#include "proto.h"
#include "main.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>

static void (*s_tx)(const char *) = 0;
static void (*s_on_frame)(const ProtoFrame *) = 0;

static char buf[PROTO_MAX_LEN];
static int  blen = 0;
static int  dropping = 0;
static volatile ProtoStats s_stats;
static uint8_t s_binary, s_binary_buf[PROTO_BINARY_MAX_LEN];
static unsigned s_binary_len;
static uint32_t s_binary_tick;
static uint16_t s_binary_seq;
static uint8_t s_binary_have_seq;

static int token_int(const char *tok, int *out);
static int parse_scene(const char *tok);
static void dispatch(char *line);
static void binary_feed(uint8_t ch);

/* robot_init 挂接三连:proto_init 清收帧缓冲;proto_set_tx 注册发送口(=USART2 轮询发);
 * proto_set_on_frame 注册整帧回调(=steps 暂存)。 */
void proto_init(void)
{
    blen = dropping = 0;
    s_binary = 0u; s_binary_len = 0u; s_binary_have_seq = 0u;
    memset((void *)&s_stats, 0, sizeof s_stats);
}
void proto_set_binary_mode(int enabled)
{
    s_binary = enabled ? 1u : 0u;
    blen = dropping = 0; s_binary_len = 0u; s_binary_have_seq = 0u;
}
void proto_set_tx(void (*tx)(const char *s))          { s_tx = tx; }
void proto_set_on_frame(void (*cb)(const ProtoFrame *f)) { s_on_frame = cb; }

/* 串口逐字节喂入(USART2 RxCplt 回调里被调):攒行,\n 触发一次整帧 dispatch,孤立 \r 忽略 */
void proto_feed_byte(uint8_t ch)
{
    if (s_binary) { binary_feed(ch); return; }
    if (ch == '\r') return;                  /* 忽略帧内 \r，仅 \n 作尾 */
    if (ch == '\n') {
        if (!dropping) {
            s_stats.lines++;
            buf[blen] = '\0';
            if (s_on_frame) dispatch(buf);
        }
        blen = 0;
        dropping = 0;
    } else if (dropping) {
        return;                               /* 超长后丢到行尾，禁止尾巴伪装成新帧 */
    } else if (blen < PROTO_MAX_LEN - 1) {
        buf[blen++] = (char)ch;
    } else {
        blen = 0;
        dropping = 1;
        s_stats.overflow++;
    }
}

/* token 与字面量是否相等(dispatch 里比首字段/值用) */
static int tokcmp(const char *tok, const char *lit) { return strcmp(tok, lit) == 0; }

/* 场景名字符串(idle/qr/eod/anti/rescue)→ 场景枚举;不认识回 -1 */
static int parse_scene(const char *tok)
{
    if (tokcmp(tok, "idle")) return SCENE_IDLE;
    if (tokcmp(tok, "qr"))   return SCENE_QR;
    if (tokcmp(tok, "eod"))  return SCENE_EOD;
    if (tokcmp(tok, "anti")) return SCENE_ANTI;
    if (tokcmp(tok, "rescue")) return SCENE_RESCUE;
    return -1;
}

/* 整行按首字段分派:READY/PONG/PING/QR/ERR/SET/OBJ → 填 ProtoFrame 回调出去 */
static void dispatch(char *line)
{
    char *tok[PROTO_FIELDS];
    int n = 0;
    tok[n++] = line;
    char *p = line;
    while (n < PROTO_FIELDS && (p = strchr(p, ','))) { *p++ = '\0'; tok[n++] = p; }

    ProtoFrame f; memset(&f, 0, sizeof f);
    f.type = PF_UNKNOWN;   /* 字段数对但数值解析失败也必须拒收，不能误计为 PF_NONE 已接受 */

    if (tokcmp(tok[0], "READY"))            { f.type = PF_READY; }
    else if (tokcmp(tok[0], "PONG"))        { f.type = PF_PONG; }
    else if (tokcmp(tok[0], "PING"))        { f.type = PF_PING; }
    /* QR 必须且只能带三个任务选择值；缺字段或多字段都不能伪装成完整识别结果。 */
    else if (tokcmp(tok[0], "QR") && n == 4) {
        if (token_int(tok[1], &f.a) && token_int(tok[2], &f.b) && token_int(tok[3], &f.c))
            f.type = PF_QR;
    }
    else if (tokcmp(tok[0], "ERR") && n >= 4) {
        if (token_int(tok[1], &f.a) && token_int(tok[2], &f.b) && token_int(tok[3], &f.c))
            f.type = PF_ERR;
    }
    else if (tokcmp(tok[0], "SET") && n >= 2) {
        f.a = parse_scene(tok[1]);
        if (f.a >= 0) f.type = PF_SET;
    }
    else if (tokcmp(tok[0], "OBJ") && n >= 8) {
        if (token_int(tok[1], &f.cls) && token_int(tok[2], &f.label)
            && token_int(tok[3], &f.cx) && token_int(tok[4], &f.cy)
            && token_int(tok[5], &f.w) && token_int(tok[6], &f.h)
            && token_int(tok[7], &f.conf)) f.type = PF_OBJ;
    }
    else f.type = PF_UNKNOWN;

    if (f.type != PF_UNKNOWN) {
        s_stats.accepted++;
        if (f.type == PF_QR) s_stats.qr++;
        if (f.type == PF_OBJ) s_stats.obj++;
        s_on_frame(&f);
    } else {
        s_stats.rejected++;
    }
}

void proto_stats_get(ProtoStats *out)
{
    uint32_t pm;
    if (!out) return;
    pm = __get_PRIMASK();
    __disable_irq();
    *out = s_stats;
    __set_PRIMASK(pm);
}

static uint16_t read_le16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t binary_crc(const uint8_t *p, unsigned length)
{
    uint16_t crc = 0xFFFFu;
    while (length--) {
        crc ^= (uint16_t)((uint16_t)*p++ << 8);
        for (unsigned bit = 0u; bit < 8u; bit++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

/* 0 oblate / 2 truncated_cone are unresolved. Current model has no bucket. */
static int binary_class(uint8_t model_class, int *cls, int *label)
{
    switch (model_class) {
        case 1: *cls = CLS_HOSTAGE; *label = LAB_CYL; return 1;
        case 3: *cls = CLS_BALL; *label = LAB_B; return 1;
        case 4: *cls = CLS_BALL; *label = LAB_R; return 1;
        case 5: *cls = CLS_BALL; *label = LAB_G; return 1;
        case 6: *cls = CLS_TARGET; *label = LAB_R; return 1;
        case 7: *cls = CLS_TARGET; *label = LAB_B; return 1;
        case 8: *cls = CLS_TARGET; *label = LAB_G; return 1;
        default: return 0;
    }
}

/* Validate the complete CRC-checked packet before any business callback. */
static int binary_dispatch(const uint8_t *p)
{
    const uint8_t type = p[0], count = p[3];
    const uint16_t sequence = read_le16(p + 1);
    ProtoFrame f;
    memset(&f, 0, sizeof f);
    f.sequence = sequence;
    if (type == 0x51u) {
        if (count != 1u || p[4] != 3u || p[5] < '1' || p[5] > '3'
            || p[6] < '1' || p[6] > '3' || p[7] < '1' || p[7] > '3') return 0;
        f.type = PF_QR; f.a = p[5] - '0'; f.b = p[6] - '0'; f.c = p[7] - '0';
    } else {
        f.img_w = read_le16(p + 4); f.img_h = read_le16(p + 6);
        if (f.img_w == 0u || f.img_h == 0u) return 0;
        for (unsigned i = 0u; i < count; i++) {
            const uint8_t *obj = p + 14u + 11u * i;
            const uint16_t score = read_le16(obj + 1);
            const uint16_t cx = read_le16(obj + 3), cy = read_le16(obj + 5);
            const uint16_t w = read_le16(obj + 7), h = read_le16(obj + 9);
            if (obj[0] > 8u || score > 1000u || cx >= f.img_w || cy >= f.img_h
                || w == 0u || h == 0u || w > f.img_w || h > f.img_h) return 0;
        }
    }
    if (s_binary_have_seq && sequence == s_binary_seq) { s_stats.duplicate++; return 1; }
    s_binary_seq = sequence; s_binary_have_seq = 1u;
    s_stats.accepted++;
    if (type == 0x51u) {
        s_stats.qr++;
        if (s_on_frame) s_on_frame(&f);
    } else {
        s_stats.obj++;
        for (unsigned i = 0u; i < count; i++) {
            const uint8_t *obj = p + 14u + 11u * i;
            const uint16_t score = read_le16(obj + 1);
            int best = 1;
            if (!binary_class(obj[0], &f.cls, &f.label)) { s_stats.binary_unmapped++; continue; }
            for (unsigned j = 0u; j < count; j++) {
                const uint8_t *other = p + 14u + 11u * j;
                if (j != i && other[0] == obj[0]
                    && (read_le16(other + 1) > score || (read_le16(other + 1) == score && j < i))) best = 0;
            }
            if (!best) continue;
            f.type = PF_OBJ; f.cx = read_le16(obj + 3); f.cy = read_le16(obj + 5);
            f.w = read_le16(obj + 7); f.h = read_le16(obj + 9);
            f.conf = (int)((score + 5u) / 10u); /* 0..1000 -> existing 0..100 */
            if (s_on_frame) s_on_frame(&f);
        }
    }
    return 1;
}

static void binary_drop(unsigned length)
{
    s_binary_len -= length;
    memmove(s_binary_buf, s_binary_buf + length, s_binary_len);
}

static void binary_feed(uint8_t ch)
{
    const uint32_t now = HAL_GetTick();
    if (s_binary_len && (uint32_t)(now - s_binary_tick) > PROTO_BINARY_GAP_MS) {
        s_binary_len = 0u; s_stats.binary_gap++;
    }
    s_binary_tick = now;
    if (s_binary_len >= PROTO_BINARY_MAX_LEN) { s_binary_len = 0u; s_stats.overflow++; }
    s_binary_buf[s_binary_len++] = ch;
    while (s_binary_len) {
        unsigned length;
        uint8_t type, count;
        if (s_binary_buf[0] != 0xAAu) { binary_drop(1u); continue; }
        if (s_binary_len < 2u) return;
        if (s_binary_buf[1] != 0x55u) { binary_drop(1u); continue; }
        if (s_binary_len < 6u) return;
        type = s_binary_buf[2]; count = s_binary_buf[5];
        if (type == 0x01u && count <= PROTO_BINARY_MAX_OBJECTS) length = 18u + 11u * count;
        else if (type == 0x51u && count <= 1u) {
            if (count == 0u) length = 8u;
            else {
                if (s_binary_len < 7u) return;
                length = 17u + s_binary_buf[6];
            }
        } else { s_stats.binary_bad++; s_stats.rejected++; binary_drop(1u); continue; }
        if (s_binary_len < length) return;
        s_stats.lines++;
        if (binary_crc(s_binary_buf + 2, length - 4u) != read_le16(s_binary_buf + length - 2u)) {
            s_stats.crc_bad++; s_stats.rejected++;
            binary_drop(1u); /* keep subsequent headers after corrupt length/CRC */
            continue;
        }
        if (!binary_dispatch(s_binary_buf + 2)) { s_stats.binary_bad++; s_stats.rejected++; }
        binary_drop(length);
    }
}

/* 严格解析一个完整的有符号十进制字段；空串、孤立负号、夹杂字符一律拒收。 */
static int token_int(const char *tok, int *out)
{
    int v = 0, neg = 0, have = 0;
    const char *p = tok;
    if (!tok || !out) return 0;
    if (*p == '-') { neg = 1; ++p; }
    while (*p >= '0' && *p <= '9') {
        int digit = *p - '0';
        have = 1;
        if (v > (INT_MAX - digit) / 10) return 0;
        v = v * 10 + digit;
        ++p;
    }
    if (!have || *p != '\0') return 0;
    *out = neg ? -v : v;
    return 1;
}

/* 发 "SET,scene" 给视觉:切当前任务的上报场景(mission 每区开头调) */
void proto_send_scene(ProtoScene sc)
{
    /* Current Maix binary app has no RX command handler. Do not send ASCII to it. */
    if (s_binary) return;
    const char *name = "idle";
    switch (sc) {
        case SCENE_QR: name = "qr"; break;
        case SCENE_EOD: name = "eod"; break;
        case SCENE_ANTI: name = "anti"; break;
        case SCENE_RESCUE: name = "rescue"; break;
        default: break;
    }
    char s[32];
    int n = snprintf(s, sizeof s, "SET,%s\r\n", name);
    (void)n;
    if (s_tx) s_tx(s);
}

/* 发 PING 给视觉探活(链路测试用) */
void proto_send_ping(void) { if (!s_binary && s_tx) s_tx("PING\r\n"); }
