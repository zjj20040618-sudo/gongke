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

static int token_int(const char *tok, int *out);
static int parse_scene(const char *tok);
static void dispatch(char *line);

/* robot_init 挂接三连:proto_init 清收帧缓冲;proto_set_tx 注册发送口(=USART2 轮询发);
 * proto_set_on_frame 注册整帧回调(=steps 暂存)。 */
void proto_init(void) { blen = 0; dropping = 0; memset((void *)&s_stats, 0, sizeof s_stats); }
void proto_set_tx(void (*tx)(const char *s))          { s_tx = tx; }
void proto_set_on_frame(void (*cb)(const ProtoFrame *f)) { s_on_frame = cb; }

/* 串口逐字节喂入(USART2 RxCplt 回调里被调):攒行,\n 触发一次整帧 dispatch,孤立 \r 忽略 */
void proto_feed_byte(uint8_t ch)
{
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
void proto_send_ping(void) { if (s_tx) s_tx("PING\r\n"); }
