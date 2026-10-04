/*
 * 初学者导读：视觉串口协议解析器。字节通过长度和校验后才转成 ProtoFrame。
 * 当前 robot_init 选择二进制；旧 ASCII 文本解析仍保留，不能混作当前线缆格式。
 * 一个“帧”是一次完整消息；帧头 AA55 用来找起点，CRC 用来检查传输内容是否损坏。
 * request_id 区分哪次模式请求；sequence 区分结果帧序号，两者用途不同。
 * static void (*s_on_frame)(const ProtoFrame *) 是函数指针：保存“收到帧后要调用谁”。
 * 注册时传函数名，接收后调用 s_on_frame(&f)；与 Python 把函数当参数传递有相似用途。
 * QR 与 OBJ 的字段含义见 proto.h；像素坐标还不是毫米，也不是夹爪位置。
 */

#include "proto.h"
#include "main.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>

static void (*s_tx)(const char *) = 0;
static void (*s_binary_tx)(const uint8_t *, uint16_t) = 0;
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
static volatile uint16_t s_request;
static volatile uint8_t s_controlled, s_mode, s_ack, s_fresh, s_failed, s_due;
static uint32_t s_send_tick;

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
    s_request = 0u; s_controlled = s_mode = s_ack = s_fresh = s_failed = s_due = 0u;
    memset((void *)&s_stats, 0, sizeof s_stats);
}
void proto_set_binary_mode(int enabled)
{
    s_binary = enabled ? 1u : 0u;
    blen = dropping = 0; s_binary_len = 0u; s_binary_have_seq = 0u;
}
void proto_set_tx(void (*tx)(const char *s))          { s_tx = tx; }
void proto_set_binary_tx(void (*tx)(const uint8_t *, uint16_t)) { s_binary_tx = tx; }
/**
 * @brief 保存完整视觉帧的接收回调地址。
 * @param cb 返回void、参数为只读ProtoFrame指针的函数；当前注册robot_vision_frame。
 * @retval 无。
 * @note 函数指针不是普通数据指针；回调接收的是临时帧地址，要保留内容需复制结构体。
 */
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
    /* strchr找到逗号地址，把逗号改为结束符，再把下一段地址放进tok数组；没有复制字符串。 */
    while (n < PROTO_FIELDS && (p = strchr(p, ','))) { *p++ = '\0'; tok[n++] = p; }

    /* sizeof f得到整份结构体的字节数，memset把这些字节清零；&f是起始地址。 */
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

/**
 * @brief 从两个连续字节读取小端16位整数。
 * @param p 指向至少两个字节；p[0]是低8位，p[1]是高8位。
 * @retval 合成的无符号16位数。
 * @note 例如字节34 12表示0x1234；<<8将高字节移到高8位，|合并两部分。
 */
static uint16_t read_le16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/**
 * @brief 计算当前协议的CRC16/CCITT-FALSE。
 * @param p 待校验字节区域首地址。
 * @param length 本区域字节数；不包含帧头和末尾CRC。
 * @retval 16位CRC值。
 * @note ^是按位异或，不是乘方；每个字节处理8个bit，0x1021是校验算法多项式。
 */
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

/* model_9541 training aliases: 0 waist-drum, 2 cone, 9 common bucket. */
/**
 * @brief 把视觉模型类别号转换成电控类别和标签。
 * @param model_class 模型类别0..9。
 * @param cls 输出地址，写入球/靶/人质/桶类别。
 * @param label 输出地址，写入颜色或形状标签。
 * @retval 1=已转换，0=不支持的类别。
 * @note 一个return只能直接返回一个值，因此另外两个结果通过指针参数写回。
 */
static int binary_class(uint8_t model_class, int *cls, int *label)
{
    switch (model_class) {
        case 0: *cls = CLS_HOSTAGE; *label = LAB_WAIST; return 1;
        case 1: *cls = CLS_HOSTAGE; *label = LAB_CYL; return 1;
        case 2: *cls = CLS_HOSTAGE; *label = LAB_CONE; return 1;
        case 3: *cls = CLS_BALL; *label = LAB_B; return 1;
        case 4: *cls = CLS_BALL; *label = LAB_R; return 1;
        case 5: *cls = CLS_BALL; *label = LAB_G; return 1;
        case 6: *cls = CLS_TARGET; *label = LAB_R; return 1;
        case 7: *cls = CLS_TARGET; *label = LAB_B; return 1;
        case 8: *cls = CLS_TARGET; *label = LAB_G; return 1;
        case 9: *cls = CLS_BUCKET; *label = 0; return 1;
        default: return 0;
    }
}

/* Validate the complete CRC-checked packet before any business callback. */
/**
 * @brief 先验证整包字段，再按类别选最高分目标回调给业务层。
 * @param p 内层业务帧首地址；调用前已完成长度与CRC检查。
 * @retval 1=处理成功（含重复帧被忽略），0=字段非法。
 * @note 先检查全部目标，再开始回调，防止只接受坏包的前半部分。
 */
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
        /* 字符'3'不是整数3；减字符'0'后才得到数值3，前面已限定字符范围1..3。 */
        f.type = PF_QR; f.a = p[5] - '0'; f.b = p[6] - '0'; f.c = p[7] - '0';
    } else {
        f.img_w = read_le16(p + 4); f.img_h = read_le16(p + 6);
        if (f.img_w == 0u || f.img_h == 0u) return 0;
        for (unsigned i = 0u; i < count; i++) {
            const uint8_t *obj = p + 14u + 11u * i;
            const uint16_t score = read_le16(obj + 1);
            const uint16_t cx = read_le16(obj + 3), cy = read_le16(obj + 5);
            const uint16_t w = read_le16(obj + 7), h = read_le16(obj + 9);
            if (obj[0] > 9u || score > 1000u || cx >= f.img_w || cy >= f.img_h
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

/* Called only after outer CRC and exact length validation. Never transmit in RX ISR. */
static int control_dispatch(const uint8_t *p, unsigned length)
{
    if (p[0] == 0x61u) {
        if (length != 5u || p[3] > 2u || p[4] > 1u) return 0;
        if (!s_controlled || read_le16(p + 1) != s_request) return 1;
        if (p[4] != 0u || p[3] != s_mode) { s_failed = 1u; s_ack = 0u; return 1; }
        s_ack = 1u;
        return 1;
    }
    if (p[0] == 0x62u) {
        const unsigned n = read_le16(p + 3);
        const uint8_t *body = p + 5;
        if (n + 5u != length || n < 4u) return 0;
        if (!s_controlled || !s_ack || s_failed || s_mode == 0u
            || read_le16(p + 1) != s_request) { s_stats.rejected++; return 1; }
        if (s_mode == 1u && body[0] == 0x51u) {
            if (body[3] == 0u && n == 4u) { s_fresh = 1u; return 1; }
            if (body[3] != 1u || n < 5u || n != 13u + body[4]) return 0;
            /* A structurally fresh but invalid QR is not a valid task selection. */
            s_fresh = 1u;
        } else if (s_mode == 2u && body[0] == 0x01u) {
            if (body[3] > PROTO_BINARY_MAX_OBJECTS || n != 14u + 11u * body[3]) return 0;
        } else return 0;
        if (!binary_dispatch(body)) return 0;
        s_fresh = 1u;
        return 1;
    }
    /* Legacy frames remain available for standalone replay, never for an armed run. */
    if (s_controlled) { s_stats.rejected++; return 1; }
    return binary_dispatch(p);
}

static void binary_drop(unsigned length)
{
    /* memmove会把后面的未处理字节前移；源和目的区重叠，所以这里不能随意换memcpy。 */
    s_binary_len -= length;
    memmove(s_binary_buf, s_binary_buf + length, s_binary_len);
}

/**
 * @brief 逐字节拼二进制帧，依次检查帧头、长度、CRC与业务结构。
 * @param ch 当前新收到的字节。
 * @retval 无。
 * @note 长度未收够就return等下次调用；校验失败只丢开头一个字节，再找后续AA55。
 */
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
        if (type == 0x61u) length = 9u;
        else if (type == 0x62u) {
            if (s_binary_len < 7u) return;
            length = 9u + read_le16(s_binary_buf + 5);
            if (length > PROTO_BINARY_MAX_LEN || length < 13u) {
                s_stats.binary_bad++; s_stats.rejected++; binary_drop(1u); continue;
            }
        }
        else if (type == 0x01u && count <= PROTO_BINARY_MAX_OBJECTS) length = 18u + 11u * count;
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
        if (!control_dispatch(s_binary_buf + 2, length - 4u)) { s_stats.binary_bad++; s_stats.rejected++; }
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
        /* 十进制逐位累加：读到1是1，再读2得到1*10+2=12；前面先检查整数溢出。 */
        v = v * 10 + digit;
        ++p;
    }
    if (!have || *p != '\0') return 0;
    *out = neg ? -v : v;
    return 1;
}

/* 发 "SET,scene" 给视觉:切当前任务的上报场景(mission 每区开头调) */
/**
 * @brief 申请视觉切换到所需模式，更新请求号以隔离旧帧。
 * @param sc SCENE_IDLE停止、SCENE_QR扫码；其余任务场景都请求OBJECT识别。
 * @retval 无。
 * @note 当前二进制分支仅登记待发请求，真正发送在DefaultTask的proto_service中。
 */
void proto_send_scene(ProtoScene sc)
{
    if (s_binary) {
        uint32_t pm = __get_PRIMASK();
        __disable_irq();
        s_controlled = 1u;
        s_mode = sc == SCENE_IDLE ? 0u : sc == SCENE_QR ? 1u : 2u;
        s_ack = s_fresh = 0u; s_binary_have_seq = 0u;
        /* Do not silently reuse an in-run request number on wrap. */
        if (s_request == 65535u) { s_failed = 1u; s_due = 0u; }
        else { s_request++; s_failed = 0u; s_due = 1u; }
        __set_PRIMASK(pm);
        return;
    }
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

/**
 * @brief 读取当前请求的握手进度。
 * @retval -1=失败，0=等待，1=收到成功ACK和本轮结构有效结果；IDLE只需ACK。
 * @note 返回1不等于看到了任务目标，OBJECT空包也可能合法，业务需另查目标帧。
 */
int proto_scene_status(void)
{
    uint32_t pm = __get_PRIMASK();
    int status;
    __disable_irq();
    status = s_failed ? -1 : (s_ack && (s_mode == 0u || s_fresh)) ? 1 : 0;
    __set_PRIMASK(pm);
    return status;
}

void proto_service(void)
{
    uint8_t packet[8] = {0xAAu, 0x55u, 0x60u, 0u, 0u, 0u, 0u, 0u};
    uint16_t crc;
    uint32_t pm = __get_PRIMASK(), now = HAL_GetTick();
    __disable_irq();
    if (!s_binary || !s_controlled || s_failed || s_ack || !s_binary_tx
        || (!s_due && (uint32_t)(now - s_send_tick) < 500u)) {
        __set_PRIMASK(pm); return;
    }
    packet[3] = (uint8_t)s_request; packet[4] = (uint8_t)(s_request >> 8);
    packet[5] = s_mode; s_due = 0u; s_send_tick = now;
    __set_PRIMASK(pm);
    crc = binary_crc(packet + 2, 4u);
    packet[6] = (uint8_t)crc; packet[7] = (uint8_t)(crc >> 8);
    s_binary_tx(packet, sizeof packet);
}

/* 发 PING 给视觉探活(链路测试用) */
void proto_send_ping(void) { if (!s_binary && s_tx) s_tx("PING\r\n"); }
