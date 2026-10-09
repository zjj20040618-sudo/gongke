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
static volatile ProtoWireDiag s_wire;
static uint8_t s_binary, s_binary_buf[PROTO_BINARY_MAX_LEN];
static unsigned s_binary_len;
static uint32_t s_binary_tick;
static uint16_t s_binary_seq;
static uint8_t s_binary_have_seq;
static volatile uint16_t s_request;
static volatile uint8_t s_controlled, s_mode, s_ack, s_fresh, s_failed, s_due;
static volatile uint8_t s_receiving;
static volatile uint8_t s_task, s_selection;
static volatile int s_target_cls, s_target_label;
static volatile uint8_t s_qr_valid, s_qr_notice_pending;
static int32_t s_qr_tuple[3]; /* First legal QR of the current controlled request. */
static int32_t s_qr_notice_tuple[3]; /* Survives automatic IDLE/OBJECT transition. */
static volatile ProtoTargetRank s_rank;
static volatile uint8_t s_rank_valid;
static uint32_t s_send_tick;

static int token_int(const char *tok, int *out);
static int parse_scene(const char *tok);
static void dispatch(char *line);
static void binary_feed(uint8_t ch);
static void rank_reset(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    s_rank_valid = 0u; memset((void *)&s_rank, 0, sizeof s_rank);
    __set_PRIMASK(pm);
}

/* robot_init 挂接三连:proto_init 清收帧缓冲;proto_set_tx 注册发送口(=USART2 轮询发);
 * proto_set_on_frame 注册整帧回调(=steps 暂存)。 */
void proto_init(void)
{
    blen = dropping = 0;
    s_binary = 0u; s_binary_len = 0u; s_binary_have_seq = 0u;
    s_request = 0u; s_controlled = s_mode = s_ack = s_fresh = s_failed = s_due = 0u;
    s_receiving = 0u;
    s_task = s_selection = 0u; s_target_cls = s_target_label = -1;
    s_qr_valid = s_qr_notice_pending = 0u; memset(s_qr_tuple, 0, sizeof s_qr_tuple);
    memset(s_qr_notice_tuple, 0, sizeof s_qr_notice_tuple);
    memset((void *)&s_stats, 0, sizeof s_stats);
    memset((void *)&s_wire, 0, sizeof s_wire);
    rank_reset();
}
void proto_set_binary_mode(int enabled)
{
    s_binary = enabled ? 1u : 0u;
    s_receiving = 0u;
    blen = dropping = 0; s_binary_len = 0u; s_binary_have_seq = 0u;
    s_qr_valid = s_qr_notice_pending = 0u; memset(s_qr_tuple, 0, sizeof s_qr_tuple);
    rank_reset();
}
void proto_set_tx(void (*tx)(const char *s))          { s_tx = tx; }
void proto_set_binary_tx(void (*tx)(const uint8_t *, uint16_t)) { s_binary_tx = tx; }
void proto_set_on_frame(void (*cb)(const ProtoFrame *f)) { s_on_frame = cb; }

/* 串口逐字节喂入(USART2 RxCplt 回调里被调):攒行,\n 触发一次整帧 dispatch,孤立 \r 忽略 */
void proto_feed_byte(uint8_t ch)
{
    s_wire.rx_bytes++;
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

void proto_wire_diag_get(ProtoWireDiag *out)
{
    uint32_t pm;
    if (!out) return;
    pm = __get_PRIMASK(); __disable_irq();
    *out = s_wire;
    out->request = s_request; out->mode = s_mode;
    out->controlled = s_controlled; out->ack = s_ack;
    out->fresh = s_fresh; out->failed = s_failed;
    out->receiving = s_receiving;
    out->task = s_task; out->selection = s_selection;
    __set_PRIMASK(pm);
}

int proto_target_rank_get(ProtoTargetRank *out)
{
    uint32_t pm = __get_PRIMASK();
    int ready;
    __disable_irq();
    ready = s_binary && s_controlled && s_receiving && s_ack && !s_failed &&
            s_mode == 2u && s_rank_valid && s_rank.request == s_request &&
            s_rank.task == s_task && s_rank.digit == s_selection &&
            (s_task == PROTO_TASK_HOSTAGE || s_task == PROTO_TASK_BALL);
    if (out) {
        if (ready) *out = s_rank;
        else memset(out, 0, sizeof *out);
    }
    __set_PRIMASK(pm);
    return ready;
}

/* Cache a bounded prefix in RX context; formatting/TX belongs to DefaultTask.
 * length may be the expected size of a malformed frame. Only available bytes
 * are copied. No pointer to the parser's mutable buffer escapes the ISR. */
static void binary_reject_evidence(unsigned length)
{
    unsigned n = s_binary_len;
    if (n > length) n = length;
    if (n > PROTO_WIRE_PREFIX_LEN) n = PROTO_WIRE_PREFIX_LEN;
    s_wire.last_reject_type = s_binary_len > 2u ? s_binary_buf[2] : 0u;
    s_wire.last_reject_len = (uint16_t)length;
    s_wire.prefix_len = (uint8_t)n;
    for (unsigned i = 0u; i < n; i++) s_wire.prefix[i] = s_binary_buf[i];
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

/* model_9541 training aliases: 0 waist-drum, 2 cone, 9 common bucket. */
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

/* Only62/request-bound54 reaches here, after current ACK/window validation.
 * Maintain a separate sequence domain from01, which intentionally uses the
 * SAME sequence on camera. Never set s_fresh or call a coordinate consumer. */
static int rank_dispatch(const uint8_t *p, unsigned length)
{
    ProtoTargetRank next;
    int cls, label, selected_cls, selected_label;
    if (length != 9u || (s_task != PROTO_TASK_HOSTAGE && s_task != PROTO_TASK_BALL) ||
        !proto_target_filter((ProtoTask)s_task, s_selection, &selected_cls, &selected_label)) return 0;
    memset(&next, 0, sizeof next);
    next.task = s_task; next.digit = s_selection; next.request = s_request;
    next.sequence = read_le16(p + 1); next.target_model = p[3];
    next.rank = p[4]; next.seen_count = p[5];
    memcpy(next.slots, p + 6, sizeof next.slots);
    if (!binary_class(next.target_model, &cls, &label) || cls != selected_cls ||
        label != selected_label || next.rank > 3u || next.seen_count > 3u) return 0;
    int target_at = -1;
    for (unsigned i = 0u; i < 3u; ++i) {
        if (i >= next.seen_count) { if (next.slots[i] != 0xFFu) return 0; continue; }
        if (!binary_class(next.slots[i], &cls, &label) || cls != selected_cls) return 0;
        for (unsigned j = 0u; j < i; ++j) if (next.slots[i] == next.slots[j]) return 0;
        if (next.slots[i] == next.target_model) target_at = (int)i;
    }
    if (next.rank == 0u ? target_at >= 0 :
        (next.rank > next.seen_count || target_at != (int)next.rank - 1)) return 0;
    if (s_rank_valid) {
        uint16_t delta = (uint16_t)(next.sequence - s_rank.sequence);
        if (delta == 0u) {
            if (next.rank != s_rank.rank || next.seen_count != s_rank.seen_count ||
                memcmp(next.slots, (const void *)s_rank.slots, sizeof next.slots) != 0) {
                s_wire.rank_conflict++; return 0;
            }
            s_wire.rank_duplicate++; return 1;
        }
        if (delta >= 0x8000u) { s_wire.rank_stale++; s_stats.rejected++; return 1; }
        if (next.seen_count < s_rank.seen_count || (s_rank.rank && next.rank != s_rank.rank)) {
            s_wire.rank_conflict++; return 0;
        }
        for (unsigned i = 0u; i < s_rank.seen_count; ++i)
            if (next.slots[i] != s_rank.slots[i]) { s_wire.rank_conflict++; return 0; }
    }
    /* Getter/phase changes mask IRQs; RX update is bounded and never calls TX. */
    s_rank = next; s_rank_valid = 1u; s_wire.rank_packets++;
    return 1;
}

/* Validate the complete CRC-checked packet before any business callback. */
static int binary_dispatch(const uint8_t *p)
{
    const uint8_t type = p[0], count = p[3];
    const uint16_t sequence = read_le16(p + 1);
    ProtoFrame f;
    memset(&f, 0, sizeof f);
    f.sequence = sequence;
    if (type == 0x51u || type == 0x53u) {
        const unsigned offset = type == 0x53u ? 4u : 5u;
        if (count != 1u || (type == 0x51u && p[4] != 3u)
            || p[offset] < '1' || p[offset] > '3'
            || p[offset + 1u] < '1' || p[offset + 1u] > '3'
            || p[offset + 2u] < '1' || p[offset + 2u] > '3') return 0;
        f.type = PF_QR; f.a = p[offset] - '0';
        f.b = p[offset + 1u] - '0'; f.c = p[offset + 2u] - '0';
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
    if (type == 0x51u || type == 0x53u) {
        s_stats.qr++;
        /* This dispatcher is reached only after CRC/length validation. Legacy
         * frames cannot fill the request-bound startup cache. Keep the first
         * complete legal task choice, even while the chassis is still in R1. */
        if (s_controlled && s_mode == 1u && s_ack && !s_failed && !s_qr_valid) {
            s_qr_tuple[0] = f.a; s_qr_tuple[1] = f.b; s_qr_tuple[2] = f.c;
            memcpy(s_qr_notice_tuple, s_qr_tuple, sizeof s_qr_notice_tuple);
            s_qr_notice_pending = 1u;
            s_qr_valid = 1u;
        }
        if (s_on_frame) s_on_frame(&f);
    } else {
        s_stats.obj++;
        for (unsigned i = 0u; i < count; i++) {
            const uint8_t *obj = p + 14u + 11u * i;
            const uint16_t score = read_le16(obj + 1);
            int best = 1;
            if (!binary_class(obj[0], &f.cls, &f.label)) { s_stats.binary_unmapped++; continue; }
            /* Even if the camera returns all detections, a selected request
             * must never deliver another task/color/shape to the chassis. */
            if (s_task && (f.cls != s_target_cls
                || (s_target_label >= 0 && f.label != s_target_label))) continue;
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
    s_wire.last_type = p[0];
    /* A CRC-valid returned request is echo/loopback evidence, not an ACK.
     * Continue rejecting it; do not turn transport echo into scan success. */
    if (p[0] == 0x60u || p[0] == 0x63u) {
        s_wire.command_echo++;
        return 0;
    }
    if (p[0] == 0x61u) {
        if (length != 5u || p[3] > 2u || p[4] > 1u) return 0;
        s_wire.ack_packets++;
        if (!s_controlled || read_le16(p + 1) != s_request) {
            s_wire.ack_mismatch++; return 1;
        }
        if (!s_receiving) { s_wire.outside_phase++; return 1; }
        if (p[4] != 0u || p[3] != s_mode) {
            s_wire.ack_failed++;
            s_failed = 1u; s_ack = 0u; s_qr_notice_pending = 0u; rank_reset(); return 1;
        }
        s_ack = 1u;
        return 1;
    }
    if (p[0] == 0x62u) {
        const unsigned n = read_le16(p + 3);
        const uint8_t *body = p + 5;
        if (n + 5u != length || n < 4u) return 0;
        if (!s_controlled || read_le16(p + 1) != s_request) {
            s_wire.result_mismatch++;
            binary_reject_evidence(length + 4u);
            s_stats.rejected++; return 1;
        }
        if (!s_receiving) { s_wire.outside_phase++; return 1; }
        if (!s_ack || s_failed || s_mode == 0u) {
            s_wire.result_preack++;
            binary_reject_evidence(length + 4u);
            s_stats.rejected++; return 1;
        }
        if (s_mode == 1u && body[0] == 0x51u) {
            if (body[3] == 0u && n == 4u) { s_fresh = 1u; return 1; }
            if (body[3] != 1u || n < 5u || n != 13u + body[4]) return 0;
            /* A structurally fresh but invalid QR is not a valid task selection. */
            s_fresh = 1u;
        } else if (s_mode == 1u && body[0] == 0x53u) {
            if (body[3] == 0u && n == 4u) { s_fresh = 1u; return 1; }
            if (body[3] != 1u || n != 7u) return 0;
            s_fresh = 1u;
        } else if (s_mode == 2u && body[0] == 0x54u) {
            return rank_dispatch(body, n); /* Does not authorize coordinate freshness. */
        } else if (s_mode == 2u && body[0] == 0x01u) {
            if (body[3] > PROTO_BINARY_MAX_OBJECTS || n != 14u + 11u * body[3]) return 0;
        } else return 0;
        if (!binary_dispatch(body)) return 0;
        s_fresh = 1u;
        return 1;
    }
    /* Legacy frames remain available for standalone replay, never for an armed run. */
    if (s_controlled) {
        if (p[0] == 0x51u || p[0] == 0x53u) s_wire.legacy_qr++;
        else if (p[0] == 0x01u) s_wire.legacy_obj++;
        binary_reject_evidence(length + 4u);
        s_stats.rejected++; return 1;
    }
    return binary_dispatch(p);
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
        if (type == 0x60u) length = 8u; /* capture valid echo, never accept it */
        else if (type == 0x63u) length = 9u;
        else if (type == 0x61u) length = 9u;
        else if (type == 0x62u) {
            if (s_binary_len < 7u) return;
            length = 9u + read_le16(s_binary_buf + 5);
            if (length > PROTO_BINARY_MAX_LEN || length < 13u) {
                s_wire.bad_length++; binary_reject_evidence(length);
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
        } else if (type == 0x53u && count <= 1u) {
            length = count == 0u ? 8u : 11u;
        } else {
            s_wire.unknown_type++; binary_reject_evidence(s_binary_len);
            s_stats.binary_bad++; s_stats.rejected++; binary_drop(1u); continue;
        }
        if (s_binary_len < length) return;
        s_stats.lines++;
        if (binary_crc(s_binary_buf + 2, length - 4u) != read_le16(s_binary_buf + length - 2u)) {
            binary_reject_evidence(length);
            s_stats.crc_bad++; s_stats.rejected++;
            binary_drop(1u); /* keep subsequent headers after corrupt length/CRC */
            continue;
        }
        if (!control_dispatch(s_binary_buf + 2, length - 4u)) {
            if (type != 0x60u && type != 0x63u) s_wire.invalid_payload++;
            binary_reject_evidence(length);
            s_stats.binary_bad++; s_stats.rejected++;
        }
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
    if (s_binary) {
        uint32_t pm = __get_PRIMASK();
        __disable_irq();
        s_controlled = 1u;
        s_receiving = 1u;
        s_mode = sc == SCENE_IDLE ? 0u : sc == SCENE_QR ? 1u : 2u;
        s_task = s_selection = 0u; s_target_cls = s_target_label = -1;
        s_ack = s_fresh = 0u; s_binary_have_seq = 0u;
        rank_reset();
        s_qr_valid = 0u; memset(s_qr_tuple, 0, sizeof s_qr_tuple);
        /* A fresh QR round cancels any previous notice. Automatic IDLE/OBJECT
         * transitions preserve it: an RX IRQ may arrive after the task's
         * notice poll but before its route/mode transition. */
        if (sc == SCENE_QR) s_qr_notice_pending = 0u;
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

int proto_target_filter(ProtoTask task, uint8_t digit, int *cls, int *label)
{
    int c, l;
    if (task == PROTO_TASK_BUCKET) {
        if (digit != 0u) return 0;
        c = CLS_BUCKET; l = -1;
    } else {
        if (digit < 1u || digit > 3u) return 0;
        switch (task) {
            case PROTO_TASK_BALL: c = CLS_BALL; l = (int)digit - 1; break;
            case PROTO_TASK_TARGET: c = CLS_TARGET; l = (int)digit - 1; break;
            case PROTO_TASK_HOSTAGE: c = CLS_HOSTAGE; l = (int)digit + 2; break;
            default: return 0;
        }
    }
    if (cls) *cls = c;
    if (label) *label = l;
    return 1;
}

int proto_send_target(ProtoTask task, uint8_t digit)
{
    int cls, label;
    uint32_t pm;
    if (!s_binary || !proto_target_filter(task, digit, &cls, &label)) return 0;
    pm = __get_PRIMASK(); __disable_irq();
    if (s_request == 65535u) {
        s_failed = 1u; s_due = 0u;
        rank_reset();
        __set_PRIMASK(pm); return 0;
    }
    s_controlled = 1u; s_mode = 2u; s_receiving = 1u;
    s_task = (uint8_t)task; s_selection = digit;
    s_target_cls = cls; s_target_label = label;
    s_request++; s_ack = s_fresh = s_failed = 0u; s_due = 1u;
    s_binary_have_seq = 0u;
    rank_reset();
    /* The mission owns its locked QR tuple. Clear only parser readiness;
     * preserve the DefaultTask's pending one-shot QR success notice. */
    s_qr_valid = 0u; memset(s_qr_tuple, 0, sizeof s_qr_tuple);
    __set_PRIMASK(pm);
    return 1;
}

/* Startup, READY and R1 all share one QR request. Starting R1 must not erase
 * a task choice already scanned at power-on. Cancellation/new scene clears it. */
void proto_qr_begin(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    if (!s_binary || !s_controlled || !s_receiving || s_mode != 1u || s_failed)
        proto_send_scene(SCENE_QR);
    __set_PRIMASK(pm);
}

void proto_receive_end(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    s_receiving = 0u;
    s_due = s_ack = s_fresh = 0u;
    s_qr_valid = 0u;
    memset(s_qr_tuple, 0, sizeof s_qr_tuple);
    rank_reset();
    /* Keep the parser draining input. The next phase creates a new request,
     * so a late tail from this phase cannot become a new target coordinate. */
    __set_PRIMASK(pm);
}

void proto_qr_cancel(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    s_qr_notice_pending = 0u; /* Explicit abort/mode change is not auto transition. */
    if (s_binary && s_controlled && s_mode == 1u)
        proto_receive_end();
    __set_PRIMASK(pm);
}

int proto_qr_get(int32_t out[3])
{
    uint32_t pm = __get_PRIMASK();
    int ready;
    __disable_irq();
    ready = s_binary && s_controlled && s_receiving && s_mode == 1u && s_ack && s_fresh
            && !s_failed && s_qr_valid;
    if (ready && out) {
        out[0] = s_qr_tuple[0]; out[1] = s_qr_tuple[1]; out[2] = s_qr_tuple[2];
    }
    __set_PRIMASK(pm);
    return ready;
}

int proto_qr_take_notice(int32_t out[3])
{
    uint32_t pm = __get_PRIMASK();
    int ready;
    __disable_irq();
    ready = s_qr_notice_pending;
    if (ready) {
        if (out) memcpy(out, s_qr_notice_tuple, sizeof s_qr_notice_tuple);
        s_qr_notice_pending = 0u;
    }
    __set_PRIMASK(pm);
    return ready;
}

int proto_scene_status(void)
{
    uint32_t pm = __get_PRIMASK();
    int status;
    __disable_irq();
    status = s_failed ? -1 : (s_receiving && s_ack && (s_mode == 0u || s_fresh)) ? 1 : 0;
    __set_PRIMASK(pm);
    return status;
}

void proto_service(void)
{
    uint8_t packet[9] = {0xAAu, 0x55u, 0x60u, 0u, 0u, 0u, 0u, 0u, 0u};
    unsigned length = 8u;
    uint16_t crc;
    uint32_t pm = __get_PRIMASK(), now = HAL_GetTick();
    __disable_irq();
    if (!s_binary || !s_controlled || !s_receiving || s_failed || s_ack || !s_binary_tx
        || (!s_due && (uint32_t)(now - s_send_tick) < 500u)) {
        __set_PRIMASK(pm); return;
    }
    packet[3] = (uint8_t)s_request; packet[4] = (uint8_t)(s_request >> 8);
    if (s_task) {
        packet[2] = 0x63u; packet[5] = s_task; packet[6] = s_selection;
        length = 9u;
    } else packet[5] = s_mode;
    s_due = 0u; s_send_tick = now;
    s_wire.tx_attempts++; /* callback invoked below; not proof of physical TX */
    __set_PRIMASK(pm);
    crc = binary_crc(packet + 2, length - 4u);
    packet[length - 2u] = (uint8_t)crc; packet[length - 1u] = (uint8_t)(crc >> 8);
    s_binary_tx(packet, (uint16_t)length);
}

/* 发 PING 给视觉探活(链路测试用) */
void proto_send_ping(void) { if (!s_binary && s_tx) s_tx("PING\r\n"); }
