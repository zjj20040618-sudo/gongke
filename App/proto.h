#ifndef PROTO_H
#define PROTO_H

#include <stdint.h>

#define PROTO_MAX_LEN 48
#define PROTO_FIELDS  8
#define PROTO_BINARY_MAX_OBJECTS 10u
#define PROTO_BINARY_MAX_LEN     280u /* legacy payload plus request envelope */
#define PROTO_BINARY_GAP_MS      100u /* incomplete-frame gap, not mission timeout */

typedef enum {
    PF_NONE = 0,
    PF_READY, PF_PONG, PF_PING,          /* 无字段 */
    PF_QR,                               /* a,b,c = 排爆球色/反恐靶色/救援形状；必须三项齐全 */
    PF_OBJ,                              /* cls,label,cx,cy,w,h,conf */
    PF_ERR,                              /* a=dx,b=dy,c=ok */
    PF_SET,                              /* a = scene enum */
    PF_UNKNOWN
} ProtoType;

typedef enum { SCENE_IDLE = 0, SCENE_QR, SCENE_EOD, SCENE_ANTI, SCENE_RESCUE } ProtoScene;

/* 0x63 task selector: task + QR digit, NOT detector/model class_id.
 * Ball/target/hostage take numeric 1..3; the common bucket takes 0. */
typedef enum {
    PROTO_TASK_BALL = 1, PROTO_TASK_TARGET, PROTO_TASK_HOSTAGE, PROTO_TASK_BUCKET
} ProtoTask;

/* OBJ: class + label 编码（按 proto.md 语义） */
enum { CLS_BALL = 0, CLS_TARGET, CLS_HOSTAGE, CLS_BUCKET };
enum { LAB_R = 0, LAB_G, LAB_B,          /* ball/target 颜色 */
       LAB_CYL, LAB_CONE, LAB_WAIST };   /* hostage 形状 */
/* CLS_BUCKET(排爆桶):EOD 放桶也要视觉锁对准 → MaixCam 需在 EOD 场景上报桶。
 *   全场单只无色、无同类并列 → 对齐时不挑 label(step_align 传 -1),
 *   当前9541模型ID9映射为CLS_BUCKET/label0；实际桶口工作点仍须标定。 */

typedef struct {
    ProtoType type;
    int a, b, c;        /* QR / ERR / SET */
    int cls, label;     /* OBJ */
    int cx, cy, w, h;   /* OBJ 像素 */
    int conf;           /* 0..100 */
    uint16_t sequence, img_w, img_h; /* binary metadata; ASCII leaves zero */
} ProtoFrame;

typedef struct {
    uint32_t lines;
    uint32_t accepted;
    uint32_t qr;
    uint32_t obj;
    uint32_t rejected;
    uint32_t overflow;
    uint32_t crc_bad, binary_bad, binary_gap, binary_unmapped, duplicate;
} ProtoStats;

/* Request-bound historical first-seen order (62 inner54), NOT coordinates,
 * visibility, alignment or a grab-complete signal. rank0 means unknown.
 * Only BALL task1 and HOSTAGE task3. Slots are detector model IDs, never QR
 * digits or ranks. BALL54 uses3/4/5, HOSTAGE54 uses0/1/2; groups cannot mix.
 * TARGET/BUCKET54 is rejected; rank54 never authorizes coordinate freshness. */
typedef struct {
    uint8_t task, digit, target_model, rank, seen_count, slots[3];
    uint16_t request, sequence;
} ProtoTargetRank;

/* Read-only wire evidence. A rejected/legacy/echoed packet NEVER authorizes
 * QR or motion. Counters are cumulative since proto_init; request is live. */
#define PROTO_WIRE_PREFIX_LEN 16u
typedef struct {
    uint32_t rx_bytes, tx_attempts;
    uint32_t ack_packets, ack_mismatch, ack_failed;
    uint32_t result_preack, result_mismatch;
    uint32_t legacy_qr, legacy_obj, command_echo, unknown_type;
    uint32_t bad_length, invalid_payload, outside_phase;
    uint32_t rank_packets, rank_duplicate, rank_stale, rank_conflict;
    uint16_t request, last_reject_len;
    uint8_t mode, controlled, ack, fresh, failed, receiving;
    uint8_t task, selection; /* 0 task = generic 0x60 session */
    uint8_t last_type, last_reject_type, prefix_len;
    uint8_t prefix[PROTO_WIRE_PREFIX_LEN];
} ProtoWireDiag;

void proto_init(void);
void proto_set_binary_mode(int enabled); /* before arming RX; no autodetection */
void proto_set_tx(void (*tx)(const char *s));          /* 用户提供串口发送 */
void proto_set_binary_tx(void (*tx)(const uint8_t *data, uint16_t length));
void proto_set_on_frame(void (*cb)(const ProtoFrame *f));
void proto_feed_byte(uint8_t ch);                       /* 每收到 1 字节调一次 */
void proto_stats_get(ProtoStats *out);
void proto_wire_diag_get(ProtoWireDiag *out);
/* Non-consuming IRQ-safe snapshot of the current selected BALL/HOSTAGE54.
 * Return1 for a legal received status, INCLUDING rank0 unknown; return0 and
 * zero out when none/current request notACKed/closed/failed. Lifetime is only
 * this RX phase: copy before local receive_end/new task/cancel/reset/NACK.
 * No coordinate freshness is conferred, and no motion callback is emitted. */
int proto_target_rank_get(ProtoTargetRank *out);

/* Binary: queue a new request; only proto_service (DefaultTask) transmits. */
void proto_send_scene(ProtoScene sc);
/* Pure validation/mapping. Invalid task/digit leaves output pointers untouched. */
int proto_target_filter(ProtoTask task, uint8_t digit, int *cls, int *label);
/* Queue a request-bound selected OBJECT session (0x63). Binary only; no TX in
 * this call. Returns 0 on invalid selection/unavailable request number. Camera
 * needs matching 0x63 support; never fall back to generic OBJECT confirmation. */
int proto_send_target(ProtoTask task, uint8_t digit);
/* Reuse the current nonfailed controlled QR request; otherwise queue a new
 * one. No transmission, blocking wait or chassis motion in this entry. */
void proto_qr_begin(void);
/* End only local business reception/retries. UART still drains/validates input;
 * never sends an IDLE/STOP command to the camera. A pending validated QR OK
 * notice survives; callers must copy their QR tuple before ending its phase. */
void proto_receive_end(void);
/* Cancel only an active QR session, including a pending/failed one. */
void proto_qr_cancel(void);
/* Non-consuming first legal 1..3 task tuple bound to current QR ACK/request.
 * New scene/cancellation/init clears it. out may be NULL for readiness only. */
int proto_qr_get(int32_t out[3]);
/* DefaultTask consumes one validated success notice per QR request, never RX
 * ISR. Automatic IDLE/OBJECT preserves pending notice; cancel/new QR clears.
 * This report snapshot does not authorize motion (only proto_qr_get does). */
int proto_qr_take_notice(int32_t out[3]);
void proto_service(void);
int proto_scene_status(void); /* -1 failed, 0 waiting, 1 ACK + fresh frame (IDLE: ACK only) */
void proto_send_ping(void);

#endif
