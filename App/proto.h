#ifndef PROTO_H
#define PROTO_H

#include <stdint.h>

#define PROTO_MAX_LEN 48
#define PROTO_FIELDS  8
#define PROTO_BINARY_MAX_OBJECTS 10u
#define PROTO_BINARY_MAX_LEN     272u /* one QR, UTF-8 payload up to 255 bytes */
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

/* OBJ: class + label 编码（按 proto.md 语义） */
enum { CLS_BALL = 0, CLS_TARGET, CLS_HOSTAGE, CLS_BUCKET };
enum { LAB_R = 0, LAB_G, LAB_B,          /* ball/target 颜色 */
       LAB_CYL, LAB_CONE, LAB_WAIST };   /* hostage 形状 */
/* CLS_BUCKET(排爆桶):EOD 放桶也要视觉锁对准 → MaixCam 需在 EOD 场景上报桶。
 *   全场单只无色、无同类并列 → 对齐时不挑 label(step_align 传 -1),
 *   视觉报个固定 label 占位即可。⚠️ 协议新增,待视觉队友确认。 */

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

void proto_init(void);
void proto_set_binary_mode(int enabled); /* before arming RX; no autodetection */
void proto_set_tx(void (*tx)(const char *s));          /* 用户提供串口发送 */
void proto_set_on_frame(void (*cb)(const ProtoFrame *f));
void proto_feed_byte(uint8_t ch);                       /* 每收到 1 字节调一次 */
void proto_stats_get(ProtoStats *out);

/* Legacy ASCII TX only. Binary RX mode leaves these no-op until commands are agreed. */
void proto_send_scene(ProtoScene sc);
void proto_send_ping(void);

#endif
