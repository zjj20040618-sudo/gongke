#ifndef APP_VISION_ALIGN_TEST_H
#define APP_VISION_ALIGN_TEST_H

#include <stdint.h>
#include "proto.h"

/* Independent bench candidates; never write mission/32 workpoints. Image
 * coordinates and signs still require physical validation after installation. */
#define VAT_X_PX               190
#define VAT_BALL_Y_PX          420
#define VAT_BUCKET_Y_PX        400
#define VAT_HOSTAGE_Y_PX       220
#define VAT_TOL_PX             10
/* Overrun tuning: integer Bluetooth v16, below one third of former v50.
 * Includes initial ball/hostage search; yaw/180 turn tuning is independent. */
#define VAT_SPEED_MMS          16.0f
#define VAT_FRESH_MS           300u
#define VAT_GOOD_FRAMES        5u
#define VAT_POLL_MS            20u
#define VAT_PLACEHOLDER_MS     5000u

typedef enum {
    VAT_OFF = 0, VAT_BRAKE, VAT_YAW_FIX, VAT_RECHECK, VAT_ALIGN,
    VAT_HOLD_BALL, VAT_TURN_REQUEST, VAT_TURN_ACTIVE, VAT_HOLD_BUCKET,
    VAT_DONE, VAT_STOPPED
} VisionAlignTestState;

typedef struct {
    VisionAlignTestState state;
    uint8_t mode, task, digit, good, axis, latest;
    int cls, label, cx, cy;
    uint16_t sequence, img_w, img_h, request;
    uint32_t age_ms;
    float heading_target_deg, heading_error_deg, vx, vy, w;
    const char *reason; /* Static ASCII string, never an ISR-formatted buffer. */
} VisionAlignTestStatus;

void vision_align_test_init(void);
/* Wrapper must open a NEW QR request when selecting38..41 and start only after
 * g plus its legal QR. This entry also verifies qr exactly matches proto_qr_get.
 * 38=ball,39=bucket,40=hostage,41=ball/hold5/+180/bucket/hold5.
 * heading_kp is caller-owned0..5. Start captures continuous absolute IMU yaw;
 * it never zeroes heading/encoders, blocks, drives arm or turns laser on. */
int vision_align_test_start(uint8_t mode, const int32_t qr[3], float heading_kp);
void vision_align_test_poll(void); /* Owner task, nonblocking, internally20ms. */
void vision_align_test_feed_frame(const ProtoFrame *frame); /* Parser callback. */
void vision_align_test_cancel(void); /* Immediate brake/localRX close/laser off. */
int vision_align_test_active(void);
void vision_align_test_status(VisionAlignTestStatus *out);
/* Only41: take once, then wrapper starts the EXISTING mode22 +180 controller.
 * Poll is inert while VAT_TURN_ACTIVE so it cannot brake/overwrite mode22.
 * Notify only after controller DONE/STOP; success still requires restoring
 * the ORIGINAL absolute heading +180, not accepting residual current yaw. */
int vision_align_test_take_turn_request(void);
void vision_align_test_notify_turn_result(int success);

#endif
