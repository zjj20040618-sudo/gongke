#ifndef APP_VISION_ALIGN_TEST_H
#define APP_VISION_ALIGN_TEST_H

#include <stdint.h>
#include "proto.h"
#include "turn_profile.h"

/* Independent bench candidates; never write mission/32 workpoints. Image
 * coordinates and signs still require physical validation after installation. */
#define VAT_X_PX               190
#define VAT_BALL_Y_PX          420
#define VAT_BUCKET_Y_PX        400
#define VAT_HOSTAGE_Y_PX       220
#define VAT_TOL_PX             10
/* X/search retains the user's accurate v20; lateral Y now uses v30.
 * Stop Y, restore the original heading, then revalidate NEW joint X/Y frames.
 * This local correction floor does not change route/90/180 turn tuning. */
#define VAT_SPEED_MMS          20.0f
#define VAT_X_SPEED_MMS        VAT_SPEED_MMS
#define VAT_Y_SPEED_MMS        30.0f
#define VAT_YAW_MIN_W          TURN_HOLD_MIN_W_RADS
/* One encoder-projected small step, then a stopped/new-image decision.
 * Wheel odometry is not a guarantee of physical millimetres (slip/calibration).
 * The time cap also bounds a step when an encoder reports no movement. */
#define VAT_STEP_MM             3.0f
#define VAT_STEP_MAX_MS          250u
#define VAT_FRESH_MS           300u
#define VAT_GOOD_FRAMES        5u
#define VAT_POLL_MS            20u
#define VAT_PLACEHOLDER_MS     5000u

typedef enum {
    VAT_OFF = 0, VAT_BRAKE, VAT_YAW_FIX, VAT_RECHECK, VAT_ALIGN, VAT_STEP_MOVE,
    VAT_HOLD_BALL, VAT_TURN_REQUEST, VAT_TURN_ACTIVE, VAT_HOLD_BUCKET,
    VAT_DONE, VAT_STOPPED
} VisionAlignTestState;

typedef struct {
    VisionAlignTestState state;
    uint8_t mode, task, digit, good, axis, latest;
    int cls, label, cx, cy;
    uint16_t sequence, img_w, img_h, request;
    /* Receive evidence only: may be shown during braking, but never
     * used as arrival proof. cx/cy/latest above remain the control snapshot. */
    int rx_cx, rx_cy;
    uint16_t rx_sequence, rx_img_w, rx_img_h;
    uint8_t rx_fresh;
    uint32_t age_ms;
    float vx, vy, w;
    float yaw_target, yaw_error; /* Continuous original heading / wrapped error. */
    uint8_t yaw_dirty, yaw_ever; /* Pending correction / this task moved in Y. */
    uint32_t step, step_ms;
    float step_mm; /* Signed encoder progress along the latched step direction. */
    uint8_t step_capped;
    const char *reason; /* Static ASCII string, never an ISR-formatted buffer. */
} VisionAlignTestStatus;

void vision_align_test_init(void);
/* Wrapper opens a NEW QR request for38/40/41 and starts only after g plus its
 * legal QR. This entry verifies qr exactly matches proto_qr_get for those modes.
 * Only39 starts from g directly as bucket task4/digit0; qr may be NULL/ignored.
 * 38=ball,39=bucket(noQR),40=hostage,41=ball/hold5/+180/bucket/hold5.
 * Valid IMU required. XY translation uses w=0, X20/Y30 in bounded 3mm steps;
 * do not reverse/retarget on each incoming image while a step is moving.
 * Every step brakes/settles and needs new imagery. After lateral motion,
 * brake, restore this task's original heading, discard old images and recheck
 * joint X/Y. No initial correction or heading/encoder zeroing, arm or laser. */
int vision_align_test_start(uint8_t mode, const int32_t qr[3]);
void vision_align_test_poll(void); /* Owner task, nonblocking, internally20ms. */
void vision_align_test_feed_frame(const ProtoFrame *frame); /* Parser callback. */
void vision_align_test_cancel(void); /* Immediate brake/localRX close/laser off. */
int vision_align_test_active(void);
void vision_align_test_status(VisionAlignTestStatus *out);
/* Only41: take once, then wrapper starts the EXISTING mode22 +180 controller.
 * Poll is inert while VAT_TURN_ACTIVE so it cannot brake/overwrite mode22.
 * Notify only after controller DONE/STOP; DONE records the new bucket heading
 * once, not on every XY recheck, and requires new bucket ACK/images. */
int vision_align_test_take_turn_request(void);
void vision_align_test_notify_turn_result(int success);

#endif
