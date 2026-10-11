#ifndef APP_ROUTE_STEP_TEST_H
#define APP_ROUTE_STEP_TEST_H

#include "route_test_plan.h"

/* Mode43 is an independent RAM-tuned, g-gated copy of the current31 trial.
 * Reuse the existing executors, not a second motion or vision controller.
 * A business action keeps its original fresh-image and mechanical timing;
 * every chassis leg, rack predeploy and ball/bucket business action stops
 * before the NEXT g. Running g/a/0 cancels, never resumes a partial move. */
#define ROUTE_STEP_MODE 43
enum { R43_PENDING_NONE = 0, R43_PENDING_PREP, R43_PENDING_BUCKET, R43_PENDING_HOSTAGE };
typedef struct {
    uint16_t road_mm[ROUTE31_STAGES];
    float leg_v[ROUTE31_STAGES]; /* 0 uses the global v/pv or fixed crossing/contact seed. */
    float turn_deg[ROUTE31_STAGES], pair_turn_deg;
    float turn_max_w, turn_kp;
    uint16_t pair_left_mm;
    uint16_t return_left_mm; /* Internal post-bucket180 left strafe, independently d-tunable while WAIT_G. */
    uint16_t hostage_left_mm;
    float pair_v, return_v, hostage_v; /* 0 follows pv; pair/return retain the80mm/s ceiling. */
    uint16_t corner_mm[3], rank_mm[3];
    uint32_t rack_steps;
    uint16_t rack_pps;
    uint32_t hostage_rack_steps, ball_down_steps, bucket_down_steps;
    uint16_t hostage_rack_pps, lift_pps, ball_grip_us, hostage_grip_us;
    float straight_v, lateral_v;
    float exit_yaw_kp; /* xkp0..5: only post-board RIGHT800, not common ykp. */
    float heading_kp, left_yaw_kp;
    float forward_ff, backward_ff, left_ff, right_ff;
    float ball_ff, target_ff, hostage_ff, corner_ff, exit_ff;
    uint8_t chinese_ui; /* UTF-8 bytes; cn0 keeps the ASCII diagnostic-only display. */
} RouteStepTune;

#endif
