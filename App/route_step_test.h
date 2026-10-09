#ifndef APP_ROUTE_STEP_TEST_H
#define APP_ROUTE_STEP_TEST_H

#include "route_test_plan.h"

/* Mode43 is an independent RAM-tuned, g-gated copy of the current31 trial.
 * Reuse the existing executors, not a second motion or vision controller.
 * A business action keeps its original fresh-image and mechanical timing;
 * every chassis leg, rack predeploy and ball/bucket business action stops
 * before the NEXT g. Running g/a/0 cancels, never resumes a partial move. */
#define ROUTE_STEP_MODE 43
enum { R43_PENDING_NONE = 0, R43_PENDING_PREP, R43_PENDING_BUCKET };
typedef struct {
    uint16_t road_mm[ROUTE31_STAGES];
    uint16_t pair_left_mm;
    uint16_t return_left_mm; /* Internal post-bucket180 left strafe, independently d-tunable while WAIT_G. */
    uint16_t corner_mm[3], rank_mm[3];
    uint32_t rack_steps;
    uint16_t rack_pps;
    float straight_v, lateral_v;
    float exit_yaw_kp; /* xkp0..5: only post-board RIGHT800, not common ykp. */
} RouteStepTune;

#endif
