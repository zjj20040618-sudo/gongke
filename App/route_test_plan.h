#ifndef APP_ROUTE_TEST_PLAN_H
#define APP_ROUTE_TEST_PLAN_H

#include <stdint.h>

/* 2026-10-05: r1=530, r2=650; scan QR from boot/along R1.
 * R1 must complete before R2; if no legal tuple yet, stop and wait there.
 * 750 includes the entire obstacle road. Do not add a second crossing leg.
 * Only that road uses v300 (user reports no slip); existing other legs stay v100.
 * After crossing: reverse d170/v20, forward d210/v100, then left d730.
 * These are encoder-distance commands, not contact sensing or homing.
 * 2450 and 2125 are whole trial corridors, not task-to-task station offsets.
 * Modes reuse the stable distance/turn machines. Forward v100 uses its own
 * half-strength RAM correction; manual15/mode32 and reverse/strafe unchanged. */
#define ROUTE_TEST_MODE 31
#define ROUTE_TEST_V_MMS 100.0f
#define ROUTE_TEST_CROSS_V_MMS 300.0f
#define ROUTE_TEST_ACC_MMS2 700.0f
#define ROUTE_TEST_DEC_MMS2 350.0f
#define ROUTE_TEST_FORWARD_FF_SEED (6.25f / 1000.0f)
#define ROUTE_TEST_HEADING_KP_SEED 0.3f /* user prefers less physical drift, 2026-10-05 */
#define ROUTE_TEST_STAGES 12u

typedef struct {
    uint8_t mode;
    uint16_t distance_mm; /* positive slot value; direction comes from mode */
    float speed_mms;
    const char *name;
} RouteTestLeg;

static const RouteTestLeg s_route_test_plan[ROUTE_TEST_STAGES] = {
    { 17u,  530u, ROUTE_TEST_V_MMS, "START_LEFT" },
    { 16u,  650u, ROUTE_TEST_V_MMS, "BACK_TO_3RD" },
    { 30u,    0u, ROUTE_TEST_V_MMS, "LEFT92" },
    { 15u,  750u, ROUTE_TEST_CROSS_V_MMS, "FULL_OBSTACLE_ROAD" },
    { 16u,  170u, 20.0f, "POST_CROSS_BACK" },
    { 15u,  210u, ROUTE_TEST_V_MMS, "POST_CROSS_FORWARD" },
    { 17u,  730u, ROUTE_TEST_V_MMS, "EXIT_LEFT" },
    { 15u,  780u, ROUTE_TEST_V_MMS, "FWD_TO_TASK_CORNER" },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT90_TO_TASKS" },
    { 15u, 2450u, ROUTE_TEST_V_MMS, "TASK_CORRIDOR_NO_TASKS" },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT90_TO_RESCUE" },
    { 15u, 2125u, ROUTE_TEST_V_MMS, "RESCUE_TO_FINAL_NO_TASK" }
};

#endif
