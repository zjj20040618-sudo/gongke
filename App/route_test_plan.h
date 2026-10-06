#ifndef APP_ROUTE_TEST_PLAN_H
#define APP_ROUTE_TEST_PLAN_H

#include <stdint.h>

/* 2026-10-06: mode31 has its own 12-node configured trial below. R1=520/v100;
 * select31 starts QR reception but only g starts movement. R1 still/yaw-fix
 * must finish and a legal QR tuple must be cached before R2 can start.
 * Mode31 uses BACK650/v300, FWD70/v20 against the board, BACK190/v100,
 * then RIGHT730/BACK780/LEFT90 and the original 2450/2125 corridors.
 * Its direct path has no bucket alignment, manual-d wait or task actions.
 * All mode31 right-angle turns are +/-90; other turn profiles are unchanged.
 * The legacy 15-node table is retained for34, including FWD80 and its
 * bucket/manual-d tail; independent36/37 retain their existing FWD80.
 * Crossing/board-contact legs do not correct yaw. IMU RX/health remain on.
 * After contact, stop, re-zero and settle before BACK190 resumes yaw hold.
 * These are encoder-distance commands, not contact sensing or homing.
 * 2450 and 2125 are whole trial corridors, not task-to-task station offsets.
 * Routes reuse the existing distance/yaw-fix executor and route RAM tuning;
 * manual/mode32 tuning and the formal mission are not changed here. */
#define ROUTE_TEST_MODE 31
#define ROUTE_TEST_V_MMS 100.0f
/* Existing reverse-crossing recipe for34/36/37;31 overrides contact only. */
#define ROUTE_CROSS_BACK_MM       650u
#define ROUTE_CROSS_BACK_V_MMS    300.0f
#define ROUTE_POST_CROSS_ALIGN_MM  80u
#define ROUTE_POST_CROSS_ALIGN_V_MMS 20.0f
#define ROUTE_POST_CROSS_CLEAR_MM 190u
#define ROUTE_POST_CROSS_CLEAR_V_MMS 100.0f
#define ROUTE_TEST_CROSS_V_MMS ROUTE_CROSS_BACK_V_MMS
#define ROUTE_TEST_ACC_MMS2 700.0f
#define ROUTE_TEST_DEC_MMS2 350.0f
#define ROUTE_TEST_FORWARD_FF_SEED (-6.25f / 1000.0f) /* negative: forward -> right */
#define ROUTE_TEST_LEFT_FF_SEED 0.0f /* route v100 not measured; lff sets RAM trial */
#define ROUTE_TEST_RIGHT_FF_SEED 0.0f /* do not silently copy manual v300 calibration */
#define ROUTE_TEST_HEADING_KP_SEED 0.3f /* user prefers less physical drift, 2026-10-05 */
#define ROUTE31_STAGES 12u
#define ROUTE31_POST_CROSS_ALIGN_MM 70u
#define ROUTE31_RIGHT_TARGET_DEG 90.0f
#define ROUTE31_LEFT_TARGET_DEG (-90.0f)
/* Legacy bucket/manual-d route for34 only. */
#define ROUTE_TEST_STAGES 15u
#define ROUTE_TEST_ALIGN_STAGE 7u
#define ROUTE_TEST_BACK_STAGE 8u

typedef struct {
    uint8_t mode;
    uint16_t distance_mm; /* positive slot value; direction comes from mode */
    float speed_mms;
    const char *name;
    uint8_t heading_hold; /* 0: no yaw correction; 1: normal route RAM gain */
} RouteTestLeg;

/* Private mode31 recipe: no changes to34/36/37 or mission routes. */
static const RouteTestLeg s_route31_plan[ROUTE31_STAGES] = {
    { 17u,  520u, ROUTE_TEST_V_MMS, "START_LEFT", 1u },
    { 16u,  650u, ROUTE_TEST_V_MMS, "BACK_TO_3RD", 1u },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT90_BEFORE_CROSS", 1u },
    { 16u, ROUTE_CROSS_BACK_MM, ROUTE_CROSS_BACK_V_MMS, "BACK_FULL_OBSTACLE_ROAD", 0u },
    { 15u, ROUTE31_POST_CROSS_ALIGN_MM, ROUTE_POST_CROSS_ALIGN_V_MMS, "POST_CROSS_FORWARD_ALIGN", 0u },
    { 16u, ROUTE_POST_CROSS_CLEAR_MM, ROUTE_POST_CROSS_CLEAR_V_MMS, "POST_CROSS_BACK_CLEAR", 1u },
    { 18u,  730u, ROUTE_TEST_V_MMS, "EXIT_RIGHT", 1u },
    { 16u,  780u, ROUTE_TEST_V_MMS, "BACK_TO_TASK_CORNER", 1u },
    { 30u,    0u, ROUTE_TEST_V_MMS, "LEFT90_TO_TASKS", 1u },
    { 15u, 2450u, ROUTE_TEST_V_MMS, "TASK_CORRIDOR_NO_TASKS", 1u },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT90_TO_RESCUE", 1u },
    { 15u, 2125u, ROUTE_TEST_V_MMS, "RESCUE_TO_FINAL_NO_TASK", 1u }
};

static const RouteTestLeg s_route_test_plan[ROUTE_TEST_STAGES] = {
    { 17u,  530u, ROUTE_TEST_V_MMS, "START_LEFT", 1u },
    { 16u,  650u, ROUTE_TEST_V_MMS, "BACK_TO_3RD", 1u },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT90_BEFORE_CROSS", 1u },
    { 16u, ROUTE_CROSS_BACK_MM, ROUTE_CROSS_BACK_V_MMS, "BACK_FULL_OBSTACLE_ROAD", 0u },
    { 15u, ROUTE_POST_CROSS_ALIGN_MM, ROUTE_POST_CROSS_ALIGN_V_MMS, "POST_CROSS_FORWARD_ALIGN", 0u },
    { 16u, ROUTE_POST_CROSS_CLEAR_MM, ROUTE_POST_CROSS_CLEAR_V_MMS, "POST_CROSS_BACK_CLEAR", 1u },
    { 30u,    0u, ROUTE_TEST_V_MMS, "LEFT92_TO_BUCKET_VIEW", 1u },
    {  0u,    0u, 50.0f, "BUCKET_X500_ALIGN", 1u },
    { 16u,    0u, ROUTE_TEST_V_MMS, "MANUAL_BACK_D", 1u },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT90_AFTER_MANUAL_BACK", 1u },
    { 15u,  780u, ROUTE_TEST_V_MMS, "FWD_TO_TASK_CORNER", 1u },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT90_TO_TASKS", 1u },
    { 15u, 2450u, ROUTE_TEST_V_MMS, "TASK_CORRIDOR_NO_TASKS", 1u },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT90_TO_RESCUE", 1u },
    { 15u, 2125u, ROUTE_TEST_V_MMS, "RESCUE_TO_FINAL_NO_TASK", 1u }
};

#endif
