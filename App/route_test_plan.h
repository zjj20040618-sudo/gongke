#ifndef APP_ROUTE_TEST_PLAN_H
#define APP_ROUTE_TEST_PLAN_H

#include <stdint.h>

/* 2026-10-06: r1=530, r2=650; mode31 scans QR from boot/along R1.
 * R1 must complete before R2; if no legal tuple yet, stop and wait there.
 * All 31/34/36/37 now share the reverse-crossing constants below:
 * RIGHT90, BACK650/v300, FWD80/v20 against the board, BACK190/v100.
 * The board leg is the measured previous50 plus30, not an extra route leg.
 * No d35 lateral offset. Crossing/board-contact legs do not correct yaw.
 * IMU RX/health remain active. After contact, stop, re-zero the leg heading
 * and settle before BACK190 resumes the normal heading gain.
 * Then LEFT92, request bucket4/0 and align cx500. Wait for a NEW d<mm>
 * to go backward/v100, RIGHT90, FWD780, RIGHT90 and the original corridors.
 * Manual backward replaces the old fixed LEFT730, not an extra route leg.
 * These are encoder-distance commands, not contact sensing or homing.
 * 2450 and 2125 are whole trial corridors, not task-to-task station offsets.
 * Modes reuse the shared distance/yaw-fix executor. Ordinary route legs use
 * independent route fff/bff/lff/rff RAM slots, not manual/mode32 tunes.
 * This update retains the separate31/34 bucket/manual-d tail unchanged. */
#define ROUTE_TEST_MODE 31
#define ROUTE_TEST_V_MMS 100.0f
/* Shared measured reverse-crossing recipe: change one place, not four copies. */
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
