#ifndef APP_ROUTE_TEST_PLAN_H
#define APP_ROUTE_TEST_PLAN_H

#include <stdint.h>

/* Mode31 has its own 16-stage task trial. R1=535/pv250;
 * select31 starts QR reception but only g starts movement. R1 still/yaw-fix
 * must finish and a legal QR tuple must be cached before R2 can start.
 * Mode31 R2 uses BACK630. Crossing uses BACK650/v300, FWD/v40 until tilt1.5deg,
 * BACK10/FWD15 atv40 (no yaw zero between), stopped IMU zero then BACK190/v200,
 * then RIGHT800/BACK760/LEFT90. Its old 2450/2125 road-only tail is REPLACED
 * by rack predeploy3200@5000 BEFORE entry LEFT90, ball search/X-only,
 * down32000@10000/grip1900/wait2/up32000/hold_grip/+180/no_offset,
 * ball rank54 required before paired180; bucket rank1BACK/rank3FWD/rank2wait,
 * only |cx-X125|<30 latches fine, then aligned-or-loss2s/down20000/release1150/
 * after1s_ready1500/wait2_total/up20000/retract3200@5000,
 * return+180/no_offset/yawfix, selected target search_route_v/until_x350/fine30/X240_band237..243/fire2s,
 * target-to-corner, RIGHT90 and hostage search_route_v/X215-or-loss2s/
 * open1150/extend3200@5000/grip1900/retract3000@5000/rank/forward-to-end_hold_grip.
 * Ordinary forward/back and initial task search use the mode31 RAM v slot
 * (boot200). Independent pv RAM slot controls route left/right (boot250);
 * crossing, board contact and visual fine speeds stay fixed. Mode31/43 turns
 * double the coarse angular command, smoothly returning to the original end profile.
 * Visual Y adjustment is temporarily disabled in vision_align_test.h; cy stays logged.
 * Ball/bucket/hostage workpoints remain TEMPORARY, not calibrated grab points.
 * Hostage: stopped alignment or explicit seen-then-lost2s trigger; extend and
 * grip1900 then retract3000, no vertical movement/release; legitimate rank gates final route.
 * No bucket-anchor detour,
 * manual-d wait or scan. Standalone42 retains its independent reset recipe.
 * Lift uses axis1 nr=DIR1/down, nl=DIR0/up at10000pulses/s. Emitted counts
 * are open-loop, not position sensing or homing; manual stop cancels the chain.
 * All mode31 right-angle turns are +/-90; independent turn profiles are unchanged.
 * The legacy 15-node table is retained for34, including FWD80 and its
 * bucket/manual-d tail; independent36/37 retain their existing FWD80.
 * Crossing/board-contact legs do not correct yaw. IMU RX/health remain on.
 * After contact and both nudges, stop, re-zero and settle before BACK190 resumes yaw hold.
 * The RIGHT800 exit alone has a private xkp3.0 trial seed (0..5 RAM tunable);
 * ordinary ykp/fine/independent modes stay unchanged. Target search/corner use
 * body-right6.5%, hostage search3% and final exit2% feedforward; ball is unchanged.
 *31 board freezes relative pitch/roll zero after300ms stationary (43 remains1s)
 * wait; after a300ms startup guard, tilt1.5deg must persist100ms in new frames.
 * it is a user-observed contact trial, not guaranteed squaring or homing.
 * A10s missing-trigger abort must not continue as successful contact.
 * Target-to-corner 520/420/320 are user trial distances. Hostage first-seen
 * rank1/2/3 selects1415/1315/1215 from its grab stop, never QR shape.
 * Legacy34 still owns its old2450/2125 road-only corridors.
 * Routes reuse the existing distance/yaw-fix executor and route RAM tuning;
 * manual/mode32 tuning and the formal mission are not changed here. */
#define ROUTE_TEST_MODE 31
#define ROUTE_TEST_V_MMS 100.0f /* Keep legacy34/36/37 defaults independent. */
#define ROUTE31_STRAIGHT_V_MMS 200.0f /* Private31/43 ordinary/back/search RAM seed. */
#define ROUTE31_LATERAL_V_MMS 250.0f /* pv1..600 replaces only31/43 route17/18 speed in RAM. */
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
#define ROUTE31_STAGES 16u
#define ROUTE31_PAIR_STAGE            9u
#define ROUTE31_PREDEPLOY_STAGE       8u /* Before LEFT90: clear the camera view. */
#define ROUTE31_RETURN180_STAGE      10u
#define ROUTE31_TARGET_STAGE         11u
#define ROUTE31_TARGET_CORNER_STAGE  12u
#define ROUTE31_HOSTAGE_TURN_STAGE   13u
#define ROUTE31_HOSTAGE_STAGE        14u
#define ROUTE31_HOSTAGE_EXIT_STAGE   15u
#define ROUTE31_RED_TO_CORNER_MM    520u /* Latest request subtracts70 from each selected target-to-corner leg. */
#define ROUTE31_GREEN_TO_CORNER_MM  420u
#define ROUTE31_BLUE_TO_CORNER_MM   320u
#define ROUTE31_HOSTAGE_HOLD_MS       0u /* Extra observation pause removed; rank and mechanical/still gates remain. */
#define ROUTE31_HOSTAGE_RANK1_MM   1415u /* Each final route adds90mm, 2026-10-09. */
#define ROUTE31_HOSTAGE_RANK2_MM   1315u
#define ROUTE31_HOSTAGE_RANK3_MM   1215u
#define ROUTE31_LASER_MS           2000u
#define ROUTE31_TARGET_SEARCH_V_MMS ROUTE31_STRAIGHT_V_MMS
#define ROUTE31_TARGET_FINE_V_MMS    30.0f /*31 only; independent35 remains50. */
#define ROUTE31_TARGET_CX             240 /*31/43 route band237..243; standalone35 remains255/250..260. */
#define ROUTE31_TARGET_LOW_CX         237
#define ROUTE31_TARGET_HIGH_CX        243
#define ROUTE31_FINE_ENTER_CX       350 /* Target ONLY: one-way coarse -> fine latch. */
#define ROUTE31_TARGET_SEARCH_RIGHT_FF_RATIO 0.065f /* Trial:65mm/m right while finding the selected target. */
#define ROUTE31_HOSTAGE_SEARCH_RIGHT_FF_RATIO 0.03f /* Trial:30mm/m right while finding the selected hostage. */
#define ROUTE31_CORNER_RIGHT_FF_RATIO 0.065f /* Trial:65mm/m right from target stop to the corner. */
#define ROUTE31_HOSTAGE_EXIT_RIGHT_FF_RATIO 0.02f /* Trial:20mm/m right on the final ranked exit. */
#define ROUTE31_LIFT_AXIS             1
#define ROUTE31_LIFT_PPS          10000u /*31/43 only; emitted rate is a trial, not verified no-loss speed. */
#define ROUTE31_LIFT_DOWN_STEPS   32000u
#define ROUTE31_LIFT_BALL_UP_STEPS 32000u
#define ROUTE31_LIFT_BUCKET_DOWN_STEPS 20000u
#define ROUTE31_LIFT_BUCKET_UP_STEPS 20000u
#define ROUTE31_BALL_GRIP_WAIT_MS   2000u /* Servo grip dwell, not observation. */
#define ROUTE31_BUCKET_RELEASE_WAIT_MS 2000u /* Total from release; includes ready1500 at1s. */
#define ROUTE31_BUCKET_LIFT_WAIT_MS    0u /* Extra post-lift observation pause removed. */
#define ROUTE31_BALL_GRIP_US        1900u /*31/43 ball; independent42 grip also1900. Legacy close stays separate. */
#define ROUTE31_HOSTAGE_GRIP_US     1900u /* No descent/lift; extend/grip/retract3000, retain grip to final. */
#define ROUTE31_CLAW_READY_US       1500u /*31: staged after bucket release, not at the hostage turn. */
#define ROUTE31_BUCKET_RELEASE_READY_MS 1000u /* From actual release1150; original total wait2 remains. */
#define ROUTE31_PAIR_LEFT_MM          40u /* Body-left AFTER first180; prior50 minus10. */
#define ROUTE31_RETURN_LEFT_MM        20u /* Body-left AFTER second180 completes unchanged angle/stability gates. */
#define ROUTE31_RACK_AXIS              0
#define ROUTE31_RACK_EXTEND_DIR        0 /* Latest user correction restores extension to raw nl, 2026-10-08. */
#define ROUTE31_RACK_RETRACT_DIR       1 /* Opposite of extension; never change lift/raw nl/nr here. */
#define ROUTE31_RACK_PPS            5000u /*31/43 extend and same-frequency return; lift10000. */
#define ROUTE31_RACK_EXTEND_STEPS    3200u /* Ball predeploy and return use the same count. */
#define ROUTE31_HOSTAGE_RACK_EXTEND_STEPS 3200u /*31 hostage;43 reuses its frozen rack count/frequency. */
#define ROUTE31_HOSTAGE_RACK_RETRACT_STEPS 3000u /* Fixed partial return only after1900 grip; never release. */
#define ROUTE31_HOSTAGE_GRIP_SETTLE_MS     250u /* Trial servo wait, not physical grip feedback. */
#define ROUTE31_RACK_SETTLE_MS        250u
#define ROUTE31_POST_CROSS_ALIGN_MM 40u /* Latest user request: prior25 plus15, 2026-10-08. */
#define ROUTE31_BOARD_CONTACT_V_MMS 40.0f
#define ROUTE31_BOARD_CONTACT_TILT_DEG 1.5f /* User trial, either mounting axis/sign; not yaw. */
#define ROUTE31_BOARD_CONTACT_FRAMES 2u /* Only separate validated IMU frames count. */
#define ROUTE31_BOARD_CONTACT_ZERO_WAIT_MS 1000u /* After wheel-still, freeze a new stationary tilt zero. */
#define ROUTE31_BOARD_ZERO_WAIT_MS 300u /*31 only;43 retains the prior1s trial. Wheel-still gate remains. */
#define ROUTE31_STABLE_MS 400u /*31 only: continuous yaw/turn hold; shared standalone/43 remain700. */
#define ROUTE31_BOARD_CONTACT_START_GUARD_MS 300u /* Trial: reject initial dynamic tilt as contact. */
#define ROUTE31_BOARD_CONTACT_CONFIRM_MS 100u /* Require sustained fresh-frame tilt, not just two polls. */
#define ROUTE31_BOARD_CONTACT_MAX_MS 10000u /* Abort if no lift; never claim contact from distance/time. */
#define ROUTE31_BOARD_NUDGE_BACK_MM    10u
#define ROUTE31_BOARD_NUDGE_FORWARD_MM 15u
#define ROUTE31_R2_BACK_MM          630u /* R2 only: prior650 minus20; obstacle road stays650. */
#define ROUTE31_EXIT_RIGHT_MM      800u /* User adds70mm to the prior730, 2026-10-07. */
#define ROUTE31_EXIT_YAW_KP_SEED   3.0f /* Trial only:31/43 stage6 yaw while translating, not other legs/turns. */
#define ROUTE31_ENTRY_BACK_MM      760u /* Prior770 minus10 before entry LEFT90. Direction remains backward. */
/* 31 ordinary endpoints: restore continuous correction to the ORIGINAL heading.
 * Either side may settle with abs(error)<0.4 and400ms still (43 retains700), without crossing
 * zero or pulse/gap cycles. Raise only31's minimum angular command from0.08
 * to0.18rad/s; keep the shared0.30rad/s cap and12s fault budget.
 * This is a trial velocity command, not measured torque/angle accuracy.
 * Manual stop and all other turn/XY/mode profiles remain unchanged. */
#define ROUTE31_POST_YAW_TOL_DEG   0.4f
#define ROUTE31_POST_YAW_MIN_W     0.18f
#define ROUTE31_RIGHT_TARGET_DEG 90.0f
#define ROUTE31_LEFT_TARGET_DEG (-90.0f)
/* Route31/43 +/-90 and +180 only: command2x outside25deg, unchanged within15deg,
 * linear blend between. These are trial speed scheduling thresholds, not
 * measured angle compensation. Do not scale endpoint yaw-fix or standalone turns. */
#define ROUTE31_TURN_SPEED_SCALE 2.0f
#define ROUTE31_TURN_SLOW_DEG 15.0f
#define ROUTE31_TURN_FAST_DEG 25.0f
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
    { 17u,  535u, ROUTE31_LATERAL_V_MMS, "START_LEFT", 1u },
    { 16u, ROUTE31_R2_BACK_MM, ROUTE31_STRAIGHT_V_MMS, "BACK_TO_3RD", 1u },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT90_BEFORE_CROSS", 1u },
    { 16u, ROUTE_CROSS_BACK_MM, ROUTE_CROSS_BACK_V_MMS, "BACK_FULL_OBSTACLE_ROAD", 0u },
    { 15u, 0u, ROUTE31_BOARD_CONTACT_V_MMS, "POST_CROSS_FORWARD_TILT_CONTACT", 0u },
    { 16u, ROUTE_POST_CROSS_CLEAR_MM, ROUTE31_STRAIGHT_V_MMS, "POST_CROSS_BACK_CLEAR", 1u },
    { 18u, ROUTE31_EXIT_RIGHT_MM, ROUTE31_LATERAL_V_MMS, "EXIT_RIGHT", 1u },
    { 16u, ROUTE31_ENTRY_BACK_MM, ROUTE31_STRAIGHT_V_MMS, "BACK_TO_TASK_CORNER", 1u },
    { 30u,    0u, ROUTE_TEST_V_MMS, "LEFT90_TO_TASKS", 1u },
    { 41u,    0u, ROUTE31_STRAIGHT_V_MMS, "BALL_BUCKET_X_GRAB_RELEASE", 1u },
    { 22u,    0u, ROUTE_TEST_V_MMS, "RETURN180_AFTER_BUCKET", 1u },
    { 35u,    0u, ROUTE31_TARGET_SEARCH_V_MMS, "SELECTED_TARGET_X_FIRE2S", 1u },
    { 15u,    0u, ROUTE31_STRAIGHT_V_MMS, "TARGET_TO_CORNER_UNMEASURED", 1u },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT90_TO_HOSTAGE", 1u },
    { 40u,    0u, ROUTE31_STRAIGHT_V_MMS, "HOSTAGE_X_GRAB_RANK", 1u },
    { 15u,    0u, ROUTE31_STRAIGHT_V_MMS, "HOSTAGE_RANK_TO_END_HOLD_GRIP", 1u }
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
