#ifndef APP_ROUTE_TEST_PLAN_H
#define APP_ROUTE_TEST_PLAN_H

#include <stdint.h>

/* Mode31 has its own 16-stage task trial. Active31 R1=565/pv100;
 * select31 starts QR reception but only g starts movement. R1 still/yaw-fix
 * must finish and a legal QR tuple must be cached before R2 can start.
 * Mode31 R2 uses BACK610. Crossing uses BACK620/v300, restore pre-cross yaw,
 * then FWD/v40 until tilt2.0deg,
 * stopped IMU zero then BACK190/route_v (no post-contact nudges),
 * then RIGHT780/BACK760/LEFT90. Its old 2450/2125 road-only tail is REPLACED
 * by ball search/X-only, first NEW selected ball coordinate deploys
 * rack3400@5000 alongside search (not before entry LEFT90),
 * LEFT30/yawfix/down32000@10000/grip2100/wait2/up32000/RIGHT30/yawfix/+180,
 * request the bucket only AFTER completed first180; no post-turn LEFT15,
 * ball rank54 required before paired180; bucket rank1BACK/rank3FWD/rank2wait,
 * only |cx-X105|<15 latches continuous fine v20, final95..115 or seen-loss>300ms/down20000/release1150/
 * after1s_ready1500/wait2_total/up20000/retract3400@5000,
 * return+180/stable400/direct_target_no_lateral_offset, selected target search50/until_x360/fine30/X250_band247..253/fire2s,
 * target-to-corner, RIGHT92 and hostage search_route_v/X215-or-seen-loss>300ms/
 * LEFT30/yawfix/recheck_X/open1150/extend3400@5000/grip2100/retract3000@5000/rank/forward-to-end_hold_grip.
 * Pre-task ordinary forward/back use the mode31 RAM v slot (boot100).
 * Independent pv RAM slot controls pre-task left/right (boot100).
 * From the ball/bucket stage onward, ordinary travel/search/offsets use50
 * and lateral feedforward is0, with moving yaw hold retained. Mode31 stays automatic.
 *31 hostage pre-grab body-left30 executes once, followed by yaw-fix and X recheck;
 * crossing, board contact and visual fine speeds stay fixed. Mode31/43 turns
 * both use the original coarse angular command. End profile is retained.
 * Visual Y adjustment is temporarily disabled in vision_align_test.h; cy stays logged.
 * Ball/bucket/hostage workpoints remain TEMPORARY, not calibrated grab points.
 * Ball/bucket/hostage: stopped alignment or explicit seen-then-lost>300ms trigger;
 * loss never means alignment. First legal NEW selected coordinate is required,
 * then brake and confirm still250ms before the action; never-seen cannot trigger.
 * Hostage: extend and
 * grip2100 then retract3000, no vertical movement/release; legitimate rank gates final route.
 * No bucket-anchor detour,
 * manual-d wait or scan. Standalone42 retains its independent reset recipe.
 * Lift uses axis1 nr=DIR1/down, nl=DIR0/up at10000pulses/s. Emitted counts
 * are open-loop, not position sensing or homing; manual stop cancels the chain.
 * Mode31 before-cross right uses+90, hostage right uses+92, left remains-90;
 *43 seeds the same goals in private RAM; independent turns remain unchanged.
 * The legacy 15-node table is retained for34, including FWD80 and its
 * bucket/manual-d tail; independent36/37 retain their existing FWD80.
 * Crossing/board-contact translation does not correct yaw. Modes31/43
 * restores the captured pre-cross heading before a fresh stopped contact zero.
 * After contact, stop, re-zero and settle before BACK190 resumes yaw hold.
 * Mode31 left strafes use a private lkp3.0 trial seed and lff+0.01 toward
 * the vehicle rear. The04:30 log showed R1 yaw+6.35deg and the user measured
 * about20mm forward drift; this is a SMALL candidate, not a full slip model.
 * RIGHT780 retains private xkp3.0; ordinary ykp/fine/other modes stay unchanged.
 * Historical43 seeds (NOT active31's zero-FF task tail): target search/corner use
 * body-right target-search2%/target-to-corner9.5%, hostage search5.5% and final exit7% feedforward;
 * ball forward coarse search uses trial1.5% body-right.
 *31/43 board freezes relative pitch/roll zero after300ms stationary
 * wait; after a300ms startup guard, tilt2.0deg must persist100ms in new frames.
 * it is a user-observed contact trial, not guaranteed squaring or homing.
 * A10s missing-trigger abort must not continue as successful contact.
 * Target-to-corner red/green/blue560/410/335 are user trial distances. Hostage first-seen
 * rank1/2/3 selects1415/1315/1215 from its grab stop, never QR shape.
 * Legacy34 still owns its old2450/2125 road-only corridors.
 * Routes reuse the existing distance/yaw-fix executor and route RAM tuning;
 *31 normal straight after accepted QR uses yaw>2.5 before crossing,
 * yaw>1.5 after crossing;43 retains1.5 throughout. Task coarse-search yaw>1.5
 * brakes and restores the original heading; crossing/contact remain exempt,
 * and resumes the SAME signed remaining distance/task. No mid-leg zero/reset;
 * correction rotation does not consume the remaining straight distance.
 * A mid correction still beyond its triggering limit after2s stops,
 * not endpoint-style false ACCEPT. Crossing31 alone multiplies body-left
 * front/rear wheel TARGET speeds by1.30;44 retains its separate1.10 trial.
 * The table below retains43's545/780/182 seeds; active31 copies/overrides
 * its own legs without changing43 RAM or44's physical-node recipe.
 * manual/mode32 tuning and the formal mission are not changed here. */
#define ROUTE_TEST_MODE 31
#define ROUTE_TEST_V_MMS 100.0f /* Keep legacy34/36/37 defaults independent. */
#define ROUTE31_STRAIGHT_V_MMS 100.0f /*31 slow-route trial; crossing/contact/fine are independent. */
#define ROUTE31_LATERAL_V_MMS 100.0f /*31 slow-route trial; pv remains independently adjustable. */
#define ROUTE31_START_LEFT_MM 565u /* Active31 only;43 table retains545. */
#define ROUTE31_TASK_ENTRY_BACK_MM 760u /* Active31 only;43 table retains780. */
#define ROUTE31_TASK_V_MMS 50.0f /* Ordinary travel/search/offsets from pair stage onward; not fine/laser. */
#define ROUTE31_CROSS_LEFT_SCALE 1.30f /* Only31 reverse crossing; target RPM of m0(LR)/m3(LF). */
#define ROUTE43_STRAIGHT_V_MMS ROUTE31_STRAIGHT_V_MMS /*43 restored as current31's stepped copy. */
#define ROUTE43_LATERAL_V_MMS ROUTE31_LATERAL_V_MMS
/* Legacy reverse-crossing recipe for34/36/37;31/43 use the private table below. */
#define ROUTE_CROSS_BACK_MM       650u
#define ROUTE31_CROSS_BACK_MM     620u /*31 only: user shortens prior630 by10. */
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
#define ROUTE31_LEFT_YAW_KP_SEED 3.0f /*31 left only: trial stronger continuous hold, not ordinary ykp. */
#define ROUTE31_LEFT_FF_SEED 0.010f /*31 left only: positive gives rearward1%; physical result pending. */
#define ROUTE31_STAGES 16u
#define ROUTE31_PAIR_STAGE            9u
#define ROUTE31_PREDEPLOY_STAGE       8u /*43 legacy predeploy before LEFT90;31 waits for first selected ball. */
#define ROUTE31_RETURN180_STAGE      10u
#define ROUTE31_TARGET_STAGE         11u
#define ROUTE31_TARGET_CORNER_STAGE  12u
#define ROUTE31_HOSTAGE_TURN_STAGE   13u
#define ROUTE31_HOSTAGE_STAGE        14u
#define ROUTE31_HOSTAGE_EXIT_STAGE   15u
#define ROUTE31_GREEN_TO_CORNER_MM  410u /*31: green only, prior435 minus15; red/blue independent and unchanged. */
#define ROUTE31_RED_TO_CORNER_MM    560u
#define ROUTE31_BLUE_TO_CORNER_MM   335u

/* User trial: no additional in-place heading repair after target fire.
 * Primary turns, moving heading hold and normal road endpoints stay unchanged.
 * Set1 only to retain the historical correction candidate's host coverage. */
#ifndef ROUTE31_TARGET_STOP_YAW_ENABLE
#define ROUTE31_TARGET_STOP_YAW_ENABLE 0
#endif
#define ROUTE31_HOSTAGE_HOLD_MS       0u /* Extra observation pause removed; rank and mechanical/still gates remain. */
#define ROUTE31_HOSTAGE_RANK1_MM   1415u /*31: user shortens all three final routes by80mm, 2026-10-11. */
#define ROUTE31_HOSTAGE_RANK2_MM   1315u
#define ROUTE31_HOSTAGE_RANK3_MM   1215u
#define ROUTE43_HOSTAGE_RANK1_MM   1415u /* Deferred43 preserves its previous final distances. */
#define ROUTE43_HOSTAGE_RANK2_MM   1315u
#define ROUTE43_HOSTAGE_RANK3_MM   1215u
#define ROUTE31_LASER_MS           2000u
#define ROUTE31_TARGET_SEARCH_V_MMS ROUTE31_STRAIGHT_V_MMS
#define ROUTE31_TARGET_FINE_V_MMS    30.0f /*31 only; independent35 remains50. */
#define ROUTE31_TARGET_CX             250 /*31 user shifts prior245 and all absolute X thresholds right5px. */
#define ROUTE31_TARGET_LOW_CX         247
#define ROUTE31_TARGET_HIGH_CX        253
#define ROUTE31_FINE_ENTER_CX       360 /* Target ONLY: one-way coarse -> fine latch. */
#define ROUTE43_TARGET_CX             ROUTE31_TARGET_CX
#define ROUTE43_TARGET_LOW_CX         ROUTE31_TARGET_LOW_CX
#define ROUTE43_TARGET_HIGH_CX        ROUTE31_TARGET_HIGH_CX
#define ROUTE43_FINE_ENTER_CX         ROUTE31_FINE_ENTER_CX
#define ROUTE31_BALL_SEARCH_RIGHT_FF_RATIO 0.015f /*31 only: halve30 to15mm/m body-right, forward ball coarse search only. */
#define ROUTE31_TARGET_SEARCH_RIGHT_FF_RATIO 0.020f /*31: previous65 minus20mm/m body-right while searching. */
#define ROUTE43_TARGET_SEARCH_RIGHT_FF_RATIO ROUTE31_TARGET_SEARCH_RIGHT_FF_RATIO
#define ROUTE31_HOSTAGE_SEARCH_RIGHT_FF_RATIO 0.055f /*31/43 seed:55mm/m, forward hostage coarse search. */
#define ROUTE43_HOSTAGE_SEARCH_RIGHT_FF_RATIO ROUTE31_HOSTAGE_SEARCH_RIGHT_FF_RATIO
#define ROUTE31_CORNER_RIGHT_FF_RATIO 0.095f /* Trial:65+30=95mm/m right from target stop to the corner; vy, never a yaw offset. */
#define ROUTE43_CORNER_RIGHT_FF_RATIO ROUTE31_CORNER_RIGHT_FF_RATIO
#define ROUTE31_HOSTAGE_EXIT_RIGHT_FF_RATIO 0.07f /*31/43 seed:70mm/m body-right on the final ranked exit. */
#define ROUTE43_HOSTAGE_EXIT_RIGHT_FF_RATIO ROUTE31_HOSTAGE_EXIT_RIGHT_FF_RATIO
#define ROUTE31_LIFT_AXIS             1
#define ROUTE31_LIFT_PPS          10000u /*31/43 only; emitted rate is a trial, not verified no-loss speed. */
#define ROUTE31_LIFT_DOWN_STEPS   32000u
#define ROUTE31_LIFT_BALL_UP_STEPS 32000u
#define ROUTE31_LIFT_BUCKET_DOWN_STEPS 20000u
#define ROUTE31_LIFT_BUCKET_UP_STEPS 20000u
#define ROUTE31_BALL_GRIP_WAIT_MS   2000u /* Servo grip dwell, not observation. */
#define ROUTE31_BUCKET_RELEASE_WAIT_MS 2000u /* Total from release; includes ready1500 at1s. */
#define ROUTE31_BUCKET_LIFT_WAIT_MS    0u /* Extra post-lift observation pause removed. */
#define ROUTE31_BALL_GRIP_US        2100u /*31 only: user increases1900 by200; open/ready unchanged. */
#define ROUTE31_HOSTAGE_GRIP_US     2100u /*31 only: user increases1900 by200; retain grip through final. */
#define ROUTE43_BALL_GRIP_US        ROUTE31_BALL_GRIP_US
#define ROUTE43_HOSTAGE_GRIP_US     ROUTE31_HOSTAGE_GRIP_US
#define ROUTE31_CLAW_READY_US       1500u /*31: staged after bucket release, not at the hostage turn. */
#define ROUTE31_BUCKET_RELEASE_READY_MS 1000u /* From actual release1150; original total wait2 remains. */
#define ROUTE31_BALL_TO_BUCKET_LEFT_MM 15u /*31: after first180, body-left15 then original-heading yaw fix. */
#define ROUTE31_BALL_TO_BUCKET_LEFT_V_MMS 80.0f /*31-only trial ceiling, not all lateral legs; obey lower RAM pv. */
#define ROUTE31_PAIR_LEFT_MM          40u /* Historical macro; current31/43 seed BALL_TO_BUCKET_LEFT_MM instead. */
#define ROUTE31_RETURN_LEFT_MM        20u /* Historical macro; current31/43 seed RETURN_RIGHT_MM instead. */
#define ROUTE31_RETURN_RIGHT_MM        0u /*31: no lateral offset after return180; zero skips the job, never starts a distance test. */
#define ROUTE31_RACK_AXIS              0
#define ROUTE31_RACK_EXTEND_DIR        0 /* Latest user correction restores extension to raw nl, 2026-10-08. */
#define ROUTE31_RACK_RETRACT_DIR       1 /* Opposite of extension; never change lift/raw nl/nr here. */
#define ROUTE31_RACK_PPS            5000u /*31/43 extend and same-frequency return; lift10000. */
#define ROUTE31_RACK_EXTEND_STEPS    3400u /*31 first-ball deploy and full return use the same count. */
#define ROUTE31_HOSTAGE_RACK_EXTEND_STEPS 3400u /*31 hostage extends3400, partial return remains3000. */
#define ROUTE43_RACK_EXTEND_STEPS    ROUTE31_RACK_EXTEND_STEPS
#define ROUTE31_HOSTAGE_PREGRAB_LEFT_MM    30u /*31 only: once after X confirmation, before arm; yaw-fix then recheck X. */
#define ROUTE31_HOSTAGE_RACK_RETRACT_STEPS 3000u /* Partial return only after owner-selected grip; never release. */
#define ROUTE31_HOSTAGE_GRIP_SETTLE_MS     250u /* Trial servo wait, not physical grip feedback. */
#define ROUTE31_RACK_SETTLE_MS        250u
#define ROUTE31_POST_CROSS_ALIGN_MM 40u /* Latest user request: prior25 plus15, 2026-10-08. */
#define ROUTE31_BOARD_CONTACT_V_MMS 40.0f
#define ROUTE31_BOARD_CONTACT_TILT_DEG 2.0f /*31: prior1.5 plus0.5; either mounting axis/sign, not yaw. */
#define ROUTE43_BOARD_CONTACT_TILT_DEG ROUTE31_BOARD_CONTACT_TILT_DEG
#define ROUTE31_BOARD_CONTACT_FRAMES 2u /* Only separate validated IMU frames count. */
#define ROUTE31_BOARD_CONTACT_ZERO_WAIT_MS 1000u /* After wheel-still, freeze a new stationary tilt zero. */
#define ROUTE31_BOARD_ZERO_WAIT_MS 300u /*31/43; wheel-still gate remains. */
#define ROUTE31_STABLE_MS 400u /*31/43 continuous yaw/turn hold; standalone retains its profile. */
#define ROUTE31_BOARD_CONTACT_START_GUARD_MS 300u /* Trial: reject initial dynamic tilt as contact. */
#define ROUTE31_BOARD_CONTACT_CONFIRM_MS 100u /* Require sustained fresh-frame tilt, not just two polls. */
#define ROUTE31_BOARD_CONTACT_MAX_MS 10000u /* Abort if no lift; never claim contact from distance/time. */
#define ROUTE31_BOARD_NUDGE_BACK_MM    10u /* Historical macro; current31/43 skip post-contact nudges. */
#define ROUTE31_BOARD_NUDGE_FORWARD_MM 15u /* Historical macro; current31/43 skip this leg. */
#define ROUTE31_R2_BACK_MM          610u /* R2 unchanged this round; independent from crossing630. */
#define ROUTE31_EXIT_RIGHT_MM      780u /* 2026-10-10: prior800 minus20. */
#define ROUTE31_EXIT_YAW_KP_SEED   3.0f /* Trial only:31/43 stage6 yaw while translating, not other legs/turns. */
#define ROUTE31_ENTRY_BACK_MM      780u /* User: move task-entry corner away from ball-side boundary. */
/* 31 ordinary endpoints: restore continuous correction to the ORIGINAL heading.
 * Either side may settle with abs(error)<0.4 and400ms still, without crossing
 * zero or pulse/gap cycles. Modes31/43 use minimum angular command0.30rad/s
 * and a0.60rad/s cap after insufficient progress.
 * Mode31 accepts after2s even
 * if still outside tolerance, reporting the remaining error before re-zero.
 * Other modes and primary90/180 turns keep their12s failure budgets.
 * This is a trial velocity command, not measured torque/angle accuracy.
 * Manual stop and all other turn/XY/mode profiles remain unchanged. */
#define ROUTE31_POST_YAW_TOL_DEG   0.4f
#define ROUTE31_POST_YAW_MIN_W     0.30f /* Loaded31 trial: previous0.18; commanded speed, not sensed torque. */
#define ROUTE31_POST_YAW_MAX_W     0.60f /*31 endpoint only; no progress400ms uses this floor/cap. */
#define ROUTE43_POST_YAW_MIN_W     ROUTE31_POST_YAW_MIN_W
#define ROUTE31_POST_YAW_MAX_MS    2000u /* User trial: accept endpoint correction at2s; not physical alignment proof. */
#define ROUTE31_MID_YAW_LIMIT_DEG 1.5f /*31 QR-accepted straight/coarse-search brake/correct/resume threshold; strictly >. */
#define ROUTE31_PRE_CROSS_MID_YAW_LIMIT_DEG 2.5f /* Active31 stage<4 only;43/44 unchanged. */
#define ROUTE31_RIGHT_TARGET_DEG 90.0f /*31 before-cross right; hostage has its own goal below. */
#define ROUTE31_HOSTAGE_RIGHT_TARGET_DEG 92.0f /*31 hostage entry; seeds private43 RAM, not standalone turns. */
#define ROUTE43_RIGHT_TARGET_DEG 90.0f /* Historical macro; current43 seeds the per-stage31 goals. */
#define ROUTE31_LEFT_TARGET_DEG (-90.0f)
#define ROUTE31_RETURN_TARGET_DEG 182.0f /* Historical43 RAM seed; active31/44 both returns are exactly180. */
/* Route31 +/-90 and +180: remove the earlier2x coarse boost for a slow trial.
 *43 uses the same seed; its private tv/tkp remain adjustable while stopped.
 * These are trial speed scheduling thresholds, not
 * measured angle compensation. Do not scale endpoint yaw-fix or standalone turns. */
#define ROUTE31_TURN_SPEED_SCALE 1.0f
#define ROUTE43_TURN_SPEED_SCALE ROUTE31_TURN_SPEED_SCALE
#define ROUTE31_TURN_MIN_W       0.30f /*31/43 turn end correction; standalone retains0.18. */
#define ROUTE31_TURN_TOL_DEG     0.4f /*31/43 inclusive arrival/settle tolerance; standalone retains0.3. */
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

/* Shared historical seeds for43. Active31 copies stage0/7 before overriding
 *565/760; tail speeds/FF and paired offsets are scoped in its executor. */
static const RouteTestLeg s_route31_plan[ROUTE31_STAGES] = {
    { 17u,  545u, ROUTE31_LATERAL_V_MMS, "START_LEFT", 1u },
    { 16u, ROUTE31_R2_BACK_MM, ROUTE31_STRAIGHT_V_MMS, "BACK_TO_3RD", 1u },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT90_BEFORE_CROSS", 1u },
    { 16u, ROUTE31_CROSS_BACK_MM, ROUTE_CROSS_BACK_V_MMS, "BACK_FULL_OBSTACLE_ROAD", 0u },
    { 15u, 0u, ROUTE31_BOARD_CONTACT_V_MMS, "POST_CROSS_FORWARD_TILT_CONTACT", 0u },
    { 16u, ROUTE_POST_CROSS_CLEAR_MM, ROUTE31_STRAIGHT_V_MMS, "POST_CROSS_BACK_CLEAR", 1u },
    { 18u, ROUTE31_EXIT_RIGHT_MM, ROUTE31_LATERAL_V_MMS, "EXIT_RIGHT", 1u },
    { 16u, ROUTE31_ENTRY_BACK_MM, ROUTE31_STRAIGHT_V_MMS, "BACK_TO_TASK_CORNER", 1u },
    { 30u,    0u, ROUTE_TEST_V_MMS, "LEFT90_TO_TASKS", 1u },
    { 41u,    0u, ROUTE31_STRAIGHT_V_MMS, "BALL_BUCKET_X_GRAB_RELEASE", 1u },
    { 22u,    0u, ROUTE_TEST_V_MMS, "RETURN180_AFTER_BUCKET", 1u },
    { 35u,    0u, ROUTE31_TARGET_SEARCH_V_MMS, "SELECTED_TARGET_X_FIRE2S", 1u },
    { 15u,    0u, ROUTE31_STRAIGHT_V_MMS, "TARGET_TO_CORNER_UNMEASURED", 1u },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT_TO_HOSTAGE", 1u },
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
