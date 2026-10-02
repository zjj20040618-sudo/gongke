#ifndef APP_TEST_CONFIG_H
#define APP_TEST_CONFIG_H

/* Bluetooth bench-only limits and defaults. These are not mission route values.
 * Keep validated turn-90 settings shared through turn_profile.h. */
#include "turn_profile.h"

/* Command parser and report cadence. */
#define T_LINE_MAX   20u
#define T_IDLE_MS    20u
#define T_V_MAX      600
#define T_D_MAX      20000
#define T_MODE_MAX   32
#define T_JOG_MAX_STEPS 50u
#define T_JOG_RETURN_WAIT_MS 2000u
#define T_SERVO_RETURN_WAIT_MS 2000u
#define T_DIST_STILL_MS 250u
#define T_WALK_ORIGIN_TOL_MM 0.5f /* Encoder-axis stop tolerance, not physical position accuracy. */
#define T_ENC_REPORT_MS 1000u
#define T_DIST_TRACE_MS 500u
#define T_PREP_MAX_MS 5000u /* physical stillness guard; failure never starts motion */
#define T_FORMAL_TURN_MAX_MS 20000u /* bench-only run-away guard for the slower formal profile */
#define T_CROSS_MAX_MS 8000u /* same physical run-away guard as the formal crossing */
#define T_DIST_NO_PROGRESS_MS 1500u /* bench fault stop, never skip/pretend arrival */
#define T_DIST_WRONGWAY_MM 10.0f

/* Modes 19-23: 19 is an elevated sign check; 20 is the ground-tested +90.
 * Mode 22's +180 profile remains a bench candidate, not the mission tune. */
#define T_TURN_SIGN_DUTY 45
#define T_TURN_SIGN_MS 250u
#define T_TURN_MAX_MS TURN90_MAX_MS
#define T_TURN_SETTLE_MS TURN90_SETTLE_MS
#define T_TURN_TARGET_DEG TURN90_TARGET_DEG
#define T_TURN_TOL_DEG TURN90_TOL_DEG
#define T_TURN_LIMIT_DEG TURN90_LIMIT_DEG
#define T_TURN_KP TURN90_KP_RADS_DEG
#define T_TURN_MAX_W TURN90_MAX_W_RADS
#define T_TURN_MIN_W TURN90_MIN_W_RADS
#define T_TURN180_MAX_MS 12000u
#define T_TURN_TRACE_MS 500u
#define T_SPEED_PROBE_W 0.5f
#define T_SPEED_PROBE_PHASE_MS 800u
#define T_SPEED_PROBE_TRACE_MS 200u
#define T_SPEED_PROBE_SETTLE_MS 300u
#define T_FWD_PULSE_DUTY 45
#define T_FWD_OPEN_MS 400u
#define T_FWD_PAUSE_MS 800u
#define T_FWD_CLOSED_MS 650u
#define T_FWD_TRACE_MS 200u
#define T_FWD_V_MMS 80.0f

/* Modes 17/18 only: measured trial defaults; the formal mission does not
 * inherit these. The feed-forward seeds remain RAM-tunable via Bluetooth. */
#define T_LATERAL_DEFAULT_V_MMS 300.0f
#define T_LATERAL_DEFAULT_D_MM 1500.0f
#define T_LEFT_FF_SEED (35.0f / 1500.0f)
#define T_RIGHT_FF_SEED T_LEFT_FF_SEED

/* Power-on safety: never spin wheels automatically. A bench run needs an
 * explicit mode and g. Keep BENCH_AUTO at 0 for normal builds. */
#define BENCH_AUTO     0u
#define BENCH_DUTY     100
#define BENCH_RUN_MS   1500u
#define BENCH_GAP_MS   1000u
#define BENCH_DELAY_MS 5000u
#define BENCH_FWD_MS   3000u
#define BENCH_FWD_GAP  3000u

#endif /* APP_TEST_CONFIG_H */
