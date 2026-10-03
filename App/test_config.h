#ifndef APP_TEST_CONFIG_H
#define APP_TEST_CONFIG_H

/* Bluetooth bench-only limits and defaults. These are not mission route values.
 * Keep the successful bench turn-hold settings shared through turn_profile.h. */
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
#define T_ENC_REPORT_MS 1000u
#define T_DIST_TRACE_MS 500u

/* Modes 19-23: 19 is an elevated sign check; 20 is the +90 turn family.
 * Mode 22's +180 profile remains a bench candidate, not the mission tune.
 * 2026-10-03 user sets independent corner trials: right85, left95.
 * Modes20/30 command +85/-95; nominal mission geometry remains +/-90.
 * Mode22 is explicitly 180, not twice this compensated target. */
#define T_TURN_SIGN_DUTY 45
#define T_TURN_SIGN_MS 250u
#define T_TURN_MAX_MS TURN90_MAX_MS
#define T_TURN_SETTLE_MS TURN90_SETTLE_MS
#define T_TURN90_RIGHT_COMP_DEG (-5.0f)
#define T_TURN90_LEFT_COMP_DEG 5.0f
#define T_TURN_RIGHT_TARGET_DEG (TURN90_TARGET_DEG + T_TURN90_RIGHT_COMP_DEG)
#define T_TURN_LEFT_TARGET_DEG (-(TURN90_TARGET_DEG + T_TURN90_LEFT_COMP_DEG))
#define T_TURN_TOL_DEG TURN90_TOL_DEG
#define T_TURN_LIMIT_DEG TURN90_LIMIT_DEG
#define T_TURN_KP TURN90_KP_RADS_DEG
#define T_TURN_MAX_W TURN90_MAX_W_RADS
#define T_TURN_MIN_W TURN90_MIN_W_RADS
#define T_TURN180_MAX_MS TURN_HOLD_MAX_MS
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

/* Forward only: user reports 10..15mm right drift per metre, 2026-10-03.
 * Positive fff requests body-left vy. Start at the midpoint, RAM-tunable.
 * Apply at current route v100 and legacy trial v200 only; no reverse/strafe.
 * This is a nominal velocity ratio; integer wheel RPM quantizes it. */
#define T_FORWARD_FF_SEED (12.5f / 1000.0f)

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
