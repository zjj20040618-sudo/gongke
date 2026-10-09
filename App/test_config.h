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
#define T_MODE_MAX   43
#define T_GRAB_MODE  42
/* Independent grab bench: counts are STEP pulses, not sensed positions. */
#define T_GRAB_EXTEND_DEFAULT_STEPS 2500u /* User measured nl2500 -> forward90mm. */
#define T_GRAB_DOWN_DEFAULT_STEPS   0u
#define T_GRAB_GRIP_DEFAULT_US    1900u /* Independent42 ball/hostage bench; not legacy mission/cc close. */
#define T_GRAB_SETTLE_MS            250u
#define T_GRAB_HOLD_MS              2000u
#define T_GRAB_SERVO_RESET_MS       2000u
#define T_JOG_MAX_STEPS 100000u /* Finite bench pulse count; not millimetres or a calibrated travel limit. */
#define T_JOG_DEFAULT_PPS 500u  /* Bench STEP frequency only; not chassis v or calibrated cm/s. */
#define T_JOG_MAX_PPS 20000u    /* TIM7-paced bench range; not a verified motor/driver speed limit. */
#define T_JOG_RETURN_WAIT_MS 2000u
#define T_SERVO_RETURN_WAIT_MS 2000u
#define T_DIST_STILL_MS 250u
#define T_ENC_REPORT_MS 1000u
#define T_DIST_TRACE_MS 500u
#define T_YKP_PROFILE_SLOTS 32u /* RAM-only exact (mode15..18, cruise mm/s) keys */
/* Normal distance DONE only: restore the ORIGINAL leg heading before zeroing.
 * Bench candidates, not a promise of physical 0.3-degree accuracy. */
#define T_DIST_ALIGN_TOL_DEG    TURN_HOLD_TOL_DEG
#define T_DIST_ALIGN_STABLE_MS  TURN_HOLD_SETTLE_MS
#define T_DIST_ALIGN_STILL_DEG  TURN_HOLD_STILL_DEG
#define T_DIST_ALIGN_MAX_MS     TURN_HOLD_MAX_MS
#define T_DIST_ALIGN_KP         TURN_HOLD_KP_RADS_DEG
#define T_DIST_ALIGN_MAX_W      0.30f
#define T_DIST_ALIGN_MIN_W      0.08f

/* Mode35: isolated QR-selected target aim/fire trial, camera faces LEFT.
 * User measured x=255; below250 goes backward, above260 goes forward.
 * These settings do not change mode32/formal task calibration. */
#define T_TARGET35_V_MMS       50.0f
#define T_TARGET35_CX          255
#define T_TARGET35_LOW_CX      250
#define T_TARGET35_HIGH_CX     260
#define T_TARGET35_GOOD_FRAMES 5u
#define T_TARGET35_FRESH_MS    300u
#define T_TARGET35_REPORT_MS   500u

/* Modes 19-23: 19 is an elevated sign check; 20 is the +90 turn family.
 * Mode 22's +180 profile remains a bench candidate, not the mission tune.
 * 2026-10-05 user keeps right90 and changes the left trial to92.
 * Modes20/30 command +90/-92; active mode31 locally uses +90/-90 only.
 * Mode22 is explicitly 180, not twice this compensated target. */
#define T_TURN_SIGN_DUTY 45
#define T_TURN_SIGN_MS 250u
#define T_TURN_MAX_MS TURN90_MAX_MS
#define T_TURN_SETTLE_MS TURN90_SETTLE_MS
#define T_TURN90_RIGHT_COMP_DEG 0.0f
#define T_TURN90_LEFT_COMP_DEG 2.0f
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

/* 2026-10-06 loaded-car trial: forward drifts body-left, reverse body-right.
 * fff/bff are independent RAM ratios; POSITIVE always requests body-left vy.
 * Small +/-6.25mm/m trial seeds, NOT newly measured final coefficients.
 * Ordinary straight distance legs apply them at all supported speeds;
 * crossing/mechanical contact legs explicitly bypass both. */
#define T_FORWARD_FF_SEED (-6.25f / 1000.0f)
#define T_BACKWARD_FF_SEED (6.25f / 1000.0f)
/* Preserve mode32's pre-existing tune until its separate integration review. */
#define T_MISSION_FORWARD_FF_SEED (12.5f / 1000.0f)

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
