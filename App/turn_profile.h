#ifndef APP_TURN_PROFILE_H
#define APP_TURN_PROFILE_H

/* Right +90 deg: mode 20 on-ground validation, 2026-09-25, four DONE runs.
 * Keep test and mission on one profile; other turn angles are not validated. */
#define TURN90_MAX_MS          7000u
#define TURN90_SETTLE_MS       700u
#define TURN90_TARGET_DEG      90.0f
#define TURN90_TOL_DEG         0.3f
#define TURN90_LIMIT_DEG       15.0f
#define TURN90_KP_RADS_DEG     0.15f
#define TURN90_MAX_W_RADS      2.0f
#define TURN90_MIN_W_RADS      0.18f
#define TURN90_STILL_DEG       0.2f

#endif
