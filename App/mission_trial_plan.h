#ifndef APP_MISSION_TRIAL_PLAN_H
#define APP_MISSION_TRIAL_PLAN_H

#include <stdint.h>

/* Mode 32 is a separate no-arm trial. These are wheel-odometry command
 * distances confirmed by the user, not physical station coordinates. */
#define MISSION_TRIAL_MODE                 32u
#define MISSION_TRIAL_ROUTE_LEGS           10u
#define MISSION_TRIAL_ROUTE_SPEED_MMS       100.0f
#define MISSION_TRIAL_TASK_CORRIDOR_MM      2450.0f
#define MISSION_TRIAL_RESCUE_CORRIDOR_MM    2125.0f

/* Each no-arm action is a stopped, interruptible wait in the caller.
 * This module does not wait, run a motor, operate a servo, or detect objects. */
#define MISSION_TRIAL_BALL_HOLD_MS          10000u
#define MISSION_TRIAL_BUCKET_HOLD_MS        10000u
#define MISSION_TRIAL_HOSTAGE_HOLD_MS       10000u

typedef struct {
    uint8_t mode;          /* Existing bench mode: 15/16/17/20/30. */
    uint16_t distance_mm;  /* Positive command; direction comes from mode. */
    int16_t turn_deg;      /* Clockwise positive; zero for translations. */
} MissionTrialRouteLeg;

extern const MissionTrialRouteLeg
    mission_trial_route_plan[MISSION_TRIAL_ROUTE_LEGS];

/* One local road axis, not a global pose. fore/lat inputs are continuous
 * signed wheel odometry: body-forward positive, body-right positive.
 * heading is continuous IMU heading, clockwise positive; it must NOT be the
 * leg-relative heading that is software-zeroed at each node.
 *
 * The caller updates this ledger throughout scanning, alignment, braking,
 * and turns, before any odometry reset. At 180 degrees body-forward progress
 * is subtracted from road progress. Pure lateral motion is also projected,
 * so alignment is not silently omitted. This is still wheel odometry: it
 * does not prove the actual car position or that its projection stays on-road. */
typedef struct {
    float road_heading_deg;
    float total_mm;
    float progress_mm;
    float last_fore_mm;
    float last_lat_mm;
    uint8_t initialized;
} MissionTrialRoad;

/* Start a NEW corridor at the current sample, with road progress zero. */
void mission_trial_road_init(MissionTrialRoad *road, float total_mm,
                            float road_heading_deg,
                            float fore_mm, float lat_mm);

/* Project each signed odometry increment onto the fixed road heading.
 * Call frequently during motion; a single sample spanning both a turn and
 * translation cannot reconstruct which heading applied to each increment. */
void mission_trial_road_update(MissionTrialRoad *road,
                              float continuous_heading_deg,
                              float fore_mm, float lat_mm);

/* After an EXPLICIT caller-controlled counter reset, replace only the sample
 * baseline. Preserve progress, total, and road heading. First update with
 * the final pre-reset sample, reset counters, then rebase to the new values.
 * No discontinuity/reset is guessed automatically. Do not use this to hide
 * ordinary movement or to start a different road (use init for that). */
void mission_trial_road_rebase_odometry(MissionTrialRoad *road,
                                      float fore_mm, float lat_mm);

/* Signed remaining distance: negative values retain overshoot evidence.
 * This is a measurement, not an arrival/recognition or motion decision.
 * Null/uninitialized ledgers return zero; callers must initialize before use. */
float mission_trial_road_remaining(const MissionTrialRoad *road);

#endif /* APP_MISSION_TRIAL_PLAN_H */
