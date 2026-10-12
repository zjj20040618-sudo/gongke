#ifndef APP_ARC_TURN_TRIAL_H
#define APP_ARC_TURN_TRIAL_H

#include <stdint.h>

/* Pure wheel-rim speed planner. All gains/limits below are trial seeds,
 * not measured acceleration, turn radius or physical clearance evidence. */
#define ARC_TRIAL_MAX_WHEEL_MMS      600.0f
#define ARC_TRIAL_ACC_MMS2           700.0f
#define ARC_TRIAL_DEC_MMS2           350.0f
#define ARC_TRIAL_FINE_DEG           15.0f
#define ARC_TRIAL_TOL_DEG            0.3f
#define ARC_TRIAL_OVER_CORR_RATIO    0.25f
#define ARC_TRIAL_MAX_DT_MS          100u

typedef enum {
    ARC_TRIAL_OFF = 0, ARC_TRIAL_APPROACH, ARC_TRIAL_ARC,
    ARC_TRIAL_EXIT, ARC_TRIAL_DONE, ARC_TRIAL_STOPPED
} ArcTrialPhase;

typedef enum {
    ARC_TRIAL_REASON_NONE = 0, ARC_TRIAL_REASON_DONE,
    ARC_TRIAL_REASON_CANCELLED, ARC_TRIAL_REASON_BAD_INPUT,
    ARC_TRIAL_REASON_ANGLE_LIMIT, ARC_TRIAL_REASON_TIMEOUT
} ArcTrialReason;

typedef struct {
    float before_mm;       /* Existing forward approach, not extra travel. */
    float after_mm;        /* 0: finish at the arc; never add a hidden exit. */
    float wheel_v_mms;     /* Rim speed of the two forward body-left wheels. */
    float target_deg;      /* Positive clockwise angle from the arc's first frame. */
    uint32_t max_ms;       /* Owner-selected total deadline, including approach. */
} ArcTrialConfig;

typedef struct {
    ArcTrialConfig config;
    ArcTrialPhase phase;
    ArcTrialReason reason;
    uint32_t start_ms, last_ms;
    float yaw_start, yaw_goal;
    float leg_odo0, remaining_mm, error_deg;
    float left_mms, right_mms; /* Commands only: no claim of measured wheel speed. */
} ArcTrial;

void arc_trial_init(ArcTrial *trial);
int arc_trial_begin(ArcTrial *trial, const ArcTrialConfig *config,
                    uint32_t now, float yaw, float forward_odo_mm);
/* Direct arc entry without a zero-speed gap; copies config but forces before=0.
 * Inputs must be current nonnegative rim-speed commands, not PWM duties. */
int arc_trial_begin_at_arc(ArcTrial *trial, const ArcTrialConfig *config,
                           uint32_t now, float yaw, float forward_odo_mm,
                           float left_mms, float right_mms);
ArcTrialPhase arc_trial_update(ArcTrial *trial, uint32_t now,
                               float yaw, float forward_odo_mm);
void arc_trial_stop(ArcTrial *trial, ArcTrialReason reason);
int arc_trial_active(const ArcTrial *trial);

#endif
