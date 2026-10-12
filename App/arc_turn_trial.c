#include "arc_turn_trial.h"
#include <math.h>
#include <string.h>

static int finite_speed(float value)
{ return isfinite(value) && value >= 0.0f && value <= ARC_TRIAL_MAX_WHEEL_MMS; }

static int config_valid(const ArcTrialConfig *config)
{
    return config && isfinite(config->before_mm) && config->before_mm >= 0.0f &&
        config->before_mm <= 65535.0f && isfinite(config->after_mm) &&
        config->after_mm >= 0.0f && config->after_mm <= 65535.0f &&
        finite_speed(config->wheel_v_mms) && config->wheel_v_mms >= 1.0f &&
        isfinite(config->target_deg) && config->target_deg > 0.0f &&
        config->target_deg <= 180.0f && config->max_ms > 0u &&
        config->max_ms <= 0x7fffffffu;
}

void arc_trial_init(ArcTrial *trial)
{
    if (!trial) return;
    memset(trial, 0, sizeof *trial);
    trial->phase = ARC_TRIAL_OFF;
}

void arc_trial_stop(ArcTrial *trial, ArcTrialReason reason)
{
    if (!trial) return;
    trial->left_mms = trial->right_mms = 0.0f;
    trial->phase = ARC_TRIAL_STOPPED;
    trial->reason = reason == ARC_TRIAL_REASON_NONE || reason == ARC_TRIAL_REASON_DONE
        ? ARC_TRIAL_REASON_CANCELLED : reason;
}

int arc_trial_active(const ArcTrial *trial)
{
    return trial && (trial->phase == ARC_TRIAL_APPROACH ||
                    trial->phase == ARC_TRIAL_ARC || trial->phase == ARC_TRIAL_EXIT);
}

static void enter_arc(ArcTrial *trial, float yaw)
{
    trial->phase = ARC_TRIAL_ARC;
    trial->yaw_start = yaw;
    trial->yaw_goal = yaw + trial->config.target_deg;
    trial->error_deg = trial->config.target_deg;
    trial->remaining_mm = 0.0f;
}

static int begin_with_speed(ArcTrial *trial, const ArcTrialConfig *config,
                             uint32_t now, float yaw, float odo,
                             float left_mms, float right_mms)
{
    ArcTrialConfig copied;
    if (!trial) return 0;
    /* Copy first: callers may restart using &trial->config. */
    if (config) copied = *config;
    if (!config_valid(config) || !isfinite(yaw) || !isfinite(odo) ||
        !finite_speed(left_mms) || !finite_speed(right_mms) ||
        !isfinite(yaw + config->target_deg)) {
        arc_trial_stop(trial, ARC_TRIAL_REASON_BAD_INPUT); return 0;
    }
    arc_trial_init(trial);
    trial->config = copied;
    trial->start_ms = trial->last_ms = now;
    trial->leg_odo0 = odo;
    trial->remaining_mm = copied.before_mm;
    trial->left_mms = left_mms; trial->right_mms = right_mms;
    if (copied.before_mm > 0.0f) trial->phase = ARC_TRIAL_APPROACH;
    else enter_arc(trial, yaw);
    return 1;
}

int arc_trial_begin(ArcTrial *trial, const ArcTrialConfig *config,
                    uint32_t now, float yaw, float forward_odo_mm)
{
    return begin_with_speed(trial, config, now, yaw, forward_odo_mm, 0.0f, 0.0f);
}

int arc_trial_begin_at_arc(ArcTrial *trial, const ArcTrialConfig *config,
                           uint32_t now, float yaw, float forward_odo_mm,
                           float left_mms, float right_mms)
{
    ArcTrialConfig copied;
    if (!config) { arc_trial_stop(trial, ARC_TRIAL_REASON_BAD_INPUT); return 0; }
    copied = *config; copied.before_mm = 0.0f;
    return begin_with_speed(trial, &copied, now, yaw, forward_odo_mm,
                            left_mms, right_mms);
}

static float ramp_speed(float current, float target, float dt)
{
    float change = (target < current ? ARC_TRIAL_DEC_MMS2 : ARC_TRIAL_ACC_MMS2) * dt;
    if (target > current) return fminf(target, current + change);
    return fmaxf(target, current - change);
}

static void set_targets(ArcTrial *trial, float left, float right, float dt)
{
    trial->left_mms = ramp_speed(trial->left_mms, left, dt);
    trial->right_mms = ramp_speed(trial->right_mms, right, dt);
}

ArcTrialPhase arc_trial_update(ArcTrial *trial, uint32_t now,
                               float yaw, float forward_odo_mm)
{
    uint32_t elapsed;
    float dt, left, right, progress;
    if (!trial) return ARC_TRIAL_STOPPED;
    if (!arc_trial_active(trial)) return trial->phase;
    if (!config_valid(&trial->config) || !isfinite(yaw) || !isfinite(forward_odo_mm) ||
        !finite_speed(trial->left_mms) || !finite_speed(trial->right_mms)) {
        arc_trial_stop(trial, ARC_TRIAL_REASON_BAD_INPUT); return trial->phase;
    }
    if ((uint32_t)(now - trial->start_ms) >= trial->config.max_ms) {
        arc_trial_stop(trial, ARC_TRIAL_REASON_TIMEOUT); return trial->phase;
    }
    elapsed = now - trial->last_ms;
    trial->last_ms = now;
    /* A late scheduler call must not jump straight to a large wheel target. */
    if (elapsed > ARC_TRIAL_MAX_DT_MS) elapsed = ARC_TRIAL_MAX_DT_MS;
    dt = (float)elapsed * 0.001f;
    left = right = trial->config.wheel_v_mms;

    if (trial->phase == ARC_TRIAL_APPROACH) {
        progress = forward_odo_mm - trial->leg_odo0;
        if (!isfinite(progress)) {
            arc_trial_stop(trial, ARC_TRIAL_REASON_BAD_INPUT); return trial->phase;
        }
        trial->remaining_mm = fmaxf(0.0f, trial->config.before_mm - progress);
        if (progress < trial->config.before_mm) {
            set_targets(trial, left, right, dt); return trial->phase;
        }
        /* Lock the fresh continuous heading BEFORE the first steering command.
         * No brake, encoder reset or IMU zero at this moving handoff. */
        if (!isfinite(yaw + trial->config.target_deg)) {
            arc_trial_stop(trial, ARC_TRIAL_REASON_BAD_INPUT); return trial->phase;
        }
        enter_arc(trial, yaw);
    }

    if (trial->phase == ARC_TRIAL_ARC) {
        float traveled = yaw - trial->yaw_start;
        trial->error_deg = trial->yaw_goal - yaw;
        if (!isfinite(traveled) || !isfinite(trial->error_deg)) {
            arc_trial_stop(trial, ARC_TRIAL_REASON_BAD_INPUT); return trial->phase;
        }
        if (traveled < -15.0f || traveled > trial->config.target_deg + 15.0f) {
            arc_trial_stop(trial, ARC_TRIAL_REASON_ANGLE_LIMIT); return trial->phase;
        }
        if (trial->error_deg >= ARC_TRIAL_FINE_DEG) right = 0.0f;
        else if (trial->error_deg >= ARC_TRIAL_TOL_DEG) {
            /* Smoothly rejoin straight travel instead of braking both sides. */
            right = left * (1.0f - trial->error_deg / ARC_TRIAL_FINE_DEG);
        } else if (trial->error_deg <= -ARC_TRIAL_TOL_DEG) {
            float correction = fminf(left * (-trial->error_deg) / ARC_TRIAL_FINE_DEG,
                                      left * ARC_TRIAL_OVER_CORR_RATIO);
            /* Small counter-steer without reversing either side; at the speed
             * ceiling reduce left instead of commanding right above600. */
            right = fminf(ARC_TRIAL_MAX_WHEEL_MMS, left + correction);
            left = right - correction;
        }
        set_targets(trial, left, right, dt);
        /* Do not report success while right is still accelerating back. These
         * are planner COMMANDS, not encoder-confirmed actual wheel speeds. */
        if (fabsf(trial->error_deg) < ARC_TRIAL_TOL_DEG &&
            fabsf(trial->left_mms - trial->right_mms) <= 0.001f &&
            trial->left_mms > 0.0f && trial->right_mms > 0.0f) {
            if (trial->config.after_mm <= 0.0f) {
                trial->phase = ARC_TRIAL_DONE; trial->reason = ARC_TRIAL_REASON_DONE;
                /* Owner brakes for its next visual task; no extra exit metres. */
            } else {
                trial->phase = ARC_TRIAL_EXIT; trial->leg_odo0 = forward_odo_mm;
                trial->remaining_mm = trial->config.after_mm;
            }
        }
        return trial->phase;
    }

    progress = forward_odo_mm - trial->leg_odo0;
    if (!isfinite(progress)) {
        arc_trial_stop(trial, ARC_TRIAL_REASON_BAD_INPUT); return trial->phase;
    }
    trial->remaining_mm = fmaxf(0.0f, trial->config.after_mm - progress);
    left = progress >= trial->config.after_mm ? 0.0f
        : fminf(left, sqrtf(2.0f * ARC_TRIAL_DEC_MMS2 * trial->remaining_mm));
    set_targets(trial, left, left, dt);
    if (progress >= trial->config.after_mm &&
        trial->left_mms <= 0.001f && trial->right_mms <= 0.001f) {
        trial->left_mms = trial->right_mms = 0.0f;
        trial->phase = ARC_TRIAL_DONE; trial->reason = ARC_TRIAL_REASON_DONE;
    }
    return trial->phase;
}
