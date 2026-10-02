#include "auto_steps.h"
#include "main.h"
#include "cmsis_os.h"
#include "motion.h"
#include "imu.h"
#include "steps.h"

/* Pitch peak-to-peak removes the fixed installation offset, not the need to
 * measure geometry or exit position. A chassis stuck at constant tilt can look
 * flat. DONE is a posture criterion, NOT proof the rear wheels cleared all
 * boards. Timestamp the window: both BT 20ms and mission 5ms ticks use 1s. */
#define X_CTRL_MS    5u
#define X_SAMPLE_MS  10u
#define X_WIN_MS     1000u
#define X_WIN_N      100u
#define X_W_MAX      2.0f
#define X_V_MAX      600.0f
static float s_rise_deg = 2.0f, s_flat_deg = 0.5f; /* uncalibrated RAM seeds */
static float s_pitch[X_WIN_N];
static uint32_t s_sample_ms[X_WIN_N];
static uint8_t s_next, s_count;
static uint32_t s_start_ms, s_first_sample_ms, s_last_sample_ms, s_deadline_ms;
static float s_vx, s_vy, s_heading0;
static CrossSnapshot s_state;

static float wrap180(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}
static CrossStatus finish(CrossStatus status)
{
    motion_brake();
    s_state.elapsed_ms = (uint32_t)(HAL_GetTick() - s_start_ms);
    s_state.status = status;
    return status;
}
static float pitch_pp(uint32_t now)
{
    float mn = 0.0f, mx = 0.0f;
    uint8_t have = 0u;
    for (uint8_t i = 0u; i < s_count; ++i) {
        if ((uint32_t)(now - s_sample_ms[i]) >= X_WIN_MS) continue;
        if (!have) { mn = mx = s_pitch[i]; have = 1u; }
        else {
            if (s_pitch[i] < mn) mn = s_pitch[i];
            if (s_pitch[i] > mx) mx = s_pitch[i];
        }
    }
    return have ? mx - mn : 0.0f;
}

int cross_begin(float vx_mms, float vy_mms, uint32_t run_ms)
{
    if (s_state.status == CROSS_RUNNING) return 0;
    s_state = (CrossSnapshot){0};
    s_start_ms = HAL_GetTick();
    s_count = s_next = 0u;
    s_first_sample_ms = s_last_sample_ms = s_start_ms;
    motion_brake();
    if (run_aborted()) { finish(CROSS_ABORT); return 0; }
    /* Positive range tests also reject NaN/infinity. */
    if (!(vx_mms >= -X_V_MAX && vx_mms <= X_V_MAX &&
          vy_mms >= -X_V_MAX && vy_mms <= X_V_MAX) ||
        (vx_mms == 0.0f && vy_mms == 0.0f)) {
        finish(CROSS_BAD_CONFIG); return 0;
    }
    if (!run_ms) { finish(CROSS_TIMEOUT); return 0; }
    if (!imu_ok()) { finish(CROSS_IMUERR); return 0; }
    s_vx = vx_mms; s_vy = vy_mms; s_deadline_ms = run_ms;
    s_heading0 = imu_yaw_deg();
    s_state.yaw_deg = s_heading0;
    s_state.pitch_deg = imu_pitch_deg();
    s_state.status = CROSS_RUNNING;
    return 1;
}

CrossStatus cross_tick(void)
{
    float w;
    uint32_t now;
    if (s_state.status != CROSS_RUNNING) return s_state.status;
    now = HAL_GetTick();
    s_state.elapsed_ms = (uint32_t)(now - s_start_ms);
    /* Every stop condition precedes this tick's motor command. */
    if (run_aborted()) return finish(CROSS_ABORT);
    if (s_state.elapsed_ms >= s_deadline_ms) return finish(CROSS_TIMEOUT);
    if (!imu_ok()) return finish(CROSS_IMUERR);
    s_state.yaw_deg = imu_yaw_deg();
    s_state.yaw_error_deg = wrap180(s_heading0 - s_state.yaw_deg);
    s_state.pitch_deg = imu_pitch_deg();

    if (s_count == 0u || (uint32_t)(now - s_last_sample_ms) >= X_SAMPLE_MS) {
        /* An unobserved >=1s interval cannot count as a stable window.
         * imu_ok checks valid-frame age; samples need not be unique frames. */
        if (s_count && (uint32_t)(now - s_last_sample_ms) >= X_WIN_MS) {
            s_count = s_next = 0u;
            s_state.window_full = s_state.rise_seen = 0u;
        }
        if (s_count == 0u) s_first_sample_ms = now;
        s_pitch[s_next] = s_state.pitch_deg;
        s_sample_ms[s_next] = now;
        s_next = (uint8_t)((s_next + 1u) % X_WIN_N);
        if (s_count < X_WIN_N) s_count++;
        s_last_sample_ms = now;
        s_state.window_full = (uint8_t)((uint32_t)(now - s_first_sample_ms) >= X_WIN_MS);
        s_state.pp_deg = pitch_pp(now);
        if (!s_state.rise_seen) {
            if (s_state.pp_deg > s_rise_deg) s_state.rise_seen = 1u;
        } else if (s_state.window_full && s_state.pp_deg < s_flat_deg) {
            return finish(CROSS_DONE);
        }
    }
    w = step_heading_kp_deg() * s_state.yaw_error_deg * 0.0174533f;
    if (w > X_W_MAX) w = X_W_MAX;
    if (w < -X_W_MAX) w = -X_W_MAX;
    motion_vel_set(s_vx, s_vy, w);
    return CROSS_RUNNING;
}

void cross_cancel(void)
{
    if (s_state.status == CROSS_RUNNING) (void)finish(CROSS_ABORT);
    else motion_brake();
}
CrossStatus cross_status(void) { return s_state.status; }
void cross_get(CrossSnapshot *out) { if (out) *out = s_state; }
const char *cross_status_name(CrossStatus status)
{
    switch (status) {
    case CROSS_IDLE: return "IDLE";
    case CROSS_RUNNING: return "RUNNING";
    case CROSS_DONE: return "DONE";
    case CROSS_ABORT: return "ABORT";
    case CROSS_IMUERR: return "IMUERR";
    case CROSS_TIMEOUT: return "TIMEOUT";
    case CROSS_BAD_CONFIG: return "BAD_CONFIG";
    default: return "UNKNOWN";
    }
}
int cross_tune_set(float rise_deg, float flat_deg)
{
    if (s_state.status == CROSS_RUNNING) return 0;
    if (!(flat_deg >= 0.01f && flat_deg < rise_deg && rise_deg <= 20.0f)) return 0;
    s_rise_deg = rise_deg; s_flat_deg = flat_deg;
    return 1;
}
void cross_tune_get(float *rise_deg, float *flat_deg)
{
    if (rise_deg) *rise_deg = s_rise_deg;
    if (flat_deg) *flat_deg = s_flat_deg;
}
int step_cross_obstacle(float vx_mms, float vy_mms, uint32_t run_ms)
{
    if (!cross_begin(vx_mms, vy_mms, run_ms)) return 0;
    while (cross_tick() == CROSS_RUNNING) osDelay(X_CTRL_MS);
    return cross_status() == CROSS_DONE;
}
