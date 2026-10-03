#include "mission_trial.h"
#include "mission_trial_plan.h"
#include "steps.h"
#include "motion.h"
#include "imu.h"
#include "proto.h"
#include "board_pins.h"
#include "test.h"
#include "turn_profile.h"
#include "main.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdio.h>

#define DEG_RAD 0.01745329252f
#define TRIAL_CX_TOL 8.0f
#define TRIAL_CX_FRAMES 5u
#define TRIAL_CX_KP 0.6f
#define TRIAL_CX_MIN 12.0f
#define TRIAL_CX_MAX 80.0f
#define TRIAL_FRAME_AGE_MS 300u

static MissionTrialRoad s_road;
static volatile uint8_t s_tracking;
static volatile uint8_t s_running;
static const char *volatile s_phase = "BOOT";
static float s_heading;
static int s_cx[4], s_sign;
static uint8_t s_cx_ready[4];
static int32_t s_qr[3];
static unsigned s_hits;

static float wrap(float d)
{
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}
static float clamp(float v, float lim)
{
    if (v > lim) return lim;
    if (v < -lim) return -lim;
    return v;
}
static void phase(const char *name)
{
    s_phase = name;
    bp_debug_send("\r\nTRIAL32 PHASE="); bp_debug_send(name); bp_debug_send("\r\n");
}
void mission_trial_init(void)
{
    s_tracking = s_running = 0u; s_road.initialized = 0u;
    s_sign = 0; s_phase = "BOOT"; s_hits = 0u;
    for (int i = 0; i < 4; ++i) { s_cx[i] = 0; s_cx_ready[i] = 0u; }
    s_qr[0] = s_qr[1] = s_qr[2] = 0;
}
const char *mission_trial_config_missing(void)
{
    static const char *const names[4] = { "bcx", "tcx", "hcx", "kcx" };
    if (s_sign != 1 && s_sign != -1) return "vsg1_or_vsg2";
    for (int i = 0; i < 4; ++i) if (!s_cx_ready[i]) return names[i];
    return 0;
}
int mission_trial_set_alignment(int cls, int cx, int sign)
{
    if (cls < -1 || cls > 3 || cx < -1 || cx >= 480
        || (sign != 0 && sign != 1 && sign != -1) || s_running) return 0;
    if (cls >= 0 && cx >= 0) { s_cx[cls] = cx; s_cx_ready[cls] = 1u; }
    if (sign) s_sign = sign;
    return 1;
}
const char *mission_trial_phase(void) { return s_phase; }
void mission_trial_get_qr(int32_t out[3])
{
    if (out) { out[0] = s_qr[0]; out[1] = s_qr[1]; out[2] = s_qr[2]; }
}
void mission_trial_get_progress(float *done, float *total)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    if (done) *done = s_road.initialized ? s_road.progress_mm : 0.0f;
    if (total) *total = s_road.initialized ? s_road.total_mm : 0.0f;
    __set_PRIMASK(pm);
}
void mission_trial_report(void)
{
    char b[160]; /* Both MissionTask and DefaultTask call this: formatter is per-call. */
    float done, total;
    mission_trial_get_progress(&done, &total);
    snprintf(b, sizeof b, "\r\nTRIAL32 v=100 task=2450 rescue=2125 hold=10000 sign=%d cx=%d,%d,%d,%d ready=%u%u%u%u\r\n",
        s_sign, s_cx[0], s_cx[1], s_cx[2], s_cx[3],
        s_cx_ready[0], s_cx_ready[1], s_cx_ready[2], s_cx_ready[3]);
    bp_debug_send(b);
    for (unsigned i = 0u; i < MISSION_TRIAL_ROUTE_LEGS; ++i) {
        const MissionTrialRouteLeg *leg = &mission_trial_route_plan[i];
        snprintf(b, sizeof b, "TRIAL32 R%u mode=%u d=%u turn=%d\r\n",
                 i + 1u, leg->mode, leg->distance_mm, leg->turn_deg);
        bp_debug_send(b);
    }
    snprintf(b, sizeof b, "TRIAL32 phase=%s road=%ld/%ld hits=%u QR=%ld,%ld,%ld no_arm=1 no_retry=1\r\n",
        s_phase, (long)done, (long)total, s_hits,
        (long)s_qr[0], (long)s_qr[1], (long)s_qr[2]);
    bp_debug_send(b);
}
void mission_trial_tick_1ms(void)
{
    if (s_tracking && imu_ok())
        mission_trial_road_update(&s_road, imu_heading_deg(),
                                 motion_odo_mm(), motion_lateral_odo_mm());
}
static void road_start(float total, float road_heading)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    s_tracking = 0u;
    mission_trial_road_init(&s_road, total, road_heading,
                           motion_odo_mm(), motion_lateral_odo_mm());
    s_tracking = 1u;
    __set_PRIMASK(pm);
}
static float remaining(void)
{
    float done, total;
    mission_trial_get_progress(&done, &total);
    return total - done;
}
static float heading_w(void)
{
    return clamp(step_heading_kp_deg() * DEG_RAD * wrap(s_heading - imu_heading_deg()), 2.0f);
}
/* Translation keeps the route's original heading, not each stop's drifted
 * heading. The precise entry preserves corrections below one integer RPM.
 * Turning deliberately retains the successful integer-wheel hold profile. */
static void road_velocity(float fore, float lat)
{
    float a = wrap(imu_heading_deg() - s_heading) * DEG_RAD;
    float c = cosf(a), s = sinf(a);
    /* Preserve the user's positive-forward small LEFT compensation only.
     * No mirroring onto backward/left strafe or uncalibrated image alignment. */
    if (fore > 0.0f && lat == 0.0f) lat -= test_forward_ff_ratio() * fore;
    motion_vel_set_precise(fore * c + lat * s, -fore * s + lat * c, heading_w());
}
static int stopped_hold(uint32_t ms)
{
    motion_brake();
    if (!step_prepare_leg()) return 0;
    wait_ms(ms);
    return !run_aborted();
}
static int translate(int lateral, float signed_distance, const char *name)
{
    MotionRamp ramp;
    float dir = signed_distance > 0.0f ? 1.0f : -1.0f;
    float axis_heading = s_heading + (lateral ? dir * 90.0f : (dir < 0.0f ? 180.0f : 0.0f));
    phase(name);
    if (!step_prepare_leg()) return 0;
    road_start(fabsf(signed_distance), axis_heading);
    motion_linear_ramp_init(&ramp);
    while (!run_aborted()) {
        float rest = remaining();
        float speed;
        if (!imu_ok()) break;
        if (rest <= 0.0f) { motion_brake(); return 1; }
        speed = motion_linear_profile_step(&ramp, dir * MISSION_TRIAL_ROUTE_SPEED_MMS, rest, 0.005f);
        road_velocity(lateral ? 0.0f : speed, lateral ? speed : 0.0f);
        osDelay(5);
    }
    motion_brake(); return 0;
}
static int turn(float angle, const char *name)
{
    uint32_t t0, settled = 0u;
    float settle_yaw;
    phase(name);
    if (!step_prepare_leg()) return 0;
    s_heading += angle; /* +180 means exactly +180, not 2 times calibrated +85. */
    t0 = HAL_GetTick(); settle_yaw = imu_heading_deg();
    while (!run_aborted()) {
        uint32_t now = HAL_GetTick();
        /* Continuous error preserves the commanded +180 direction even if
         * the pre-turn heading is slightly below the planned heading. */
        float h = imu_heading_deg(), e = s_heading - h, ae = fabsf(e);
        if (!imu_ok() || (uint32_t)(now - t0) >= TURN_HOLD_MAX_MS) break;
        if (ae <= TURN_HOLD_TOL_DEG) {
            motion_brake();
            if (!settled || fabsf(h - settle_yaw) > TURN_HOLD_STILL_DEG) {
                settled = now;
                settle_yaw = h;
            }
            if ((uint32_t)(now - settled) >= TURN_HOLD_SETTLE_MS) return 1;
        } else {
            float w = clamp(TURN_HOLD_KP_RADS_DEG * e, TURN_HOLD_MAX_W_RADS);
            settled = 0u;
            if (w > 0.0f && w < TURN_HOLD_MIN_W_RADS) w = TURN_HOLD_MIN_W_RADS;
            if (w < 0.0f && w > -TURN_HOLD_MIN_W_RADS) w = -TURN_HOLD_MIN_W_RADS;
            motion_vel_set(0.0f, 0.0f, w);
        }
        osDelay(5);
    }
    motion_brake(); return 0;
}
static int align(int cls, int label, const char *name)
{
    uint32_t last_frame;
    uint32_t trace_t0;
    unsigned good = 0u;
    phase(name);
    if (!step_prepare_leg()) return 0;
    step_object_select(cls, label);
    last_frame = HAL_GetTick();
    trace_t0 = last_frame;
    while (!run_aborted()) {
        ProtoFrame f;
        if (!imu_ok()) break;
        if (step_object_take(&f) && f.cls == cls && (label < 0 || f.label == label)) {
            float e = (float)(f.cx - s_cx[cls]);
            float applied = 0.0f;
            last_frame = HAL_GetTick();
            if (fabsf(e) <= TRIAL_CX_TOL) {
                motion_brake();
                ++good;
            } else {
                float v = clamp((float)s_sign * TRIAL_CX_KP * e, TRIAL_CX_MAX);
                good = 0u;
                if (fabsf(v) < TRIAL_CX_MIN) v = v > 0.0f ? TRIAL_CX_MIN : -TRIAL_CX_MIN;
                applied = v;
                motion_vel_set_precise(v, 0.0f, heading_w());
            }
            if (good >= TRIAL_CX_FRAMES || (uint32_t)(last_frame - trace_t0) >= 1000u) {
                static char trace[152]; /* Only the MissionTask alignment path uses it. */
                trace_t0 = last_frame;
                snprintf(trace, sizeof trace,
                    "TRIAL32 ALIGN cls=%d lab=%d cx=%d target=%d err_px=%.1f cmd_vx=%.1f yaw=%.2f good=%u seq=%u\r\n",
                    cls, label, f.cx, s_cx[cls], e, applied, imu_heading_deg(), good, f.sequence);
                bp_debug_send(trace);
            }
            if (good >= TRIAL_CX_FRAMES) return 1;
        } else if ((uint32_t)(HAL_GetTick() - last_frame) > TRIAL_FRAME_AGE_MS) {
            good = 0u; motion_brake(); /* no stale velocity; no return/retry path */
        }
        osDelay(5);
    }
    motion_brake(); return 0;
}
/* A single continuous corridor, never a new full-length leg after parking.
 * 1=selected object found, 0=road end, -1=manual/sensor stop. */
static int pass_until(int cls, int label, const char *name)
{
    MotionRamp ramp;
    phase(name);
    if (!step_prepare_leg()) return -1;
    step_object_select(cls, label);
    motion_linear_ramp_init(&ramp);
    while (!run_aborted()) {
        ProtoFrame f;
        float rest = remaining();
        if (!imu_ok()) break;
        if (rest <= 0.0f) { motion_brake(); return 0; }
        if (cls >= 0 && step_object_take(&f) && f.cls == cls && (label < 0 || f.label == label)) {
            motion_brake(); return 1;
        }
        road_velocity(motion_linear_profile_step(&ramp, MISSION_TRIAL_ROUTE_SPEED_MMS, rest, 0.005f), 0.0f);
        osDelay(5);
    }
    motion_brake(); return -1;
}
static int read_qr(void)
{
    int32_t d[3];
    phase("QR_STOP10S");
    if (!step_vision_scene(SCENE_QR) || !stopped_hold(10000u)) return 0;
    while (!run_aborted()) {
        if (!wait_qr(d, 0u)) return 0;
        if (d[0] >= 1 && d[0] <= 3 && d[1] >= 1 && d[1] <= 3 && d[2] >= 1 && d[2] <= 3) {
            uint32_t pm = __get_PRIMASK();
            __disable_irq();
            s_qr[0] = d[0]; s_qr[1] = d[1]; s_qr[2] = d[2];
            __set_PRIMASK(pm);
            phase("QR_VALID"); mission_trial_report();
            return 1;
        }
    }
    return 0;
}
int mission_trial_run(void)
{
    static const char *const leg_name[7] = {
        "R1_LEFT500", "R2_BACK600", "R3_LEFT95", "R4_CROSS_ROAD750",
        "R5_LEFT730", "R6_FORWARD830", "R7_RIGHT85"
    };
    int found;
    if (run_aborted() || mission_trial_config_missing() || !imu_ok()) return 0;
    s_tracking = 0u; s_running = 1u; s_road.initialized = 0u; s_hits = 0u;
    s_qr[0] = s_qr[1] = s_qr[2] = 0;
    s_heading = imu_heading_deg();
    bp_laser_set(0);
    for (unsigned i = 0u; i < 7u; ++i) {
        const MissionTrialRouteLeg *leg = &mission_trial_route_plan[i];
        if (leg->turn_deg) {
            if (!turn((float)leg->turn_deg, leg_name[i])) goto stop;
        } else {
            float d = (float)leg->distance_mm;
            if (leg->mode == 16u || leg->mode == 17u) d = -d;
            if (!translate(leg->mode == 17u, d, leg_name[i])) goto stop;
        }
        if (i == 0u && !read_qr()) goto stop;
    }
    road_start((float)mission_trial_route_plan[7].distance_mm, s_heading);
    if (!step_vision_scene(SCENE_EOD)) goto stop;
    found = pass_until(CLS_BALL, (int)s_qr[0] - 1, "BALL_SINGLE_PASS");
    if (found < 0) goto stop;
    if (found > 0) {
        if (!align(CLS_BALL, (int)s_qr[0] - 1, "BALL_ALIGN")) goto stop;
        phase("BALL_HOLD10S");
        if (!stopped_hold(MISSION_TRIAL_BALL_HOLD_MS)
            || !turn(180.0f, "BALL_TURN180")
            || !align(CLS_BUCKET, -1, "BUCKET_ALIGN")) goto stop;
        phase("BUCKET_HOLD10S");
        if (!stopped_hold(MISSION_TRIAL_BUCKET_HOLD_MS)
            || !turn(180.0f, "BUCKET_TURN180_BACK")) goto stop;
        s_hits |= 1u;
    }
    if (!step_vision_scene(SCENE_ANTI)) goto stop;
    found = pass_until(CLS_TARGET, (int)s_qr[1] - 1, "TARGET_SINGLE_PASS");
    if (found < 0) goto stop;
    if (found > 0) {
        if (!align(CLS_TARGET, (int)s_qr[1] - 1, "TARGET_ALIGN")) goto stop;
        phase("TARGET_LASER");
        if (!step_fire(2000u)) goto stop;
        s_hits |= 2u;
    }
    if (pass_until(-1, -1, "TASK_REMAINDER2450") < 0
        || !turn((float)mission_trial_route_plan[8].turn_deg, "R9_RIGHT85")) goto stop;
    road_start((float)mission_trial_route_plan[9].distance_mm, s_heading);
    if (!step_vision_scene(SCENE_RESCUE)) goto stop;
    found = pass_until(CLS_HOSTAGE, (int)s_qr[2] + 2, "HOSTAGE_SINGLE_PASS");
    if (found < 0) goto stop;
    if (found > 0) {
        if (!align(CLS_HOSTAGE, (int)s_qr[2] + 2, "HOSTAGE_ALIGN")) goto stop;
        phase("HOSTAGE_HOLD10S");
        if (!stopped_hold(MISSION_TRIAL_HOSTAGE_HOLD_MS)) goto stop;
        s_hits |= 4u;
    }
    if (pass_until(-1, -1, "RESCUE_REMAINDER2125") < 0) goto stop;
    motion_brake();
    if (!step_prepare_leg()) goto stop;
    s_tracking = s_running = 0u;
    bp_laser_set(0); proto_send_scene(SCENE_IDLE);
    phase("ROUTE_END"); mission_trial_report();
    return 1; /* road end only; hits is evidence, not an invented task pass */
stop:
    motion_brake(); bp_laser_set(0); s_tracking = s_running = 0u;
    proto_send_scene(SCENE_IDLE); phase("STOP"); mission_trial_report();
    return 0;
}
