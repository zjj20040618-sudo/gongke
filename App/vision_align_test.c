#include "vision_align_test.h"
#include "main.h"
#include "motion.h"
#include "control.h"
#include "imu.h"
#include "steps.h"
#include "board_pins.h"
#include "test_config.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    ProtoFrame frame;
    uint32_t packet, at;
    uint8_t seen;
} VatSample;
static volatile VatSample s_sample;
static VisionAlignTestStatus s_vat;
static int32_t s_counts[4];
static int32_t s_qr[3];
static uint32_t s_last_poll, s_still_from, s_fix_from, s_fix_stable_from;
static uint32_t s_hold_from, s_used_packet, s_recheck_packet, s_session_packet, s_rejected_packet;
static uint16_t s_last_seq;
static uint16_t s_width, s_height;
static float s_kp, s_fix_last_yaw;
static int s_y_target, s_direction;
static uint8_t s_have_seq, s_seen_target, s_need_new, s_initial_search, s_stable;

static int vat_running(void)
{
    return s_vat.state != VAT_OFF && s_vat.state != VAT_DONE && s_vat.state != VAT_STOPPED;
}
int vision_align_test_active(void) { return vat_running(); }
static void vat_brake(void)
{
    motion_brake(); s_vat.vx = s_vat.vy = s_vat.w = 0.0f;
}
static void vat_sample_clear(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq(); memset((void *)&s_sample, 0, sizeof s_sample);
    __set_PRIMASK(pm);
}
static void vat_stop(const char *reason, VisionAlignTestState state)
{
    vat_brake(); bp_laser_set(0); proto_receive_end(); vat_sample_clear();
    s_vat.state = state; s_vat.reason = reason; s_vat.latest = 0u;
    s_vat.good = state == VAT_DONE ? VAT_GOOD_FRAMES : 0u;
    s_vat.axis = 0u; s_direction = 0;
}
void vision_align_test_init(void)
{
    memset(&s_vat, 0, sizeof s_vat); vat_sample_clear();
    s_vat.reason = "OFF"; s_vat.age_ms = UINT32_MAX;
    bp_laser_set(0);
}
void vision_align_test_cancel(void) { vat_stop("CANCEL", VAT_STOPPED); }
static void vat_count_reset(uint32_t now)
{
    for (int i = 0; i < 4; ++i) s_counts[i] = ctrl_enc_total(i);
    s_still_from = now;
}
static int vat_counts_changed(uint32_t now)
{
    int changed = 0;
    for (int i = 0; i < 4; ++i) {
        int32_t value = ctrl_enc_total(i);
        if (value != s_counts[i]) { s_counts[i] = value; changed = 1; }
    }
    if (changed) { s_still_from = now; s_vat.good = 0u; }
    return changed;
}
static int vat_finite(float value)
{
    uint32_t bits;
    /* STM32/host IEEE754 float: keep the fault gate valid with fast-math,
     * which may otherwise fold isfinite() into an unconditional true. */
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x7f800000u) != 0x7f800000u;
}
static int vat_error(float *out)
{
    float yaw = imu_heading_deg(), error;
    if (!vat_finite(yaw) || !vat_finite(s_vat.heading_target_deg)) return 0;
    error = s_vat.heading_target_deg - yaw;
    if (!vat_finite(error)) return 0;
    error = fmodf(error, 360.0f);
    if (!vat_finite(error)) return 0;
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;
    *out = error; return 1;
}
static void vat_begin_brake(uint32_t now, int need_new)
{
    vat_brake(); vat_count_reset(now);
    s_vat.state = VAT_BRAKE; s_vat.reason = "BRAKE_YAW";
    s_vat.axis = s_vat.good = s_vat.latest = 0u; s_direction = 0;
    s_need_new = (uint8_t)need_new; s_stable = 0u;
    s_fix_from = now;
}
static int vat_request(ProtoTask task, uint8_t digit, uint32_t now)
{
    ProtoStats stats;
    ProtoWireDiag diag;
    int cls, label;
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    if (!proto_target_filter(task, digit, &cls, &label) || !proto_send_target(task, digit)) {
        __set_PRIMASK(pm); return 0;
    }
    proto_wire_diag_get(&diag); proto_stats_get(&stats);
    memset((void *)&s_sample, 0, sizeof s_sample);
    s_vat.task = (uint8_t)task; s_vat.digit = digit;
    s_vat.cls = cls; s_vat.label = label; s_vat.request = diag.request;
    s_session_packet = s_used_packet = s_recheck_packet = stats.obj;
    s_have_seq = s_seen_target = 0u; s_width = s_height = 0u; s_rejected_packet = 0u;
    s_vat.latest = s_vat.good = 0u; s_vat.age_ms = UINT32_MAX;
    __set_PRIMASK(pm);
    s_y_target = task == PROTO_TASK_BALL ? VAT_BALL_Y_PX :
                 task == PROTO_TASK_BUCKET ? VAT_BUCKET_Y_PX : VAT_HOSTAGE_Y_PX;
    s_initial_search = task != PROTO_TASK_BUCKET;
    vat_begin_brake(now, 0);
    return 1;
}
int vision_align_test_start(uint8_t mode, const int32_t qr[3], float heading_kp)
{
    int32_t current[3];
    float heading = imu_heading_deg();
    if (vat_running() || mode < 38u || mode > 41u || !qr || !vat_finite(heading_kp)
        || heading_kp < 0.0f || heading_kp > 5.0f || !imu_ok() || !vat_finite(heading)
        || run_aborted() || !proto_qr_get(current)) return 0;
    for (unsigned i = 0; i < 3u; ++i)
        if (qr[i] < 1 || qr[i] > 3 || qr[i] != current[i]) return 0;
    memset(&s_vat, 0, sizeof s_vat); s_vat.mode = mode; s_vat.heading_target_deg = heading;
    memcpy(s_qr, qr, sizeof s_qr); s_kp = heading_kp; s_last_poll = HAL_GetTick();
    bp_laser_set(0); vat_brake();
    ProtoTask task = mode == 39u ? PROTO_TASK_BUCKET : mode == 40u ? PROTO_TASK_HOSTAGE : PROTO_TASK_BALL;
    uint8_t digit = task == PROTO_TASK_BUCKET ? 0u : (uint8_t)s_qr[task == PROTO_TASK_HOSTAGE ? 2u : 0u];
    if (!vat_request(task, digit, s_last_poll)) { vat_stop("REQUEST", VAT_STOPPED); return 0; }
    return 1;
}
void vision_align_test_feed_frame(const ProtoFrame *frame)
{
    ProtoStats stats;
    ProtoWireDiag diag;
    if (!frame || frame->type != PF_OBJ || !vat_running()) return;
    proto_wire_diag_get(&diag);
    if (!diag.receiving || !diag.ack || diag.failed || diag.mode != 2u
        || diag.request != s_vat.request || diag.task != s_vat.task || diag.selection != s_vat.digit) return;
    proto_stats_get(&stats);
    s_sample.frame = *frame; s_sample.packet = stats.obj; s_sample.at = HAL_GetTick();
    s_sample.seen = (uint8_t)(frame->cls == s_vat.cls && (s_vat.label < 0 || frame->label == s_vat.label));
}
static int vat_snapshot(VatSample *sample, ProtoStats *stats)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq(); *sample = s_sample; proto_stats_get(stats); __set_PRIMASK(pm);
    return sample->seen && sample->packet == stats->obj && sample->packet != s_session_packet;
}
static int vat_ack_ready(void)
{
    ProtoWireDiag diag;
    proto_wire_diag_get(&diag);
    return diag.receiving && diag.ack && !diag.failed && diag.request == s_vat.request
        && diag.mode == 2u && diag.task == s_vat.task && diag.selection == s_vat.digit;
}
static int vat_geometry(const ProtoFrame *f)
{
    if (!f->img_w || !f->img_h || VAT_X_PX + VAT_TOL_PX >= f->img_w
        || s_y_target + VAT_TOL_PX >= f->img_h || f->cx < 0 || f->cx >= f->img_w
        || f->cy < 0 || f->cy >= f->img_h) return 0;
    if (s_width && (s_width != f->img_w || s_height != f->img_h)) return 0;
    s_width = f->img_w; s_height = f->img_h; return 1;
}
static void vat_drive(float vx, float vy, float w)
{
    s_vat.vx = vx; s_vat.vy = vy; s_vat.w = w;
    motion_vel_set_precise(vx, vy, w);
}
static float vat_motion_w(void)
{
    float w = s_kp * 0.0174533f * s_vat.heading_error_deg;
    if (fabsf(w) > 2.0f) w = copysignf(2.0f, w);
    return w;
}
static void vat_drive_target(uint32_t packet, uint32_t now, float vx, float vy)
{
    VatSample current;
    ProtoStats stats;
    float error = 0.0f;
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    /* A newer empty/missing packet can arrive between the main snapshot and
     * dispatch. Never issue an old-coordinate command after that invalidation. */
    if (vat_snapshot(&current, &stats) && current.packet == packet
        && packet != s_rejected_packet && (uint32_t)(now - current.at) <= VAT_FRESH_MS
        && proto_scene_status() == 1 && !run_aborted() && imu_ok() && vat_error(&error)) {
        s_vat.heading_error_deg = error;
        vat_drive(vx, vy, vat_motion_w());
    } else {
        s_initial_search = 0u; vat_begin_brake(now, 1);
    }
    __set_PRIMASK(pm);
}
static void vat_finish_alignment(uint32_t now)
{
    vat_brake(); proto_receive_end(); vat_sample_clear(); s_hold_from = now;
    s_vat.latest = s_vat.axis = 0u; s_vat.good = VAT_GOOD_FRAMES;
    if (s_vat.mode == 41u) {
        s_vat.state = s_vat.task == PROTO_TASK_BALL ? VAT_HOLD_BALL : VAT_HOLD_BUCKET;
        s_vat.reason = "HOLD5";
    } else vat_stop("ALIGNED", VAT_DONE);
}
static int vat_final_check(uint32_t packet, uint32_t now)
{
    VatSample final;
    ProtoStats stats;
    float error = 0.0f;
    uint32_t pm = __get_PRIMASK();
    int valid;
    __disable_irq();
    valid = vat_snapshot(&final, &stats) && final.packet == packet
        && (uint32_t)(now - final.at) <= VAT_FRESH_MS && !run_aborted() && imu_ok()
        && proto_scene_status() == 1 && vat_error(&error) && fabsf(error) <= T_DIST_ALIGN_TOL_DEG
        && abs(final.frame.cx - VAT_X_PX) <= VAT_TOL_PX && abs(final.frame.cy - s_y_target) <= VAT_TOL_PX;
    for (int i = 0; i < 4; ++i) if (ctrl_enc_total(i) != s_counts[i]) valid = 0;
    if (valid) vat_finish_alignment(now);
    __set_PRIMASK(pm); return valid;
}
void vision_align_test_poll(void)
{
    uint32_t now = HAL_GetTick();
    VatSample sample;
    ProtoStats stats;
    float heading_error = 0.0f;
    int matching, fresh, is_new, changed;
    if (!vat_running()) return;
    if (run_aborted() || !imu_ok() || !vat_error(&heading_error)) {
        vat_stop(run_aborted() ? "ABORT" : "IMU", VAT_STOPPED); return;
    }
    if (proto_scene_status() < 0) { vat_stop("NACK", VAT_STOPPED); return; }
    if (s_vat.state == VAT_TURN_ACTIVE) return; /* Mode22 exclusively owns motors. */
    if ((uint32_t)(now - s_last_poll) < VAT_POLL_MS) return;
    s_last_poll = now;
    s_vat.heading_error_deg = heading_error;
    changed = vat_counts_changed(now);
    if (s_vat.state == VAT_HOLD_BALL || s_vat.state == VAT_HOLD_BUCKET) {
        vat_brake();
        if (changed) s_hold_from = now;
        if ((uint32_t)(now - s_hold_from) < VAT_PLACEHOLDER_MS) return;
        if (s_vat.state == VAT_HOLD_BUCKET) { vat_stop("ALIGNED_BALL_BUCKET", VAT_DONE); return; }
        if (!vat_request(PROTO_TASK_BUCKET, 0u, now)) { vat_stop("REQUEST", VAT_STOPPED); return; }
        s_vat.state = VAT_TURN_REQUEST; s_vat.reason = "TURN180_REQUEST"; return;
    }
    if (s_vat.state == VAT_TURN_REQUEST) { vat_brake(); return; }
    if (s_vat.state == VAT_BRAKE) {
        vat_brake();
        if ((uint32_t)(now - s_still_from) < T_DIST_STILL_MS) return;
        s_vat.state = VAT_YAW_FIX; s_fix_from = now; s_stable = 0u;
    }
    if (s_vat.state == VAT_YAW_FIX) {
        float error = s_vat.heading_error_deg, yaw = imu_heading_deg();
        if (!vat_finite(yaw)) { vat_stop("IMU", VAT_STOPPED); return; }
        if ((uint32_t)(now - s_fix_from) >= T_DIST_ALIGN_MAX_MS) { vat_stop("YAW_TIMEOUT", VAT_STOPPED); return; }
        if (fabsf(error) > T_DIST_ALIGN_TOL_DEG) {
            float w = error * T_DIST_ALIGN_KP;
            if (fabsf(w) > T_DIST_ALIGN_MAX_W) w = copysignf(T_DIST_ALIGN_MAX_W, w);
            if (fabsf(w) < T_DIST_ALIGN_MIN_W) w = copysignf(T_DIST_ALIGN_MIN_W, w);
            s_stable = 0u; s_still_from = now; vat_drive(0.0f, 0.0f, w); return;
        }
        vat_brake();
        if (!s_stable || changed || fabsf(yaw - s_fix_last_yaw) > T_DIST_ALIGN_STILL_DEG) {
            s_stable = 1u; s_fix_stable_from = now; s_fix_last_yaw = yaw;
        }
        if ((uint32_t)(now - s_fix_stable_from) < T_DIST_ALIGN_STABLE_MS
            || (uint32_t)(now - s_still_from) < T_DIST_STILL_MS) return;
        uint32_t pm = __get_PRIMASK();
        __disable_irq();
        proto_stats_get(&stats); s_recheck_packet = stats.obj;
        memset((void *)&s_sample, 0, sizeof s_sample);
        s_vat.latest = s_vat.good = 0u;
        if (!s_need_new) {
            /* Initial correction must discard its own cached image too.
             * Keep ALIGN so the permitted no-first-frame search still works. */
            s_session_packet = s_used_packet = stats.obj;
            s_have_seq = 0u; s_rejected_packet = 0u;
        }
        s_vat.state = s_need_new ? VAT_RECHECK : VAT_ALIGN;
        s_vat.reason = s_need_new ? "NEW_IMAGE_AFTER_YAW" : "ALIGN";
        __set_PRIMASK(pm);
        return; /* A frame captured before heading settled never proves arrival. */
    }
    matching = vat_snapshot(&sample, &stats);
    fresh = matching && sample.packet != s_rejected_packet
        && (uint32_t)(now - sample.at) <= VAT_FRESH_MS && proto_scene_status() == 1;
    s_vat.latest = (uint8_t)fresh; s_vat.age_ms = sample.seen ? now - sample.at : UINT32_MAX;
    if (fresh && !vat_geometry(&sample.frame)) { vat_stop("IMAGE_GEOMETRY", VAT_STOPPED); return; }
    if (!fresh) {
        s_vat.good = 0u;
        if (s_vat.state == VAT_ALIGN && !s_seen_target && s_initial_search && vat_ack_ready()) {
            s_vat.axis = 1u; s_direction = 1; s_still_from = now;
            vat_drive(VAT_SPEED_MMS, 0.0f, vat_motion_w()); return;
        }
        if (s_vat.axis) { s_initial_search = 0u; vat_begin_brake(now, 1); }
        else vat_brake();
        return;
    }
    s_seen_target = 1u;
    is_new = sample.packet != s_used_packet;
    if (is_new) {
        uint16_t delta = (uint16_t)(sample.frame.sequence - s_last_seq);
        if (s_have_seq && (delta == 0u || delta >= 0x8000u)) {
            s_rejected_packet = s_used_packet = sample.packet; s_vat.good = s_vat.latest = 0u;
            if (s_vat.axis) vat_begin_brake(now, 1); else vat_brake();
            return;
        }
        if (s_have_seq && (delta != 1u || sample.packet - s_used_packet != 1u)) s_vat.good = 0u;
        s_have_seq = 1u; s_last_seq = sample.frame.sequence; s_used_packet = sample.packet; s_rejected_packet = 0u;
    }
    s_vat.cx = sample.frame.cx; s_vat.cy = sample.frame.cy; s_vat.sequence = sample.frame.sequence;
    s_vat.img_w = sample.frame.img_w; s_vat.img_h = sample.frame.img_h;
    if (s_vat.state == VAT_RECHECK) {
        vat_brake();
        if (!is_new || sample.packet == s_recheck_packet) { s_vat.latest = 0u; return; }
        s_vat.state = VAT_ALIGN; s_vat.reason = "ALIGN";
    }
    int ex = sample.frame.cx - VAT_X_PX, ey = sample.frame.cy - s_y_target;
    uint8_t axis = abs(ex) > VAT_TOL_PX ? 1u : abs(ey) > VAT_TOL_PX ? 2u : 0u;
    int direction = axis == 1u ? (ex > 0 ? 1 : -1) : axis == 2u ? (ey > 0 ? 1 : -1) : 0;
    if (s_vat.axis && (axis != s_vat.axis || direction != s_direction)) {
        s_initial_search = 0u; vat_begin_brake(now, 1); return;
    }
    if (fabsf(s_vat.heading_error_deg) > T_DIST_ALIGN_TOL_DEG && !axis) {
        vat_begin_brake(now, 1); return;
    }
    if (axis) {
        s_vat.good = 0u;
        if (!s_vat.axis && !is_new) { vat_brake(); return; }
        s_vat.axis = axis; s_direction = direction; s_still_from = now;
        float speed = VAT_SPEED_MMS * (float)direction;
        vat_drive_target(sample.packet, now, axis == 1u ? speed : 0.0f,
                         axis == 2u ? speed : 0.0f); return;
    }
    vat_brake();
    if (is_new && s_vat.good < VAT_GOOD_FRAMES) s_vat.good++;
    if (is_new && s_vat.good >= VAT_GOOD_FRAMES && (uint32_t)(now - s_still_from) >= T_DIST_STILL_MS)
        (void)vat_final_check(sample.packet, now);
}
int vision_align_test_take_turn_request(void)
{
    float error = 0.0f;
    if (s_vat.state != VAT_TURN_REQUEST) return 0;
    if (run_aborted() || !imu_ok() || !vat_error(&error) || proto_scene_status() < 0) {
        vat_stop("TURN180_PREP", VAT_STOPPED); return 0;
    }
    s_vat.state = VAT_TURN_ACTIVE; s_vat.reason = "TURN180_ACTIVE"; return 1;
}
void vision_align_test_notify_turn_result(int success)
{
    float error = 0.0f;
    if (s_vat.state != VAT_TURN_ACTIVE) return;
    if (!success || !imu_ok() || !vat_error(&error) || run_aborted() || proto_scene_status() < 0) {
        vat_stop("TURN180_FAILED", VAT_STOPPED); return;
    }
    s_vat.heading_target_deg += 180.0f;
    if (!vat_error(&error)
        || fabsf(error) > T_TURN_LIMIT_DEG) { vat_stop("TURN180_RESIDUAL", VAT_STOPPED); return; }
    if (!vat_request(PROTO_TASK_BUCKET, 0u, HAL_GetTick())) { vat_stop("REQUEST", VAT_STOPPED); return; }
    s_initial_search = 0u; s_need_new = 1u;
}
void vision_align_test_status(VisionAlignTestStatus *out)
{
    VatSample current;
    ProtoStats stats;
    uint32_t now;
    if (!out) return;
    *out = s_vat;
    now = HAL_GetTick();
    int matching = vat_snapshot(&current, &stats);
    out->age_ms = current.seen ? now - current.at : UINT32_MAX;
    /* cx/cy remain historical values in logs; latest never blesses stale data
     * during yaw correction, holds, cancellation or a rejected sequence. */
    if (!matching || current.packet != s_used_packet || current.packet == s_rejected_packet || out->age_ms > VAT_FRESH_MS
        || proto_scene_status() != 1) out->latest = 0u;
}
