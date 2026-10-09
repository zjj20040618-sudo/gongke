#include "vision_align_test.h"
#include "main.h"
#include "motion.h"
#include "control.h"
#include "imu.h"
#include "steps.h"
#include "board_pins.h"
#include "test_config.h"
#include "yaw_progress_boost.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    ProtoFrame frame;
    uint32_t packet, at;
    uint8_t seen;
} VatSample;
static volatile VatSample s_sample;
static volatile VatSample s_route_search_sample; /* First fine-gate frame survives a later empty packet. */
static volatile VatSample s_route_bucket_sample; /* First |cx-goal|<30 survives empty/replay; never arrival proof. */
static volatile VatSample s_route_bucket_loss_sample; /* Last NEW bucket01 survives empty/replay packets. */
static volatile uint8_t s_route_bucket_geometry_bad;
static uint8_t s_route_bucket_loss_braking;
static uint32_t s_route_bucket_session_packet;
/* Independent receive evidence: a later empty packet cannot erase first/last
 * selected hostage. Only the task owner poll may turn this into an action. */
static volatile VatSample s_route_hostage_sample;
static volatile uint8_t s_route_hostage_geometry_bad;
static uint8_t s_route_hostage_loss_braking;
static uint32_t s_route_hostage_session_packet;
static VisionAlignTestStatus s_vat;
static int32_t s_counts[4];
static int32_t s_qr[3];
static uint32_t s_last_poll, s_still_from;
static uint32_t s_hold_from, s_used_packet, s_recheck_packet, s_session_packet, s_rejected_packet;
static uint16_t s_last_seq;
static uint16_t s_width, s_height;
static int s_y_target, s_direction;
static uint8_t s_have_seq, s_seen_target, s_need_new, s_initial_search;
static uint32_t s_yaw_from, s_yaw_stable_from;
static float s_yaw_stable_heading;
static uint8_t s_yaw_stable;
static uint32_t s_step_from, s_step_image_at;
static float s_step_odo0;
static uint8_t s_step_has_target;
static volatile uint8_t s_step_saw_target;
static uint8_t s_route_owned, s_route_action_taken;
static uint8_t s_route_search_started;
static uint32_t s_route_search_from;
static float s_route_search_kp, s_route_search_ff;
static float s_route_search_speed;
static uint32_t s_route_yaw_settle_ms;
static YawProgressBoost s_vat_yaw_progress;
static uint16_t s_route_coarse_seq;
static uint32_t s_route_coarse_packet;
static uint8_t s_route_coarse_have_seq;
static volatile uint8_t s_route_bucket_eligible, s_route_bucket_seen;
static uint8_t s_route_bucket_missing_active;
static uint32_t s_route_bucket_missing_from;
static int vat_ack_ready(void);
static int vat_hostage_rank_capture(void);
static void vat_ball_rank_capture(void);
static void vat_ball_begin_bucket_turn(void);
static int vat_route_hostage_loss_poll(uint32_t now);
static int vat_route_bucket_loss_poll(uint32_t now);

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
    yaw_progress_reset(&s_vat_yaw_progress);
    vat_brake(); bp_laser_set(0); proto_receive_end(); vat_sample_clear();
    s_vat.state = state; s_vat.reason = reason; s_vat.latest = 0u;
    s_vat.good = state == VAT_DONE && !s_vat.hostage_fallback && !s_vat.bucket_fallback ? VAT_GOOD_FRAMES : 0u;
    s_vat.axis = 0u; s_direction = 0;
    s_route_owned = s_route_action_taken = 0u;
    s_route_search_started = 0u; s_route_search_from = 0u;
    s_route_search_kp = s_route_search_ff = 0.0f;
    s_route_coarse_have_seq = 0u;
    s_route_bucket_eligible = s_route_bucket_seen = s_route_bucket_missing_active = 0u;
    memset((void *)&s_route_bucket_sample, 0, sizeof s_route_bucket_sample);
    memset((void *)&s_route_bucket_loss_sample, 0, sizeof s_route_bucket_loss_sample);
    s_route_bucket_geometry_bad = s_route_bucket_loss_braking = 0u;
    memset((void *)&s_route_search_sample, 0, sizeof s_route_search_sample);
    memset((void *)&s_route_hostage_sample, 0, sizeof s_route_hostage_sample);
    s_route_hostage_geometry_bad = s_route_hostage_loss_braking = 0u;
}
void vision_align_test_init(void)
{
    yaw_progress_reset(&s_vat_yaw_progress);
    memset(&s_vat, 0, sizeof s_vat); vat_sample_clear();
    s_route_owned = s_route_action_taken = 0u;
    s_route_search_started = 0u; s_route_search_from = 0u;
    s_route_search_kp = s_route_search_ff = 0.0f;
    s_route_coarse_have_seq = 0u;
    s_route_bucket_eligible = s_route_bucket_seen = s_route_bucket_missing_active = 0u;
    memset((void *)&s_route_bucket_sample, 0, sizeof s_route_bucket_sample);
    memset((void *)&s_route_bucket_loss_sample, 0, sizeof s_route_bucket_loss_sample);
    s_route_bucket_geometry_bad = s_route_bucket_loss_braking = 0u;
    memset((void *)&s_route_search_sample, 0, sizeof s_route_search_sample);
    memset((void *)&s_route_hostage_sample, 0, sizeof s_route_hostage_sample);
    s_route_hostage_geometry_bad = s_route_hostage_loss_braking = 0u;
    s_vat.hostage_age_ms = UINT32_MAX;
    s_vat.bucket_age_ms = UINT32_MAX;
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
static float vat_heading_error(float target, float yaw)
{
    float e = target - yaw;
    if (!vat_finite(e)) return e;
    e = fmodf(e, 360.0f);
    if (e > 180.0f) e -= 360.0f;
    if (e < -180.0f) e += 360.0f;
    return e;
}
static int vat_heading_update(void)
{
    float yaw = imu_heading_deg();
    if (!imu_ok() || !vat_finite(yaw) || !vat_finite(s_vat.yaw_target)) return 0;
    s_vat.yaw_error = vat_heading_error(s_vat.yaw_target, yaw);
    return vat_finite(s_vat.yaw_error);
}
static void vat_begin_brake(uint32_t now, int need_new)
{
    yaw_progress_reset(&s_vat_yaw_progress);
    vat_brake(); vat_count_reset(now);
    s_vat.state = VAT_BRAKE; s_vat.reason = "BRAKE_STILL";
    s_vat.axis = s_vat.good = s_vat.latest = 0u; s_direction = 0;
    s_need_new = (uint8_t)need_new;
}
static int vat_request(ProtoTask task, uint8_t digit, uint32_t now)
{
    yaw_progress_reset(&s_vat_yaw_progress);
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
    s_vat.target_rank = 0u; s_vat.rank_sequence = 0u;
    s_vat.alignment_confirmed = 0u;
    s_vat.hostage_seen = s_vat.hostage_fallback = 0u;
    s_vat.hostage_age_ms = UINT32_MAX;
    s_vat.bucket_seen = s_vat.bucket_fallback = 0u;
    s_vat.bucket_age_ms = UINT32_MAX;
    s_vat.x_goal = !s_route_owned ? VAT_X_PX :
        task == PROTO_TASK_BUCKET ? VAT_ROUTE_BUCKET_X_PX :
        task == PROTO_TASK_HOSTAGE ? VAT_ROUTE_HOSTAGE_X_PX : VAT_ROUTE_BALL_X_PX;
    s_vat.cls = cls; s_vat.label = label; s_vat.request = diag.request;
    s_session_packet = s_used_packet = s_recheck_packet = stats.obj;
    s_route_hostage_session_packet = stats.obj;
    s_route_bucket_session_packet = stats.obj;
    s_have_seq = s_seen_target = 0u; s_width = s_height = 0u; s_rejected_packet = 0u;
    s_vat.yaw_dirty = s_vat.yaw_ever = 0u;
    s_step_saw_target = 0u;
    s_route_search_started = 0u; s_route_search_from = 0u;
    s_route_coarse_have_seq = 0u;
    s_route_bucket_eligible = s_route_bucket_seen = s_route_bucket_missing_active = 0u;
    memset((void *)&s_route_bucket_sample, 0, sizeof s_route_bucket_sample);
    memset((void *)&s_route_bucket_loss_sample, 0, sizeof s_route_bucket_loss_sample);
    s_route_bucket_geometry_bad = s_route_bucket_loss_braking = 0u;
    memset((void *)&s_route_search_sample, 0, sizeof s_route_search_sample);
    memset((void *)&s_route_hostage_sample, 0, sizeof s_route_hostage_sample);
    s_route_hostage_geometry_bad = s_route_hostage_loss_braking = 0u;
    s_vat.latest = s_vat.good = 0u; s_vat.age_ms = UINT32_MAX;
    __set_PRIMASK(pm);
    s_y_target = task == PROTO_TASK_BALL ? VAT_BALL_Y_PX :
                 task == PROTO_TASK_BUCKET ? VAT_BUCKET_Y_PX : VAT_HOSTAGE_Y_PX;
    s_initial_search = task != PROTO_TASK_BUCKET;
    vat_begin_brake(now, 0);
    return 1;
}
static int vat_start(uint8_t mode, const int32_t qr[3], int route)
{
    int32_t current[3];
    static const ProtoTask qr_tasks[3] = { PROTO_TASK_BALL, PROTO_TASK_TARGET, PROTO_TASK_HOSTAGE };
    float heading = imu_heading_deg();
    float search_kp = route ? step_heading_kp_deg() : 0.0f;
    if (vat_running() || mode < 38u || mode > 41u || run_aborted()) return 0;
    if (route && mode != 40u && mode != 41u) return 0;
    if (route && (!vat_finite(search_kp) || search_kp < 0.0f || search_kp > 5.0f)) return 0;
    if (!imu_ok() || !vat_finite(heading)) return 0;
    /* Only the standalone black bucket has no QR-selected color/shape.
     * Independent trials require parser-current QR;31 instead supplies its
     * owner-locked legal tuple after closing the R1 QR window. */
    if (mode != 39u) {
        if (!qr || (!route && !proto_qr_get(current))) return 0;
        for (unsigned i = 0; i < 3u; ++i)
            if (qr[i] < 1 || qr[i] > 3 || (!route && qr[i] != current[i])
                || !proto_target_filter(qr_tasks[i], (uint8_t)qr[i], NULL, NULL)) return 0;
    }
    memset(&s_vat, 0, sizeof s_vat); s_vat.mode = mode;
    s_route_owned = (uint8_t)(route != 0); s_route_action_taken = 0u;
    s_route_search_kp = search_kp; s_route_search_ff = 0.0f;
    s_route_search_speed = VAT_ROUTE_SEARCH_SPEED_MMS;
    s_route_yaw_settle_ms = T_DIST_ALIGN_STABLE_MS;
    yaw_progress_reset(&s_vat_yaw_progress);
    s_vat.yaw_target = heading; /* Capture once; never zero a twisted heading. */
    memset(s_qr, 0, sizeof s_qr);
    if (mode != 39u) memcpy(s_qr, qr, sizeof s_qr);
    s_last_poll = HAL_GetTick();
    bp_laser_set(0); vat_brake();
    ProtoTask task = mode == 39u ? PROTO_TASK_BUCKET : mode == 40u ? PROTO_TASK_HOSTAGE : PROTO_TASK_BALL;
    uint8_t digit = task == PROTO_TASK_BUCKET ? 0u : (uint8_t)s_qr[task == PROTO_TASK_HOSTAGE ? 2u : 0u];
    if (!vat_request(task, digit, s_last_poll)) { vat_stop("REQUEST", VAT_STOPPED); return 0; }
    return 1;
}
int vision_align_test_start(uint8_t mode, const int32_t qr[3])
{
    return vat_start(mode, qr, 0);
}
int vision_align_test_start_route(uint8_t mode, const int32_t qr[3])
{
    return vat_start(mode, qr, 1);
}
static int vat_take_route_action_locked(void)
{
    int hostage = s_vat.mode == 40u && s_vat.task == PROTO_TASK_HOSTAGE &&
        s_vat.state == VAT_WAIT_HOSTAGE_ACTION;
    int bucket_loss = s_vat.mode == 41u && s_vat.task == PROTO_TASK_BUCKET &&
        s_vat.state == VAT_WAIT_BUCKET_ACTION && s_vat.bucket_fallback;
    if (!s_route_owned || s_route_action_taken ||
        (!hostage && (s_vat.mode != 41u ||
        (s_vat.state != VAT_WAIT_BALL_ACTION && s_vat.state != VAT_WAIT_BUCKET_ACTION))))
        return VAT_ROUTE_ACTION_NONE;
    if (run_aborted() || !imu_ok() || !vat_finite(imu_heading_deg())) {
        vat_stop("LIFT_ERROR", VAT_STOPPED); return VAT_ROUTE_ACTION_NONE;
    }
    if ((hostage || bucket_loss || s_vat.task == PROTO_TASK_BALL) && (!vat_ack_ready() || proto_scene_status() < 0)) {
        vat_stop("NACK", VAT_STOPPED); return VAT_ROUTE_ACTION_NONE;
    }
    if (hostage && s_vat.hostage_fallback) {
        (void)vat_route_hostage_loss_poll(HAL_GetTick());
        if (s_vat.state != VAT_WAIT_HOSTAGE_ACTION) return VAT_ROUTE_ACTION_NONE;
    }
    if (bucket_loss) {
        (void)vat_route_bucket_loss_poll(HAL_GetTick());
        if (s_vat.state != VAT_WAIT_BUCKET_ACTION || !s_vat.bucket_fallback)
            return VAT_ROUTE_ACTION_NONE;
    }
    vat_brake();
    if (hostage || bucket_loss) {
        uint32_t now = HAL_GetTick();
        (void)vat_counts_changed(now);
        if ((uint32_t)(now - s_still_from) < T_DIST_STILL_MS) return VAT_ROUTE_ACTION_NONE;
    }
    s_route_action_taken = 1u;
    vat_ball_rank_capture();
    if (hostage) return VAT_ROUTE_ACTION_HOSTAGE;
    return s_vat.state == VAT_WAIT_BALL_ACTION ? VAT_ROUTE_ACTION_BALL : VAT_ROUTE_ACTION_BUCKET;
}
int vision_align_test_take_route_action(void)
{
    uint32_t pm = __get_PRIMASK();
    int action;
    __disable_irq(); action = vat_take_route_action_locked(); __set_PRIMASK(pm);
    return action;
}
static void vat_notify_route_action_result_locked(int success)
{
    int hostage = s_vat.mode == 40u && s_vat.task == PROTO_TASK_HOSTAGE &&
        s_vat.state == VAT_WAIT_HOSTAGE_ACTION;
    if (!s_route_owned || !s_route_action_taken ||
        (!hostage && (s_vat.mode != 41u ||
        (s_vat.state != VAT_WAIT_BALL_ACTION && s_vat.state != VAT_WAIT_BUCKET_ACTION)))) return;
    if (!success || run_aborted() || !imu_ok() || !vat_finite(imu_heading_deg())) {
        vat_stop("LIFT_ERROR", VAT_STOPPED); return;
    }
    if ((hostage || s_vat.bucket_fallback || s_vat.task == PROTO_TASK_BALL) && (!vat_ack_ready() || proto_scene_status() < 0)) {
        vat_stop("NACK", VAT_STOPPED); return;
    }
    s_route_action_taken = 0u;
    if (hostage) {
        vat_brake();
        if (vat_hostage_rank_capture()) {
            vat_stop(s_vat.hostage_fallback ? "HOSTAGE_LOST2S_GRABBED" : "HOSTAGE_ALIGNED_GRABBED", VAT_DONE);
        } else {
            s_vat.state = VAT_WAIT_HOSTAGE_RANK; s_vat.reason = "WAIT_HOSTAGE_RANK";
        }
        return;
    }
    if (s_vat.state == VAT_WAIT_BUCKET_ACTION) {
        vat_stop(s_vat.bucket_fallback ? "BUCKET_LOST2S_RELEASED" : "ALIGNED_BALL_BUCKET", VAT_DONE); return;
    }
    vat_ball_rank_capture();
    if (!s_vat.ball_rank) {
        vat_brake(); s_vat.state = VAT_WAIT_BALL_RANK; s_vat.reason = "WAIT_BALL_RANK";
        return; /* Camera still reports54; do not guess a direction from QR color. */
    }
    vat_ball_begin_bucket_turn();
}
void vision_align_test_notify_route_action_result(int success)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq(); vat_notify_route_action_result_locked(success); __set_PRIMASK(pm);
}
static int vat_route_search_enabled(void)
{
    return s_route_owned && (s_vat.task == PROTO_TASK_BALL || s_vat.task == PROTO_TASK_HOSTAGE);
}
int vision_align_test_route_search_kp_set(float kp)
{
    if (!vat_finite(kp) || kp < 0.0f || kp > 5.0f || !vat_route_search_enabled()
        || s_vat.state != VAT_BRAKE || !s_initial_search || s_seen_target || s_route_search_started)
        return 0;
    s_route_search_kp = kp; return 1;
}
int vision_align_test_route_search_ff_set(float ratio)
{
    if (!vat_finite(ratio) || ratio < 0.0f || ratio > 0.1f || !vat_route_search_enabled()
        || s_vat.state != VAT_BRAKE || !s_initial_search || s_seen_target || s_route_search_started)
        return 0;
    s_route_search_ff = ratio; return 1;
}
int vision_align_test_route_search_speed_set(float speed_mms)
{
    if (!vat_finite(speed_mms) || speed_mms < 1.0f || speed_mms > 600.0f ||
        !vat_route_search_enabled() || s_vat.state != VAT_BRAKE ||
        !s_initial_search || s_seen_target || s_route_search_started)
        return 0;
    s_route_search_speed = speed_mms; return 1;
}
int vision_align_test_route_yaw_settle_ms_set(uint32_t ms)
{
    if ((ms != 400u && ms != T_DIST_ALIGN_STABLE_MS) ||
        !vat_route_search_enabled() || s_vat.state != VAT_BRAKE ||
        !s_initial_search || s_seen_target || s_route_search_started) return 0;
    s_route_yaw_settle_ms = ms; return 1;
}
static int vat_geometry_fields_valid(const ProtoFrame *f)
{
    return f->img_w && f->img_h && s_vat.x_goal + VAT_TOL_PX < f->img_w
#if VAT_Y_ALIGN_ENABLE
        && s_y_target + VAT_TOL_PX < f->img_h
#endif
        && f->cx >= 0 && f->cx < f->img_w
        && f->cy >= 0 && f->cy < f->img_h
        && (!s_width || (s_width == f->img_w && s_height == f->img_h));
}
/* Called only under the same successful current request/ACK gate as s_sample.
 * Preserve the last NEW selected01 independently of empty packets, rejected
 * classes, rank54 and the alignment engine's brake/recheck slot clearing. */
static void vat_route_hostage_observe(const ProtoFrame *frame, uint32_t packet)
{
    uint16_t delta;
    uint32_t packet_delta;
    if (!s_route_owned || s_vat.mode != 40u || s_vat.task != PROTO_TASK_HOSTAGE ||
        (s_vat.state == VAT_WAIT_HOSTAGE_ACTION && (!s_vat.hostage_fallback || s_route_action_taken)) ||
        s_vat.state == VAT_WAIT_HOSTAGE_RANK ||
        frame->cls != s_vat.cls || (s_vat.label >= 0 && frame->label != s_vat.label)) return;
    packet_delta = packet - (s_route_hostage_sample.seen ?
        s_route_hostage_sample.packet : s_route_hostage_session_packet);
    if (!packet_delta || packet_delta >= 0x80000000u) return;
    delta = (uint16_t)(frame->sequence - s_route_hostage_sample.frame.sequence);
    if (s_route_hostage_sample.seen && (delta == 0u || delta >= 0x8000u)) return;
    if (!vat_geometry_fields_valid(frame) || frame->w <= 0 || frame->h <= 0 ||
        frame->w > frame->img_w || frame->h > frame->img_h || frame->conf < 0 || frame->conf > 100 ||
        (s_route_hostage_sample.seen && (frame->img_w != s_route_hostage_sample.frame.img_w ||
         frame->img_h != s_route_hostage_sample.frame.img_h))) {
        s_route_hostage_geometry_bad = 1u; return;
    }
    s_route_hostage_sample.frame = *frame;
    s_route_hostage_sample.packet = packet; s_route_hostage_sample.at = HAL_GetTick();
    s_route_hostage_sample.seen = 1u;
}
/* Caller has checked the current task ACK/request. Only record evidence here;
 * no ISR wheel command, heading correction or arrival decision is permitted. */
static void vat_route_search_observe(const ProtoFrame *frame, uint32_t packet)
{
    uint16_t delta = (uint16_t)(frame->sequence -
        (s_route_coarse_have_seq ? s_route_coarse_seq : s_last_seq));
    if (!vat_route_search_enabled() || !s_initial_search || s_seen_target
        || s_route_search_sample.seen || packet == s_session_packet
        || (s_vat.state != VAT_BRAKE && s_vat.state != VAT_ALIGN && s_vat.state != VAT_ROUTE_SEARCH)
        || frame->cls != s_vat.cls || (s_vat.label >= 0 && frame->label != s_vat.label)
        || !vat_geometry_fields_valid(frame)
        || (s_route_coarse_have_seq && packet == s_route_coarse_packet)
        || ((s_route_coarse_have_seq || s_have_seq) && (delta == 0u || delta >= 0x8000u))) return;
    /* A far target is still fresh evidence, but it must not end approach.
     * Retain its sequence so an old/duplicate crossing cannot latch fine. */
    s_route_coarse_have_seq = 1u; s_route_coarse_seq = frame->sequence;
    s_route_coarse_packet = packet;
    if (abs(frame->cx - s_vat.x_goal) >= VAT_ROUTE_FINE_ERROR_PX) return;
    s_route_search_sample.frame = *frame;
    s_route_search_sample.packet = packet; s_route_search_sample.at = HAL_GetTick();
    s_route_search_sample.seen = 1u;
}
static void vat_route_bucket_observe(const ProtoFrame *frame, uint32_t packet)
{
    uint16_t delta = (uint16_t)(frame->sequence -
        (s_route_coarse_have_seq ? s_route_coarse_seq : s_last_seq));
    if (!s_route_bucket_eligible || s_route_bucket_seen || s_route_bucket_sample.seen
        || packet == s_session_packet
        || (s_vat.state != VAT_BRAKE && s_vat.state != VAT_ALIGN &&
            s_vat.state != VAT_RECHECK && s_vat.state != VAT_ROUTE_BUCKET_SEARCH)
        || frame->cls != s_vat.cls || (s_vat.label >= 0 && frame->label != s_vat.label)
        || !vat_geometry_fields_valid(frame)
        || (s_route_coarse_have_seq && packet == s_route_coarse_packet)
        || ((s_route_coarse_have_seq || s_have_seq) && (delta == 0u || delta >= 0x8000u))) return;
    s_route_coarse_have_seq = 1u; s_route_coarse_seq = frame->sequence;
    s_route_coarse_packet = packet;
    if (abs(frame->cx - s_vat.x_goal) >= VAT_ROUTE_BUCKET_FINE_ERROR_PX) return;
    s_route_bucket_sample.frame = *frame;
    s_route_bucket_sample.packet = packet; s_route_bucket_sample.at = HAL_GetTick();
    s_route_bucket_sample.seen = 1u;
}
static void vat_route_bucket_loss_observe(const ProtoFrame *frame, uint32_t packet)
{
    uint16_t delta;
    uint32_t packet_delta;
    if (!s_route_bucket_eligible || !s_route_owned || s_vat.mode != 41u ||
        s_vat.task != PROTO_TASK_BUCKET ||
        (s_vat.state == VAT_WAIT_BUCKET_ACTION && (!s_vat.bucket_fallback || s_route_action_taken)) ||
        frame->cls != s_vat.cls || (s_vat.label >= 0 && frame->label != s_vat.label)) return;
    packet_delta = packet - (s_route_bucket_loss_sample.seen ?
        s_route_bucket_loss_sample.packet : s_route_bucket_session_packet);
    if (!packet_delta || packet_delta >= 0x80000000u) return;
    delta = (uint16_t)(frame->sequence - s_route_bucket_loss_sample.frame.sequence);
    if (s_route_bucket_loss_sample.seen && (delta == 0u || delta >= 0x8000u)) return;
    if (!vat_geometry_fields_valid(frame) || frame->w <= 0 || frame->h <= 0 ||
        frame->w > frame->img_w || frame->h > frame->img_h || frame->conf < 0 || frame->conf > 100 ||
        (s_route_bucket_loss_sample.seen && (frame->img_w != s_route_bucket_loss_sample.frame.img_w ||
         frame->img_h != s_route_bucket_loss_sample.frame.img_h))) {
        s_route_bucket_geometry_bad = 1u; return;
    }
    s_route_bucket_loss_sample.frame = *frame;
    s_route_bucket_loss_sample.packet = packet; s_route_bucket_loss_sample.at = HAL_GetTick();
    s_route_bucket_loss_sample.seen = 1u;
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
    vat_route_search_observe(frame, stats.obj);
    vat_route_bucket_observe(frame, stats.obj);
    vat_route_bucket_loss_observe(frame, stats.obj);
    vat_route_hostage_observe(frame, stats.obj);
    /* Keep the fact that a blind step saw its selected target even if an empty
     * packet arrives before the next poll. No ISR motor/geometry decisions. */
    if (s_vat.state == VAT_STEP_MOVE && s_sample.seen) s_step_saw_target = 1u;
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
static int vat_hostage_rank_capture(void)
{
    ProtoTargetRank rank;
    if (!vat_ack_ready() || !proto_target_rank_get(&rank) || rank.request != s_vat.request ||
        rank.task != s_vat.task || rank.digit != s_vat.digit || rank.rank < 1u || rank.rank > 3u) return 0;
    s_vat.target_rank = rank.rank; s_vat.rank_sequence = rank.sequence; return 1;
}
static void vat_ball_rank_capture(void)
{
    if (s_route_owned && s_vat.task == PROTO_TASK_BALL && vat_hostage_rank_capture())
        s_vat.ball_rank = s_vat.target_rank;
}
static void vat_ball_begin_bucket_turn(void)
{
    if (!vat_request(PROTO_TASK_BUCKET, 0u, HAL_GetTick())) {
        vat_stop("REQUEST", VAT_STOPPED); return;
    }
    s_vat.state = VAT_TURN_REQUEST; s_vat.reason = "TURN180_REQUEST";
}
/* This intentional seen-then-lost grab must never fabricate pixel alignment.
 * Once its2s trigger brakes, no continuous blind search may restart. A new
 * selected01 before the owner takes the pending action cancels the dropout and
 * returns through stopped/new-image recheck with the refreshed timer. */
static int vat_route_hostage_loss_poll(uint32_t now)
{
    uint32_t pm, age;
    if (!s_route_owned || s_vat.mode != 40u || s_vat.task != PROTO_TASK_HOSTAGE ||
        (s_vat.state == VAT_WAIT_HOSTAGE_ACTION && (!s_vat.hostage_fallback || s_route_action_taken)) ||
        s_vat.state == VAT_WAIT_HOSTAGE_RANK) return 0;
    pm = __get_PRIMASK(); __disable_irq();
    if (s_route_hostage_geometry_bad) {
        vat_stop("IMAGE_GEOMETRY", VAT_STOPPED); __set_PRIMASK(pm); return 1;
    }
    if (!s_route_hostage_sample.seen) { __set_PRIMASK(pm); return 0; }
    s_vat.hostage_seen = 1u;
    now = HAL_GetTick(); /* Sample may have arrived after this poll captured its earlier tick. */
    age = now - s_route_hostage_sample.at; s_vat.hostage_age_ms = age;
    if (!vat_ack_ready()) {
        vat_brake(); s_vat.reason = "HOSTAGE_WAIT_ACK"; __set_PRIMASK(pm); return 1;
    }
    if (age < VAT_ROUTE_HOSTAGE_LOST_MS) {
        if (!s_route_hostage_loss_braking) { __set_PRIMASK(pm); return 0; }
        s_route_hostage_loss_braking = 0u; s_vat.hostage_fallback = 0u;
        vat_begin_brake(now, 1); s_vat.reason = "HOSTAGE_NEW_TARGET_BRAKE";
        __set_PRIMASK(pm); return 1;
    }
    if (!s_route_hostage_loss_braking) {
        s_route_hostage_loss_braking = 1u;
        s_initial_search = 0u; s_seen_target = 1u;
        memset((void *)&s_route_search_sample, 0, sizeof s_route_search_sample);
        vat_begin_brake(now, 1); s_vat.reason = "HOSTAGE_LOST2S_BRAKE";
        __set_PRIMASK(pm); return 1;
    }
    vat_brake(); (void)vat_counts_changed(now);
    s_vat.good = s_vat.latest = s_vat.axis = 0u;
    s_vat.reason = "HOSTAGE_LOST2S_BRAKE";
    if ((uint32_t)(now - s_still_from) >= T_DIST_STILL_MS) {
        s_vat.hostage_fallback = 1u; s_vat.alignment_confirmed = 0u;
        s_route_action_taken = 0u;
        s_vat.state = VAT_WAIT_HOSTAGE_ACTION; s_vat.reason = "HOSTAGE_LOST2S_GRAB";
        (void)vat_hostage_rank_capture();
    }
    __set_PRIMASK(pm); return 1;
}
/* Same explicit seen-then-lost trial as hostage, but expose the real BUCKET
 * action; never manufacture five good frames or pixel alignment. Only the
 * route owner may begin release after this pending action is taken. */
static int vat_route_bucket_loss_poll(uint32_t now)
{
    uint32_t pm, age;
    if (!s_route_bucket_eligible || !s_route_bucket_seen || !s_route_owned || s_vat.mode != 41u ||
        s_vat.task != PROTO_TASK_BUCKET ||
        (s_vat.state == VAT_WAIT_BUCKET_ACTION && (!s_vat.bucket_fallback || s_route_action_taken))) return 0;
    pm = __get_PRIMASK(); __disable_irq();
    if (s_route_bucket_geometry_bad) {
        vat_stop("IMAGE_GEOMETRY", VAT_STOPPED); __set_PRIMASK(pm); return 1;
    }
    if (!s_route_bucket_loss_sample.seen) { __set_PRIMASK(pm); return 0; }
    s_vat.bucket_seen = 1u;
    now = HAL_GetTick();
    age = now - s_route_bucket_loss_sample.at; s_vat.bucket_age_ms = age;
    if (!vat_ack_ready()) {
        vat_brake(); s_vat.reason = "BUCKET_WAIT_ACK"; __set_PRIMASK(pm); return 1;
    }
    if (age < VAT_ROUTE_BUCKET_LOST_MS) {
        if (!s_route_bucket_loss_braking) { __set_PRIMASK(pm); return 0; }
        s_route_bucket_loss_braking = 0u; s_vat.bucket_fallback = 0u;
        vat_begin_brake(now, 1); s_vat.reason = "BUCKET_NEW_TARGET_BRAKE";
        __set_PRIMASK(pm); return 1;
    }
    if (!s_route_bucket_loss_braking) {
        s_route_bucket_loss_braking = 1u;
        s_route_bucket_seen = s_seen_target = 1u; s_initial_search = 0u;
        s_have_seq = 1u; s_last_seq = s_route_bucket_loss_sample.frame.sequence;
        s_used_packet = s_route_bucket_loss_sample.packet;
        memset((void *)&s_route_bucket_sample, 0, sizeof s_route_bucket_sample);
        vat_begin_brake(now, 1); s_vat.reason = "BUCKET_LOST2S_BRAKE";
        __set_PRIMASK(pm); return 1;
    }
    vat_brake(); (void)vat_counts_changed(now);
    s_vat.good = s_vat.latest = s_vat.axis = 0u;
    s_vat.reason = "BUCKET_LOST2S_BRAKE";
    if ((uint32_t)(now - s_still_from) >= T_DIST_STILL_MS) {
        s_vat.bucket_fallback = 1u; s_vat.alignment_confirmed = 0u;
        s_route_action_taken = 0u;
        s_vat.state = VAT_WAIT_BUCKET_ACTION; s_vat.reason = "BUCKET_LOST2S_RELEASE";
    }
    __set_PRIMASK(pm); return 1;
}
static int vat_geometry(const ProtoFrame *f)
{
    if (!vat_geometry_fields_valid(f)) return 0;
    s_width = f->img_w; s_height = f->img_h; return 1;
}
static void vat_drive(float vx, float vy, float w)
{
    s_vat.vx = vx; s_vat.vy = vy; s_vat.w = w;
    motion_vel_set_precise(vx, vy, w);
}
/* Crossing the one-way X gate ends search, not alignment: brake and discard
 * all moving images. Keep the observed sequence as the freshness baseline. */
static int vat_route_search_finish_if_seen(uint32_t now)
{
    VatSample first;
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    if (!vat_route_search_enabled() || !s_route_search_sample.seen) {
        __set_PRIMASK(pm); return 0;
    }
    first = s_route_search_sample;
    memset((void *)&s_route_search_sample, 0, sizeof s_route_search_sample);
    s_seen_target = 1u; s_initial_search = 0u;
    s_have_seq = 1u; s_last_seq = first.frame.sequence; s_used_packet = first.packet;
    s_rejected_packet = 0u; s_width = first.frame.img_w; s_height = first.frame.img_h;
    if (s_route_search_started) {
        s_vat.yaw_ever = 1u;
        if (fabsf(s_vat.yaw_error) > T_DIST_ALIGN_TOL_DEG) s_vat.yaw_dirty = 1u;
    }
    if (s_vat.state == VAT_BRAKE) { vat_brake(); s_need_new = 1u; }
    else vat_begin_brake(now, 1);
    s_vat.reason = "SEARCH_TARGET_BRAKE";
    __set_PRIMASK(pm); return 1;
}
static void vat_route_search_poll(uint32_t now)
{
    VatSample sample;
    ProtoStats stats;
    float leg, target, w, speed;
    int direction = 1; /* No current selected coordinates: preserve forward search. */
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    if (!vat_ack_ready()) {
        vat_brake(); s_vat.reason = "SEARCH_WAIT_ACK"; __set_PRIMASK(pm); return;
    }
    if (vat_snapshot(&sample, &stats) && sample.packet != s_rejected_packet
        && (uint32_t)(now - sample.at) <= VAT_FRESH_MS && proto_scene_status() == 1) {
        if (!vat_geometry(&sample.frame)) {
            vat_stop("IMAGE_GEOMETRY", VAT_STOPPED); __set_PRIMASK(pm); return;
        }
        /* A fresh-but-replayed sequence must not reverse an existing search.
         * With no selected snapshot at all, the default remains forward. */
        if (s_route_search_started) direction = s_direction;
        vat_route_search_observe(&sample.frame, sample.packet);
        /* Only the accepted coarse sequence may change the search direction.
         * Replayed/old frames cannot drive an opposite-direction command. */
        if (s_route_coarse_have_seq && sample.packet == s_route_coarse_packet &&
            sample.frame.sequence == s_route_coarse_seq)
            direction = sample.frame.cx < s_vat.x_goal ? -1 : 1;
    }
    if (vat_route_search_finish_if_seen(now)) { __set_PRIMASK(pm); return; }
    /* VAT owns a continuous heading target; the existing hold helper takes a
     * leg-relative target. Bridge the CURRENT leg zero explicitly, no re-zero.
     * Its limiter is reused with the route owner's validated gain snapshot. */
    leg = imu_leg_heading_deg(); target = leg + s_vat.yaw_error;
    if (!vat_finite(leg) || !vat_finite(target)) {
        vat_stop("IMUERR", VAT_STOPPED); __set_PRIMASK(pm); return;
    }
    w = step_heading_hold_w_kp(target, s_route_search_kp);
    if (!vat_finite(w)) {
        vat_stop("IMUERR", VAT_STOPPED); __set_PRIMASK(pm); return;
    }
    s_vat.axis = 1u; s_vat.good = s_vat.latest = 0u;
    s_vat.reason = "ROUTE_SEARCH";
    if (!s_route_search_started || s_direction != direction) {
        s_route_search_started = 1u; s_route_search_from = now;
    }
    s_direction = direction;
    speed = VAT_ROUTE_SEARCH_ACC_MMS2 * (float)(uint32_t)(now - s_route_search_from) * 0.001f;
    if (speed > s_route_search_speed) speed = s_route_search_speed;
    speed *= (float)direction;
    vat_drive(speed, speed > 0.0f ? s_route_search_ff * speed : 0.0f, w);
    __set_PRIMASK(pm);
}
static int vat_route_bucket_finish_if_seen(uint32_t now)
{
    VatSample first;
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    if (!s_route_bucket_eligible || !s_route_bucket_sample.seen) {
        __set_PRIMASK(pm); return 0;
    }
    first = s_route_bucket_sample;
    memset((void *)&s_route_bucket_sample, 0, sizeof s_route_bucket_sample);
    s_route_bucket_seen = 1u; s_route_bucket_missing_active = 0u;
    s_seen_target = 1u; s_initial_search = 0u;
    s_have_seq = 1u; s_last_seq = first.frame.sequence; s_used_packet = first.packet;
    s_rejected_packet = 0u; s_width = first.frame.img_w; s_height = first.frame.img_h;
    if (s_route_search_started) {
        s_vat.yaw_ever = 1u;
        if (fabsf(s_vat.yaw_error) > T_DIST_ALIGN_TOL_DEG) s_vat.yaw_dirty = 1u;
    }
    if (s_vat.state == VAT_BRAKE) { vat_brake(); s_need_new = 1u; }
    else vat_begin_brake(now, 1);
    s_vat.reason = "BUCKET_TARGET_BRAKE";
    __set_PRIMASK(pm); return 1;
}
/* Return1 only when this route-only wait/search took ownership of the poll.
 * BeforeACK or during the ordinary brake it cannot issue a wheel command. */
static int vat_route_bucket_poll(uint32_t now)
{
    VatSample sample;
    ProtoStats stats;
    float leg, target, w, speed;
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    if (!s_route_bucket_eligible || s_route_bucket_seen ||
        (s_vat.state != VAT_BRAKE && s_vat.state != VAT_ALIGN &&
         s_vat.state != VAT_RECHECK && s_vat.state != VAT_ROUTE_BUCKET_SEARCH)) {
        __set_PRIMASK(pm); return 0;
    }
    if (!vat_ack_ready()) {
        s_route_bucket_missing_active = 0u;
        if (s_vat.state == VAT_ROUTE_BUCKET_SEARCH) {
            vat_brake(); s_vat.reason = "BUCKET_WAIT_ACK"; __set_PRIMASK(pm); return 1;
        }
        __set_PRIMASK(pm); return 0;
    }
    if (vat_snapshot(&sample, &stats) && sample.packet != s_rejected_packet &&
        (uint32_t)(now - sample.at) <= VAT_FRESH_MS && proto_scene_status() == 1) {
        if (!vat_geometry(&sample.frame)) {
            vat_stop("IMAGE_GEOMETRY", VAT_STOPPED); __set_PRIMASK(pm); return 1;
        }
        vat_route_bucket_observe(&sample.frame, sample.packet);
    }
    if (vat_route_bucket_finish_if_seen(now)) { __set_PRIMASK(pm); return 1; }
    if (!s_route_bucket_missing_active) {
        s_route_bucket_missing_active = 1u; s_route_bucket_missing_from = now;
    }
    if (s_route_bucket_loss_sample.seen &&
        (uint32_t)(now - s_route_bucket_loss_sample.at) > VAT_FRESH_MS &&
        (uint32_t)(now - s_route_bucket_loss_sample.at) < VAT_ROUTE_BUCKET_MISSING_MS) {
        s_route_search_started = 0u;
        vat_brake(); s_vat.reason = "BUCKET_WAIT2_NO_NEW_FRAME";
        __set_PRIMASK(pm); return 1;
    }
    if ((!s_route_bucket_loss_sample.seen &&
         (uint32_t)(now - s_route_bucket_missing_from) < VAT_ROUTE_BUCKET_MISSING_MS) ||
        s_vat.state == VAT_BRAKE || (uint32_t)(now - s_still_from) < T_DIST_STILL_MS) {
        int search = s_vat.state == VAT_ROUTE_BUCKET_SEARCH;
        if (search) { vat_brake(); s_vat.reason = "BUCKET_WAIT2_NO_FRAME"; }
        __set_PRIMASK(pm); return search;
    }
    leg = imu_leg_heading_deg(); target = leg + s_vat.yaw_error;
    if (!vat_finite(leg) || !vat_finite(target)) {
        vat_stop("IMUERR", VAT_STOPPED); __set_PRIMASK(pm); return 1;
    }
    w = step_heading_hold_w_kp(target, s_route_search_kp);
    if (!vat_finite(w)) { vat_stop("IMUERR", VAT_STOPPED); __set_PRIMASK(pm); return 1; }
    if (s_vat.ball_rank == 0u || (s_vat.ball_rank == 2u &&
        (!s_route_bucket_loss_sample.seen ||
         (uint32_t)(now - s_route_bucket_loss_sample.at) > VAT_FRESH_MS))) {
        s_route_search_started = 0u;
        vat_brake(); s_vat.reason = s_vat.ball_rank ? "BUCKET_MIDDLE_WAIT_FRAME" : "BUCKET_WAIT_BALL_RANK";
        __set_PRIMASK(pm); return 1;
    }
    /* Coarse approach stays fast and position-specific until |cx-goal|<40:
     * rank1 backward, rank3 forward, whether the far bucket is visible or not.
     * Rank2 has no blind move and follows its fresh coordinate error. Fine
     * steps use coordinate signs independently of this latched coarse rule. */
    if (s_vat.ball_rank == 1u) s_direction = -1;
    else if (s_vat.ball_rank == 3u) s_direction = 1;
    else s_direction = s_route_bucket_loss_sample.frame.cx > s_vat.x_goal ? 1 : -1;
    if (!s_route_search_started) { s_route_search_started = 1u; s_route_search_from = now; }
    speed = VAT_ROUTE_SEARCH_ACC_MMS2 * (float)(uint32_t)(now - s_route_search_from) * 0.001f;
    if (speed > s_route_search_speed) speed = s_route_search_speed;
    s_vat.state = VAT_ROUTE_BUCKET_SEARCH; s_vat.axis = 1u;
    s_vat.good = s_vat.latest = 0u;
    s_vat.reason = s_direction > 0 ? "BUCKET_SEARCH_FORWARD_RANK3_OR_CX" : "BUCKET_SEARCH_BACK_RANK1_OR_CX";
    vat_drive((float)s_direction * speed, 0.0f, w);
    __set_PRIMASK(pm); return 1;
}
static float vat_step_odo(void)
{
    return s_vat.axis == 1u ? motion_odo_mm() : motion_lateral_odo_mm();
}
static void vat_begin_step(uint32_t now, float vx, float vy, int has_target, uint32_t image_at)
{
    float odo = vat_step_odo();
    if (!vat_finite(odo)) { vat_stop("ODOERR", VAT_STOPPED); return; }
    s_step_from = now; s_step_odo0 = odo; s_step_image_at = image_at;
    s_step_has_target = (uint8_t)has_target;
    s_vat.step++; s_vat.step_mm = 0.0f; s_vat.step_ms = 0u; s_vat.step_capped = 0u;
    s_vat.state = VAT_STEP_MOVE; s_vat.reason = has_target ? "STEP_MOVE" : "SEARCH_STEP";
    s_vat.good = 0u;
    vat_drive(vx, vy, 0.0f);
#if VAT_Y_ALIGN_ENABLE
    if (vy != 0.0f) s_vat.yaw_dirty = s_vat.yaw_ever = 1u;
#endif
}
static void vat_drive_target(uint32_t packet, uint32_t now, float vx, float vy)
{
    VatSample current;
    ProtoStats stats;
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    /* A newer empty/missing packet can arrive between the main snapshot and
     * dispatch. Never issue an old-coordinate command after that invalidation. */
    if (vat_snapshot(&current, &stats) && current.packet == packet
        && packet != s_rejected_packet && (uint32_t)(now - current.at) <= VAT_FRESH_MS
        && proto_scene_status() == 1 && !run_aborted()) {
        vat_begin_step(now, vx, vy, 1, current.at);
    } else {
        s_initial_search = 0u; vat_begin_brake(now, 1);
    }
    __set_PRIMASK(pm);
}
static void vat_step_poll(uint32_t now)
{
    VatSample sample;
    ProtoStats stats;
    float odo = vat_step_odo(), progress;
    if (!vat_finite(odo)) { vat_stop("ODOERR", VAT_STOPPED); return; }
    progress = (odo - s_step_odo0) * (float)s_direction;
    if (!vat_finite(progress)) { vat_stop("ODOERR", VAT_STOPPED); return; }
    s_vat.step_mm = progress; s_vat.step_ms = now - s_step_from;
    if (s_step_saw_target) { s_seen_target = 1u; s_initial_search = 0u; }
    /* A newly seen object ends future blind search, but never reverses or
     * changes THIS bounded step. Empty/intermittent frames do not chatter the
     * motors; their old coordinates still cannot start another step. */
    if (vat_snapshot(&sample, &stats) &&
        (uint32_t)(now - sample.at) <= VAT_FRESH_MS && proto_scene_status() == 1) {
        if (!vat_geometry(&sample.frame)) { vat_stop("IMAGE_GEOMETRY", VAT_STOPPED); return; }
        s_seen_target = 1u; s_initial_search = 0u;
    }
    int expired = s_step_has_target && (uint32_t)(now - s_step_image_at) > VAT_FRESH_MS;
    if (progress >= VAT_STEP_MM || s_vat.step_ms >= VAT_STEP_MAX_MS || expired) {
        s_vat.step_capped = (uint8_t)(s_vat.step_ms >= VAT_STEP_MAX_MS && progress < VAT_STEP_MM);
        vat_begin_brake(now, s_seen_target ? 1 : 0);
        s_vat.reason = expired ? "STEP_OLD_BRAKE" : s_vat.step_capped ? "STEP_CAP_BRAKE" : "STEP_DONE_BRAKE";
    }
    /* Direction/velocity stay latched; only cancellation/faults bypass this
     * cadence. Coordinates received during motion never count as arrival. */
}
/* Discard images captured before the wheels/yaw became stable. Preserve the
 * task's seq history on rechecks so duplicate/backwards frames stay rejected. */
static void vat_after_stop(int initial)
{
    ProtoStats stats;
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    proto_stats_get(&stats); s_recheck_packet = stats.obj;
    memset((void *)&s_sample, 0, sizeof s_sample);
    s_vat.latest = s_vat.good = 0u;
    if (initial) {
        s_session_packet = s_used_packet = stats.obj;
        s_have_seq = 0u; s_rejected_packet = 0u;
    }
    s_vat.state = initial ? VAT_ALIGN : VAT_RECHECK;
    yaw_progress_reset(&s_vat_yaw_progress);
    s_vat.reason = initial ? "ALIGN" : "NEW_XY_AFTER_STOP";
    __set_PRIMASK(pm);
}
static void vat_yaw_poll(uint32_t now, int changed)
{
    float yaw = imu_heading_deg(), e, w;
    if (!imu_ok() || !vat_finite(yaw)) { vat_stop("IMUERR", VAT_STOPPED); return; }
    e = vat_heading_error(s_vat.yaw_target, yaw);
    if (!vat_finite(e)) { vat_stop("IMUERR", VAT_STOPPED); return; }
    s_vat.yaw_error = e;
    if ((uint32_t)(now - s_yaw_from) >= T_DIST_ALIGN_MAX_MS) {
        vat_stop("YAW_TIMEOUT", VAT_STOPPED); return;
    }
    if (fabsf(e) > T_DIST_ALIGN_TOL_DEG) {
        float min_w = VAT_YAW_MIN_W;
        s_yaw_stable = 0u; s_vat.reason = "YAW_CORRECT";
        if (s_route_owned && s_route_yaw_settle_ms == 400u)
            min_w = yaw_progress_floor(&s_vat_yaw_progress, e, now, min_w, T_DIST_ALIGN_MAX_W);
        else yaw_progress_reset(&s_vat_yaw_progress);
        w = T_DIST_ALIGN_KP * e;
        if (w > T_DIST_ALIGN_MAX_W) w = T_DIST_ALIGN_MAX_W;
        if (w < -T_DIST_ALIGN_MAX_W) w = -T_DIST_ALIGN_MAX_W;
        if (fabsf(w) < min_w) w = e > 0.0f ? min_w : -min_w;
        vat_drive(0.0f, 0.0f, w); return;
    }
    yaw_progress_reset(&s_vat_yaw_progress);
    vat_brake(); s_vat.reason = "YAW_SETTLE";
    if (!s_yaw_stable || changed ||
        fabsf(vat_heading_error(s_yaw_stable_heading, yaw)) > T_DIST_ALIGN_STILL_DEG) {
        s_yaw_stable = 1u; s_yaw_stable_from = now; s_yaw_stable_heading = yaw;
    }
    if ((uint32_t)(now - s_yaw_stable_from) >=
        (s_route_owned ? s_route_yaw_settle_ms : T_DIST_ALIGN_STABLE_MS) &&
        (uint32_t)(now - s_still_from) >= T_DIST_STILL_MS) {
        s_vat.yaw_dirty = 0u;
        vat_after_stop(0); /* Not the old five frames; require post-yaw images. */
    }
}
static void vat_finish_alignment(uint32_t now)
{
    if (s_route_owned && s_vat.mode == 40u && s_vat.task == PROTO_TASK_HOSTAGE) {
        vat_brake(); s_hold_from = now;
        s_vat.latest = s_vat.axis = 0u; s_vat.good = VAT_GOOD_FRAMES;
        s_vat.alignment_confirmed = 1u; s_vat.hostage_fallback = 0u;
        s_route_action_taken = 0u;
        s_vat.state = VAT_WAIT_HOSTAGE_ACTION; s_vat.reason = "HOSTAGE_ALIGNED_GRAB";
        (void)vat_hostage_rank_capture();
        return; /* Keep task3 RXopen for54 while the owner moves the arm. */
    }
    vat_brake();
    if (!(s_route_owned && s_vat.task == PROTO_TASK_BALL)) proto_receive_end();
    vat_sample_clear(); s_hold_from = now;
    s_vat.latest = s_vat.axis = 0u; s_vat.good = VAT_GOOD_FRAMES;
    if (s_vat.mode == 41u) {
        if (s_route_owned) {
            s_route_action_taken = 0u;
            s_vat.state = s_vat.task == PROTO_TASK_BALL ? VAT_WAIT_BALL_ACTION : VAT_WAIT_BUCKET_ACTION;
            s_vat.reason = "LIFT_WAIT";
        } else {
            s_vat.state = s_vat.task == PROTO_TASK_BALL ? VAT_HOLD_BALL : VAT_HOLD_BUCKET;
            s_vat.reason = "HOLD5";
        }
    } else vat_stop("ALIGNED", VAT_DONE);
}
static int vat_final_check(uint32_t packet, uint32_t now)
{
    VatSample final;
    ProtoStats stats;
    uint32_t pm = __get_PRIMASK();
    int valid;
    __disable_irq();
    valid = vat_ack_ready() && vat_snapshot(&final, &stats) && final.packet == packet
        && (uint32_t)(now - final.at) <= VAT_FRESH_MS && !run_aborted()
        && proto_scene_status() == 1
        && abs(final.frame.cx - s_vat.x_goal) <= VAT_TOL_PX;
#if VAT_Y_ALIGN_ENABLE
    valid = valid && abs(final.frame.cy - s_y_target) <= VAT_TOL_PX;
#endif
    if (!vat_heading_update() || (s_vat.yaw_ever &&
        fabsf(s_vat.yaw_error) > T_DIST_ALIGN_TOL_DEG)) valid = 0;
    for (int i = 0; i < 4; ++i) if (ctrl_enc_total(i) != s_counts[i]) valid = 0;
    if (valid && s_route_owned && s_vat.task == PROTO_TASK_HOSTAGE) {
        s_vat.alignment_confirmed = 1u; /* Pixels/still/freshness already passed; do not delay owner claw on rank0. */
        s_vat.target_rank = 0u; s_vat.rank_sequence = 0u;
        (void)vat_hostage_rank_capture();
    }
    if (valid) vat_finish_alignment(now);
    __set_PRIMASK(pm); return valid;
}
void vision_align_test_poll(void)
{
    uint32_t now = HAL_GetTick();
    VatSample sample;
    ProtoStats stats;
    int matching, fresh, is_new, changed;
    if (!vat_running()) return;
    if (run_aborted()) { vat_stop("ABORT", VAT_STOPPED); return; }
    if (proto_scene_status() < 0) { vat_stop("NACK", VAT_STOPPED); return; }
    if (s_vat.state == VAT_TURN_ACTIVE) return; /* Mode22 exclusively owns motors. */
    if (!vat_heading_update()) { vat_stop("IMUERR", VAT_STOPPED); return; }
    vat_ball_rank_capture();
    if (s_vat.state == VAT_WAIT_BALL_RANK) {
        vat_brake();
        if (!vat_ack_ready()) { vat_stop("NACK", VAT_STOPPED); return; }
        if (s_vat.ball_rank) vat_ball_begin_bucket_turn();
        return;
    }
    if (vat_route_hostage_loss_poll(now)) return;
    if (vat_route_bucket_loss_poll(now)) return;
    if (s_vat.state == VAT_WAIT_BALL_ACTION || s_vat.state == VAT_WAIT_BUCKET_ACTION) {
        vat_brake(); return; /* The route owner must explicitly complete the action. */
    }
    if (s_vat.state == VAT_WAIT_HOSTAGE_ACTION || s_vat.state == VAT_WAIT_HOSTAGE_RANK) {
        vat_brake(); (void)vat_counts_changed(now);
        if (!vat_ack_ready()) { vat_stop("NACK", VAT_STOPPED); return; }
        if (s_vat.state == VAT_WAIT_HOSTAGE_RANK && vat_hostage_rank_capture())
            vat_stop(s_vat.hostage_fallback ? "HOSTAGE_LOST2S_GRABBED" : "HOSTAGE_ALIGNED_GRABBED", VAT_DONE);
        return;
    }
    if (vat_route_search_finish_if_seen(now)) return;
    if (vat_route_bucket_finish_if_seen(now)) return;
    if (s_vat.state == VAT_ROUTE_SEARCH) { vat_route_search_poll(now); return; }
    if (vat_route_bucket_poll(now)) return;
    if ((uint32_t)(now - s_last_poll) < VAT_POLL_MS) return;
    s_last_poll = now;
    /* Also harvest at BRAKE: a target ISR can set the sticky bit just after
     * the final STEP poll read. Never restart a blind step across that race. */
    if (s_step_saw_target) {
        s_seen_target = 1u; s_initial_search = 0u;
        if (s_vat.state == VAT_BRAKE) s_need_new = 1u;
    }
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
    if (s_vat.state == VAT_STEP_MOVE) { vat_step_poll(now); return; }
    if (s_vat.state == VAT_YAW_FIX) { vat_yaw_poll(now, changed); return; }
    if (s_vat.state == VAT_BRAKE) {
        vat_brake();
        if ((uint32_t)(now - s_still_from) < T_DIST_STILL_MS) return;
        if (s_route_search_started && fabsf(s_vat.yaw_error) > T_DIST_ALIGN_TOL_DEG)
            s_vat.yaw_dirty = s_vat.yaw_ever = 1u;
        if (s_vat.yaw_dirty) {
            s_vat.state = VAT_YAW_FIX; s_vat.reason = "YAW_CORRECT";
            s_yaw_from = now; s_yaw_stable = 0u; s_vat.latest = s_vat.good = 0u;
            return;
        }
        vat_after_stop(!s_need_new);
        return; /* A frame captured before stopping never proves arrival. */
    }
    matching = vat_snapshot(&sample, &stats);
    fresh = matching && sample.packet != s_rejected_packet
        && (uint32_t)(now - sample.at) <= VAT_FRESH_MS && proto_scene_status() == 1;
    s_vat.latest = (uint8_t)fresh; s_vat.age_ms = sample.seen ? now - sample.at : UINT32_MAX;
    if (fresh && !vat_geometry(&sample.frame)) { vat_stop("IMAGE_GEOMETRY", VAT_STOPPED); return; }
    if (fresh && vat_route_search_enabled() && s_initial_search && !s_seen_target) {
        uint32_t pm = __get_PRIMASK();
        __disable_irq(); vat_route_search_observe(&sample.frame, sample.packet);
        int seen = vat_route_search_finish_if_seen(now);
        __set_PRIMASK(pm);
        if (seen) return;
        /* Far selected coordinates use the owner's signed coarse approach.
         * Once the fine gate latches, s_initial_search stays false forever. */
        if (s_vat.state == VAT_ALIGN && vat_ack_ready()) {
            s_vat.state = VAT_ROUTE_SEARCH;
            vat_route_search_poll(now); return;
        }
    }
    if (!fresh) {
        s_vat.good = 0u;
        if (s_vat.state == VAT_ALIGN && !s_seen_target && !s_step_saw_target && s_initial_search && vat_ack_ready()) {
            if (vat_route_search_enabled()) {
                s_vat.state = VAT_ROUTE_SEARCH;
                vat_route_search_poll(now); return;
            }
            s_vat.axis = 1u; s_direction = 1; s_still_from = now;
            vat_begin_step(now, VAT_X_SPEED_MMS, 0.0f, 0, now); return;
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
    int ex = sample.frame.cx - s_vat.x_goal;
#if VAT_Y_ALIGN_ENABLE
    int ey = sample.frame.cy - s_y_target;
    uint8_t axis = abs(ex) > VAT_TOL_PX ? 1u : abs(ey) > VAT_TOL_PX ? 2u : 0u;
    int direction = axis == 1u ? (ex > 0 ? 1 : -1) : axis == 2u ? (ey > 0 ? 1 : -1) : 0;
#else
    uint8_t axis = abs(ex) > VAT_TOL_PX ? 1u : 0u;
    int direction = axis ? (ex > 0 ? 1 : -1) : 0;
#endif
    if (axis) {
        s_vat.good = 0u;
        if (!s_vat.axis && !is_new) { vat_brake(); return; }
        s_vat.axis = axis; s_direction = direction; s_still_from = now;
#if VAT_Y_ALIGN_ENABLE
        float speed = (axis == 1u ? VAT_X_SPEED_MMS : VAT_Y_SPEED_MMS) * (float)direction;
        vat_drive_target(sample.packet, now, axis == 1u ? speed : 0.0f,
                         axis == 2u ? speed : 0.0f);
#else
        vat_drive_target(sample.packet, now, VAT_X_SPEED_MMS * (float)direction, 0.0f);
#endif
        return;
    }
    vat_brake();
    /* A later X repair/inertia may twist a task after route search or enabled Y.
     * Do not call it aligned merely because the earlier correction completed. */
    if (s_vat.yaw_ever && fabsf(s_vat.yaw_error) > T_DIST_ALIGN_TOL_DEG) {
        s_vat.yaw_dirty = 1u; s_initial_search = 0u; vat_begin_brake(now, 1); return;
    }
    if (is_new && s_vat.good < VAT_GOOD_FRAMES) s_vat.good++;
    if (is_new && s_vat.good >= VAT_GOOD_FRAMES && (uint32_t)(now - s_still_from) >= T_DIST_STILL_MS)
        (void)vat_final_check(sample.packet, now);
}
int vision_align_test_take_turn_request(void)
{
    if (s_vat.state != VAT_TURN_REQUEST) return 0;
    if (run_aborted() || !imu_ok() || !vat_finite(imu_heading_deg()) || proto_scene_status() < 0) {
        vat_stop("TURN180_PREP", VAT_STOPPED); return 0;
    }
    s_vat.state = VAT_TURN_ACTIVE; s_vat.reason = "TURN180_ACTIVE"; return 1;
}
void vision_align_test_notify_turn_result(int success)
{
    if (s_vat.state != VAT_TURN_ACTIVE) return;
    float heading = imu_heading_deg();
    if (!success || run_aborted() || !imu_ok() || !vat_finite(heading) || proto_scene_status() < 0) {
        vat_stop("TURN180_FAILED", VAT_STOPPED); return;
    }
    /* Explicit completion includes any route-owned offset after +180. */
    s_vat.yaw_target = heading; s_vat.yaw_error = 0.0f;
    if (!vat_request(PROTO_TASK_BUCKET, 0u, HAL_GetTick())) { vat_stop("REQUEST", VAT_STOPPED); return; }
    s_initial_search = 0u; s_need_new = 1u;
    /* The pre-turn bucket request cannot arm this timer. Only this explicit
     * completed180/offset/new request may wait for its matchingACK then count2s. */
    s_route_bucket_eligible = (uint8_t)(s_route_owned && s_vat.mode == 41u);
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
    out->rx_cx = out->rx_cy = 0;
    out->rx_sequence = out->rx_img_w = out->rx_img_h = 0u;
    out->rx_fresh = (uint8_t)(matching && out->age_ms <= VAT_FRESH_MS && vat_ack_ready());
    {
        uint32_t pm = __get_PRIMASK();
        __disable_irq();
        if (s_route_hostage_sample.seen && s_route_owned && s_vat.task == PROTO_TASK_HOSTAGE) {
            out->hostage_seen = 1u;
            out->hostage_age_ms = HAL_GetTick() - s_route_hostage_sample.at;
        }
        if (s_route_bucket_loss_sample.seen && s_route_owned && s_vat.task == PROTO_TASK_BUCKET) {
            out->bucket_seen = 1u;
            out->bucket_age_ms = HAL_GetTick() - s_route_bucket_loss_sample.at;
        }
        __set_PRIMASK(pm);
    }
    if (out->rx_fresh) {
        out->rx_cx = current.frame.cx; out->rx_cy = current.frame.cy;
        out->rx_sequence = current.frame.sequence;
        out->rx_img_w = current.frame.img_w; out->rx_img_h = current.frame.img_h;
    }
    /* cx/cy remain historical values in logs; latest never blesses stale data
     * during braking, holds, cancellation or a rejected sequence. */
    if (!matching || current.packet != s_used_packet || current.packet == s_rejected_packet || out->age_ms > VAT_FRESH_MS
        || proto_scene_status() != 1) out->latest = 0u;
}
