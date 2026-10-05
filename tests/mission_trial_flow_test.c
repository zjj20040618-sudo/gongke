/* Real mode32 runner + real road ledger, synthetic host sensors only.
 * No arm implementation is linked: adding an arm/sweep/return API dependency
 * to the runner makes this fixture fail to link. No physical acceptance claim.
 * The g-stop model below injects the same run_abort latch + immediate brake;
 * it is NOT a Bluetooth byte-parser test (covered by g_command_stop_test).
 */
#include "mission_trial.h"
#include "mission_trial_plan.h"
#include "steps.h"
#include "motion.h"
#include "imu.h"
#include "board_pins.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "trial flow line %d: %s; phase=%s time=%lu\n", \
    __LINE__, #x, mission_trial_phase(), (unsigned long)now_ms); return 1; } } while (0)
#define RAD_DEG 57.2957795131f
#define MAX_EVENTS 40u
/* Arbitrary synthetic distances/target positions, not measured station values
 * or a fixed ball-color arrangement. Physical ball order is drawn by lot. */
#define SYNTHETIC_FIRST_MM 1000u
#define SYNTHETIC_SECOND_MM 1450.0f

typedef struct {
    char name[40];
    uint32_t at;
    /* Phase entry is before the next road_start(), so road/total describe
     * the completed previous leg at that boundary, not the new leg. */
    float fore, lat, heading, road, total;
} Event;
typedef struct { const char *name; uint32_t duration; } Hold;
typedef struct {
    ProtoTask task;
    uint8_t digit;
    uint32_t at;
    float heading;
    unsigned positive_180_before;
} TargetRequest;
enum { IN_NONE, IN_WAIT, IN_TURN, IN_ALIGN, IN_PASS, IN_VISION, IN_LASER, IN_PREP, IN_TRANSLATE, IN_QR, IN_RETURN, IN_TARGET_SETTLE };
/* Fixture-only marker; the bucket is a distinct request, not another EOD
 * scene. Do not assign this test value to the real ProtoScene enum. */
enum { VISION_BUCKET_MARKER = 100 };

static Event events[MAX_EVENTS];
static unsigned event_count, issues, qr_reads, commands, tuning_locks, first_leg_locks;
static Hold holds[8];
static unsigned hold_count, fire_calls, laser_on_calls;
static unsigned target_settle_calls;
static uint32_t target_settle_started, target_settle_finished, laser_started, laser_finished;
static int scenes[8], vision_abort_marker;
static unsigned receive_end_calls, forbidden_scene_calls;
static int receive_open;
static unsigned scene_count;
static TargetRequest target_requests[4];
static unsigned target_request_count;
static int32_t qr_digits[3];
static uint32_t now_ms, phase_at, abort_at, context_at;
static float fore_mm, lat_mm, heading_deg, cmd_fore, cmd_lat, cmd_w;
static int aborted, laser_on, all_missing, in_context, scene_in_progress, sensor_ok;
static const char *abort_name;
static int abort_context, abort_kind, selected_cls, selected_label;
static unsigned align_index, align_signs[4], first_turn_seen, positive_180;
static unsigned grab_align_calls[4], grab_y_locks;
static float abort_fore, abort_lat, abort_w;
static uint32_t next_frame_at;
static char last_report[180];
static char last_split_report[120];
static unsigned return_commands[2], qr_prepared, qr_ready;
static int task_errors[2];
static float align_anchors[4], settle_shift_mm, ball_at_mm, target_at_mm, hostage_at_mm;
static int task_exact_heading;
static uint32_t qr_valid_after_ms;
static uint32_t qr_request_at;
static int qr_precached, qr_request_failed;

static int is_phase(const char *name) { return strcmp(mission_trial_phase(), name) == 0; }
static int returning(void) { return is_phase("TARGET_RETURN") || is_phase("HOSTAGE_RETURN"); }
static float progress(void) { float d; mission_trial_get_progress(&d, 0); return d; }
static int close_value(float a, float b, float tol) { return fabsf(a - b) <= tol; }

static void observe(void)
{
    const char *name = mission_trial_phase();
    if (strcmp(name, "BOOT") == 0) return;
    if (!event_count || strcmp(events[event_count - 1u].name, name) != 0) {
        Event *e;
        if (event_count >= MAX_EVENTS) { issues++; return; }
        e = &events[event_count++];
        snprintf(e->name, sizeof e->name, "%s", name);
        e->at = phase_at = now_ms;
        e->fore = fore_mm; e->lat = lat_mm; e->heading = heading_deg;
        mission_trial_get_progress(&e->road, &e->total);
        first_turn_seen = 0u;
        /* Configuration writes must be locked even before road tracking starts. */
        if (strcmp(name, "STOP") != 0 && strcmp(name, "ROUTE_END") != 0) {
            if (mission_trial_set_alignment(0, 241, 1)) issues++;
            else tuning_locks++;
            if (mission_trial_set_grab_y(CLS_BALL, 151, 1)) issues++;
            else grab_y_locks++;
            if (mission_trial_set_first_leg(1111u)) issues++;
            else first_leg_locks++;
        }
    }
}

static const Event *event(const char *name)
{
    for (unsigned i = 0u; i < event_count; ++i)
        if (strcmp(events[i].name, name) == 0) return &events[i];
    return 0;
}

uint32_t HAL_GetTick(void) { return now_ms; }
uint8_t imu_ok(void) { return (uint8_t)sensor_ok; }
float imu_heading_deg(void) { return heading_deg; }
float motion_odo_mm(void) { return fore_mm; }
float motion_lateral_odo_mm(void) { return lat_mm; }
float step_heading_kp_deg(void) { return 0.3f; }
float test_forward_ff_ratio(void) { return 0.0125f; }
int run_aborted(void) { return aborted; }
void run_abort(void) { aborted = 1; step_vision_receive_end(); }

void bp_debug_send(const char *s)
{
    observe();
    if (strstr(s, "hits=")) snprintf(last_report, sizeof last_report, "%s", s);
    if (strstr(s, "first=")) snprintf(last_split_report, sizeof last_split_report, "%s", s);
}
void bp_laser_set(int on)
{
    observe();
    if (on && (!is_phase("TARGET_LASER") || run_aborted() ||
               target_settle_calls != 1u || now_ms - target_settle_started != 1000u ||
               target_settle_finished != now_ms)) issues++;
    laser_on = on != 0;
    if (on) laser_on_calls++;
}
void motion_brake(void) { observe(); cmd_fore = cmd_lat = cmd_w = 0.0f; }

static void velocity(float vx, float vy, float w, int precise)
{
    const char *name;
    observe(); name = mission_trial_phase();
    if (aborted) issues++;
    /* A 100 mm/s road-axis command plus -1.25 mm/s feedforward has a
     * 100.0078 mm/s vector magnitude before heading-frame conversion. */
    if (fabsf(vx) > 100.02f || fabsf(vy) > 100.02f) issues++;
    if (returning()) {
        if (!precise || vx <= 0.0f) issues++;
        return_commands[is_phase("HOSTAGE_RETURN") ? 1u : 0u]++;
        if (is_phase("TARGET_RETURN") ? fire_calls != 1u || laser_on
            : !hold_count || strcmp(holds[hold_count - 1u].name, "HOSTAGE_HOLD10S") != 0) issues++;
    }
    if (strstr(name, "ALIGN")) {
        if (!precise || selected_cls < 0 || selected_cls > 3) issues++;
        else if (vx > 0.0f) align_signs[selected_cls] |= 1u;
        else if (vx < 0.0f) align_signs[selected_cls] |= 2u;
        if (fabsf(vy) > 0.001f) issues++;
    }
    if ((!returning() && strstr(name, "TURN")) || is_phase("R3_LEFT95") || is_phase("R7_RIGHT85") || is_phase("R9_RIGHT85")) {
        if (precise || fabsf(vx) > 0.001f || fabsf(vy) > 0.001f) issues++;
        if (!first_turn_seen++) {
            if (is_phase("BALL_TURN180") || is_phase("BUCKET_TURN180_BACK")) {
                if (w <= 0.0f) issues++;
                else positive_180++;
            } else if (is_phase("R3_LEFT95") ? w >= 0.0f : w <= 0.0f) issues++;
        }
    } else if (!strstr(name, "ALIGN") && !precise) issues++;
    cmd_fore = vx; cmd_lat = vy; cmd_w = w; commands++;
}
void motion_vel_set(float vx, float vy, float w) { velocity(vx, vy, w, 0); }
void motion_vel_set_precise(float vx, float vy, float w) { velocity(vx, vy, w, 1); }
void motion_linear_ramp_init(MotionRamp *r) { memset(r, 0, sizeof *r); }
float motion_linear_profile_step(MotionRamp *r, float speed, float rest, float dt)
{
    (void)r; (void)rest; (void)dt;
    return speed; /* Deliberately ideal plant; no claim about actual acceleration. */
}

static void inject_stop(void)
{
    int matched;
    if (!abort_name || aborted || !sensor_ok) return;
    if (abort_context == IN_VISION)
        matched = in_context == IN_VISION && scene_in_progress == vision_abort_marker;
    else matched = strcmp(mission_trial_phase(), abort_name) == 0 && in_context == abort_context;
    if (matched && now_ms - context_at >= 15u) {
        abort_at = now_ms;
        abort_fore = cmd_fore; abort_lat = cmd_lat; abort_w = cmd_w;
        if (abort_kind == 2) sensor_ok = 0;
        else run_abort();
        if (abort_kind == 1) { /* model the g router's immediate latch and brake */
            motion_brake(); bp_laser_set(0);
        }
    }
}

void osDelay(uint32_t ms)
{
    for (uint32_t i = 0u; i < ms; ++i) {
        int saved = in_context;
        if (in_context == IN_NONE) {
            const char *p = mission_trial_phase();
            in_context = is_phase("QR_WAIT") ? (abort_context == IN_VISION ? IN_VISION : IN_QR)
                : returning() ? IN_RETURN : strstr(p, "ALIGN") ? IN_ALIGN : strstr(p, "PASS") || strstr(p, "REMAINDER") ? IN_PASS
                : strstr(p, "TURN") || is_phase("R3_LEFT95") || is_phase("R7_RIGHT85") || is_phase("R9_RIGHT85") ? IN_TURN : IN_TRANSLATE;
            context_at = is_phase("QR_WAIT") ? qr_request_at : phase_at;
        }
        now_ms++;
        if (in_context != IN_VISION && (in_context == IN_WAIT || in_context == IN_TARGET_SETTLE || in_context == IN_LASER
                || returning() || is_phase("BALL_TURN180") || is_phase("BUCKET_TURN180_BACK")
                || is_phase("BUCKET_TO_CORNER_REMAINDER") || is_phase("RESCUE_REMAINDER2125"))
            && receive_open) issues++; /* Continued camera frames must not enter task control. */
        if (in_context == IN_TARGET_SETTLE || in_context == IN_LASER) {
            if (cmd_fore != 0.0f || cmd_lat != 0.0f || cmd_w != 0.0f) issues++;
            if (in_context == IN_TARGET_SETTLE && laser_on) issues++;
            if (in_context == IN_LASER && !aborted && !laser_on) issues++;
        }
        fore_mm += cmd_fore * 0.001f;
        lat_mm += cmd_lat * 0.001f;
        heading_deg += cmd_w * RAD_DEG * 0.001f;
        mission_trial_tick_1ms();
        inject_stop();
        in_context = saved;
        if (now_ms > 300000u) { issues++; run_abort(); }
    }
}

int step_prepare_leg(void)
{
    int saved = in_context;
    observe(); motion_brake();
    in_context = IN_PREP; /* stop-injection turn/align/pass cases exercise motion, not this mock wait */
    context_at = now_ms;
    if (is_phase("BALL_TURN180") && !first_turn_seen) heading_deg = 27.0f - 0.2f;
    if (is_phase("BUCKET_TURN180_BACK") && !first_turn_seen) heading_deg = 207.0f - 0.2f;
    if (task_exact_heading && is_phase("TARGET_ALIGN")) heading_deg = 387.0f;
    if (task_exact_heading && is_phase("HOSTAGE_ALIGN")) heading_deg = 472.0f;
    if (is_phase("TARGET_ALIGN") || is_phase("HOSTAGE_ALIGN")) fore_mm += settle_shift_mm;
    mission_trial_tick_1ms();
    /* Stop settling is mocked, rather than exercising real encoder stillness. */
    osDelay(20u); in_context = saved;
    if (is_phase("QR_WAIT") && !aborted) qr_prepared = 1u;
    return !aborted;
}
void wait_ms(uint32_t ms)
{
    int saved = in_context;
    uint32_t started = now_ms;
    if (hold_count >= sizeof holds / sizeof holds[0]) { issues++; return; }
    holds[hold_count].name = mission_trial_phase(); holds[hold_count++].duration = ms;
    in_context = IN_WAIT; context_at = now_ms;
    while (!aborted && now_ms - started < ms) osDelay(1u);
    in_context = saved;
}
void proto_qr_begin(void)
{
    observe();
    if (!is_phase("QR_WAIT") || !qr_prepared || hold_count ||
        scene_count >= sizeof scenes / sizeof scenes[0]) { issues++; return; }
    /* Nonblocking QR request. Four task requests use step_vision_target below. */
    scenes[scene_count++] = SCENE_QR;
    receive_open = 1;
    qr_request_at = now_ms;
    scene_in_progress = SCENE_QR;
    if (qr_precached) qr_ready = 1u; /* Existing boot request may already be valid. */
}

int proto_scene_status(void)
{
    if (qr_request_failed) return -1;
    if (now_ms - qr_request_at >= 40u) qr_ready = 1u;
    return qr_ready ? 1 : 0; /* Fake ACK/fresh; real request/CRC checked elsewhere. */
}

int proto_qr_get(int32_t d[3])
{
    if (!is_phase("QR_WAIT") || hold_count || !qr_prepared
        || cmd_fore != 0.0f || cmd_lat != 0.0f || cmd_w != 0.0f) issues++;
    qr_reads++;
    if (aborted) return 0;
    /* The parser hides partial/invalid tuples; the nonblocking getter must
     * not release R2 or publish them, even after fresh status becomes true. */
    if (!qr_ready || (!qr_precached && now_ms - qr_request_at < 50u) ||
        now_ms - phase_at < qr_valid_after_ms || abort_context == IN_QR) {
        int32_t latched[3];
        mission_trial_get_qr(latched);
        if (latched[0] || latched[1] || latched[2] || event("R2_BACK600")) issues++;
        return 0;
    }
    d[0] = qr_digits[0]; d[1] = qr_digits[1]; d[2] = qr_digits[2];
    return 1;
}

int step_vision_scene(ProtoScene scene)
{
    (void)scene;
    issues++; /* Tasks must carry their selected digit, not a broad scene. */
    return 0;
}
int step_vision_target(ProtoTask task, uint8_t digit)
{
    int saved = in_context, marker;
    unsigned index;
    if (scene_count >= sizeof scenes / sizeof scenes[0] || target_request_count >= 4u) {
        issues++; return 0;
    }
    /* Task IDs are wire values: ball=1, target=2, hostage=3, bucket=4.
     * Wire/CRC/request-ID validation is covered by the real parser tests. */
    if (task == PROTO_TASK_BALL) {
        marker = SCENE_EOD;
        if (digit != qr_digits[0] || !event("R7_RIGHT85") || event("BALL_SINGLE_PASS")) issues++;
    } else if (task == PROTO_TASK_TARGET) {
        marker = SCENE_ANTI;
        if (digit != qr_digits[1] || event("TARGET_SINGLE_PASS")) issues++;
    } else if (task == PROTO_TASK_HOSTAGE) {
        marker = SCENE_RESCUE;
        if (digit != qr_digits[2] || !event("R9_RIGHT85") || event("HOSTAGE_SINGLE_PASS")) issues++;
    } else if (task == PROTO_TASK_BUCKET) {
        marker = VISION_BUCKET_MARKER;
        if (digit != 0u || positive_180 != 1u || !event("BALL_TURN180") || event("BUCKET_ALIGN")
            || !close_value(heading_deg, 207.0f, 0.31f)) issues++;
    } else { issues++; return 0; }
    index = target_request_count++;
    target_requests[index].task = task;
    target_requests[index].digit = digit;
    target_requests[index].at = now_ms;
    target_requests[index].heading = heading_deg;
    target_requests[index].positive_180_before = positive_180;
    scenes[scene_count++] = marker;
    receive_open = 1;
    in_context = IN_VISION; context_at = now_ms; scene_in_progress = marker;
    motion_brake();
    for (unsigned i = 0u; i < 40u && !aborted; ++i) osDelay(1u);
    in_context = saved;
    return !aborted; /* command/ACK framing is covered by the binary replay suite */
}
void proto_send_scene(ProtoScene scene)
{
    (void)scene;
    forbidden_scene_calls++; issues++; /* No stop/IDLE command is permitted in mode32. */
}
void step_vision_receive_end(void)
{
    receive_open = 0;
    receive_end_calls++;
}

void step_object_select(int cls, int label)
{
    selected_cls = cls; selected_label = label;
    align_index = 0u; next_frame_at = now_ms;
    if (strstr(mission_trial_phase(), "ALIGN") && cls >= 0 && cls < 4) align_anchors[cls] = progress();
    if ((cls == CLS_BALL && label != qr_digits[0] - 1) || (cls == CLS_TARGET && label != qr_digits[1] - 1)
        || (cls == CLS_HOSTAGE && label != qr_digits[2] + 2) || (cls == CLS_BUCKET && label != -1)) issues++;
}
int step_object_take(ProtoFrame *out)
{
    float at = progress();
    if (!receive_open) { issues++; return 0; }
    if (all_missing) return 0;
    memset(out, 0, sizeof *out);
    out->type = PF_OBJ; out->cls = selected_cls; out->label = selected_label;
    out->conf = 90; out->img_w = 480; out->img_h = 320; out->w = 30; out->h = 40;
    out->cy = 150 + selected_cls * 10;
    if (strstr(mission_trial_phase(), "ALIGN")) {
        if (now_ms < next_frame_at) return 0;
        /* Two out-of-window frames require +36 then -18 mm/s for 100 ms.
         * Net body-forward displacement is +1.8 mm; bucket at +180 must
         * DECREASE fixed-road progress rather than reset/ignore this motion. */
        out->cx = 200 + selected_cls * 10 + (align_index < 2u
            ? (selected_cls == CLS_TARGET || selected_cls == CLS_HOSTAGE ? task_errors[align_index]
               : align_index == 0u ? 60 : -30) : 0);
        out->sequence = (uint16_t)++align_index;
        next_frame_at = now_ms + 100u;
        return 1;
    }
    out->cx = 200 + selected_cls * 10;
    if (is_phase("BALL_SINGLE_PASS")) return at >= ball_at_mm;
    if (is_phase("TARGET_SINGLE_PASS")) return at >= target_at_mm;
    if (is_phase("HOSTAGE_SINGLE_PASS")) return at >= hostage_at_mm;
    return 0;
}
/* Shared real XY loop is covered by grab_xy_alignment_test. This mock checks
 * mode32 supplies BOTH measured workpoints/signs to that loop and preserves
 * the synthetic road ledger while consuming independent current frames. */
int step_align_xy(int cls, int label, const GrabAlignConfig *cfg,
                  float absolute_heading_deg, uint32_t to)
{
    uint32_t started = now_ms, last_frame = now_ms, still_at = now_ms;
    unsigned good = 0u;
    uint16_t previous_sequence = 0u;
    int seen = 0, moving = 0;
    float heading_error = absolute_heading_deg - heading_deg;
    while (heading_error > 180.0f) heading_error -= 360.0f;
    while (heading_error < -180.0f) heading_error += 360.0f;
    if ((cls != CLS_BALL && cls != CLS_HOSTAGE) || !cfg ||
        cfg->cx != 200 + cls * 10 || cfg->cy != 150 + cls * 10 ||
        cfg->x_sign != 1 || cfg->y_sign != 1 ||
        cfg->x_tol_px != GRAB_XY_TOL_PX || cfg->y_tol_px != GRAB_XY_TOL_PX ||
        !isfinite(absolute_heading_deg) || fabsf(heading_error) > 0.31f || to != 0u) {
        issues++; return 0;
    }
    grab_align_calls[cls]++;
    step_object_select(cls, label);
    while (!aborted) {
        ProtoFrame f;
        if (!sensor_ok || (to && now_ms - started >= to)) break;
        if (step_object_take(&f)) {
            float ex = (float)f.cx - (float)cfg->cx;
            float ey = (float)f.cy - (float)cfg->cy;
            int x_bad = fabsf(ex) > cfg->x_tol_px;
            int y_bad = fabsf(ey) > cfg->y_tol_px;
            if (f.cls != cls || f.label != label || !f.img_w || !f.img_h ||
                cfg->cx >= f.img_w || cfg->cy >= f.img_h ||
                (seen && (uint16_t)(f.sequence - previous_sequence) != 1u)) {
                issues++; break;
            }
            seen = 1; previous_sequence = f.sequence; last_frame = now_ms;
            if (x_bad || y_bad) {
                float v = 0.6f * (x_bad ? ex * (float)cfg->x_sign : ey * (float)cfg->y_sign);
                if (v > 80.0f) v = 80.0f;
                if (v < -80.0f) v = -80.0f;
                if (fabsf(v) < 12.0f) v = v > 0.0f ? 12.0f : -12.0f;
                good = 0u; moving = 1;
                motion_vel_set_precise(x_bad ? v : 0.0f, x_bad ? 0.0f : v, 0.0f);
            } else {
                motion_brake();
                if (moving) { still_at = now_ms; good = 0u; moving = 0; }
                if (++good >= GRAB_XY_GOOD_FRAMES && now_ms - still_at >= GRAB_XY_STILL_MS) {
                    step_vision_receive_end(); return 1;
                }
            }
        } else if (now_ms - last_frame > GRAB_XY_FRESH_MS) {
            motion_brake(); good = 0u; moving = 0;
        }
        osDelay(5u);
    }
    motion_brake(); return 0;
}
int step_fire(uint32_t ms)
{
    int saved = in_context;
    uint32_t started = now_ms;
    if (ms != 2000u || !is_phase("TARGET_LASER") || target_settle_calls != 1u ||
        now_ms - target_settle_started != 1000u || target_settle_finished != now_ms ||
        cmd_fore != 0.0f || cmd_lat != 0.0f || cmd_w != 0.0f) issues++;
    if (aborted) return 0;
    laser_started = now_ms;
    fire_calls++; bp_laser_set(1);
    in_context = IN_LASER; context_at = now_ms;
    while (!aborted && now_ms - started < ms) osDelay(1u);
    bp_laser_set(0); laser_finished = now_ms; in_context = saved;
    return !aborted;
}

int step_target_settle(void)
{
    int saved = in_context;
    uint32_t started = now_ms;
    observe();
    if (!is_phase("TARGET_SETTLE1S")) issues++;
    target_settle_calls++;
    target_settle_started = now_ms;
    bp_laser_set(0); motion_brake();
    if (aborted) return 0;
    in_context = IN_TARGET_SETTLE; context_at = now_ms;
    while (!aborted && now_ms - started < 1000u) osDelay(1u);
    target_settle_finished = now_ms;
    in_context = saved;
    return !aborted;
}

static void reset_fixture(void)
{
    memset(events, 0, sizeof events); memset(holds, 0, sizeof holds);
    memset(align_signs, 0, sizeof align_signs); memset(last_report, 0, sizeof last_report);
    memset(last_split_report, 0, sizeof last_split_report); memset(return_commands, 0, sizeof return_commands);
    memset(align_anchors, 0, sizeof align_anchors);
    memset(grab_align_calls, 0, sizeof grab_align_calls); grab_y_locks = 0u;
    memset(scenes, 0, sizeof scenes); memset(target_requests, 0, sizeof target_requests);
    target_request_count = 0u; qr_digits[0] = 2; qr_digits[1] = 3; qr_digits[2] = 1;
    vision_abort_marker = -1;
    event_count = issues = qr_reads = commands = tuning_locks = hold_count = fire_calls = laser_on_calls = scene_count = 0u;
    target_settle_calls = 0u;
    target_settle_started = target_settle_finished = laser_started = laser_finished = 0u;
    now_ms = phase_at = context_at = abort_at = 0u;
    fore_mm = lat_mm = cmd_fore = cmd_lat = cmd_w = 0.0f; heading_deg = 37.0f;
    abort_fore = abort_lat = abort_w = 0.0f;
    aborted = laser_on = all_missing = 0; in_context = IN_NONE; scene_in_progress = -1;
    abort_name = 0; abort_context = abort_kind = 0;
    receive_open = 1; receive_end_calls = forbidden_scene_calls = 0u; /* boot QR may be active */
    selected_cls = selected_label = -1; align_index = positive_180 = first_turn_seen = 0u;
    first_leg_locks = qr_prepared = qr_ready = 0u; sensor_ok = 1;
    task_errors[0] = 60; task_errors[1] = -30;
    settle_shift_mm = 0.0f; ball_at_mm = 300.0f; target_at_mm = 800.0f; hostage_at_mm = 400.0f;
    task_exact_heading = 0; qr_valid_after_ms = qr_request_at = 0u;
    qr_precached = qr_request_failed = 0;
    mission_trial_init();
    for (int cls = 0; cls < 4; ++cls)
        if (!mission_trial_set_alignment(cls, 200 + cls * 10, 1)) issues++;
    if (!mission_trial_set_grab_y(CLS_BALL, 150, 1) ||
        !mission_trial_set_grab_y(CLS_HOSTAGE, 170, 0)) issues++;
    if (!mission_trial_set_first_leg(SYNTHETIC_FIRST_MM)) issues++;
}

static int test_success(void)
{
    static const char *const order[] = { "R1_LEFT500", "QR_WAIT", "QR_VALID", "R2_BACK600", "R3_LEFT95", "R4_CROSS_ROAD750",
        "R5_LEFT730", "R6_FORWARD830", "R7_RIGHT85", "BALL_SINGLE_PASS", "BALL_ALIGN", "BALL_HOLD10S", "BALL_TURN180",
        "BUCKET_ALIGN", "BUCKET_HOLD10S", "BUCKET_TURN180_BACK", "BUCKET_LEG_START", "TARGET_SINGLE_PASS", "TARGET_ALIGN", "TARGET_SETTLE1S", "TARGET_LASER",
        "TARGET_RETURN", "BUCKET_TO_CORNER_REMAINDER", "R9_RIGHT85", "HOSTAGE_SINGLE_PASS", "HOSTAGE_ALIGN", "HOSTAGE_HOLD10S",
        "HOSTAGE_RETURN", "RESCUE_REMAINDER2125", "ROUTE_END" };
    const Event *a, *b;
    int32_t qr[3];
    reset_fixture();
    CHECK(mission_trial_run() == 1);
    CHECK(issues == 0u && !aborted && !laser_on);
    CHECK(event_count == sizeof order / sizeof order[0]);
    for (unsigned i = 0u; i < event_count; ++i) CHECK(strcmp(events[i].name, order[i]) == 0);
    CHECK(qr_reads >= 2u && fire_calls == 1u && laser_on_calls == 1u && positive_180 == 2u);
    CHECK(TARGET_AIM_SETTLE_MS == 1000u && TARGET_LASER_ON_MS == 2000u);
    CHECK(target_settle_calls == 1u && target_settle_finished - target_settle_started == 1000u);
    CHECK(laser_started == target_settle_finished && laser_finished - laser_started == 2000u);
    mission_trial_get_qr(qr); CHECK(qr[0] == 2 && qr[1] == 3 && qr[2] == 1);
    CHECK(scene_count == 5u && scenes[0] == SCENE_QR && scenes[1] == SCENE_EOD
        && scenes[2] == VISION_BUCKET_MARKER && scenes[3] == SCENE_ANTI && scenes[4] == SCENE_RESCUE
        && forbidden_scene_calls == 0u && !receive_open && receive_end_calls >= 5u);
    CHECK(target_request_count == 4u);
    CHECK(target_requests[0].task == PROTO_TASK_BALL && target_requests[0].digit == 2u);
    CHECK(target_requests[1].task == PROTO_TASK_BUCKET && target_requests[1].digit == 0u);
    CHECK(target_requests[2].task == PROTO_TASK_TARGET && target_requests[2].digit == 3u);
    CHECK(target_requests[3].task == PROTO_TASK_HOSTAGE && target_requests[3].digit == 1u);
    CHECK(target_requests[0].positive_180_before == 0u && target_requests[1].positive_180_before == 1u);
    CHECK(target_requests[2].positive_180_before == 2u && target_requests[3].positive_180_before == 2u);
    CHECK(target_requests[1].at > event("BALL_TURN180")->at
        && target_requests[1].at < event("BUCKET_ALIGN")->at
        && close_value(target_requests[1].heading, 207.0f, 0.31f));
    CHECK(hold_count == 3u && tuning_locks > 15u && first_leg_locks == tuning_locks);
    CHECK(grab_y_locks == tuning_locks && grab_align_calls[CLS_BALL] == 1u &&
        grab_align_calls[CLS_HOSTAGE] == 1u && !grab_align_calls[CLS_TARGET] && !grab_align_calls[CLS_BUCKET]);
    for (unsigned i = 0u; i < 3u; ++i) CHECK(holds[i].duration == 10000u);
    for (unsigned i = 0u; i < 4u; ++i) CHECK(align_signs[i] == 3u);
    CHECK(strcmp(holds[0].name, "BALL_HOLD10S") == 0 && strcmp(holds[1].name, "BUCKET_HOLD10S") == 0
        && strcmp(holds[2].name, "HOSTAGE_HOLD10S") == 0);
    CHECK(strstr(last_report, "hits=7") && strstr(last_report, "no_arm=1 no_retry=1"));
    CHECK(strstr(last_split_report, "first=1000 second=1450 bucket_anchor=VISION"));
    a = event("R1_LEFT500"); b = event("QR_WAIT"); CHECK(a && b && close_value(b->lat - a->lat, -500.0f, 0.7f));
    CHECK(close_value(b->road, 500.0f, 0.7f) && close_value(b->total, 500.0f, 0.01f));
    a = event("R2_BACK600"); b = event("R3_LEFT95"); CHECK(a && b && close_value(b->fore - a->fore, -600.0f, 0.7f));
    CHECK(close_value(b->road, 600.0f, 0.7f) && close_value(b->total, 600.0f, 0.01f));
    a = event("R4_CROSS_ROAD750"); b = event("R5_LEFT730"); CHECK(a && b && close_value(b->fore - a->fore, 750.0f, 0.7f));
    CHECK(close_value(b->road, 750.0f, 0.7f) && close_value(b->total, 750.0f, 0.01f));
    a = event("R5_LEFT730"); b = event("R6_FORWARD830"); CHECK(a && b && close_value(b->lat - a->lat, -730.0f, 0.7f));
    CHECK(close_value(b->road, 730.0f, 0.7f) && close_value(b->total, 730.0f, 0.01f));
    a = event("R6_FORWARD830"); b = event("R7_RIGHT85"); CHECK(a && b && close_value(b->fore - a->fore, 830.0f, 0.7f));
    CHECK(close_value(b->road, 830.0f, 0.7f) && close_value(b->total, 830.0f, 0.01f));
    a = event("QR_WAIT"); b = event("R2_BACK600"); CHECK(a && b && b->at - a->at < 10000u
        && close_value(b->fore, a->fore, 0.001f) && close_value(b->lat, a->lat, 0.001f));
    a = event("BALL_HOLD10S"); b = event("BALL_TURN180"); CHECK(a && b && b->at - a->at >= 10000u);
    CHECK(close_value(a->road, b->road, 0.001f) && close_value(a->fore, b->fore, 0.001f) && close_value(a->lat, b->lat, 0.001f));
    a = event("TARGET_SETTLE1S"); b = event("TARGET_LASER");
    CHECK(a && b && b->at - a->at == 1000u);
    CHECK(close_value(a->road, b->road, 0.001f) && close_value(a->fore, b->fore, 0.001f) && close_value(a->lat, b->lat, 0.001f));
    a = event("TARGET_LASER"); b = event("TARGET_RETURN");
    CHECK(a && b && b->at - a->at == 2000u && b->at == laser_finished);
    CHECK(close_value(a->road, b->road, 0.001f) && close_value(a->fore, b->fore, 0.001f) && close_value(a->lat, b->lat, 0.001f));
    a = event("BUCKET_HOLD10S"); b = event("BUCKET_TURN180_BACK"); CHECK(a && b && b->at - a->at >= 10000u);
    CHECK(close_value(a->road, b->road, 0.001f) && close_value(a->fore, b->fore, 0.001f) && close_value(a->lat, b->lat, 0.001f));
    a = event("HOSTAGE_HOLD10S"); b = event("HOSTAGE_RETURN"); CHECK(a && b && b->at - a->at >= 10000u);
    CHECK(close_value(a->road, b->road, 0.001f) && close_value(a->fore, b->fore, 0.001f) && close_value(a->lat, b->lat, 0.001f));
    a = event("BUCKET_ALIGN"); b = event("BUCKET_HOLD10S"); CHECK(a && b && close_value(b->fore - a->fore, 1.8f, 0.06f) && close_value(b->road - a->road, -1.8f, 0.06f));
    a = event("BUCKET_LEG_START"); b = event("TARGET_SINGLE_PASS");
    CHECK(a && b && close_value(a->total, 2450.0f, 0.01f));
    CHECK(close_value(b->road, 0.0f, 0.001f) && close_value(b->total, SYNTHETIC_SECOND_MM, 0.01f)
        && close_value(b->fore, a->fore, 0.001f) && close_value(b->lat, a->lat, 0.001f));
    CHECK(event("BALL_SINGLE_PASS") && close_value(event("BALL_SINGLE_PASS")->total, 2450.0f, 0.01f));
    a = event("BUCKET_TO_CORNER_REMAINDER"); b = event("R9_RIGHT85"); CHECK(a && b && a->road > 801.0f && a->road < 803.0f
        && close_value(b->road, SYNTHETIC_SECOND_MM, 0.7f) && close_value(b->total, SYNTHETIC_SECOND_MM, 0.01f));
    CHECK(b->fore - a->fore < 700.0f); /* no new full-length leg after alignment */
    CHECK(return_commands[0] == 0u && return_commands[1] == 0u); /* net forward: no backward return */
    a = event("RESCUE_REMAINDER2125"); b = event("ROUTE_END"); CHECK(a && b && a->road > 401.0f && a->road < 403.0f && close_value(b->road, 2125.0f, 0.7f) && close_value(b->total, 2125.0f, 0.01f));
    CHECK(b->fore - a->fore < 1800.0f && close_value(b->heading, 472.0f, 0.31f));
    CHECK(mission_trial_set_alignment(0, 200, 1)); /* unlocked only after run */
    CHECK(mission_trial_set_grab_y(CLS_BALL, 150, 1));
    CHECK(mission_trial_set_first_leg(1001u));
    return 0;
}

static int test_all_qr_target_requests(void)
{
    for (int ball = 1; ball <= 3; ++ball)
        for (int target = 1; target <= 3; ++target)
            for (int hostage = 1; hostage <= 3; ++hostage) {
                int32_t latched[3];
                reset_fixture();
                qr_digits[0] = ball; qr_digits[1] = target; qr_digits[2] = hostage;
                CHECK(mission_trial_run() == 1 && issues == 0u);
                mission_trial_get_qr(latched);
                CHECK(latched[0] == ball && latched[1] == target && latched[2] == hostage);
                CHECK(target_request_count == 4u && scene_count == 5u && scenes[0] == SCENE_QR);
                CHECK(target_requests[0].task == PROTO_TASK_BALL && target_requests[0].digit == ball);
                CHECK(target_requests[1].task == PROTO_TASK_BUCKET && target_requests[1].digit == 0u);
                CHECK(target_requests[2].task == PROTO_TASK_TARGET && target_requests[2].digit == target);
                CHECK(target_requests[3].task == PROTO_TASK_HOSTAGE && target_requests[3].digit == hostage);
                CHECK(target_requests[0].at < target_requests[1].at
                    && target_requests[1].at < target_requests[2].at
                    && target_requests[2].at < target_requests[3].at);
                CHECK(target_requests[1].positive_180_before == 1u
                    && target_requests[1].at > event("BALL_TURN180")->at
                    && target_requests[1].at < event("BUCKET_ALIGN")->at
                    && close_value(target_requests[1].heading, 207.0f, 0.31f));
                CHECK(positive_180 == 2u && hold_count == 3u && fire_calls == 1u && !laser_on);
            }
    return 0;
}

static int test_no_objects(void)
{
    reset_fixture(); all_missing = 1;
    CHECK(mission_trial_run() == 1 && issues == 0u);
    CHECK(!event("BALL_ALIGN") && !event("BUCKET_ALIGN") && !event("TARGET_ALIGN") && !event("HOSTAGE_ALIGN"));
    CHECK(!event("BALL_TURN180") && !event("BUCKET_TURN180_BACK") && !event("TARGET_SETTLE1S") && !event("TARGET_LASER"));
    CHECK(hold_count == 0u && fire_calls == 0u && positive_180 == 0u && strstr(last_report, "hits=0"));
    CHECK(target_request_count == 3u && scene_count == 4u);
    CHECK(target_requests[0].task == PROTO_TASK_BALL && target_requests[1].task == PROTO_TASK_TARGET
        && target_requests[2].task == PROTO_TASK_HOSTAGE); /* No invented bucket request without a ball turn. */
    CHECK(!event("BUCKET_LEG_START") && event("BUCKET_ANCHOR_MISSING")
        && close_value(event("BUCKET_ANCHOR_MISSING")->road, 2450.0f, 0.7f));
    CHECK(strstr(last_split_report, "bucket_anchor=MISSING"));
    CHECK(event("TARGET_SINGLE_PASS") && close_value(event("TARGET_SINGLE_PASS")->road, 2450.0f, 0.7f)
        && close_value(event("TARGET_SINGLE_PASS")->total, 2450.0f, 0.01f));
    CHECK(!event("TARGET_RETURN") && !event("HOSTAGE_RETURN"));
    CHECK(event("R9_RIGHT85") && close_value(event("R9_RIGHT85")->road, 2450.0f, 0.7f));
    CHECK(event("ROUTE_END") && close_value(event("ROUTE_END")->road, 2125.0f, 0.7f));
    return 0; /* no back-and-forth retry and no invented target completion */
}

static int test_ball_beyond_common_bucket(void)
{
    const Event *bucket, *second, *end;
    reset_fixture(); ball_at_mm = (float)SYNTHETIC_FIRST_MM + 100.0f;
    CHECK(mission_trial_run() == 1 && issues == 0u);
    CHECK(event("BALL_ALIGN") && event("BALL_ALIGN")->road >= ball_at_mm);
    bucket = event("BUCKET_LEG_START"); second = event("TARGET_SINGLE_PASS"); end = event("R9_RIGHT85");
    CHECK(bucket && second && end && !event("BUCKET_ANCHOR_MISSING"));
    CHECK(close_value(second->fore, bucket->fore, 0.001f) && close_value(second->road, 0.0f, 0.001f));
    CHECK(close_value(second->total, SYNTHETIC_SECOND_MM, 0.01f)
        && close_value(end->road, SYNTHETIC_SECOND_MM, 0.7f));
    CHECK(close_value(end->fore - second->fore, SYNTHETIC_SECOND_MM, 0.7f));
    return 0;
}

static int test_net_alignment_returns(void)
{
    /* 100 ms per correction in the synthetic plant. Include both directions,
     * cancelling travel, and cases where net backward travel is smaller than
     * the backward component. Only that net deficit may be restored. */
    static const struct { int first, second; float net_mm; } cases[] = {
        {60, 0, 3.6f}, {-60, 0, -3.6f}, {60, -30, 1.8f}, {-60, 30, -1.8f},
        {60, -60, 0.0f}, {30, -60, -1.8f}
    };
    static const char *const task_phase[2] = { "TARGET_LASER", "HOSTAGE_HOLD10S" };
    static const char *const return_phase[2] = { "TARGET_RETURN", "HOSTAGE_RETURN" };
    static const char *const remainder_phase[2] = { "BUCKET_TO_CORNER_REMAINDER", "RESCUE_REMAINDER2125" };
    for (unsigned i = 0u; i < sizeof cases / sizeof cases[0]; ++i) {
        reset_fixture(); task_errors[0] = cases[i].first; task_errors[1] = cases[i].second;
        /* Exact planned headings isolate signed displacement in this fixture;
         * the final case adds synthetic braking drift before the anchor. */
        task_exact_heading = 1; settle_shift_mm = i == 5u ? 0.75f : 0.0f;
        CHECK(mission_trial_run() == 1 && issues == 0u);
        for (unsigned j = 0u; j < 2u; ++j) {
            int cls = j ? CLS_HOSTAGE : CLS_TARGET;
            const Event *task = event(task_phase[j]), *ret = event(return_phase[j]), *rest = event(remainder_phase[j]);
            CHECK(task && ret && rest && close_value(task->road - align_anchors[cls], cases[i].net_mm, 0.06f));
            CHECK(close_value(ret->road, task->road, 0.001f) && close_value(rest->total, task->total, 0.001f));
            CHECK(ret->at - task->at >= (j ? 10000u : 2000u)); /* task done before return */
            if (cases[i].net_mm < 0.0f) {
                CHECK(return_commands[j] > 0u && rest->road >= align_anchors[cls]);
                CHECK(close_value(rest->road, align_anchors[cls], 0.7f));
                CHECK(close_value(rest->fore - ret->fore, -cases[i].net_mm, 0.7f));
            } else {
                CHECK(return_commands[j] == 0u && close_value(rest->road, ret->road, 0.001f));
            }
        }
        CHECK(event("R9_RIGHT85") && close_value(event("R9_RIGHT85")->road, SYNTHETIC_SECOND_MM, 0.7f));
        CHECK(event("ROUTE_END") && close_value(event("ROUTE_END")->road, 2125.0f, 0.7f));
    }
    return 0;
}

static int test_qr_wait_without_fixed_hold(void)
{
    const Event *qr, *back;
    reset_fixture(); qr_valid_after_ms = 15000u;
    CHECK(mission_trial_run() == 1 && issues == 0u);
    qr = event("QR_WAIT"); back = event("R2_BACK600");
    CHECK(qr && back && back->at - qr->at >= 15000u && back->at - qr->at < 15100u);
    CHECK(qr_reads > 2900u && close_value(qr->fore, back->fore, 0.001f) && close_value(qr->lat, back->lat, 0.001f));
    CHECK(hold_count == 3u && strcmp(holds[0].name, "BALL_HOLD10S") == 0);
    return 0;
}

static int test_qr_precached_and_request_failure(void)
{
    const Event *qr, *back;
    reset_fixture(); qr_precached = 1;
    CHECK(mission_trial_run() == 1 && issues == 0u);
    qr = event("QR_WAIT"); back = event("R2_BACK600");
    CHECK(qr && back && back->at - qr->at == 20u && qr_reads == 1u);
    CHECK(close_value(qr->fore, back->fore, 0.001f) && close_value(qr->lat, back->lat, 0.001f));
    CHECK(event("QR_VALID") && hold_count == 3u && target_settle_calls == 1u);
    reset_fixture(); qr_request_failed = 1;
    CHECK(mission_trial_run() == 0 && issues == 0u && aborted);
    CHECK(event("QR_WAIT") && !event("QR_VALID") && !event("R2_BACK600") && qr_reads == 0u);
    CHECK(is_phase("STOP") && !receive_open && forbidden_scene_calls == 0u && !laser_on && !fire_calls);
    CHECK(cmd_fore == 0.0f && cmd_lat == 0.0f && cmd_w == 0.0f);
    return 0;
}

static int test_return_stops_at_road_end(void)
{
    static const char *const names[2] = { "TARGET_RETURN", "HOSTAGE_RETURN" };
    static const char *const next[2] = { "BUCKET_TO_CORNER_REMAINDER", "RESCUE_REMAINDER2125" };
    reset_fixture(); task_exact_heading = 1; settle_shift_mm = 2.0f;
    target_at_mm = SYNTHETIC_SECOND_MM - 1.0f; hostage_at_mm = 2124.0f;
    task_errors[0] = -60; task_errors[1] = 0;
    CHECK(mission_trial_run() == 1 && issues == 0u);
    for (unsigned i = 0u; i < 2u; ++i) {
        int cls = i ? CLS_HOSTAGE : CLS_TARGET;
        const Event *ret = event(names[i]), *rest = event(next[i]);
        CHECK(ret && rest && align_anchors[cls] > ret->total && ret->road < ret->total);
        CHECK(return_commands[i] > 0u && rest->road >= rest->total);
        CHECK(close_value(rest->road, rest->total, 0.7f) && rest->road < align_anchors[cls]);
        CHECK(close_value(rest->fore - ret->fore, ret->total - ret->road, 0.7f));
    }
    return 0; /* stop-loop sampling can overshoot; commanded target is capped */
}

static int test_cancellations(void)
{
    static const struct { const char *name; int context; int scene; } cases[] = {
        {"BALL_HOLD10S", IN_WAIT, 0}, {"BUCKET_HOLD10S", IN_WAIT, 0}, {"HOSTAGE_HOLD10S", IN_WAIT, 0},
        {"R3_LEFT95", IN_TURN, 0}, {"R7_RIGHT85", IN_TURN, 0}, {"BALL_TURN180", IN_TURN, 0}, {"BUCKET_TURN180_BACK", IN_TURN, 0}, {"R9_RIGHT85", IN_TURN, 0},
        {"QR", IN_VISION, SCENE_QR}, {"EOD", IN_VISION, SCENE_EOD}, {"BUCKET", IN_VISION, VISION_BUCKET_MARKER},
        {"ANTI", IN_VISION, SCENE_ANTI}, {"RESCUE", IN_VISION, SCENE_RESCUE},
        {"BALL_ALIGN", IN_ALIGN, 0}, {"BUCKET_ALIGN", IN_ALIGN, 0}, {"TARGET_ALIGN", IN_ALIGN, 0}, {"HOSTAGE_ALIGN", IN_ALIGN, 0},
        {"BALL_SINGLE_PASS", IN_PASS, 0}, {"TARGET_SINGLE_PASS", IN_PASS, 0}, {"HOSTAGE_SINGLE_PASS", IN_PASS, 0},
        {"TARGET_SETTLE1S", IN_TARGET_SETTLE, 0}, {"TARGET_LASER", IN_LASER, 0}, {"QR_WAIT", IN_QR, 0},
        {"TARGET_RETURN", IN_RETURN, 0}, {"HOSTAGE_RETURN", IN_RETURN, 0},
        {"R1_LEFT500", IN_TRANSLATE, 0}, {"R2_BACK600", IN_TRANSLATE, 0}, {"R4_CROSS_ROAD750", IN_TRANSLATE, 0},
        {"R5_LEFT730", IN_TRANSLATE, 0}, {"R6_FORWARD830", IN_TRANSLATE, 0},
        {"BUCKET_TO_CORNER_REMAINDER", IN_PASS, 0}, {"RESCUE_REMAINDER2125", IN_PASS, 0},
        {"R1_LEFT500", IN_PREP, 0}, {"QR_WAIT", IN_PREP, 0}, {"BALL_TURN180", IN_PREP, 0}, {"BUCKET_ALIGN", IN_PREP, 0}
    };
    for (unsigned i = 0u; i < sizeof cases / sizeof cases[0]; ++i) {
        for (int kind = 0; kind < 2; ++kind) {
            reset_fixture(); abort_name = cases[i].name; abort_context = cases[i].context; abort_kind = kind;
            if (abort_context == IN_RETURN) { task_errors[0] = -60; task_errors[1] = 30; }
            if (abort_context == IN_VISION) vision_abort_marker = cases[i].scene;
            CHECK(mission_trial_run() == 0 && aborted && issues == 0u);
            CHECK(abort_at != 0u && now_ms - abort_at <= 5u);
            if (abort_context == IN_TURN) CHECK(fabsf(abort_w) > 0.0f);
            if (abort_context == IN_ALIGN || abort_context == IN_PASS || abort_context == IN_TRANSLATE || abort_context == IN_RETURN)
                CHECK(fabsf(abort_fore) + fabsf(abort_lat) > 0.0f);
            if (abort_context == IN_WAIT || abort_context == IN_QR || abort_context == IN_VISION || abort_context == IN_PREP || abort_context == IN_TARGET_SETTLE)
                CHECK(abort_fore == 0.0f && abort_lat == 0.0f && abort_w == 0.0f);
            CHECK(strcmp(mission_trial_phase(), "STOP") == 0 && !receive_open && forbidden_scene_calls == 0u && !laser_on);
            CHECK(cmd_fore == 0.0f && cmd_lat == 0.0f && cmd_w == 0.0f);
            CHECK(!event("ROUTE_END") && mission_trial_set_alignment(0, 200, 1));
            CHECK(mission_trial_set_first_leg(1001u));
            if (strcmp(abort_name, "TARGET_LASER") == 0) CHECK(!event("TARGET_RETURN"));
            if (strcmp(abort_name, "TARGET_SETTLE1S") == 0)
                CHECK(!event("TARGET_LASER") && !event("TARGET_RETURN") && fire_calls == 0u && laser_on_calls == 0u);
            if (strcmp(abort_name, "HOSTAGE_HOLD10S") == 0) CHECK(!event("HOSTAGE_RETURN"));
            if (strcmp(abort_name, "TARGET_RETURN") == 0) CHECK(!event("BUCKET_TO_CORNER_REMAINDER"));
            if (strcmp(abort_name, "HOSTAGE_RETURN") == 0) CHECK(!event("RESCUE_REMAINDER2125"));
        }
    }
    return 0;
}

static int test_return_imu_failure(void)
{
    static const char *const names[2] = { "TARGET_RETURN", "HOSTAGE_RETURN" };
    for (unsigned i = 0u; i < 2u; ++i) {
        reset_fixture(); task_errors[0] = -60; task_errors[1] = 30;
        abort_name = names[i]; abort_context = IN_RETURN; abort_kind = 2;
        CHECK(mission_trial_run() == 0 && issues == 0u && !sensor_ok && !aborted);
        CHECK(abort_at && now_ms - abort_at <= 5u && abort_fore > 0.0f);
        CHECK(is_phase("STOP") && !laser_on && cmd_fore == 0.0f && cmd_lat == 0.0f && cmd_w == 0.0f);
        CHECK(!event("ROUTE_END") && !event(i ? "RESCUE_REMAINDER2125" : "BUCKET_TO_CORNER_REMAINDER"));
    }
    return 0;
}

static int test_configuration_and_pending_abort(void)
{
    reset_fixture(); mission_trial_init();
    CHECK(strcmp(mission_trial_config_missing(), "vsg1_or_vsg2") == 0);
    CHECK(!mission_trial_set_alignment(0, 65535, 1) && !mission_trial_set_alignment(4, 200, 1) && !mission_trial_set_alignment(0, 200, 2));
    CHECK(mission_trial_set_alignment(CLS_BALL, 480, 0)); /* Actual frame width decides validity at alignment. */
    CHECK(!mission_trial_set_grab_y(CLS_TARGET, 150, 1) &&
        !mission_trial_set_grab_y(CLS_BUCKET, 150, 1) && !mission_trial_set_grab_y(-2, 150, 1) &&
        !mission_trial_set_grab_y(CLS_BALL, -2, 1) && !mission_trial_set_grab_y(CLS_BALL, 65535, 1) &&
        !mission_trial_set_grab_y(CLS_BALL, 150, 2));
    CHECK(mission_trial_run() == 0 && commands == 0u);
    CHECK(mission_trial_set_alignment(-1, -1, -1));
    CHECK(strcmp(mission_trial_config_missing(), "tcx") == 0);
    for (int i = 0; i < 4; ++i) CHECK(mission_trial_set_alignment(i, 200 + i * 10, 0));
    CHECK(strcmp(mission_trial_config_missing(), "b1d") == 0);
    mission_trial_report(); CHECK(strstr(last_split_report, "first=0 second=0 bucket_anchor=UNSET"));
    CHECK(mission_trial_run() == 0 && commands == 0u && scene_count == 0u);
    CHECK(!mission_trial_set_first_leg(0u) && !mission_trial_set_first_leg(2450u) && !mission_trial_set_first_leg(65535u));
    CHECK(mission_trial_set_first_leg(1u)); mission_trial_report(); CHECK(strstr(last_split_report, "first=1 second=2449"));
    CHECK(mission_trial_set_first_leg(2449u)); mission_trial_report(); CHECK(strstr(last_split_report, "first=2449 second=1"));
    CHECK(strcmp(mission_trial_config_missing(), "ysg1_or_ysg2") == 0);
    CHECK(mission_trial_run() == 0 && commands == 0u && scene_count == 0u);
    CHECK(mission_trial_set_grab_y(-1, -1, 1));
    CHECK(strcmp(mission_trial_config_missing(), "bcy") == 0);
    CHECK(mission_trial_run() == 0 && commands == 0u && scene_count == 0u);
    CHECK(mission_trial_set_grab_y(CLS_BALL, 150, 0));
    CHECK(strcmp(mission_trial_config_missing(), "hcy") == 0);
    CHECK(mission_trial_run() == 0 && commands == 0u && scene_count == 0u);
    CHECK(mission_trial_set_grab_y(CLS_HOSTAGE, 170, 0));
    CHECK(mission_trial_set_grab_y(-1, -1, -1)); /* Both polarity routes are accepted in RAM. */
    CHECK(mission_trial_config_missing() == 0);
    run_abort(); CHECK(mission_trial_run() == 0 && commands == 0u && scene_count == 0u && now_ms == 0u);
    return 0;
}

int main(void)
{
    CHECK(test_success() == 0);
    CHECK(test_all_qr_target_requests() == 0);
    CHECK(test_no_objects() == 0);
    CHECK(test_ball_beyond_common_bucket() == 0);
    CHECK(test_net_alignment_returns() == 0);
    CHECK(test_qr_wait_without_fixed_hold() == 0);
    CHECK(test_qr_precached_and_request_failure() == 0);
    CHECK(test_return_stops_at_road_end() == 0);
    CHECK(test_cancellations() == 0);
    CHECK(test_return_imu_failure() == 0);
    CHECK(test_configuration_and_pending_abort() == 0);
    puts("mission trial flow: real runner/ledger, BALL/HOSTAGE shared XY config and running lock, 27 QR task selections and 4 ordered target requests (bucket only after +180), cached/new QR valid wait and request failure, 3 arm holds, target stopped1s then laser2s/off before return, common-bucket split, signed net returns, capped road-end return, 72 stop injections, 2 return IMU failures and X/Y config gates passed (synthetic host only)");
    return 0;
}
