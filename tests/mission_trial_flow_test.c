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

typedef struct {
    char name[40];
    uint32_t at;
    /* Phase entry is before the next road_start(), so road/total describe
     * the completed previous leg at that boundary, not the new leg. */
    float fore, lat, heading, road, total;
} Event;
typedef struct { const char *name; uint32_t duration; } Hold;
enum { IN_NONE, IN_WAIT, IN_TURN, IN_ALIGN, IN_PASS, IN_VISION, IN_LASER, IN_PREP, IN_TRANSLATE, IN_QR };

static Event events[MAX_EVENTS];
static unsigned event_count, issues, qr_reads, commands, tuning_locks;
static Hold holds[8];
static unsigned hold_count, fire_calls, laser_on_calls;
static ProtoScene scenes[8], idle_scene;
static unsigned scene_count;
static uint32_t now_ms, phase_at, abort_at, context_at;
static float fore_mm, lat_mm, heading_deg, cmd_fore, cmd_lat, cmd_w;
static int aborted, laser_on, all_missing, in_context, scene_in_progress;
static const char *abort_name;
static int abort_context, abort_kind, selected_cls, selected_label;
static unsigned align_index, align_signs[4], first_turn_seen, positive_180;
static float abort_fore, abort_lat, abort_w;
static uint32_t next_frame_at;
static char last_report[180];

static int is_phase(const char *name) { return strcmp(mission_trial_phase(), name) == 0; }
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
uint8_t imu_ok(void) { return 1u; }
float imu_heading_deg(void) { return heading_deg; }
float motion_odo_mm(void) { return fore_mm; }
float motion_lateral_odo_mm(void) { return lat_mm; }
float step_heading_kp_deg(void) { return 0.3f; }
float test_forward_ff_ratio(void) { return 0.0125f; }
int run_aborted(void) { return aborted; }
void run_abort(void) { aborted = 1; }

void bp_debug_send(const char *s)
{
    observe();
    if (strstr(s, "hits=")) snprintf(last_report, sizeof last_report, "%s", s);
}
void bp_laser_set(int on)
{
    observe();
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
    if (strstr(name, "ALIGN")) {
        if (!precise || selected_cls < 0 || selected_cls > 3) issues++;
        else if (vx > 0.0f) align_signs[selected_cls] |= 1u;
        else if (vx < 0.0f) align_signs[selected_cls] |= 2u;
        if (fabsf(vy) > 0.001f) issues++;
    }
    if (strstr(name, "TURN") || is_phase("R3_LEFT95") || is_phase("R7_RIGHT85") || is_phase("R9_RIGHT85")) {
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
    if (!abort_name || aborted) return;
    if (abort_context == IN_VISION)
        matched = in_context == IN_VISION && scene_in_progress == (int)idle_scene;
    else matched = strcmp(mission_trial_phase(), abort_name) == 0 && in_context == abort_context;
    if (matched && now_ms - context_at >= 15u) {
        abort_at = now_ms;
        abort_fore = cmd_fore; abort_lat = cmd_lat; abort_w = cmd_w;
        run_abort();
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
            in_context = strstr(p, "ALIGN") ? IN_ALIGN : strstr(p, "PASS") || strstr(p, "REMAINDER") ? IN_PASS
                : strstr(p, "TURN") || is_phase("R3_LEFT95") || is_phase("R7_RIGHT85") || is_phase("R9_RIGHT85") ? IN_TURN : IN_TRANSLATE;
            context_at = phase_at;
        }
        now_ms++;
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
    if (is_phase("BALL_TURN180")) heading_deg = 27.0f - 0.2f;
    if (is_phase("BUCKET_TURN180_BACK")) heading_deg = 207.0f - 0.2f;
    mission_trial_tick_1ms();
    /* Stop settling is mocked, rather than exercising real encoder stillness. */
    osDelay(20u); in_context = saved;
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
int wait_qr(int32_t d[3], uint32_t to)
{
    int saved = in_context;
    if (to != 0u || !is_phase("QR_STOP10S") || !hold_count || holds[0].duration != 10000u) issues++;
    in_context = IN_QR;
    if (qr_reads == 0u) context_at = now_ms;
    osDelay(5u);
    in_context = saved;
    if (aborted) return 0;
    d[0] = 2; d[1] = 3; d[2] = qr_reads++ == 0u || abort_context == IN_QR ? 0 : 1;
    return 1; /* first tuple deliberately invalid: runner must not accept it */
}

int step_vision_scene(ProtoScene scene)
{
    int saved = in_context;
    if (scene_count >= sizeof scenes / sizeof scenes[0]) { issues++; return 0; }
    scenes[scene_count++] = scene;
    in_context = IN_VISION; context_at = now_ms; scene_in_progress = (int)scene;
    motion_brake();
    for (unsigned i = 0u; i < 40u && !aborted; ++i) osDelay(1u);
    in_context = saved;
    return !aborted; /* command/ACK framing is covered by the binary replay suite */
}
void proto_send_scene(ProtoScene scene)
{
    if (scene != SCENE_IDLE) issues++;
    idle_scene = scene;
}

void step_object_select(int cls, int label)
{
    selected_cls = cls; selected_label = label;
    align_index = 0u; next_frame_at = now_ms;
    if ((cls == CLS_BALL && label != LAB_G) || (cls == CLS_TARGET && label != LAB_B)
        || (cls == CLS_HOSTAGE && label != LAB_CYL) || (cls == CLS_BUCKET && label != -1)) issues++;
}
int step_object_take(ProtoFrame *out)
{
    float at = progress();
    if (all_missing) return 0;
    memset(out, 0, sizeof *out);
    out->type = PF_OBJ; out->cls = selected_cls; out->label = selected_label;
    out->conf = 90; out->img_w = 480; out->img_h = 320; out->w = 30; out->h = 40;
    if (strstr(mission_trial_phase(), "ALIGN")) {
        if (now_ms < next_frame_at) return 0;
        /* Two out-of-window frames require +36 then -18 mm/s for 100 ms.
         * Net body-forward displacement is +1.8 mm; bucket at +180 must
         * DECREASE fixed-road progress rather than reset/ignore this motion. */
        out->cx = 200 + selected_cls * 10 + (align_index == 0u ? 60 : align_index == 1u ? -30 : 0);
        out->sequence = (uint16_t)++align_index;
        next_frame_at = now_ms + 100u;
        return 1;
    }
    out->cx = 200 + selected_cls * 10;
    if (is_phase("BALL_SINGLE_PASS")) return at >= 300.0f;
    if (is_phase("TARGET_SINGLE_PASS")) return at >= 800.0f;
    if (is_phase("HOSTAGE_SINGLE_PASS")) return at >= 400.0f;
    return 0;
}
int step_fire(uint32_t ms)
{
    int saved = in_context;
    uint32_t started = now_ms;
    if (ms != 2000u || !is_phase("TARGET_LASER")) issues++;
    fire_calls++; bp_laser_set(1);
    in_context = IN_LASER; context_at = now_ms;
    while (!aborted && now_ms - started < ms) osDelay(1u);
    bp_laser_set(0); in_context = saved;
    return !aborted;
}

static void reset_fixture(void)
{
    memset(events, 0, sizeof events); memset(holds, 0, sizeof holds);
    memset(align_signs, 0, sizeof align_signs); memset(last_report, 0, sizeof last_report);
    event_count = issues = qr_reads = commands = tuning_locks = hold_count = fire_calls = laser_on_calls = scene_count = 0u;
    now_ms = phase_at = context_at = abort_at = 0u;
    fore_mm = lat_mm = cmd_fore = cmd_lat = cmd_w = 0.0f; heading_deg = 37.0f;
    abort_fore = abort_lat = abort_w = 0.0f;
    aborted = laser_on = all_missing = 0; in_context = IN_NONE; scene_in_progress = -1;
    abort_name = 0; abort_context = abort_kind = 0; idle_scene = SCENE_RESCUE;
    selected_cls = selected_label = -1; align_index = positive_180 = first_turn_seen = 0u;
    mission_trial_init();
    for (int cls = 0; cls < 4; ++cls)
        if (!mission_trial_set_alignment(cls, 200 + cls * 10, 1)) issues++;
}

static int test_success(void)
{
    static const char *const order[] = { "R1_LEFT500", "QR_STOP10S", "QR_VALID", "R2_BACK600", "R3_LEFT95", "R4_CROSS_ROAD750",
        "R5_LEFT730", "R6_FORWARD830", "R7_RIGHT85", "BALL_SINGLE_PASS", "BALL_ALIGN", "BALL_HOLD10S", "BALL_TURN180",
        "BUCKET_ALIGN", "BUCKET_HOLD10S", "BUCKET_TURN180_BACK", "TARGET_SINGLE_PASS", "TARGET_ALIGN", "TARGET_LASER",
        "TASK_REMAINDER2450", "R9_RIGHT85", "HOSTAGE_SINGLE_PASS", "HOSTAGE_ALIGN", "HOSTAGE_HOLD10S", "RESCUE_REMAINDER2125", "ROUTE_END" };
    const Event *a, *b;
    int32_t qr[3];
    reset_fixture();
    CHECK(mission_trial_run() == 1);
    CHECK(issues == 0u && !aborted && !laser_on);
    CHECK(event_count == sizeof order / sizeof order[0]);
    for (unsigned i = 0u; i < event_count; ++i) CHECK(strcmp(events[i].name, order[i]) == 0);
    CHECK(qr_reads == 2u && fire_calls == 1u && laser_on_calls == 1u && positive_180 == 2u);
    mission_trial_get_qr(qr); CHECK(qr[0] == 2 && qr[1] == 3 && qr[2] == 1);
    CHECK(scene_count == 4u && scenes[0] == SCENE_QR && scenes[1] == SCENE_EOD && scenes[2] == SCENE_ANTI && scenes[3] == SCENE_RESCUE && idle_scene == SCENE_IDLE);
    CHECK(hold_count == 4u && tuning_locks > 15u);
    for (unsigned i = 0u; i < 4u; ++i) CHECK(holds[i].duration == 10000u && align_signs[i] == 3u);
    CHECK(strstr(last_report, "hits=7") && strstr(last_report, "no_arm=1 no_retry=1"));
    a = event("R1_LEFT500"); b = event("QR_STOP10S"); CHECK(a && b && close_value(b->lat - a->lat, -500.0f, 0.7f));
    CHECK(close_value(b->road, 500.0f, 0.7f) && close_value(b->total, 500.0f, 0.01f));
    a = event("R2_BACK600"); b = event("R3_LEFT95"); CHECK(a && b && close_value(b->fore - a->fore, -600.0f, 0.7f));
    CHECK(close_value(b->road, 600.0f, 0.7f) && close_value(b->total, 600.0f, 0.01f));
    a = event("R4_CROSS_ROAD750"); b = event("R5_LEFT730"); CHECK(a && b && close_value(b->fore - a->fore, 750.0f, 0.7f));
    CHECK(close_value(b->road, 750.0f, 0.7f) && close_value(b->total, 750.0f, 0.01f));
    a = event("R5_LEFT730"); b = event("R6_FORWARD830"); CHECK(a && b && close_value(b->lat - a->lat, -730.0f, 0.7f));
    CHECK(close_value(b->road, 730.0f, 0.7f) && close_value(b->total, 730.0f, 0.01f));
    a = event("R6_FORWARD830"); b = event("R7_RIGHT85"); CHECK(a && b && close_value(b->fore - a->fore, 830.0f, 0.7f));
    CHECK(close_value(b->road, 830.0f, 0.7f) && close_value(b->total, 830.0f, 0.01f));
    a = event("QR_STOP10S"); b = event("R2_BACK600"); CHECK(a && b && b->at - a->at >= 10000u && close_value(b->fore, a->fore, 0.001f) && close_value(b->lat, a->lat, 0.001f));
    a = event("BALL_HOLD10S"); b = event("BALL_TURN180"); CHECK(a && b && b->at - a->at >= 10000u);
    CHECK(close_value(a->road, b->road, 0.001f) && close_value(a->fore, b->fore, 0.001f) && close_value(a->lat, b->lat, 0.001f));
    a = event("BUCKET_HOLD10S"); b = event("BUCKET_TURN180_BACK"); CHECK(a && b && b->at - a->at >= 10000u);
    CHECK(close_value(a->road, b->road, 0.001f) && close_value(a->fore, b->fore, 0.001f) && close_value(a->lat, b->lat, 0.001f));
    a = event("HOSTAGE_HOLD10S"); b = event("RESCUE_REMAINDER2125"); CHECK(a && b && b->at - a->at >= 10000u);
    CHECK(close_value(a->road, b->road, 0.001f) && close_value(a->fore, b->fore, 0.001f) && close_value(a->lat, b->lat, 0.001f));
    a = event("BUCKET_ALIGN"); b = event("BUCKET_HOLD10S"); CHECK(a && b && close_value(b->fore - a->fore, 1.8f, 0.06f) && close_value(b->road - a->road, -1.8f, 0.06f));
    a = event("TASK_REMAINDER2450"); b = event("R9_RIGHT85"); CHECK(a && b && a->road > 801.0f && a->road < 803.0f && close_value(b->road, 2450.0f, 0.7f) && close_value(b->total, 2450.0f, 0.01f));
    CHECK(b->fore - a->fore < 1700.0f); /* not another full 2450 after alignment */
    a = event("RESCUE_REMAINDER2125"); b = event("ROUTE_END"); CHECK(a && b && a->road > 401.0f && a->road < 403.0f && close_value(b->road, 2125.0f, 0.7f) && close_value(b->total, 2125.0f, 0.01f));
    CHECK(b->fore - a->fore < 1800.0f && close_value(b->heading, 472.0f, 0.31f));
    CHECK(mission_trial_set_alignment(0, 200, 1)); /* unlocked only after run */
    return 0;
}

static int test_no_objects(void)
{
    reset_fixture(); all_missing = 1;
    CHECK(mission_trial_run() == 1 && issues == 0u);
    CHECK(!event("BALL_ALIGN") && !event("BUCKET_ALIGN") && !event("TARGET_ALIGN") && !event("HOSTAGE_ALIGN"));
    CHECK(!event("BALL_TURN180") && !event("BUCKET_TURN180_BACK") && !event("TARGET_LASER"));
    CHECK(hold_count == 1u && fire_calls == 0u && positive_180 == 0u && strstr(last_report, "hits=0"));
    CHECK(event("R9_RIGHT85") && close_value(event("R9_RIGHT85")->road, 2450.0f, 0.7f));
    CHECK(event("ROUTE_END") && close_value(event("ROUTE_END")->road, 2125.0f, 0.7f));
    return 0; /* no back-and-forth retry and no invented target completion */
}

static int test_cancellations(void)
{
    static const struct { const char *name; int context; int scene; } cases[] = {
        {"QR_STOP10S", IN_WAIT, 0}, {"BALL_HOLD10S", IN_WAIT, 0}, {"BUCKET_HOLD10S", IN_WAIT, 0}, {"HOSTAGE_HOLD10S", IN_WAIT, 0},
        {"R3_LEFT95", IN_TURN, 0}, {"R7_RIGHT85", IN_TURN, 0}, {"BALL_TURN180", IN_TURN, 0}, {"BUCKET_TURN180_BACK", IN_TURN, 0}, {"R9_RIGHT85", IN_TURN, 0},
        {"QR", IN_VISION, SCENE_QR}, {"EOD", IN_VISION, SCENE_EOD}, {"ANTI", IN_VISION, SCENE_ANTI}, {"RESCUE", IN_VISION, SCENE_RESCUE},
        {"BALL_ALIGN", IN_ALIGN, 0}, {"BUCKET_ALIGN", IN_ALIGN, 0}, {"TARGET_ALIGN", IN_ALIGN, 0}, {"HOSTAGE_ALIGN", IN_ALIGN, 0},
        {"BALL_SINGLE_PASS", IN_PASS, 0}, {"TARGET_SINGLE_PASS", IN_PASS, 0}, {"HOSTAGE_SINGLE_PASS", IN_PASS, 0},
        {"TARGET_LASER", IN_LASER, 0}, {"QR_STOP10S", IN_QR, 0},
        {"R1_LEFT500", IN_TRANSLATE, 0}, {"R2_BACK600", IN_TRANSLATE, 0}, {"R4_CROSS_ROAD750", IN_TRANSLATE, 0},
        {"R5_LEFT730", IN_TRANSLATE, 0}, {"R6_FORWARD830", IN_TRANSLATE, 0},
        {"TASK_REMAINDER2450", IN_PASS, 0}, {"RESCUE_REMAINDER2125", IN_PASS, 0},
        {"R1_LEFT500", IN_PREP, 0}, {"QR_STOP10S", IN_PREP, 0}, {"BALL_TURN180", IN_PREP, 0}, {"BUCKET_ALIGN", IN_PREP, 0}
    };
    for (unsigned i = 0u; i < sizeof cases / sizeof cases[0]; ++i) {
        for (int kind = 0; kind < 2; ++kind) {
            reset_fixture(); abort_name = cases[i].name; abort_context = cases[i].context; abort_kind = kind;
            if (abort_context == IN_VISION) idle_scene = (ProtoScene)cases[i].scene;
            CHECK(mission_trial_run() == 0 && aborted && issues == 0u);
            CHECK(abort_at != 0u && now_ms - abort_at <= 5u);
            if (abort_context == IN_TURN) CHECK(fabsf(abort_w) > 0.0f);
            if (abort_context == IN_ALIGN || abort_context == IN_PASS || abort_context == IN_TRANSLATE)
                CHECK(fabsf(abort_fore) + fabsf(abort_lat) > 0.0f);
            if (abort_context == IN_WAIT || abort_context == IN_QR || abort_context == IN_VISION || abort_context == IN_PREP)
                CHECK(abort_fore == 0.0f && abort_lat == 0.0f && abort_w == 0.0f);
            CHECK(strcmp(mission_trial_phase(), "STOP") == 0 && idle_scene == SCENE_IDLE && !laser_on);
            CHECK(cmd_fore == 0.0f && cmd_lat == 0.0f && cmd_w == 0.0f);
            CHECK(!event("ROUTE_END") && mission_trial_set_alignment(0, 200, 1));
        }
    }
    return 0;
}

static int test_configuration_and_pending_abort(void)
{
    reset_fixture(); mission_trial_init();
    CHECK(strcmp(mission_trial_config_missing(), "vsg1_or_vsg2") == 0);
    CHECK(!mission_trial_set_alignment(0, 480, 1) && !mission_trial_set_alignment(4, 200, 1) && !mission_trial_set_alignment(0, 200, 2));
    CHECK(mission_trial_run() == 0 && commands == 0u);
    CHECK(mission_trial_set_alignment(-1, -1, -1));
    CHECK(strcmp(mission_trial_config_missing(), "bcx") == 0);
    for (int i = 0; i < 4; ++i) CHECK(mission_trial_set_alignment(i, 200 + i * 10, 0));
    CHECK(mission_trial_config_missing() == 0);
    run_abort(); CHECK(mission_trial_run() == 0 && commands == 0u && scene_count == 0u && now_ms == 0u);
    return 0;
}

int main(void)
{
    CHECK(test_success() == 0);
    CHECK(test_no_objects() == 0);
    CHECK(test_cancellations() == 0);
    CHECK(test_configuration_and_pending_abort() == 0);
    puts("mission trial flow: real runner/ledger, full route, QR/4 holds, two +180 directions, signed bucket progress, remaining corridors, missing-object single-pass, 66 stop injections and config gates passed (synthetic host only)");
    return 0;
}
