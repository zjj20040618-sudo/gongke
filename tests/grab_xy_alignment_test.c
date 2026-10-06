/* Host-only coupled BALL/HOSTAGE alignment, using the real steps.c.
 * Build alone with -Itests/stubs -IApp -ffunction-sections -fdata-sections
 * -fno-asynchronous-unwind-tables -fno-unwind-tables -Wl,--gc-sections -lm.
 * These checks exercise decisions/commands, not physical pixel polarity. */
#include "steps.h"
#include "motion.h"
#include "control.h"
#include "imu.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum {
    CONSTANT, CROSS_DISTURBANCE, OVERSHOOT, DUPLICATE, EMPTY_AFTER_GOOD,
    WRONG_AFTER_GOOD, WRONG_LABEL_AFTER_GOOD, SINGLE_MOVING_FRAME,
    OLD_PENDING_FRAME, POST_STOP_DRIFT, DIMENSION_CHANGE, SEQUENCE_WRAP,
    SEQUENCE_GAP, NO_FRAMES, BACKWARD_SEQUENCE, ALTERNATING_IN_BAND
};
typedef struct { uint32_t ms; float x, y, w; } Drive;
static uint32_t now_ms, next_frame_ms, packet_at, abort_at, imu_bad_at;
static uint32_t stopped_at, last_encoder_change, first_drive_stop, coast_until;
static ProtoStats stats;
static int32_t encoder[4];
static Drive drives[4096];
static unsigned drive_count, brake_count, legacy_drives, zero_heading_count;
static unsigned receive_end_count, emitted, mode, phase, last_axis, bad_switches;
static int cls, label, error_x, error_y, origin_y, fixed_seq, frame_limit, packet_done;
static int abort_first_drive, coast_ms, imu_valid, scene_status;
static int reject_runtime_set, runtime_set_attempts, simultaneous;
static uint16_t image_width, image_height, sequence;
static float current_x, current_y, heading;
static void advance_fixture(void);

uint32_t HAL_GetTick(void) { return now_ms; }
void osDelay(uint32_t ms)
{
    now_ms += ms;
    if (current_x || current_y) {
        for (int i = 0; i < 4; ++i) ++encoder[i];
        last_encoder_change = now_ms;
    } else if (coast_until && now_ms <= coast_until) {
        ++encoder[3]; /* A single wheel still turning must reset the hold. */
        last_encoder_change = now_ms;
    }
    if (abort_at && now_ms >= abort_at) run_abort();
    if (imu_bad_at && now_ms >= imu_bad_at) imu_valid = 0;
    advance_fixture();
}
uint8_t imu_ok(void) { return (uint8_t)imu_valid; }
float imu_heading_deg(void) { return heading; }
float imu_leg_heading_deg(void) { return -1000.0f; }
uint8_t imu_zero_leg_heading(void) { ++zero_heading_count; return 1u; }
int32_t ctrl_enc_total(int wheel) { return encoder[wheel]; }
void proto_stats_get(ProtoStats *out) { *out = stats; }
int proto_scene_status(void) { return scene_status; }
void proto_receive_end(void) { ++receive_end_count; }
void bp_debug_send(const char *line) { (void)line; }
void motion_vel_set(float x, float y, float w)
{ (void)x; (void)y; (void)w; ++legacy_drives; }
void motion_brake(void)
{
    ++brake_count;
    if (current_x || current_y) {
        stopped_at = now_ms;
        if (!first_drive_stop) first_drive_stop = now_ms;
        coast_until = now_ms + (uint32_t)coast_ms;
    }
    current_x = current_y = 0.0f;
}
void motion_vel_set_precise(float x, float y, float w)
{
    unsigned axis = x != 0.0f ? 1u : (y != 0.0f ? 2u : 0u);
    if (x && y) simultaneous = 1;
    if (last_axis && axis && last_axis != axis
        && ((current_x || current_y)
            || now_ms - stopped_at < GRAB_XY_STILL_MS
            || now_ms - last_encoder_change < GRAB_XY_STILL_MS)) ++bad_switches;
    if (axis) last_axis = axis;
    if (drive_count < sizeof drives / sizeof drives[0])
        drives[drive_count] = (Drive){now_ms, x, y, w};
    ++drive_count;
    current_x = x; current_y = y;
    if (mode == CROSS_DISTURBANCE) {
        if (phase == 0 && x) phase = 1;
        else if (phase == 1 && y) phase = 2;
        else if (phase == 2 && x) phase = 3;
    } else if (mode == OVERSHOOT) {
        if (phase == 0 && x > 0.0f) phase = 1;
        else if (phase == 1 && x < 0.0f) phase = 2;
    } else if (mode == POST_STOP_DRIFT) {
        if (phase == 0 && x) phase = 1;
        else if (phase == 2 && y) phase = 3;
    }
    ++runtime_set_attempts;
    if (step_grab_alignment_set(cls, 161, 121, 1, 1)) reject_runtime_set = 1;
    if (abort_first_drive) run_abort();
}

#include "../App/steps.c"

#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "grab XY line %d: %s (mode=%u ms=%lu frames=%u drives=%u phase=%u)\n", \
                __LINE__, #expr, mode, (unsigned long)now_ms, emitted, drive_count, phase); \
        return 0; \
    } \
} while (0)

static void feed(int ex, int ey, int object_cls, int object_label)
{
    ProtoFrame f = {0};
    ++stats.obj;
    f.type = PF_OBJ; f.cls = object_cls; f.label = object_label;
    f.cx = 160 + ex; f.cy = origin_y + ey; f.w = f.h = 10; f.conf = 90;
    f.img_w = image_width; f.img_h = image_height;
    f.sequence = fixed_seq >= 0 ? (uint16_t)fixed_seq : sequence++;
    steps_feed_frame(&f);
    ++emitted;
}

static void advance_fixture(void)
{
    int ex = error_x, ey = error_y;
    if (packet_at && !packet_done && now_ms >= packet_at) {
        packet_done = 1;
        if (mode == EMPTY_AFTER_GOOD) ++stats.obj; /* Empty packets have no callback. */
        if (mode == WRONG_AFTER_GOOD) feed(0, 0, CLS_TARGET, LAB_R);
        if (mode == WRONG_LABEL_AFTER_GOOD) feed(0, 0, cls, label + 1);
    }
    if (mode == NO_FRAMES || now_ms < next_frame_ms
        || (frame_limit >= 0 && emitted >= (unsigned)frame_limit)
        || (packet_done && (mode == EMPTY_AFTER_GOOD || mode == WRONG_AFTER_GOOD
                           || mode == WRONG_LABEL_AFTER_GOOD))) return;
    next_frame_ms = now_ms + 40u;
    if (mode == CROSS_DISTURBANCE) {
        ex = phase == 0 || phase == 2 ? 40 : 0;
        ey = phase == 0 || phase == 1 ? 40 : 0;
    } else if (mode == OVERSHOOT) {
        ex = phase == 0 ? 40 : (phase == 1 ? -40 : 0); ey = 0;
    } else if (mode == POST_STOP_DRIFT) {
        ex = phase == 0 ? 40 : 0;
        ey = phase == 2 ? 40 : 0;
        if (phase == 1 && !current_x) { phase = 2; ey = 40; }
    } else if (mode == BACKWARD_SEQUENCE && emitted) {
        fixed_seq = 9; ex = ey = 0;
    } else if (mode == ALTERNATING_IN_BAND) {
        ex = emitted % 2u ? 0 : 40;
        ey = emitted % 2u ? 40 : 0;
    }
    if (mode == DIMENSION_CHANGE && emitted) --image_width;
    if (mode == SEQUENCE_GAP && emitted) ++sequence;
    feed(ex, ey, cls, label);
    if (mode == OLD_PENDING_FRAME) now_ms += GRAB_XY_FRESH_MS + 1u;
}

static void reset_fixture(unsigned new_mode)
{
    now_ms = 0; next_frame_ms = 5; packet_at = abort_at = imu_bad_at = 0;
    stopped_at = last_encoder_change = first_drive_stop = coast_until = 0;
    memset(&stats, 0, sizeof stats); memset(encoder, 0, sizeof encoder);
    memset(drives, 0, sizeof drives);
    drive_count = brake_count = legacy_drives = zero_heading_count = 0;
    receive_end_count = emitted = phase = last_axis = bad_switches = 0;
    mode = new_mode; cls = CLS_BALL; label = LAB_R; error_x = error_y = 0; origin_y = 120;
    fixed_seq = -1; frame_limit = -1; packet_done = 0;
    abort_first_drive = coast_ms = reject_runtime_set = runtime_set_attempts = simultaneous = 0;
    imu_valid = scene_status = 1; heading = 37.0f;
    image_width = 320; image_height = 240; sequence = 1;
    current_x = current_y = 0.0f;
    run_reset();
}

static GrabAlignConfig config(void)
{
    return (GrabAlignConfig){160, 120, 1, 1, GRAB_XY_TOL_PX, GRAB_XY_TOL_PX};
}
static int stopped(void)
{
    return current_x == 0.0f && current_y == 0.0f && brake_count
        && !simultaneous && !legacy_drives && !zero_heading_count
        && !reject_runtime_set && !s_grab_align_running;
}

static int check_boundaries(void)
{
    const int errors[] = {0, 8, -8};
    const int outside[] = {9, -9, 40, -40, 140, -140};
    for (int object = 0; object < 2; ++object) {
        for (unsigned x = 0; x < sizeof errors / sizeof errors[0]; ++x) {
            for (unsigned y = 0; y < sizeof errors / sizeof errors[0]; ++y) {
                GrabAlignConfig c = config();
                reset_fixture(CONSTANT); cls = object ? CLS_HOSTAGE : CLS_BALL;
                label = object ? LAB_CYL : LAB_R; error_x = errors[x]; error_y = errors[y];
                CHECK(step_align_xy(cls, label, &c, heading, 1000));
                CHECK(!drive_count && emitted >= GRAB_XY_GOOD_FRAMES
                      && now_ms >= GRAB_XY_STILL_MS && receive_end_count == 1 && stopped());
            }
        }
    }
    for (int xs = -1; xs <= 1; xs += 2) {
        for (int ys = -1; ys <= 1; ys += 2) {
            for (unsigned axis = 0; axis < 2; ++axis) {
                for (unsigned e = 0; e < sizeof outside / sizeof outside[0]; ++e) {
                    GrabAlignConfig c = config();
                    float v = fabsf((float)outside[e]) * 0.6f;
                    reset_fixture(CONSTANT); image_height = 480; abort_first_drive = 1;
                    c.x_sign = xs; c.y_sign = ys;
                    if (axis) { c.cy = origin_y = 240; error_y = outside[e]; }
                    else error_x = outside[e];
                    if (v < 12.0f) v = 12.0f;
                    if (v > 80.0f) v = 80.0f;
                    v *= (float)(outside[e] > 0 ? 1 : -1) * (float)(axis ? ys : xs);
                    CHECK(!step_align_xy(cls, label, &c, heading, 1000));
                    CHECK(drive_count == 1 && drives[0].x == (axis ? 0.0f : v)
                          && drives[0].y == (axis ? v : 0.0f)
                          && drives[0].w == 0.0f && stopped());
                }
            }
        }
    }
    {
        GrabAlignConfig c = config();
        reset_fixture(CONSTANT); error_x = 9; error_y = 100; abort_first_drive = 1;
        CHECK(!step_align_xy(cls, label, &c, heading, 1000));
        CHECK(drives[0].x == 12.0f && drives[0].y == 0.0f && stopped());
        reset_fixture(CONSTANT); c.x_tol_px = 2.0f; c.y_tol_px = 3.0f;
        error_x = 2; error_y = -3;
        CHECK(step_align_xy(cls, label, &c, heading, 1000) && !drive_count && stopped());
        reset_fixture(CONSTANT); error_x = 3; abort_first_drive = 1;
        CHECK(!step_align_xy(cls, label, &c, heading, 1000));
        CHECK(drive_count == 1 && drives[0].x == 12.0f && stopped());
    }
    return 1;
}

static int check_coupling_and_still(void)
{
    GrabAlignConfig c = config();
    int axes[3]; unsigned transitions = 0;
    reset_fixture(CROSS_DISTURBANCE); coast_ms = 75;
    CHECK(step_align_xy(cls, label, &c, heading, 5000));
    for (unsigned i = 0; i < drive_count; ++i) {
        int axis = drives[i].x ? 1 : 2;
        if (!transitions || axes[transitions - 1] != axis) {
            CHECK(transitions < 3); axes[transitions++] = axis;
        }
    }
    CHECK(phase == 3 && transitions == 3 && axes[0] == 1 && axes[1] == 2
          && axes[2] == 1 && !bad_switches
          && now_ms - last_encoder_change >= GRAB_XY_STILL_MS && stopped());
    reset_fixture(OVERSHOOT);
    CHECK(step_align_xy(cls, label, &c, heading, 2500));
    CHECK(phase == 2 && drives[0].x > 0.0f && stopped());
    {
        int saw_negative = 0;
        for (unsigned i = 0; i < drive_count; ++i) if (drives[i].x < 0.0f) saw_negative = 1;
        CHECK(saw_negative);
    }
    reset_fixture(CONSTANT); coast_until = 400;
    CHECK(step_align_xy(cls, label, &c, heading, 1500));
    CHECK(now_ms >= 650 && now_ms - last_encoder_change >= GRAB_XY_STILL_MS && stopped());
    reset_fixture(POST_STOP_DRIFT);
    CHECK(step_align_xy(cls, label, &c, heading, 2500));
    CHECK(phase == 3 && !bad_switches && stopped());
    reset_fixture(CONSTANT); error_x = 40; abort_first_drive = 1; heading = 39.0f;
    CHECK(!step_align_xy(cls, label, &c, 37.0f, 1000));
    CHECK(fabsf(drives[0].w + 2.0f * step_heading_kp_deg() * 0.0174533f) < 0.000001f
          && stopped());
    return 1;
}

static int check_packet_freshness(void)
{
    const unsigned invalid_modes[] = {EMPTY_AFTER_GOOD, WRONG_AFTER_GOOD, WRONG_LABEL_AFTER_GOOD};
    GrabAlignConfig c = config();
    reset_fixture(CONSTANT); frame_limit = 4;
    CHECK(!step_align_xy(cls, label, &c, heading, 650));
    CHECK(emitted == 4 && !drive_count && stopped());
    reset_fixture(DUPLICATE); fixed_seq = 1;
    CHECK(!step_align_xy(cls, label, &c, heading, 650));
    CHECK(!drive_count && stopped());
    reset_fixture(DUPLICATE); fixed_seq = 1; error_x = 40;
    CHECK(!step_align_xy(cls, label, &c, heading, 650));
    CHECK(drive_count && first_drive_stop == 45 && stopped());
    reset_fixture(SEQUENCE_GAP);
    CHECK(!step_align_xy(cls, label, &c, heading, 650) && !drive_count && stopped());
    reset_fixture(BACKWARD_SEQUENCE); sequence = 10; error_x = 40;
    CHECK(!step_align_xy(cls, label, &c, heading, 650));
    CHECK(drive_count && first_drive_stop == 45 && stopped());
    reset_fixture(ALTERNATING_IN_BAND);
    CHECK(!step_align_xy(cls, label, &c, heading, 1500));
    CHECK(drive_count && !bad_switches && stopped());
    reset_fixture(SEQUENCE_WRAP); sequence = 65533;
    CHECK(step_align_xy(cls, label, &c, heading, 1000));
    CHECK(sequence < 20 && !drive_count && stopped());
    for (unsigned i = 0; i < sizeof invalid_modes / sizeof invalid_modes[0]; ++i) {
        reset_fixture(invalid_modes[i]); packet_at = 180;
        CHECK(!step_align_xy(cls, label, &c, heading, 650));
        CHECK(packet_done && !drive_count && stopped());
        reset_fixture(invalid_modes[i]); packet_at = 50; error_x = 40;
        CHECK(!step_align_xy(cls, label, &c, heading, 650));
        CHECK(packet_done && drive_count && first_drive_stop == 50 && stopped());
    }
    reset_fixture(SINGLE_MOVING_FRAME); error_x = 40; frame_limit = 1;
    CHECK(!step_align_xy(cls, label, &c, heading, 650));
    CHECK(drive_count && first_drive_stop >= 305 && first_drive_stop <= 315 && stopped());
    reset_fixture(OLD_PENDING_FRAME); error_x = 40; frame_limit = 1;
    CHECK(!step_align_xy(cls, label, &c, heading, 650) && !drive_count && stopped());
    reset_fixture(NO_FRAMES);
    CHECK(!step_align_xy(cls, label, &c, heading, 60));
    CHECK(now_ms == 60 && !drive_count && stopped());
    return 1;
}

static int check_failures_and_configuration(void)
{
    GrabAlignConfig base = config();
    const GrabAlignConfig bad[] = {
        {-1,120,1,1,8,8}, {160,-1,1,1,8,8}, {65535,120,1,1,8,8},
        {160,120,0,1,8,8}, {160,120,1,0,8,8}, {160,120,2,1,8,8},
        {160,120,1,-2,8,8}, {160,120,1,1,0,8}, {160,120,1,1,8,-1},
        {160,120,1,1,NAN,8}, {160,120,1,1,8,INFINITY}, {160,120,1,1,101,8}
    };
    for (unsigned i = 0; i < sizeof bad / sizeof bad[0]; ++i) {
        reset_fixture(CONSTANT);
        CHECK(!step_align_xy(cls, label, &bad[i], heading, 1000));
        CHECK(!now_ms && !drive_count && stopped());
    }
    reset_fixture(CONSTANT);
    CHECK(!step_align_xy(cls, label, 0, heading, 1000) && !now_ms && stopped());
    reset_fixture(CONSTANT);
    CHECK(!step_align_xy(CLS_TARGET, LAB_R, &base, heading, 1000) && !now_ms && stopped());
    reset_fixture(CONSTANT);
    CHECK(!step_align_xy(CLS_BUCKET, -1, &base, heading, 1000) && !now_ms && stopped());
    reset_fixture(CONSTANT);
    CHECK(!step_align_xy(cls, label, &base, NAN, 1000) && !now_ms && stopped());
    reset_fixture(CONSTANT); base.cx = 320;
    CHECK(!step_align_xy(cls, label, &base, heading, 1000));
    CHECK(now_ms == 5 && !drive_count && stopped());
    base = config(); reset_fixture(CONSTANT); base.cy = 240;
    CHECK(!step_align_xy(cls, label, &base, heading, 1000));
    CHECK(now_ms == 5 && !drive_count && stopped());
    base = config(); reset_fixture(CONSTANT); image_width = 0;
    CHECK(!step_align_xy(cls, label, &base, heading, 1000) && !drive_count && stopped());
    reset_fixture(CONSTANT); error_x = 160;
    CHECK(!step_align_xy(cls, label, &base, heading, 1000) && !drive_count && stopped());
    reset_fixture(CONSTANT); error_y = -121;
    CHECK(!step_align_xy(cls, label, &base, heading, 1000) && !drive_count && stopped());
    reset_fixture(DIMENSION_CHANGE);
    CHECK(!step_align_xy(cls, label, &base, heading, 1000) && !drive_count && stopped());
    reset_fixture(CONSTANT); imu_valid = 0;
    CHECK(!step_align_xy(cls, label, &base, heading, 1000) && !now_ms && stopped());
    reset_fixture(CONSTANT); heading = NAN;
    CHECK(!step_align_xy(cls, label, &base, 37.0f, 1000) && !now_ms && stopped());
    reset_fixture(CONSTANT); error_x = 40; imu_bad_at = 80;
    CHECK(!step_align_xy(cls, label, &base, heading, 1000));
    CHECK(now_ms == 80 && drive_count && stopped());
    reset_fixture(CONSTANT); scene_status = -1;
    CHECK(!step_align_xy(cls, label, &base, heading, 1000) && !now_ms && stopped());
    reset_fixture(NO_FRAMES); abort_at = 125;
    CHECK(!step_align_xy(cls, label, &base, heading, 0));
    CHECK(now_ms == 125 && run_aborted() && !drive_count && stopped());
    reset_fixture(CONSTANT); error_x = 40; abort_at = 80;
    CHECK(!step_align_xy(cls, label, &base, heading, 0));
    CHECK(now_ms == 80 && run_aborted() && drive_count && stopped());
    reset_fixture(CONSTANT); run_abort();
    CHECK(!step_align_xy(cls, label, &base, heading, 1000) && !now_ms && stopped());
    reset_fixture(CONSTANT);
    CHECK(strcmp(step_grab_alignment_config_missing(), "BALL_GRAB_XY_WORKPOINT_SIGNS") == 0);
    CHECK(!step_align(CLS_BALL, LAB_R, 1000) && !now_ms && !zero_heading_count && stopped());
    CHECK(step_grab_alignment_set(CLS_BALL, 160, 120, 1, -1));
    CHECK(strcmp(step_grab_alignment_config_missing(), "HOSTAGE_GRAB_XY_WORKPOINT_SIGNS") == 0);
    CHECK(step_grab_alignment_set(CLS_HOSTAGE, 160, 120, -1, 1));
    CHECK(!step_grab_alignment_config_missing());
    CHECK(!step_grab_alignment_set(CLS_TARGET, 160, 120, 1, 1));
    CHECK(!step_grab_alignment_set(CLS_BALL, -2, 120, 1, 1));
    CHECK(!step_grab_alignment_set(CLS_BALL, 160, 120, 2, 1));
    CHECK(step_grab_alignment_set(CLS_BALL, -1, -1, 0, 0));
    CHECK(!step_grab_alignment_config_missing());
    return 1;
}

int main(void)
{
    if (!check_boundaries() || !check_packet_freshness()
        || !check_failures_and_configuration() || !check_coupling_and_still()) return 1;
    puts("grab XY: BALL/HOSTAGE thresholds/signs, X-Y-X disturbance, overshoot, four-wheel holds, freshness/empty/sequence gates, abort/IMU/configuration passed");
    return 0;
}
