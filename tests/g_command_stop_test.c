/* Actual Bluetooth command router/state machine with host-only peripheral stubs.
 * A mission-start stub models an accepted future start; production calibration
 * gates are NOT changed. Real gate/start-abort checks live in the other fixture. */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "../App/test.c"

/* Include the new bench engine in this existing single-translation-unit
 * fixture. Production Keil links it as a separate source file. */
static int host_xy_filter(ProtoTask task, uint8_t digit, int *cls, int *label)
{
    if (task == PROTO_TASK_BUCKET && digit == 0u) { *cls = CLS_BUCKET; *label = -1; return 1; }
    if (digit < 1u || digit > 3u) return 0;
    if (task == PROTO_TASK_BALL) { *cls = CLS_BALL; *label = digit - 1; return 1; }
    if (task == PROTO_TASK_HOSTAGE) { *cls = CLS_HOSTAGE; *label = digit + 2; return 1; }
    return 0;
}
#define proto_target_filter host_xy_filter
#include "../App/vision_align_test.c"
#undef proto_target_filter

static uint32_t host_tick;
static MissionState host_state;
static int host_abort, start_calls, brake_calls, pulse_calls, gate_closed;
static int host_imu_valid;
static float host_yaw, host_absolute_yaw, last_x, last_y, last_w;
static float host_heading_kp;
static unsigned host_precise_calls, host_integer_calls;
static float host_fore, host_lateral;
static int32_t host_counts[4];
static void (*host_counts_hook)(int motor);
static int zero_calls, prepare_calls, servo_calls;
static uint16_t host_servo;
static char last_message[256];
static int trial_start_calls, trial_report_calls, trial_gate_closed;
static int trial_alignment_calls, trial_alignment_cls, trial_alignment_cx, trial_alignment_sign;
static int trial_grab_y_calls, trial_grab_y_cls, trial_grab_y_cy, trial_grab_y_sign;
static int trial_distance_calls;
static uint16_t trial_first_leg_mm;
static int laser_state;
static unsigned host_laser_on_calls;
static ProtoStats host_proto_stats;
static ProtoScene host_scene;
static int host_scene_calls, host_scene_status;
static int host_target_calls, host_target_result;
static ProtoTask host_target_task;
static uint8_t host_target_digit;
static int host_receive_closed, host_receive_end_calls;
static int host_qr_valid;
static int32_t host_qr_tuple[3];
static void (*host_scene_hook)(ProtoScene);
static int (*host_target_hook)(ProtoTask task, uint8_t digit);
static void (*host_qr_begin_hook)(void), (*host_qr_cancel_hook)(void);
static void (*host_receive_end_hook)(void);
static int (*host_qr_get_hook)(int32_t out[3]);
static void (*host_wire_diag_hook)(ProtoWireDiag *out);
static char host_messages[8192];
static MotionProfileTune host_motion_profile;

uint32_t HAL_GetTick(void) { return host_tick; }
void bp_debug_send(const char *s)
{
    /* Production send() appends CRLF in a separate UART write. */
    if (strcmp(s, "\r\n") != 0)
        snprintf(last_message, sizeof last_message, "%s", s);
    if (strlen(host_messages) + strlen(s) < sizeof host_messages)
        strcat(host_messages, s);
}
void bp_laser_set(int on) { laser_state = on; if (on) host_laser_on_calls++; }
int32_t bp_enc_raw_total(int m) { (void)m; return 0; }
void bp_enc_raw_reset_all(void) { }
void motion_brake(void) { brake_calls++; last_x = last_y = last_w = 0.0f; }
void motion_vel_set(float x, float y, float w)
{ host_integer_calls++; last_x = x; last_y = y; last_w = w; }
void motion_vel_set_precise(float x, float y, float w)
{ host_precise_calls++; last_x = x; last_y = y; last_w = w; }
float motion_odo_mm(void) { return host_fore; }
float motion_lateral_odo_mm(void) { return host_lateral; }
void motion_ik(float x, float y, float w, int16_t out[4])
{ (void)x; (void)y; (void)w; memset(out, 0, 4 * sizeof *out); }
#ifndef HOST_REAL_MOTION_PROFILE
void motion_profile_get(MotionProfileTune *out) { *out = host_motion_profile; }
int motion_profile_set(const MotionProfileTune *in) { host_motion_profile = *in; return 1; }
void motion_ramp_init(MotionRamp *r, float acc, float dec)
{ r->cur = 0.0f; r->acc = acc; r->dec = dec; }
void motion_linear_ramp_init(MotionRamp *r)
{ motion_ramp_init(r, host_motion_profile.acc_mms2, host_motion_profile.dec_mms2); }
float motion_linear_profile_step(MotionRamp *r, float v, float d, float dt)
{ (void)r; (void)d; (void)dt; return v; }
float motion_linear_ramp_step(MotionRamp *r, float v, float d, float dt)
{ (void)r; (void)d; (void)dt; return v; }
#endif
void ctrl_set_speed(int m, int16_t rpm) { (void)m; (void)rpm; }
void ctrl_set_duty_open(int m, int16_t duty) { (void)m; (void)duty; }
void ctrl_coast_all(void) { }
void ctrl_enc_reset_all(void) { host_fore = host_lateral = 0.0f; memset(host_counts, 0, sizeof host_counts); }
int32_t ctrl_enc_total(int m)
{
    if (host_counts_hook) host_counts_hook(m);
    return host_counts[m];
}
void ctrl_get_rpm_est_all(int16_t out[4]) { memset(out, 0, 4 * sizeof *out); }
void ctrl_get_rpm_fast_all(float out[4]) { memset(out, 0, 4 * sizeof *out); }
int16_t ctrl_get_rpm_est(int m) { (void)m; return 0; }
void ctrl_tune_get(CtrlTune *out) { memset(out, 0, sizeof *out); }
int ctrl_tune_set(const CtrlTune *in) { (void)in; return 1; }
void arm_claw_set_us(uint16_t us) { host_servo = us; servo_calls++; }
uint16_t arm_claw_command_us(void) { return host_servo; }
void arm_claw_open(void) { host_servo = 1000u; }
void arm_claw_close(void) { host_servo = 1800u; }
void arm_stepper_dir(int axis, int dir) { (void)axis; (void)dir; }
void arm_stepper_step(int axis) { (void)axis; pulse_calls++; }
float imu_yaw_deg(void) { return 0.0f; }
float imu_heading_deg(void) { return host_absolute_yaw; }
float imu_leg_heading_deg(void) { return host_yaw; }
float imu_pitch_deg(void) { return 0.0f; }
float imu_roll_deg(void) { return 0.0f; }
uint8_t imu_ok(void) { return (uint8_t)host_imu_valid; }
uint8_t imu_zero_leg_heading(void) { zero_calls++; host_yaw = 0.0f; return (uint8_t)host_imu_valid; }
uint32_t imu_last_valid_age_ms(void) { return 0u; }
void run_abort(void) { host_abort = 1; step_vision_receive_end(); }
int run_aborted(void) { return host_abort; }
int step_prepare_leg(void) { prepare_calls++; return !host_abort && host_imu_valid; }
float step_heading_hold_w_kp(float heading, float kp)
{
    float e = heading - host_yaw;
    float w;
    while (e > 180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    w = kp * e * 0.0174533f;
    if (w > 2.0f) w = 2.0f;
    if (w < -2.0f) w = -2.0f;
    return w;
}
float step_heading_hold_w(float heading) { return step_heading_hold_w_kp(heading, host_heading_kp); }
float step_heading_kp_deg(void) { return host_heading_kp; }
int step_heading_kp_set(float v)
{ if (v < 0.0f || v > 5.0f) return 0; host_heading_kp = v; return 1; }
float step_orth_kp(void) { return 0.0f; }
int step_orth_kp_set(float v) { (void)v; return 1; }
float step_orth_hold_cmd(int lateral, float ref) { (void)lateral; (void)ref; return 0.0f; }
MissionState mission_state(void) { return host_state; }
const char *mission_config_missing(void) { return gate_closed ? "QR_START_LEFT_MM" : 0; }
int mission_start(void) { start_calls++; return !gate_closed; }
int mission_start_trial(void) { trial_start_calls++; return !trial_gate_closed; }
const char *mission_trial_config_missing(void)
{
    return trial_gate_closed ? "TRIAL_VISION_UNCONFIRMED" : 0;
}
void mission_trial_report(void) { trial_report_calls++; }
int mission_trial_set_first_leg(uint16_t mm)
{
    trial_distance_calls++;
    trial_first_leg_mm = mm;
    return mm > 0u && mm < 2450u;
}
int mission_trial_set_alignment(int cls, int cx, int sign)
{
    trial_alignment_calls++;
    trial_alignment_cls = cls; trial_alignment_cx = cx; trial_alignment_sign = sign;
    return cls >= -1 && cls <= 3 && cx >= -1 && cx < 65535 && sign >= -1 && sign <= 1;
}
int mission_trial_set_grab_y(int cls, int cy, int sign)
{
    trial_grab_y_calls++;
    trial_grab_y_cls = cls; trial_grab_y_cy = cy; trial_grab_y_sign = sign;
    return (cls == -1 || cls == CLS_BALL || cls == CLS_HOSTAGE) &&
           cy >= -1 && cy < 65535 && sign >= -1 && sign <= 1;
}
void robot_diag_report(void) { }
void proto_stats_get(ProtoStats *out) { *out = host_proto_stats; }
void proto_wire_diag_get(ProtoWireDiag *out)
{ if (host_wire_diag_hook) host_wire_diag_hook(out); else memset(out, 0, sizeof *out); }
int proto_scene_status(void) { return host_scene_status; }
void proto_send_scene(ProtoScene scene)
{
    host_scene = scene; host_scene_calls++; host_scene_status = 0;
    host_receive_closed = 0;
    host_qr_valid = 0; memset(host_qr_tuple, 0, sizeof host_qr_tuple);
    if (host_scene_hook) host_scene_hook(scene);
}
int proto_send_target(ProtoTask task, uint8_t digit)
{
    host_target_calls++; host_target_task = task; host_target_digit = digit;
    host_scene = SCENE_ANTI; host_scene_status = 0; host_receive_closed = 0;
    host_qr_valid = 0; memset(host_qr_tuple, 0, sizeof host_qr_tuple);
    if (host_target_hook) return host_target_hook(task, digit);
    return host_target_result;
}
void proto_qr_begin(void)
{
    if (host_qr_begin_hook) { host_qr_begin_hook(); return; }
    if (!host_scene_calls || host_receive_closed || host_scene != SCENE_QR || host_scene_status < 0)
        proto_send_scene(SCENE_QR);
}
void proto_receive_end(void)
{
    host_receive_closed = 1; host_receive_end_calls++; host_scene_status = 0;
    host_qr_valid = 0; memset(host_qr_tuple, 0, sizeof host_qr_tuple);
    if (host_receive_end_hook) host_receive_end_hook();
}
void step_vision_receive_end(void) { proto_receive_end(); }
void proto_qr_cancel(void)
{
    if (host_qr_cancel_hook) { host_qr_cancel_hook(); return; }
    if (host_scene_calls && host_scene == SCENE_QR) proto_receive_end();
}
int proto_qr_get(int32_t out[3])
{
    if (host_qr_get_hook) return host_qr_get_hook(out);
    if (host_receive_closed || host_scene != SCENE_QR || host_scene_status != 1 || !host_qr_valid) return 0;
    if (out) memcpy(out, host_qr_tuple, sizeof host_qr_tuple);
    return 1;
}
static void host_qr_accept(int a, int b, int c)
{
    if (host_receive_closed || host_scene != SCENE_QR || host_scene_status != 1 || host_qr_valid ||
        a < 1 || a > 3 || b < 1 || b > 3 || c < 1 || c > 3) return;
    host_qr_tuple[0] = a; host_qr_tuple[1] = b; host_qr_tuple[2] = c; host_qr_valid = 1;
}

static void reset_fixture(void)
{
    host_tick = 100u; host_state = MS_BOOT;
    host_abort = start_calls = brake_calls = pulse_calls = gate_closed = 0;
    host_imu_valid = 1; host_yaw = host_absolute_yaw = last_x = last_y = last_w = 0.0f;
    host_wire_diag_hook = NULL;
    host_heading_kp = 0.3f; host_precise_calls = host_integer_calls = 0u;
    host_fore = host_lateral = 0.0f; memset(host_counts, 0, sizeof host_counts);
    host_counts_hook = NULL;
    zero_calls = prepare_calls = servo_calls = 0;
    host_servo = 1400u; last_message[0] = '\0';
    trial_start_calls = trial_report_calls = trial_gate_closed = trial_alignment_calls = 0;
    trial_alignment_cls = trial_alignment_cx = trial_alignment_sign = 0;
    trial_grab_y_calls = trial_grab_y_cls = trial_grab_y_cy = trial_grab_y_sign = 0;
    trial_distance_calls = 0; trial_first_leg_mm = 0u;
    laser_state = 0; host_laser_on_calls = 0u;
    memset(&host_proto_stats, 0, sizeof host_proto_stats);
    host_scene = SCENE_IDLE; host_scene_calls = host_scene_status = 0;
    host_target_calls = 0; host_target_result = 1;
    host_target_task = PROTO_TASK_BALL; host_target_digit = 0u;
    host_receive_closed = host_receive_end_calls = 0;
    host_qr_valid = 0; memset(host_qr_tuple, 0, sizeof host_qr_tuple);
    host_scene_hook = NULL; host_qr_begin_hook = host_qr_cancel_hook = NULL;
    host_target_hook = NULL;
    host_receive_end_hook = NULL;
    host_qr_get_hook = NULL;
    host_messages[0] = '\0';
    memset(&host_motion_profile, 0, sizeof host_motion_profile);
    (void)motion_profile_set(&host_motion_profile);
    test_init();
}

#define CHECK(expr) do { if (!(expr)) { fprintf(stderr, "g test line %d: %s\n", __LINE__, #expr); return 1; } } while (0)

static int sequence_start_stage(void)
{
    CHECK(s_seq_state == SQ_STILL && last_w == 0.0f && last_x == 0.0f && last_y == 0.0f);
    host_tick += T_DIST_STILL_MS;
    test_poll();
    CHECK(s_seq_state == SQ_WAIT && last_w == 0.0f && last_x == 0.0f && last_y == 0.0f);
    host_tick += NAV_SETTLE_MS;
    test_poll();
    CHECK(s_seq_state == SQ_RUN && s_round == R_RUN && prepare_calls == 0);
    return 0;
}

/* Non-vision route tests supply a fresh legal camera response at R1.
 * The dedicated QR-gate regression drives the waiting phase independently. */
static int sequence_release_qr(void)
{
    CHECK(s_seq_state == SQ_QR_WAIT && host_scene == SCENE_QR);
    int requests = host_scene_calls;
    host_scene_status = 1;
    host_qr_accept(1, 2, 3);
    test_poll();
    CHECK(s_seq_state == SQ_STILL && s_seq_stage == 1u && host_scene == SCENE_QR);
    CHECK(host_receive_closed && host_scene_calls == requests && !proto_qr_get(NULL));
    return 0;
}

static void fixture_complete_distance_alignment(void)
{
    /* No vehicle plant here: dedicated post-yaw tests verify correction
     * signs and intermediate commands. This shared route driver supplies
     * the physically corrected endpoint and waits the complete hold. */
    if (s_round == R_ALIGN) {
        host_yaw = s_dist_heading0;
        test_poll();
        host_tick += T_DIST_ALIGN_STABLE_MS;
        test_poll();
    }
}

static int sequence_finish_stage(void)
{
    CHECK(s_seq_state == SQ_RUN);
    if (dist_mode()) {
        if (dist_lateral()) host_lateral = s_dist_odo0 + s_dist_target;
        else host_fore = s_dist_odo0 + s_dist_target;
        test_poll();
        CHECK(s_round == R_BRAKE && s_dist_reason == 1u && s_seq_state == SQ_RUN);
        host_tick += T_DIST_STILL_MS;
        test_poll();
        fixture_complete_distance_alignment();
    } else {
        host_yaw = turn_target_deg(); test_poll();
        CHECK(s_round == R_BRAKE && strcmp(s_turn_result, "DONE") == 0 && s_seq_state == SQ_RUN);
        host_tick += T_TURN_SETTLE_MS; test_poll();
    }
    if (s_seq_state == SQ_QR_WAIT) CHECK(sequence_release_qr() == 0);
    return 0;
}

/* Stub-only bucket samples. The dedicated QR/bucket fixtures use real wire
 * packets; ordinary sequence_start_stage must never start the mode0 node. */
static void sequence_bucket_stub_sample(void)
{
    ProtoFrame frame = {0};
    frame.type = PF_OBJ; frame.cls = CLS_BUCKET; frame.label = 0;
    frame.cx = 500; frame.cy = 120; frame.w = frame.h = 24; frame.conf = 90;
    frame.img_w = 640u; frame.img_h = 320u;
    host_proto_stats.obj++;
    frame.sequence = (uint16_t)host_proto_stats.obj;
    test_vision_feed_frame(&frame);
    test_poll();
}

static int sequence_bucket_stub_phase(unsigned phase)
{
    CHECK(phase <= 3u && s_seq_state == SQ_BUCKET_ALIGN && s_bucket36_phase == BA_STILL);
    CHECK(s_seq_stage == ROUTE_TEST_ALIGN_STAGE && host_target_task == PROTO_TASK_BUCKET &&
          host_target_digit == 0u && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    if (!phase) return 0;
    host_scene_status = 1; /* Explicit host ACK; not a real-parser assertion. */
    host_tick += T_DIST_STILL_MS; test_poll();
    CHECK(s_seq_state == SQ_BUCKET_ALIGN && s_bucket36_phase == BA_PREP_WAIT);
    if (phase == 1u) return 0;
    host_tick += NAV_SETTLE_MS; test_poll();
    CHECK(s_seq_state == SQ_BUCKET_ALIGN && s_bucket36_phase == BA_SEEK);
    if (phase == 2u) return 0;
    for (unsigned i = 0u; i < 5u; ++i) sequence_bucket_stub_sample();
    CHECK(s_seq_state == SQ_BUCKET_ALIGN && s_bucket36_phase == BA_SETTLE &&
          s_bucket36_good == 5u && last_x == 0.0f && !laser_state);
    return 0;
}

static int sequence_finish_bucket_stub(void)
{
    CHECK(sequence_bucket_stub_phase(3u) == 0);
    for (unsigned i = 0u; i < 4u; ++i) {
        host_tick += 250u; sequence_bucket_stub_sample();
    }
    CHECK(s_seq_state == SQ_MANUAL_D_WAIT && s_seq_stage == ROUTE_TEST_BACK_STAGE &&
          s_bucket36_back_mm == 0u && host_receive_closed && !laser_state &&
          last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    return 0;
}

static int check_retained34_bucket_stub(void)
{
    reset_fixture(); run_cmd("34"); run_cmd("g");
    s_seq_stage = ROUTE_TEST_ALIGN_STAGE; route_seq_prepare();
    CHECK(route_seq_bucket_enabled() && s_seq_state == SQ_BUCKET_ALIGN);
    CHECK(sequence_finish_bucket_stub() == 0);
    run_cmd("d730"); CHECK(sequence_start_stage() == 0);
    CHECK(s_msel == 16 && s_dist_target == -730.0f && s_v == 100.0f);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && s_msel == 34);
    return 0;
}

/* Two runs without power cycling pin that the disabled cross/contact gains
 * never leak into the next leg, manual profiles or a later route run. */
static int check_route_cross_heading_restart(unsigned mode)
{
    const char *selection = mode == 31u ? "31" : "34";
    CHECK(mode == 31u || mode == 34u);
    reset_fixture();
    for (unsigned run = 1u; run <= 2u; ++run) {
        run_cmd(selection); run_cmd("ykp1.2"); run_cmd("g");
        CHECK(s_seq_run == run && s_route_heading_kp == 1.2f && step_heading_kp_deg() == 0.3f);
        s_seq_stage = 3u; route_seq_prepare();
        CHECK(sequence_start_stage() == 0);
        CHECK(s_msel == 16 && s_dist_target == -650.0f && s_v == 300.0f && s_dist_heading_kp == 0.0f);
        CHECK(s_dist_ff_ratio == 0.0f && last_x == -300.0f && last_y == 0.0f);
        host_yaw = 6.0f; tick(); CHECK(last_w == 0.0f);
        host_yaw = -6.0f; tick(); CHECK(last_w == 0.0f);
        CHECK(s_route_heading_kp == 1.2f && step_heading_kp_deg() == 0.3f);
        CHECK(sequence_finish_stage() == 0 && s_seq_stage == 4u);
        CHECK(sequence_start_stage() == 0);
        CHECK(s_msel == 15 && s_dist_target == (mode == 31u ? 70.0f : 80.0f) && s_v == 20.0f && s_dist_heading_kp == 0.0f);
        CHECK(last_x == 20.0f && last_y == 0.0f && s_dist_ff_ratio == 0.0f);
        host_yaw = 6.0f; tick(); CHECK(last_w == 0.0f);
        host_yaw = -6.0f; tick(); CHECK(last_w == 0.0f);
        unsigned before_finish = (unsigned)zero_calls;
        CHECK(sequence_finish_stage() == 0);
        CHECK(s_seq_stage == 5u && s_seq_state == SQ_STILL &&
              last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
        CHECK(zero_calls == (int)before_finish + 1); /* Existing distance-result reset. */
        unsigned before_node = (unsigned)zero_calls;
        host_yaw = run == 1u ? 7.0f : -7.0f; host_counts[0]++;
        host_tick += T_DIST_STILL_MS; test_poll();
        CHECK(s_seq_state == SQ_STILL && zero_calls == (int)before_node &&
              last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
        host_tick += T_DIST_STILL_MS - 1u; test_poll();
        CHECK(s_seq_state == SQ_STILL && zero_calls == (int)before_node);
        host_tick++; test_poll();
        CHECK(s_seq_state == SQ_WAIT && zero_calls == (int)before_node + 1 && host_yaw == 0.0f);
        CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
        CHECK(NAV_SETTLE_MS == 750u);
        host_tick += NAV_SETTLE_MS - 1u; test_poll();
        CHECK(s_seq_state == SQ_WAIT && zero_calls == (int)before_node + 1 &&
              last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
        host_tick++; test_poll();
        CHECK(s_seq_state == SQ_RUN && s_round == R_RUN && s_msel == 16);
        CHECK(s_dist_target == -190.0f && s_v == 100.0f && s_dist_heading0 == 0.0f);
        CHECK(s_dist_heading_kp == 1.2f && s_route_heading_kp == 1.2f &&
              step_heading_kp_deg() == 0.3f && host_yaw == 0.0f && last_w == 0.0f);
        CHECK(s_dist_ff_ratio == 0.00625f && last_x == -100.0f && last_y == -0.625f);
        host_yaw = 3.0f; tick();
        CHECK(fabsf(last_w + 1.2f * 3.0f * 0.0174533f) < 0.000001f);
        host_yaw = -3.0f; tick();
        CHECK(fabsf(last_w - 1.2f * 3.0f * 0.0174533f) < 0.000001f);
        run_cmd("g");
        CHECK(s_seq_state == SQ_STOPPED && s_msel == (int)mode &&
              s_route_heading_kp == 1.2f && step_heading_kp_deg() == 0.3f);
    }
    run_cmd("15"); run_cmd("v100"); run_cmd("d100"); run_cmd("g");
    CHECK(s_dist_heading_kp == 0.3f && s_route_heading_kp == 1.2f);
    run_cmd("g"); run_cmd("32");
    CHECK(dist_heading_kp_get(NULL) == 0.3f && step_heading_kp_deg() == 0.3f);
    printf("route%u two-run post-cross: back650/v300 ->forward%u/v20 yaw/FF disabled; wheel-still/node zero/750ms wait ->fresh back190/v100 restoresykp + independentBFF; manual/32 unchanged passed\n", mode, mode == 31u ? 70u : 80u);
    return 0;
}

static int check_route_sequence(void)
{
    /* Independent expectation, not copied from the live recipe at runtime. */
    static const int expected_modes[12] = {17,16,20,16,15,16,18,16,30,15,20,15};
    static const int expected_commands[12] = {-520,-650,90,-650,70,-190,730,-780,-90,2450,90,2125};
    static const float expected_speeds[12] = {100,100,100,300,20,100,100,100,100,100,100,100};
    static const char *const stop_keys[] = {"g", "a", "0"};
    reset_fixture();
    host_fore = 4321.0f; host_lateral = -1234.0f; host_yaw = 47.0f;
    run_cmd("31");
    CHECK(s_seq_state == SQ_READY && s_round == R_READY && s_msel == 31);
    for (int i = 0; i < 5; i++) { host_tick += 100u; test_poll(); }
    CHECK(s_seq_state == SQ_READY && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    run_cmd("g"); CHECK(s_seq_run == 1u && s_seq_stage == 0u);
    CHECK(ROUTE31_STAGES == 12u && route_seq_stage_count() == 12u && !route_seq_bucket_enabled());
    for (int i = 0; i < 12; i++) {
        CHECK(s_seq_stage == i && sequence_start_stage() == 0);
        CHECK(s_msel == expected_modes[i] && s_route_leg == i + 1 && s_active_test == (uint32_t)i + 1u);
        CHECK(s_route31_plan[i].heading_hold == (i == 3 || i == 4 ? 0u : 1u));
        if (dist_mode()) {
            CHECK(s_dist_target == (float)expected_commands[i] && s_v == expected_speeds[i]);
            CHECK(s_dist_ramp.acc == 700.0f && s_dist_ramp.dec == 350.0f && s_dist_precise == 1u);
            CHECK(s_dist_ff_ratio == (i == 9 || i == 11 ? -0.00625f :
                  i == 1 || i == 5 || i == 7 ? 0.00625f : 0.0f));
            CHECK(s_dist_heading_kp == (i == 3 || i == 4 ? 0.0f : 0.3f));
            CHECK(last_w == 0.0f);
            if (dist_lateral()) CHECK(last_x == 0.0f && last_y == (s_msel == 18 ? 100.0f : -100.0f));
            else CHECK(last_x == (s_msel == 16 ? -expected_speeds[i] : expected_speeds[i]) &&
                       last_y == -s_dist_ff_ratio * expected_speeds[i]);
        } else {
            CHECK(turn_target_deg() == (float)expected_commands[i] && last_x == 0.0f && last_y == 0.0f);
            CHECK(last_w == (i == 8 ? -T_TURN_MAX_W : T_TURN_MAX_W));
        }
        CHECK(start_calls == 0 && pulse_calls == 0 && servo_calls == 0 && !s_go);
        CHECK(sequence_finish_stage() == 0);
        CHECK(i == 11 ? s_seq_state == SQ_DONE : s_seq_state == SQ_STILL);
        CHECK(!host_target_calls && s_seq_state != SQ_BUCKET_ALIGN && s_seq_state != SQ_MANUAL_D_WAIT);
    }
    CHECK(zero_calls == 21 && prepare_calls == 0); /* 12 node zeros + 9 distance-report resets */
    CHECK(s_msel == 31 && s_round == R_DONE && strstr(last_message, "status=DONE") != NULL);
    for (int i = 0; i < 10; i++) { host_tick += 1000u; test_poll(); }
    run_cmd("g"); CHECK(s_seq_state == SQ_DONE && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    run_cmd("31"); run_cmd("g"); CHECK(s_seq_run == 2u && s_seq_state == SQ_STILL);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED);

    /* All 12 motion-only nodes, every phase and stop key. */
    for (int stage = 0; stage < 12; stage++) {
        for (int key = 0; key < 3; key++) {
            for (int phase = 0; phase < 4; phase++) {
                reset_fixture(); run_cmd("31"); run_cmd("g");
                s_seq_stage = (uint8_t)stage; route_seq_prepare();
                if (phase == 1) { host_tick += T_DIST_STILL_MS; test_poll(); CHECK(s_seq_state == SQ_WAIT); }
                if (phase >= 2) CHECK(sequence_start_stage() == 0);
                if (phase == 3) {
                    if (dist_mode()) {
                        if (dist_lateral()) host_lateral = s_dist_odo0 + s_dist_target;
                        else host_fore = s_dist_odo0 + s_dist_target;
                    } else host_yaw = turn_target_deg();
                    test_poll(); CHECK(s_round == R_BRAKE);
                }
                run_cmd(stop_keys[key]);
                CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == stage && s_msel == 31 && s_round == R_DONE);
                for (int j = 0; j < 4; j++) { host_tick += 1000u; test_poll(); }
                run_cmd("g");
                CHECK(s_seq_state == SQ_STOPPED && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f &&
                      !s_go && start_calls == 0 && pulse_calls == 0 && servo_calls == 0);
            }
        }
    }
    reset_fixture(); run_cmd("31"); run_cmd("g");
    host_tick += T_DIST_STILL_MS; host_counts[0]++; test_poll();
    CHECK(s_seq_state == SQ_STILL && zero_calls == 0); /* no zero while wheel still changes */
    host_tick += T_DIST_STILL_MS; test_poll(); CHECK(s_seq_state == SQ_WAIT && zero_calls == 1);
    test_feed('g'); host_tick += T_IDLE_MS; test_poll();
    CHECK(s_seq_state == SQ_STOPPED && s_len == 0u && last_x == 0.0f && last_y == 0.0f);

    static const char *const writes[] = {"v300","d1500","ykp4","r1","20","31","cc","co","su1500","n5","b1d800"};
    for (int phase = 0; phase < 4; phase++) {
        reset_fixture(); run_cmd("31"); run_cmd("g");
        if (phase == 1) { host_tick += T_DIST_STILL_MS; test_poll(); }
        if (phase >= 2) CHECK(sequence_start_stage() == 0);
        if (phase == 3) { host_lateral = s_dist_odo0 + s_dist_target; test_poll(); }
        for (size_t k = 0; k < sizeof writes / sizeof writes[0]; k++) {
            run_cmd(writes[k]);
            CHECK(strstr(last_message, "ERR ROUTE_SEQ_ACTIVE") != NULL && s_v == 100.0f &&
                  s_d == 520.0f && s_seq_stage == 0u && s_msel == 17 && servo_calls == 0);
        }
        CHECK(route_seq_active()); run_cmd("g");
    }
    /* All failed sensor/turn statuses cancel rather than advancing a ready/done submode. */
    for (int stage = 0; stage < 12; stage++) {
        reset_fixture(); run_cmd("31"); run_cmd("g");
        s_seq_stage = (uint8_t)stage; route_seq_prepare();
        CHECK(sequence_start_stage() == 0);
        host_imu_valid = 0; test_poll();
        CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == stage && strstr(last_message, "status=IMUERR") != NULL);
    }
    reset_fixture(); run_cmd("31"); host_imu_valid = 0; run_cmd("g"); test_poll();
    CHECK(s_seq_state == SQ_STOPPED && s_test_seq == 0u && last_x == 0.0f && last_y == 0.0f);
    for (int phase = 0; phase < 2; phase++) {
        reset_fixture(); run_cmd("31"); run_cmd("g");
        if (phase) CHECK(sequence_start_stage() == 0);
        run_abort(); test_poll(); CHECK(s_seq_state == SQ_STOPPED && last_x == 0.0f && last_y == 0.0f);
    }
    for (int failure = 0; failure < 3; failure++) {
        reset_fixture(); run_cmd("31"); run_cmd("g"); s_seq_stage = 2u; route_seq_prepare();
        CHECK(sequence_start_stage() == 0);
        if (failure == 0) { host_tick += T_TURN_MAX_MS; test_poll(); }
        if (failure == 1) { host_yaw = 111.0f; test_poll(); }
        if (failure == 2) { turn_begin_settle("STOP"); }
        CHECK(s_round == R_BRAKE);
        host_tick += T_TURN_SETTLE_MS; test_poll();
        CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 2u && last_w == 0.0f);
    }
    reset_fixture(); run_cmd("31"); run_cmd("g"); CHECK(sequence_start_stage() == 0);
    dist_begin_finish(0u); host_tick += T_DIST_STILL_MS; test_poll();
    CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 0u);
    puts("route31: exact12 motion-only traversal incl board70 and crossing/contact yaw/FF disabled; QRgate/right730/back780/left90 withoutbucket/manuald, two forwardFFF and three backwardBFF legs, post-yaw-before-next, terminal/no-restart,144 g/a/0 cancellations, locks and IMU/abort/turn failure passed");
    return 0;
}

static int check_forward_compensation(void)
{
    static const int speeds[] = {100, 200, 300};
    for (int mode = 15; mode <= 18; mode++) {
        for (int k = 0; k < 3; k++) {
            char line[16];
            float expected_ratio = 0.0f;
            reset_fixture();
            CHECK(s_forward_ff_ratio == -0.00625f && test_forward_ff_ratio() == 0.0125f);
            snprintf(line, sizeof line, "%d", mode); run_cmd(line);
            snprintf(line, sizeof line, "v%d", speeds[k]); run_cmd(line);
            run_cmd("d1500"); run_cmd("g"); tick();
            CHECK(s_round == R_RUN);
            if (mode == 15) expected_ratio = -0.00625f;
            if (mode == 16) expected_ratio = 0.00625f;
            if (mode == 17 && speeds[k] == 300) expected_ratio = T_LEFT_FF_SEED;
            if (mode == 18 && speeds[k] == 300) expected_ratio = T_RIGHT_FF_SEED;
            CHECK(s_dist_ff_ratio == expected_ratio && last_w == 0.0f);
            if (mode <= 16) {
                CHECK(last_x == (mode == 15 ? (float)speeds[k] : -(float)speeds[k]));
                CHECK(last_y == -expected_ratio * fabsf(last_x));
            } else {
                CHECK(last_y == (mode == 17 ? -(float)speeds[k] : (float)speeds[k]));
                CHECK(last_x == expected_ratio * last_y);
            }
            run_cmd("g"); CHECK(s_round == R_BRAKE && last_x == 0.0f && last_y == 0.0f);
        }
    }
    reset_fixture(); run_cmd("15"); run_cmd("fff0");
    run_cmd("v100"); run_cmd("d1000"); run_cmd("g"); tick();
    CHECK(s_dist_ff_ratio == 0.0f && last_x == 100.0f && last_y == 0.0f);
    reset_fixture(); run_cmd("15"); run_cmd("fff0.01");
    run_cmd("v100"); run_cmd("d1000"); run_cmd("g"); tick();
    CHECK(s_dist_ff_ratio > 0.009999f && s_dist_ff_ratio < 0.010001f &&
          last_y > -1.0001f && last_y < -0.9999f);
    reset_fixture(); run_cmd("param");
    CHECK(strstr(host_messages, "fff=0.0125") != NULL);
    puts("distance FF: forward-right/reverse-left trial seeds at100/200/300; independent direction RAM and preserved v300 strafe parameters, manual/mission isolation, off/override/PARAM passed");
    return 0;
}

static int check_shared_turn_hold(void)
{
    static const int modes[] = {20, 22, 30};
    /* Literal checks pin the successful 180 settings, not just new aliases. */
    CHECK(T_TURN180_MAX_MS == 12000u && T_TURN_MAX_MS == 12000u);
    CHECK(TURN90_TARGET_DEG == 90.0f && T_TURN90_LEFT_COMP_DEG == 2.0f &&
          T_TURN90_RIGHT_COMP_DEG == 0.0f && T_TURN_LEFT_TARGET_DEG == -92.0f &&
          T_TURN_RIGHT_TARGET_DEG == 90.0f);
    CHECK(T_TURN_KP == 0.15f && T_TURN_MAX_W == 2.0f && T_TURN_MIN_W == 0.18f &&
          T_TURN_TOL_DEG == 0.3f && T_TURN_SETTLE_MS == 700u && TURN90_STILL_DEG == 0.2f);
    for (int k = 0; k < 3; k++) {
        char line[8];
        float target, dir;
        reset_fixture(); snprintf(line, sizeof line, "%d", modes[k]);
        run_cmd(line); run_cmd("g");
        target = turn_target_deg(); dir = target < 0.0f ? -1.0f : 1.0f;
        CHECK(target == (modes[k] == 22 ? 180.0f : (modes[k] == 30 ? -92.0f : 90.0f)));
        CHECK(strstr(last_message, modes[k] == 22 ? "target=+180deg" :
                                  (modes[k] == 30 ? "target=-92deg" : "target=+90deg")) != NULL);
        if (modes[k] != 22) {
            host_yaw = dir * 88.0f; tick();
            CHECK(s_round == R_RUN && last_w * dir > 0.0f); /* both turns remain short */
            host_yaw = dir * 91.0f; tick();
            CHECK(s_round == R_RUN && last_w < 0.0f); /* right corrects past90, left continues toward92 */
            CHECK(strstr(last_message, modes[k] == 20 ? "comp=0deg" : "comp=2deg") != NULL);
        }
        CHECK(turn_max_ms() == 12000u);
        host_yaw = target - dir * 10.0f; tick();
        CHECK(last_w == dir * 1.5f && last_x == 0.0f && last_y == 0.0f);
        host_yaw = target - dir * 0.7f; tick(); CHECK(last_w == dir * 0.18f);
        /* +/-90 now get the same correction budget as 180, not angle bias. */
        host_tick = s_meas_t0 + 7000u; tick(); CHECK(s_round == R_RUN);
        host_yaw = target; tick(); CHECK(s_round == R_BRAKE && last_w == 0.0f);
        host_yaw = target + dir * 0.7f; tick(); CHECK(s_round == R_RUN);
        tick(); CHECK(last_w == -dir * 0.18f); /* inertial overshoot reverses slowly */
        host_yaw = target; tick(); CHECK(s_round == R_BRAKE);
        host_tick += 350u; host_yaw = target + dir * 0.25f; tick();
        CHECK(s_round == R_BRAKE && s_turn_settle_t0 == host_tick);
        host_tick += 650u; tick(); CHECK(s_round == R_BRAKE);
        host_tick += 50u; tick(); CHECK(s_round == R_DONE && strstr(last_message, "status=DONE") != NULL);
    }
    puts("90/180 shared hold: preserved 180 constants, signed P/min/clamp, overshoot recovery, still-window reset, 12s budget passed");
    return 0;
}

static int check_trial32_router(void)
{
    static const char *const stop_keys[] = { "g", "a", "0" };
    static const char *const workpoint_keys[] = { "bcx185", "tcx173", "hcx160", "kcx201" };
    reset_fixture();
    CHECK(test_forward_ff_ratio() == T_MISSION_FORWARD_FF_SEED);
    run_cmd("fff0.0200");
    CHECK(test_forward_ff_ratio() > 0.019999f && test_forward_ff_ratio() < 0.020001f);
    run_cmd("fff0"); CHECK(test_forward_ff_ratio() == 0.0f);
    reset_fixture();
    run_cmd("32");
    CHECK(s_msel == 32 && s_round == R_READY && !s_go && trial_start_calls == 0);
    CHECK(strcmp(s_mname[32], "no_arm_single_pass_mission") == 0 && trial_report_calls == 1);
    run_cmd("b1d1"); CHECK(trial_distance_calls == 1 && trial_first_leg_mm == 1u);
    run_cmd("b1d2449"); CHECK(trial_distance_calls == 2 && trial_first_leg_mm == 2449u);
    run_cmd("b1d800"); CHECK(trial_distance_calls == 3 && trial_first_leg_mm == 800u);
    run_cmd("b1d0"); run_cmd("b1d2450"); run_cmd("b1d-1"); run_cmd("b1d");
    run_cmd("b1d800x"); run_cmd("b1d800.5"); run_cmd("b1d65537");
    CHECK(trial_distance_calls == 3 && trial_first_leg_mm == 800u);
    run_cmd("vsg1");
    CHECK(trial_alignment_calls == 1 && trial_alignment_cls == -1 &&
          trial_alignment_cx == -1 && trial_alignment_sign == 1);
    run_cmd("vsg2");
    CHECK(trial_alignment_calls == 2 && trial_alignment_sign == -1);
    for (int cls = 0; cls < 4; ++cls) {
        run_cmd(workpoint_keys[cls]);
        CHECK(trial_alignment_cls == cls && trial_alignment_cx >= 0 && trial_alignment_sign == 0);
    }
    run_cmd("ysg1");
    CHECK(trial_grab_y_calls == 1 && trial_grab_y_cls == -1 && trial_grab_y_cy == -1 && trial_grab_y_sign == 1);
    run_cmd("ysg2"); CHECK(trial_grab_y_calls == 2 && trial_grab_y_sign == -1);
    run_cmd("bcy150");
    CHECK(trial_grab_y_calls == 3 && trial_grab_y_cls == CLS_BALL && trial_grab_y_cy == 150 && trial_grab_y_sign == 0);
    run_cmd("hcy170");
    CHECK(trial_grab_y_calls == 4 && trial_grab_y_cls == CLS_HOSTAGE && trial_grab_y_cy == 170 && trial_grab_y_sign == 0);
    run_cmd("bcy0"); CHECK(trial_grab_y_calls == 5 && trial_grab_y_cy == 0);
    run_cmd("hcy65534"); CHECK(trial_grab_y_calls == 6 && trial_grab_y_cy == 65534);
    int changed = trial_alignment_calls;
    run_cmd("vsg0"); run_cmd("vsg3"); run_cmd("vsg2junk");
    run_cmd("bcx-3"); run_cmd("hcx"); run_cmd("tcx99x");
    CHECK(trial_alignment_calls == changed);
    int y_changed = trial_grab_y_calls;
    run_cmd("ysg0"); run_cmd("ysg3"); run_cmd("ysg2junk"); run_cmd("ysg");
    run_cmd("bcy-1"); run_cmd("bcy65535"); run_cmd("hcy"); run_cmd("hcy170x"); run_cmd("bcy12.5");
    CHECK(trial_grab_y_calls == y_changed);
    run_cmd("trial"); CHECK(trial_report_calls >= 8);
    run_cmd("0"); CHECK(!s_go && !host_abort && trial_start_calls == 0);

    for (unsigned i = 0; i < 3u; ++i) {
        reset_fixture(); run_cmd("32"); run_cmd("g");
        CHECK(s_go && !host_abort && trial_start_calls == 1 && start_calls == 0);
        CHECK(host_state == MS_BOOT && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
        /* Only a queued request: DefaultTask keeps handling Bluetooth. */
        host_tick += 100u; test_poll();
        CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f && pulse_calls == 0);
        run_cmd("bcx199"); run_cmd("vsg1"); run_cmd("bcy150"); run_cmd("hcy170"); run_cmd("ysg1");
        run_cmd("b1d800"); run_cmd("24"); run_cmd("co");
        CHECK(trial_alignment_calls == 0 && trial_grab_y_calls == 0 && trial_distance_calls == 0 && s_msel == 32 && host_servo == 1400u);
        run_cmd("trial");
        CHECK(trial_report_calls == 3); /* select, g automatic report, explicit report */
        laser_state = 1;
        run_cmd(stop_keys[i]);
        CHECK(host_abort && s_go && trial_start_calls == 1 && !laser_state);
        host_state = MS_DONE; run_cmd("g"); run_cmd("0");
        CHECK(host_abort && s_go && trial_start_calls == 1 && start_calls == 0);
        CHECK(pulse_calls == 0 && servo_calls == 0 && last_w == 0.0f);
    }
    reset_fixture(); run_cmd("32"); trial_gate_closed = 1; run_cmd("g");
    CHECK(!s_go && !host_abort && trial_start_calls == 0 && start_calls == 0);
    CHECK(strstr(last_message, "TRIAL32_UNCALIBRATED") != NULL);
    reset_fixture(); run_cmd("7"); run_cmd("p45"); run_cmd("g"); run_cmd("bcx180"); run_cmd("bcy150"); run_cmd("ysg2"); run_cmd("b1d800");
    CHECK(trial_alignment_calls == 0 && trial_grab_y_calls == 0 && trial_distance_calls == 0 && s_round == R_RUN);
    puts("trial32 router: selection/RAM XY workpoints/both signs/first-leg aliases, malformed writes, queued start, pending g/a/0, laser-off, terminal no-restart and write lock passed");
    return 0;
}

static void diagnostic_qr(int a, int b, int c)
{
    ProtoFrame frame = {0};
    frame.type = PF_QR; frame.a = a; frame.b = b; frame.c = c;
    test_vision_feed_frame(&frame);
}

static int check_vision33_router(void)
{
    static const char *const stops[] = { "g", "a", "0" };
    static const char *const writes[] = {
        "31", "32", "7", "r1", "v100", "d500", "p45", "fff0.02",
        "bcx160", "b1d800", "vsg1", "bcy150", "hcy170", "ysg2", "co", "cc", "su1800", "u1800", "nr10", "imu"
    };
    ProtoFrame frame = {0};
    reset_fixture(); run_cmd("33");
    CHECK(s_msel == 33 && s_round == R_READY && host_scene_calls == 0);
    CHECK(strcmp(s_mname[33], "vision_receive_only") == 0 && !s_go);
    diagnostic_qr(1, 2, 3); CHECK(!s_vdiag_qr_pending);
    run_cmd("g");
    CHECK(s_round == R_RUN && s_vdiag_phase == VD_QR_WAIT && host_scene == SCENE_QR);
    CHECK(s_active_test == 1u && !laser_state && !host_abort && !s_go);
    diagnostic_qr(0, 2, 3); CHECK(!s_vdiag_qr_pending);
    diagnostic_qr(1, 2, 3); diagnostic_qr(3, 2, 1);
    CHECK(s_vdiag_qr_pending && s_vdiag_qr[0] == 1 && s_vdiag_qr[2] == 3);
    test_poll(); CHECK(host_scene_calls == 1); /* ACK/fresh still absent. */
    host_scene_status = 1; test_poll();
    CHECK(s_vdiag_phase == VD_OBJECT_WAIT && host_scene == SCENE_EOD && host_scene_calls == 2);
    CHECK(!s_vdiag_qr_pending && !s_go && start_calls == 0 && trial_start_calls == 0);
    host_scene_status = 1; test_poll(); CHECK(s_vdiag_phase == VD_OBJECT);
    frame.type = PF_OBJ; frame.cls = CLS_HOSTAGE; frame.label = LAB_WAIST;
    frame.cx = 120; frame.cy = 135; frame.w = 30; frame.h = 70;
    frame.conf = 88; frame.sequence = 7; frame.img_w = frame.img_h = 320;
    host_proto_stats.obj = 1u;
    test_vision_feed_frame(&frame);
    CHECK(s_vdiag_sample[CLS_HOSTAGE].seen == 1u && s_vdiag_sample[CLS_HOSTAGE].packet_index == 1u);
    frame.label = LAB_CYL; test_vision_feed_frame(&frame);
    CHECK(s_vdiag_sample[CLS_HOSTAGE].seen == 1u); /* Non-selected shape is ignored. */
    run_cmd("vision");
    CHECK(strstr(host_messages, "cls=HOSTAGE lab=5 seen=1 latest=1") != NULL);
    CHECK(strstr(host_messages, "lastcx=120 lastcy=135") != NULL && strstr(host_messages, "img=320x320") != NULL);
    host_proto_stats.obj++; /* Empty packet: parser has no per-object callback. */
    host_messages[0] = '\0'; host_tick += 1000u; test_poll();
    CHECK(strstr(host_messages, "cls=HOSTAGE lab=5 seen=1 latest=0 age=1000") != NULL);
    CHECK(strstr(host_messages, "cls=BUCKET seen=0 latest=0 no_coordinate=1") != NULL);
    for (unsigned i = 0; i < sizeof writes / sizeof writes[0]; i++) {
        run_cmd(writes[i]);
        CHECK(s_msel == 33 && s_round == R_RUN && strstr(last_message, "VISION_DIAG_ACTIVE") != NULL);
        CHECK(host_servo == 1400u && pulse_calls == 0 && servo_calls == 0 &&
              last_x == 0.0f && last_y == 0.0f && last_w == 0.0f && !laser_state);
    }
    CHECK(zero_calls == 0 && prepare_calls == 0 && start_calls == 0 && trial_start_calls == 0);

    /* Every stop key in QR_WAIT, pending QR, OBJECT_WAIT, and OBJECT. */
    for (unsigned key = 0; key < 3; key++) {
        for (int phase = 0; phase < 4; phase++) {
            reset_fixture(); run_cmd("33"); run_cmd("g");
            if (phase >= 1) diagnostic_qr(1, 2, 3);
            if (phase >= 2) { host_scene_status = 1; test_poll(); }
            if (phase >= 3) { host_scene_status = 1; test_poll(); }
            int requests = host_scene_calls;
            ProtoScene previous_scene = host_scene;
            run_cmd(stops[key]);
            CHECK(s_vdiag_phase == VD_OFF && !s_vdiag_qr_pending && s_round == R_READY && s_msel == 33);
            CHECK(host_scene == previous_scene && host_scene_calls == requests && host_receive_closed);
            CHECK(strstr(last_message, "LOCAL_RX_STOP") != NULL);
            diagnostic_qr(2, 2, 2); test_vision_feed_frame(&frame);
            host_scene_status = 1; test_poll();
            CHECK(!s_vdiag_qr_pending && s_vdiag_sample[CLS_HOSTAGE].seen == 0u &&
                  host_scene == previous_scene && host_receive_closed && host_scene_calls == requests);
            CHECK(!s_go && !host_abort && start_calls == 0 && trial_start_calls == 0 &&
                  pulse_calls == 0 && servo_calls == 0 && !laser_state);
            run_cmd("g");
            CHECK(s_vdiag_phase == VD_QR_WAIT && host_scene == SCENE_QR && s_active_test == 2u);
            CHECK(!host_receive_closed && host_scene_calls == requests + 1);
            CHECK(s_vdiag_qr[0] == 0 && s_vdiag_sample[CLS_HOSTAGE].seen == 0u);
        }
    }
    reset_fixture();
    const char *batch = "33\ng\ng\n";
    while (*batch) test_feed((uint8_t)*batch++);
    test_poll(); CHECK(host_scene == SCENE_QR && host_receive_closed && host_scene_calls == 1 &&
                       s_vdiag_phase == VD_OFF && !s_go);
    reset_fixture(); host_state = MS_EOD; run_cmd("33"); CHECK(s_msel == R_FREE && !host_scene_calls);
    reset_fixture(); run_cmd("33"); run_cmd("g"); host_scene_status = -1; host_tick += 1000u; test_poll();
    CHECK(s_vdiag_phase == VD_QR_WAIT && strstr(host_messages, "link=-1") != NULL && !s_go);
    CHECK(strstr(host_messages, "VW req=") != NULL && strstr(last_message, "VR type=") != NULL);
    puts("vision33: ACK/fresh QR gating, current/empty packet snapshots, 20 write locks, 12 stops, late frames, same-batch cancellation and repeated test IDs passed; no actuator calls");
    return 0;
}

int main(void)
{
    reset_fixture();
    run_cmd("g"); /* accepted, but MissionTask is still BOOT */
    CHECK(start_calls == 1 && s_go && !host_abort);
    run_cmd("g");
    CHECK(host_abort && brake_calls >= 2 && start_calls == 1 && s_go);
    CHECK(strstr(last_message, "ABORT_REQUEST") != NULL);
    run_cmd("g"); /* never restart/clear abort with a third g */
    CHECK(host_abort && start_calls == 1);

    reset_fixture(); host_state = MS_READ_QR;
    run_cmd("g"); /* also stop starts from outside the Bluetooth router */
    CHECK(host_abort && brake_calls == 1 && start_calls == 0);
    reset_fixture(); host_state = MS_EOD; s_go = 1;
    run_cmd("g");
    CHECK(host_abort && brake_calls == 1 && start_calls == 0);
    reset_fixture(); host_state = MS_ABORT;
    run_cmd("g");
    CHECK(host_abort && start_calls == 0);
    reset_fixture(); host_state = MS_DONE;
    run_cmd("g");
    CHECK(host_abort && start_calls == 0);
    reset_fixture(); s_go = 1;
    run_cmd("a");
    CHECK(host_abort && brake_calls == 1);
    reset_fixture(); gate_closed = 1;
    run_cmd("g");
    CHECK(!host_abort && !s_go && start_calls == 0);
    CHECK(strstr(last_message, "MISSION_UNCALIBRATED") != NULL);

    reset_fixture();
    test_feed('g'); /* phone sends a single byte without a line ending */
    host_tick += T_IDLE_MS; test_poll();
    CHECK(start_calls == 1 && s_go && !host_abort && s_len == 0u);
    test_feed('g'); test_feed('\r'); test_feed('\n');
    CHECK(host_abort && start_calls == 1 && s_len == 0u);
    reset_fixture(); host_state = MS_EOD;
    test_feed('g'); host_tick += T_IDLE_MS; test_poll();
    CHECK(host_abort && start_calls == 0 && s_len == 0u);

    reset_fixture(); run_cmd("7"); run_cmd("p45"); run_cmd("g");
    CHECK(s_round == R_RUN && !s_go);
    run_cmd("g");
    CHECK(s_round == R_READY && !host_abort && !s_go && brake_calls >= 1);
    reset_fixture(); run_cmd("15"); run_cmd("v200"); run_cmd("d200"); run_cmd("g");
    CHECK(s_round == R_RUN);
    run_cmd("g");
    CHECK(s_round == R_BRAKE && !host_abort && brake_calls >= 1);
    /* Mode30 is the only new entry: left yaw, mirrored limit and signed correction. */
    reset_fixture(); run_cmd("30");
    CHECK(s_msel == 30 && strcmp(s_mname[30], "turn_left_90_hold") == 0);
    run_cmd("g"); tick();
    CHECK(s_round == R_RUN && last_x == 0.0f && last_y == 0.0f && last_w == -T_TURN_MAX_W);
    CHECK(strstr(last_message, "TURN90_LEFT") != NULL);
    host_yaw = -89.0f; tick(); CHECK(s_round == R_RUN && last_w < 0.0f);
    host_yaw = -90.0f; tick(); CHECK(s_round == R_RUN && last_w < 0.0f);
    host_yaw = -91.0f; tick(); CHECK(s_round == R_RUN && last_w < 0.0f);
    host_yaw = -91.5f; tick(); CHECK(s_round == R_RUN && last_w < 0.0f);
    host_yaw = -93.0f; tick(); CHECK(s_round == R_RUN && last_w > 0.0f);
    host_yaw = -92.0f; tick(); CHECK(s_round == R_BRAKE && last_w == 0.0f);
    host_yaw = -92.8f; tick(); CHECK(s_round == R_RUN); /* inertia reopens correction */
    tick(); CHECK(last_w > 0.0f);
    host_yaw = -92.0f; tick(); host_tick += T_TURN_SETTLE_MS; tick();
    CHECK(s_round == R_DONE && strstr(last_message, "type=TURN90_LEFT") != NULL &&
          strstr(last_message, "status=DONE") != NULL && strstr(last_message, "cmd_deg=-92") != NULL);
    run_cmd("g"); CHECK(s_round == R_READY && last_w == 0.0f); /* clear only, no reverse */

    reset_fixture(); run_cmd("30"); run_cmd("g"); tick(); run_cmd("g");
    CHECK(s_round == R_BRAKE && strcmp(s_turn_result, "STOP") == 0 && last_w == 0.0f);
    host_yaw = -20.0f; host_tick += T_TURN_SETTLE_MS; tick();
    CHECK(s_round == R_DONE && strstr(last_message, "status=STOP") != NULL && last_w == 0.0f);
    reset_fixture(); run_cmd("30"); run_cmd("g"); host_yaw = -92.0f; tick(); run_cmd("g");
    host_yaw = -93.0f; host_tick += T_TURN_SETTLE_MS; tick();
    CHECK(s_round == R_DONE && strstr(last_message, "status=STOP") != NULL && last_w == 0.0f);

    /* Same range as the old positive controller; mirrored range for left. */
    for (int mode = 20; mode <= 30; mode++) {
        if (mode != 20 && mode != 22 && mode != 30) continue;
        reset_fixture(); s_msel = mode;
        float target = turn_target_deg();
        float dir = target < 0.0f ? -1.0f : 1.0f;
        CHECK(!turn_angle_outside(-dir * T_TURN_LIMIT_DEG));
        CHECK(!turn_angle_outside(target + dir * T_TURN_LIMIT_DEG));
        CHECK(turn_angle_outside(-dir * (T_TURN_LIMIT_DEG + 0.1f)));
        CHECK(turn_angle_outside(target + dir * (T_TURN_LIMIT_DEG + 0.1f)));
    }
    reset_fixture(); run_cmd("30"); run_cmd("g"); host_yaw = -111.0f; tick();
    CHECK(s_round == R_BRAKE && strcmp(s_turn_result, "ANGLE_LIMIT") == 0 && last_w == 0.0f);
    reset_fixture(); run_cmd("30"); run_cmd("g"); host_yaw = -92.0f; tick();
    host_yaw = 16.0f; tick(); CHECK(s_round == R_DONE && strstr(last_message, "status=ANGLE_LIMIT") != NULL);
    reset_fixture(); run_cmd("30"); run_cmd("g"); host_tick += T_TURN_MAX_MS; tick();
    CHECK(s_round == R_BRAKE && strcmp(s_turn_result, "TIMEOUT") == 0 && last_w == 0.0f);
    reset_fixture(); run_cmd("30"); run_cmd("g"); host_imu_valid = 0; tick();
    CHECK(s_round == R_BRAKE && strcmp(s_turn_result, "IMUERR") == 0 && last_w == 0.0f);
    tick(); CHECK(s_round == R_DONE && strstr(last_message, "status=IMUERR") != NULL);
    reset_fixture(); run_cmd("30"); host_imu_valid = 0; run_cmd("g");
    CHECK(s_round == R_READY && last_w == 0.0f && strstr(last_message, "ERR TURN_PREP") != NULL);
    reset_fixture(); run_cmd("30"); run_cmd("g"); tick(); run_cmd("a");
    CHECK(s_round == R_READY && last_w == 0.0f);
    reset_fixture(); run_cmd("30"); run_cmd("g"); tick(); run_cmd("0");
    CHECK(s_round == R_READY && last_w == 0.0f);
    reset_fixture(); run_cmd("42"); CHECK(s_msel == R_FREE && strstr(last_message, "MODE_RANGE") != NULL);
    for (int mode = 20; mode <= 22; mode += 2) {
        reset_fixture(); s_msel = mode; run_cmd("g"); tick();
        CHECK(s_round == R_RUN && last_w == T_TURN_MAX_W);
        host_yaw = turn_target_deg(); tick(); host_tick += T_TURN_SETTLE_MS; tick();
        CHECK(s_round == R_DONE && strstr(last_message, "status=DONE") != NULL &&
              strstr(last_message, mode == 20 ? "cmd_deg=90" : "cmd_deg=180") != NULL);
    }
    puts("left90 entry: signed start/overshoot/settle/stop, mirrored limits, timeout/IMU, a/0, right90/180 preservation passed");
    reset_fixture(); run_cmd("20"); run_cmd("g");
    CHECK(s_round == R_RUN);
    run_cmd("g");
    CHECK(s_round == R_BRAKE && !host_abort && brake_calls >= 1);
    reset_fixture(); run_cmd("14"); run_cmd("g");
    CHECK(s_round == R_RUN);
    run_cmd("g");
    CHECK(s_round == R_READY && !host_abort);

    reset_fixture(); run_cmd("24"); run_cmd("nr5"); run_cmd("g");
    CHECK(s_round == R_RUN);
    run_cmd("g");
    CHECK(s_round == R_DONE && pulse_calls == 0 && !host_abort);
    run_cmd("g");
    CHECK(s_round == R_READY && pulse_calls == 0); /* clear only, no return */
    reset_fixture(); run_cmd("26"); run_cmd("nr5"); run_cmd("g");
    CHECK(s_round == R_RUN);
    run_cmd("g");
    CHECK(s_round == R_READY && pulse_calls == 0);
    host_tick += 3000u; tick();
    CHECK(s_round == R_READY && pulse_calls == 0); /* auto return canceled */
    reset_fixture(); run_cmd("28"); run_cmd("u1500"); run_cmd("g");
    CHECK(s_round == R_DONE && host_servo == 1500u);
    run_cmd("g"); host_tick += 3000u; tick();
    CHECK(s_round == R_READY && host_servo == 1500u); /* holds PWM, no auto return */

    reset_fixture(); run_cmd("5"); run_cmd("v100"); run_cmd("d100"); run_cmd("g");
    host_tick += 100u; run_cmd("g");
    CHECK(s_round == R_DONE);
    run_cmd("g"); /* existing explicit third-g return is retained */
    CHECK(s_round == R_RET);
    run_cmd("g");
    CHECK(s_round == R_READY && s_leg_run == 0u && brake_calls >= 1);
    CHECK(check_forward_compensation() == 0);
    CHECK(check_shared_turn_hold() == 0);
    CHECK(check_route_sequence() == 0);
    CHECK(check_retained34_bucket_stub() == 0);
    CHECK(check_route_cross_heading_restart(31u) == 0);
    CHECK(check_route_cross_heading_restart(34u) == 0);
    CHECK(check_trial32_router() == 0);
    CHECK(check_vision33_router() == 0);
    puts("real g router: pending/running/terminal mission stop, a alias, closed gate, CRLF/idle framing, wheel/distance/turn/IMU/jog/servo/return passed");
    return 0;
}
