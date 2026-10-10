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
    int c, l;
    if (task == PROTO_TASK_BUCKET && digit == 0u) { c = CLS_BUCKET; l = -1; }
    else {
        if (digit < 1u || digit > 3u) return 0;
        if (task == PROTO_TASK_BALL) { c = CLS_BALL; l = digit - 1; }
        else if (task == PROTO_TASK_TARGET) { c = CLS_TARGET; l = digit - 1; }
        else if (task == PROTO_TASK_HOSTAGE) { c = CLS_HOSTAGE; l = digit + 2; }
        else return 0;
    }
    if (cls) *cls = c;
    if (label) *label = l;
    return 1;
}
#define proto_target_filter host_xy_filter
#include "../App/vision_align_test.c"
#undef proto_target_filter

static uint32_t host_tick;
/* Host-only timer model: requested pps rounded to a 1 us event period.
 * This verifies IRQ/state-machine semantics, not actual TIM7 register timing. */
static uint64_t host_timer_us, host_timer_next_us, host_timer_origin_us;
static uint32_t host_timer_period_us;
static uint16_t host_timer_pps;
static int host_timer_active;
static int host_timer_start_fail;
static unsigned host_timer_start_calls, host_timer_stop_calls, host_timer_rephase_calls, host_uart_calls;
static MissionState host_state;
static int host_abort, start_calls, brake_calls, pulse_calls, gate_closed;
static int host_jog_axis, host_jog_dir;
static unsigned host_jog_pulses[2][2];
static int host_imu_valid;
static float host_yaw, host_absolute_yaw, last_x, last_y, last_w;
static float host_pitch, host_roll;
static uint32_t host_imu_age;
static float host_heading_kp;
static unsigned host_precise_calls, host_integer_calls;
static float host_fore, host_lateral;
static int32_t host_counts[4];
static void (*host_counts_hook)(int motor);
static void (*host_motion_brake_hook)(void);
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
static int (*host_rank_get_hook)(ProtoTargetRank *out);
static void (*host_wire_diag_hook)(ProtoWireDiag *out);
static char host_messages[8192];
static MotionProfileTune host_motion_profile;

uint32_t HAL_GetTick(void) { return host_tick; }
void bp_debug_send(const char *s)
{
    host_uart_calls++;
    /* Production send() appends CRLF in a separate UART write. */
    if (strcmp(s, "\r\n") != 0)
        snprintf(last_message, sizeof last_message, "%s", s);
    if (strlen(host_messages) + strlen(s) < sizeof host_messages)
        strcat(host_messages, s);
}
void bp_laser_set(int on) { laser_state = on; if (on) host_laser_on_calls++; }
int32_t bp_enc_raw_total(int m) { (void)m; return 0; }
void bp_enc_raw_reset_all(void) { }
void motion_brake(void)
{
    brake_calls++; last_x = last_y = last_w = 0.0f;
    if (host_motion_brake_hook) host_motion_brake_hook();
}
void motion_vel_set(float x, float y, float w)
{ host_integer_calls++; last_x = x; last_y = y; last_w = w; }
void motion_vel_set_precise(float x, float y, float w)
{ host_precise_calls++; last_x = x; last_y = y; last_w = w; }
void motion_vel_set_creep(float x, float y, float w)
{ motion_vel_set_precise(x, y, w); }
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
void arm_claw_close(void) { host_servo = ARM_SERVO_GRIP_US; }
void arm_stepper_dir(int axis, int dir) { host_jog_axis = axis; host_jog_dir = dir; }
void arm_stepper_step(int axis)
{
    pulse_calls++;
    if (axis >= 0 && axis < 2 && host_jog_dir >= 0 && host_jog_dir < 2)
        host_jog_pulses[axis][host_jog_dir]++;
}
static void host_timer_sync_hal(void)
{
    uint32_t observed = (uint32_t)(host_timer_us / 1000u);
    host_timer_us += (uint64_t)(uint32_t)(host_tick - observed) * 1000u;
}
int arm_stepper_clock_start(uint16_t pps)
{
    host_timer_start_calls++;
    if (host_timer_start_fail || pps == 0u || pps > 20000u) {
        host_timer_active = 0;
        return 0;
    }
    host_timer_sync_hal();
    host_timer_origin_us = host_timer_us;
    host_timer_pps = pps;
    host_timer_period_us = (1000000u + pps / 2u) / pps;
    host_timer_next_us = host_timer_us + host_timer_period_us;
    host_timer_active = 1;
    return 1;
}
void arm_stepper_clock_stop(void)
{
    host_timer_active = 0;
    host_timer_stop_calls++;
}
void arm_stepper_clock_rephase(void)
{
    host_timer_rephase_calls++;
    if (host_timer_active) host_timer_next_us = host_timer_us + host_timer_period_us;
}
float imu_yaw_deg(void) { return 0.0f; }
float imu_heading_deg(void) { return host_absolute_yaw; }
float imu_leg_heading_deg(void) { return host_yaw; }
float imu_pitch_deg(void) { return host_pitch; }
float imu_roll_deg(void) { return host_roll; }
uint8_t imu_ok(void) { return (uint8_t)host_imu_valid; }
uint8_t imu_zero_leg_heading(void) { zero_calls++; host_yaw = 0.0f; return (uint8_t)host_imu_valid; }
uint32_t imu_last_valid_age_ms(void) { return host_imu_age; }
uint8_t imu_tilt_snapshot(float *pitch, float *roll, uint32_t *sample_ms)
{
    if (!host_imu_valid || host_imu_age >= 200u) return 0u;
    if (pitch) *pitch = host_pitch;
    if (roll) *roll = host_roll;
    if (sample_ms) *sample_ms = host_tick - host_imu_age;
    return 1u;
}
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
int proto_target_rank_get(ProtoTargetRank *out)
{
    if (host_rank_get_hook) return host_rank_get_hook(out);
    if (out) memset(out, 0, sizeof *out);
    return 0; /* No fixture rank means unknown, never an invented road position. */
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
    host_timer_us = 100000u; host_timer_next_us = host_timer_origin_us = host_timer_us;
    host_timer_period_us = 0u; host_timer_pps = 0u; host_timer_active = host_timer_start_fail = 0;
    host_timer_start_calls = host_timer_stop_calls = host_timer_rephase_calls = host_uart_calls = 0u;
    host_abort = start_calls = brake_calls = pulse_calls = gate_closed = 0;
    host_jog_axis = host_jog_dir = 0;
    memset(host_jog_pulses, 0, sizeof host_jog_pulses);
    host_imu_valid = 1; host_yaw = host_absolute_yaw = last_x = last_y = last_w = 0.0f;
    host_pitch = host_roll = 0.0f; host_imu_age = 0u;
    host_wire_diag_hook = NULL;
    host_heading_kp = 0.3f; host_precise_calls = host_integer_calls = 0u;
    host_fore = host_lateral = 0.0f; memset(host_counts, 0, sizeof host_counts);
    host_counts_hook = NULL;
    host_motion_brake_hook = NULL;
    zero_calls = prepare_calls = servo_calls = 0;
    host_servo = ARM_SERVO_START_US; last_message[0] = '\0';
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
    host_rank_get_hook = NULL;
    host_messages[0] = '\0';
    memset(&host_motion_profile, 0, sizeof host_motion_profile);
    (void)motion_profile_set(&host_motion_profile);
    test_init();
}

#define CHECK(expr) do { if (!(expr)) { fprintf(stderr, "g test line %d: %s\n", __LINE__, #expr); return 1; } } while (0)

static int jog_host_ticks(unsigned steps);
static int jog_host_service_ms(unsigned ms);

static int fixture_complete_route31_predeploy(void)
{
    if (s_seq_state != SQ_ARM_PREP) return 0;
    CHECK(s_seq_mode == 31u && s_seq_stage == ROUTE31_PREDEPLOY_STAGE);
    CHECK(s_route31_lift_phase == R31_RACK_EXTEND && host_timer_active);
    CHECK(host_jog_axis == 0 && host_jog_dir == 0 && host_timer_pps == 5000u);
    /* Independent3400 expectation pins the new31 recipe,
     * rather than letting the host fixture silently copy a changed macro. */
    CHECK(ROUTE31_RACK_EXTEND_STEPS == 3400u && s_jog_clock.remaining == 3400u);
    int before = pulse_calls;
    CHECK(jog_host_ticks(3399u) == 0 && s_jog_clock.remaining == 1u && host_timer_active);
    test_poll();
    CHECK(s_seq_state == SQ_ARM_PREP && s_route31_lift_phase == R31_RACK_EXTEND &&
          pulse_calls == before + 3399 && !s_route31_rack_deployed);
    CHECK(jog_host_ticks(1u) == 0);
    test_poll();
    CHECK(s_seq_state == SQ_ARM_PREP && s_route31_lift_phase == R31_RACK_EXTEND_WAIT &&
          pulse_calls == before + 3400 && !host_timer_active &&
          host_jog_pulses[0][0] == 3400u && !host_jog_pulses[0][1]);
    CHECK(jog_host_service_ms(250u) == 0);
    test_poll();
    CHECK(s_seq_state == SQ_STILL && s_route31_rack_deployed && s_msel == 30 &&
          s_route31_lift_phase == R31_LIFT_OFF && !servo_calls &&
          last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    return 0;
}

static int fixture_complete_route31_cross_yaw(void)
{
    if (s_seq_state != SQ_CROSS_YAW) return 0;
    CHECK(s_seq_mode == 31u && s_seq_stage == 4u &&
          s_route31_cross_heading_valid && !s_route31_cross_yaw_done);
    unsigned zeros = (unsigned)zero_calls;
    /* Supply a corrected physical endpoint, never invent a missing reference.
     * Keep the absolute/local sensor frames coherent, including wrapped goals. */
    float correction = fmodf(s_route31_task_yaw_goal - host_yaw, 360.0f);
    if (correction > 180.0f) correction -= 360.0f;
    if (correction < -180.0f) correction += 360.0f;
    host_absolute_yaw += correction;
    host_yaw = s_route31_task_yaw_goal;
    test_poll();
    CHECK(s_seq_state == SQ_CROSS_YAW && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    host_tick += 400u; test_poll();
    CHECK(s_seq_state == SQ_STILL && s_seq_stage == 4u &&
          s_route31_cross_yaw_done && (unsigned)zero_calls == zeros &&
          last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    return 0;
}

static int sequence_start_stage(void)
{
    int board_contact = route31_owner() && s_seq_stage == 4u && !s_seq_contact_post;
    int contact_nudge = route31_owner() && s_seq_stage == 4u && s_seq_contact_post;
    CHECK(fixture_complete_route31_predeploy() == 0);
    CHECK(fixture_complete_route31_cross_yaw() == 0);
    CHECK(s_seq_state == SQ_STILL && last_w == 0.0f && last_x == 0.0f && last_y == 0.0f);
    host_tick += T_DIST_STILL_MS;
    test_poll();
    if (contact_nudge) {
        CHECK(s_seq_state == SQ_RUN && s_round == R_RUN && prepare_calls == 0);
        return 0;
    }
    CHECK(s_seq_state == SQ_WAIT && last_w == 0.0f && last_x == 0.0f && last_y == 0.0f);
    host_tick += board_contact ? ROUTE31_BOARD_CONTACT_ZERO_WAIT_MS : NAV_SETTLE_MS;
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

static int fixture_trigger_board_contact(void)
{
    CHECK(route31_owner() && s_seq_stage == 4u && s_seq_state == SQ_RUN && s_round == R_RUN);
    /* Contact ignores startup motion, then requires fresh observations over
     * a sustained interval. Encoder distance never substitutes for tilt. */
    host_pitch += ROUTE31_BOARD_CONTACT_TILT_DEG + 0.1f;
    host_imu_age = 0u;
    host_tick += ROUTE31_BOARD_CONTACT_START_GUARD_MS + 20u; test_poll();
    CHECK(s_round == R_RUN);
    for (unsigned elapsed = 0u; elapsed < ROUTE31_BOARD_CONTACT_CONFIRM_MS; elapsed += 20u) {
        host_tick += 20u; test_poll();
    }
    CHECK(s_round == R_BRAKE && s_dist_reason == 1u);
    return 0;
}

static int sequence_finish_stage(void)
{
    CHECK(s_seq_state == SQ_RUN);
    if (dist_mode()) {
        if (route31_owner() && s_seq_stage == 4u && !s_seq_contact_post) {
            CHECK(fixture_trigger_board_contact() == 0);
        } else {
            if (dist_lateral()) host_lateral = s_dist_odo0 + s_dist_target;
            else host_fore = s_dist_odo0 + s_dist_target;
            test_poll();
        }
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
    CHECK(fixture_complete_route31_cross_yaw() == 0);
    /* Ordinary mode31 stage drivers consume this logical contact unit's
     * BACK10 only. Mode43 still exposes BACK10/FWD15 behind fresh g gates. */
    while (s_seq_mode == ROUTE_TEST_MODE && s_seq_stage == 4u && s_seq_contact_post &&
           s_seq_state == SQ_STILL) {
        CHECK(sequence_start_stage() == 0);
        CHECK(sequence_finish_stage() == 0);
    }
    return 0;
}

/* Tests which focus only on stage4 must first establish a real legal stage3
 * reference and complete its correction, not bypass the production guard. */
static int fixture_prepare_route31_board(void)
{
    CHECK(s_seq_mode == 31u);
    s_seq_stage = 3u; route_seq_prepare();
    CHECK(sequence_start_stage() == 0);
    CHECK(s_route31_cross_heading_valid && !s_route31_cross_yaw_done);
    CHECK(sequence_finish_stage() == 0);
    CHECK(s_seq_stage == 4u && s_seq_state == SQ_STILL && s_route31_cross_yaw_done);
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
        if (mode == 31u) host_absolute_yaw = run == 1u ? 37.0f : -37.0f;
        s_seq_stage = 3u; route_seq_prepare();
        CHECK(sequence_start_stage() == 0);
        if (mode == 31u)
            CHECK(s_route31_cross_heading_valid && !s_route31_cross_yaw_done &&
                  s_route31_cross_heading == (run == 1u ? 37.0f : -37.0f));
        CHECK(s_msel == 16 && s_dist_target == (mode == 31u ? -620.0f : -650.0f) && s_v == 300.0f && s_dist_heading_kp == 0.0f);
        CHECK(s_dist_ff_ratio == 0.0f && last_x == -300.0f && last_y == 0.0f);
        host_yaw = 6.0f;
        if (mode == 31u) host_absolute_yaw = s_route31_cross_heading + host_yaw;
        tick(); CHECK(last_w == 0.0f);
        host_yaw = -6.0f;
        if (mode == 31u) host_absolute_yaw = s_route31_cross_heading + host_yaw;
        tick(); CHECK(last_w == 0.0f);
        CHECK(s_route_heading_kp == 1.2f && step_heading_kp_deg() == 0.3f);
        CHECK(sequence_finish_stage() == 0 && s_seq_stage == 4u);
        CHECK(sequence_start_stage() == 0);
        CHECK(s_msel == 15 && s_dist_target == (mode == 31u ? 0.0f : 80.0f) && s_v == (mode == 31u ? 40.0f : 20.0f) && s_dist_heading_kp == 0.0f);
        CHECK(last_x == (mode == 31u ? 40.0f : 20.0f) && last_y == 0.0f && s_dist_ff_ratio == 0.0f);
        host_yaw = 6.0f; tick(); CHECK(last_w == 0.0f);
        host_yaw = -6.0f; tick(); CHECK(last_w == 0.0f);
        unsigned before_finish = (unsigned)zero_calls;
        CHECK(sequence_finish_stage() == 0);
        CHECK(s_seq_stage == 5u && s_seq_state == SQ_STILL &&
              last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
        CHECK(zero_calls == (int)before_finish + (mode == 31u ? 0 : 1));
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
        CHECK(s_dist_target == -190.0f && s_v == (mode == 31u ? 200.0f : 100.0f) && s_dist_heading0 == 0.0f);
        CHECK(s_dist_heading_kp == 1.2f && s_route_heading_kp == 1.2f &&
              step_heading_kp_deg() == 0.3f && host_yaw == 0.0f && last_w == 0.0f);
        CHECK(s_dist_ff_ratio == 0.00625f && last_x == (mode == 31u ? -200.0f : -100.0f) &&
              last_y == (mode == 31u ? -1.25f : -0.625f));
        /* Check restored moving gain below31's1.5deg mid-yaw brake.
         * The dedicated mid-yaw fixture covers larger errors and resume. */
        host_yaw = 1.0f; tick();
        CHECK(fabsf(last_w + 1.2f * 1.0f * 0.0174533f) < 0.000001f);
        host_yaw = -1.0f; tick();
        CHECK(fabsf(last_w - 1.2f * 1.0f * 0.0174533f) < 0.000001f);
        run_cmd("g");
        CHECK(s_seq_state == SQ_STOPPED && s_msel == (int)mode &&
              s_route_heading_kp == 1.2f && step_heading_kp_deg() == 0.3f);
    }
    run_cmd("15"); run_cmd("v100"); run_cmd("d100"); run_cmd("g");
    CHECK(s_dist_heading_kp == 0.3f && s_route_heading_kp == 1.2f);
    run_cmd("g"); run_cmd("32");
    CHECK(dist_heading_kp_get(NULL) == 0.3f && step_heading_kp_deg() == 0.3f);
    printf("route%u two-run post-cross: back%u/v300 ->forward%s/v%u yaw/FF disabled; wheel-still/node zero/750ms wait ->fresh back190/v%u restoresykp + independentBFF; manual/32 unchanged passed\n", mode, mode == 31u ? 620u : 650u, mode == 31u ? "tilt_contact" : "80", mode == 31u ? 40u : 20u, mode == 31u ? 200u : 100u);
    return 0;
}

static int check_route_sequence(void)
{
    /* Independent expectation, not copied from the live recipe at runtime. */
    static const int expected_modes[9] = {17,16,20,16,15,16,18,16,30};
    static const int expected_commands[9] = {-575,-610,90,-620,0,-190,780,-810,-90};
    static const float expected_speeds[9] = {250,200,100,300,40,200,250,200,100};
    static const char *const stop_keys[] = {"g", "a", "0"};
    reset_fixture();
    host_fore = 4321.0f; host_lateral = -1234.0f; host_yaw = 47.0f;
    run_cmd("31");
    CHECK(s_seq_state == SQ_READY && s_round == R_READY && s_msel == 31);
    for (int i = 0; i < 5; i++) { host_tick += 100u; test_poll(); }
    CHECK(s_seq_state == SQ_READY && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    run_cmd("g"); CHECK(s_seq_run == 1u && s_seq_stage == 0u);
    CHECK(ROUTE31_STAGES == 16u && route_seq_stage_count() == 16u && !route_seq_bucket_enabled());
    CHECK(ROUTE31_PAIR_STAGE == 9u && ROUTE31_RETURN180_STAGE == 10u && ROUTE31_TARGET_STAGE == 11u &&
          ROUTE31_TARGET_CORNER_STAGE == 12u && ROUTE31_HOSTAGE_TURN_STAGE == 13u && ROUTE31_HOSTAGE_STAGE == 14u &&
          ROUTE31_HOSTAGE_EXIT_STAGE == 15u);
    for (int i = 0; i < 9; i++) {
        CHECK(s_seq_stage == i && sequence_start_stage() == 0);
        CHECK(s_msel == expected_modes[i] && s_route_leg == i + 1 &&
              s_active_test == (uint32_t)i + 1u + (i == 8 ? 1u : 0u));
        CHECK(s_route31_plan[i].heading_hold == (i == 3 || i == 4 ? 0u : 1u));
        if (dist_mode()) {
            CHECK(s_dist_target == (float)expected_commands[i] && s_v == expected_speeds[i]);
            CHECK(s_dist_ramp.acc == 700.0f && s_dist_ramp.dec == 350.0f && s_dist_precise == 1u);
            CHECK(s_dist_ff_ratio == (i == 1 || i == 5 || i == 7 ? 0.00625f : 0.0f));
            CHECK(s_dist_heading_kp == (i == 3 || i == 4 ? 0.0f : i == 6 ? 3.0f : 0.3f));
            CHECK(last_w == 0.0f);
            if (dist_lateral()) CHECK(last_x == 0.0f && last_y == (s_msel == 18 ? 250.0f : -250.0f));
            else CHECK(last_x == (s_msel == 16 ? -expected_speeds[i] : expected_speeds[i]) &&
                       last_y == -s_dist_ff_ratio * expected_speeds[i]);
        } else {
            CHECK(turn_target_deg() == (float)expected_commands[i] && last_x == 0.0f && last_y == 0.0f);
            CHECK(last_w == (i == 8 ? -2.0f*T_TURN_MAX_W : 2.0f*T_TURN_MAX_W));
        }
        CHECK(start_calls == 0 && pulse_calls == (i == 8 ? 3400 : 0) && servo_calls == 0 && !s_go);
        CHECK(sequence_finish_stage() == 0);
        CHECK(s_seq_state == (i == 8 ? SQ_TASK : i == 7 ? SQ_ARM_PREP : SQ_STILL));
        CHECK(host_target_calls == (i == 8 ? 1 : 0) && s_seq_state != SQ_BUCKET_ALIGN && s_seq_state != SQ_MANUAL_D_WAIT);
    }
    CHECK(zero_calls == 14 && prepare_calls == 0); /* Crossing retains yaw; board/nudges have no report zero. */
    CHECK(s_seq_stage == ROUTE31_PAIR_STAGE && vision_align_test_active() && s_vat.mode == 41u &&
          s_vat.state == VAT_BRAKE && host_target_task == PROTO_TASK_BALL && host_target_digit == 1u);
    CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f && !laser_state && pulse_calls == 3400 && !host_timer_active && !servo_calls);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && !vision_align_test_active() && host_receive_closed);
    /* Terminal router policy only; the dedicated real-wire task regression
     * verifies reaching DONE through ball/bucket/target/hostage, not this stub. */
    route_seq_end("DONE");
    CHECK(s_msel == 31 && s_round == R_DONE && strstr(last_message, "status=DONE") != NULL);
    for (int i = 0; i < 10; i++) { host_tick += 1000u; test_poll(); }
    run_cmd("g"); CHECK(s_seq_state == SQ_DONE && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    run_cmd("31"); run_cmd("g"); CHECK(s_seq_run == 2u && s_seq_state == SQ_STILL);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED);

    /* All nine prefix road nodes, every phase and stop key. Task phases have
     * their own integration/stop regression instead of pretending to be roads. */
    for (int stage = 0; stage < 9; stage++) {
        for (int key = 0; key < 3; key++) {
            for (int phase = 0; phase < 4; phase++) {
                reset_fixture(); run_cmd("31"); run_cmd("g");
                if (stage == 4) CHECK(fixture_prepare_route31_board() == 0);
                else { s_seq_stage = (uint8_t)stage; route_seq_prepare(); }
                if (phase >= 1) CHECK(fixture_complete_route31_predeploy() == 0);
                if (phase == 1) { host_tick += T_DIST_STILL_MS; test_poll(); CHECK(s_seq_state == SQ_WAIT); }
                if (phase >= 2) CHECK(sequence_start_stage() == 0);
                if (phase == 3) {
                    if (route31_owner() && s_seq_stage == 4u) {
                        CHECK(fixture_trigger_board_contact() == 0);
                    } else if (dist_mode()) {
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
                      !s_go && start_calls == 0 && pulse_calls == (stage == 8 && phase >= 1 ? 3400 : 0) && !host_timer_active && servo_calls == 0);
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
            CHECK(strstr(last_message, "ERR ROUTE_SEQ_ACTIVE") != NULL && s_v == 250.0f &&
                  s_d == 575.0f && s_seq_stage == 0u && s_msel == 17 && servo_calls == 0);
        }
        CHECK(route_seq_active()); run_cmd("g");
    }
    /* All failed sensor/turn statuses cancel rather than advancing a ready/done submode. */
    for (int stage = 0; stage < 9; stage++) {
        reset_fixture(); run_cmd("31"); run_cmd("g");
        if (stage == 4) CHECK(fixture_prepare_route31_board() == 0);
        else { s_seq_stage = (uint8_t)stage; route_seq_prepare(); }
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
    puts("route31: nine-road prefix R1left575/R2back610/right90/cross620/board_tilt1.0_wait300ms_guard300_confirm100_v40/no_contact_nudges, crossing/contact yaw/FF disabled, QRgate/right780/back810/rackDIR0nl3400predeploy/left90 then actual ball/bucket41 task handoff; independent3400 recipe and3399-not-DONE boundary, no legacybucket/manuald, three backwardBFF legs, post-yaw-before-next, terminal router policy,108 road g/a/0 cancellations, locks and IMU/abort/turn failure passed");
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
        CHECK(trial_alignment_calls == 0 && trial_grab_y_calls == 0 && trial_distance_calls == 0 && s_msel == 32 && host_servo == ARM_SERVO_START_US);
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

static int jog_host_timer_event(void)
{
    CHECK(host_timer_active && host_timer_period_us != 0u);
    int before = pulse_calls;
    unsigned uart_before = host_uart_calls;
    unsigned rephase_before = host_timer_rephase_calls;
    host_timer_us = host_timer_next_us;
    host_tick = (uint32_t)(host_timer_us / 1000u);
    test_stepper_timer_irq();
    CHECK(pulse_calls >= before && pulse_calls <= before + 1 && host_uart_calls == uart_before &&
          host_timer_rephase_calls == rephase_before + (unsigned)(pulse_calls - before));
    if (host_timer_active) host_timer_next_us = host_timer_us + host_timer_period_us;
    return 0;
}

static int jog_host_service_ms(unsigned ms)
{
    host_timer_sync_hal();
    uint64_t until = host_timer_us + (uint64_t)ms * 1000u;
    while (host_timer_active && host_timer_next_us <= until)
        CHECK(jog_host_timer_event() == 0);
    host_timer_us = until;
    host_tick = (uint32_t)(host_timer_us / 1000u);
    tick();
    return 0;
}

static uint32_t jog_host_expected_ms(unsigned steps)
{
    uint64_t gate_us = (host_timer_origin_us / 1000u + 2u) * 1000u;
    uint64_t first_event = (gate_us - host_timer_origin_us + host_timer_period_us - 1u) / host_timer_period_us;
    if (first_event == 0u) first_event = 1u;
    uint64_t end_us = host_timer_origin_us + (first_event + steps - 1u) * host_timer_period_us;
    return (uint32_t)((end_us / 1000u) - (host_timer_origin_us / 1000u));
}

/* Scheduled TIM7 events drive pulses independently of the command/report
 * task. Multiple real timer events within one HAL millisecond are expected. */
static int jog_host_ticks(unsigned steps)
{
    int target = pulse_calls + (int)steps;
    unsigned budget = steps + T_JOG_MAX_PPS / 500u + 2u;
    host_timer_sync_hal();
    while (pulse_calls < target && budget--) {
        CHECK(jog_host_timer_event() == 0);
    }
    CHECK(pulse_calls == target);
    tick();
    return 0;
}

static int check_jog_range_and_repeat(void)
{
    static const unsigned counts[] = { 1u, 500u, 1000u, 5000u, 5001u, 100000u };
    static const char *const rejected[] = {
        "nl100001", "nr100001", "n100001", "n-100001", "nl0", "nr0", "nl-1", "nr-1"
    };
    char mode_line[8], request_line[20], report_line[64];
    CHECK(T_JOG_MAX_STEPS == 100000u);
    CHECK(sizeof s_jog_request >= 4u && sizeof s_jog_done >= 4u && sizeof s_back >= 4u);
    for (int mode = 24; mode <= 27; ++mode) {
        snprintf(mode_line, sizeof mode_line, "%d", mode);
        for (unsigned dir = 0u; dir < 2u; ++dir) {
            for (unsigned k = 0u; k < sizeof counts / sizeof counts[0]; ++k) {
                reset_fixture(); run_cmd(mode_line);
                snprintf(request_line, sizeof request_line, "n%c%u", dir ? 'r' : 'l', counts[k]);
                run_cmd(request_line);
                CHECK(s_round == R_READY && pulse_calls == 0 &&
                      s_jog_request == (dir ? (int32_t)counts[k] : -(int32_t)counts[k]));
                snprintf(report_line, sizeof report_line, "steps=%u; g starts bounded", counts[k]);
                CHECK(strstr(last_message, "OK JOG_N") != NULL && strstr(last_message, report_line) != NULL);
                run_cmd("g");
                CHECK(s_round == R_RUN && s_axis == ((mode - 24) & 1) &&
                      host_jog_axis == s_axis && host_jog_dir == (int)dir && pulse_calls == 0);
                snprintf(report_line, sizeof report_line, " n=%u ", counts[k]);
                CHECK(strstr(last_message, report_line) != NULL &&
                      strstr(last_message, "g_stops_no_return") != NULL);
            }
        }
        reset_fixture(); run_cmd(mode_line); run_cmd("nl100000");
        for (unsigned k = 0u; k < sizeof rejected / sizeof rejected[0]; ++k) {
            run_cmd(rejected[k]);
            CHECK(strstr(last_message, "ERR N_RANGE") != NULL &&
                  s_round == R_READY && s_jog_request == -100000 && pulse_calls == 0);
        }
        run_cmd("n100000"); CHECK(s_jog_request == 100000);
        run_cmd("n-100000"); CHECK(s_jog_request == -100000);
    }

    /* An uninterrupted auto-return clears progress, not the chosen parameter.
     * A READY g repeats the exact direction/count with a fresh test number. */
    for (int mode = 26; mode <= 27; ++mode) {
        snprintf(mode_line, sizeof mode_line, "%d", mode);
        for (unsigned dir = 0u; dir < 2u; ++dir) {
            for (unsigned k = 0u; k < sizeof counts / sizeof counts[0]; ++k) {
                unsigned n = counts[k];
                int axis = (mode - 24) & 1;
                int32_t request = dir ? (int32_t)n : -(int32_t)n;
                reset_fixture(); run_cmd(mode_line);
                snprintf(request_line, sizeof request_line, "n%c%u", dir ? 'r' : 'l', n);
                run_cmd(request_line);
                for (unsigned round = 1u; round <= 2u; ++round) {
                    run_cmd("g");
                    CHECK(s_round == R_RUN && s_active_test == round && s_axis == axis &&
                          host_jog_axis == axis && host_jog_dir == (int)dir);
                    CHECK(pulse_calls == (int)(2u * n * (round - 1u)));
                    uint32_t out_start = host_tick;
                    uint32_t expected_out_ms = jog_host_expected_ms(n);
                    CHECK(jog_host_ticks(n) == 0);
                    uint32_t out_ms = host_tick - out_start;
                    CHECK(out_ms == expected_out_ms);
                    CHECK(s_round == R_DONE && s_jog_done == n &&
                          pulse_calls == (int)(n * (2u * round - 1u)) &&
                          host_jog_pulses[axis][dir] == n * round &&
                          strstr(last_message, "WAIT_2S_THEN_RETURN") != NULL);
                    host_tick += T_JOG_RETURN_WAIT_MS - 1u; tick();
                    CHECK(s_round == R_DONE && pulse_calls == (int)(n * (2u * round - 1u)));
                    host_tick++; tick();
                    CHECK(s_round == R_RET && host_jog_dir == (int)(dir ^ 1u) &&
                          pulse_calls == (int)(n * (2u * round - 1u)));
                    uint32_t back_start = host_tick;
                    uint32_t expected_back_ms = jog_host_expected_ms(n);
                    CHECK(jog_host_ticks(n) == 0);
                    uint32_t back_ms = host_tick - back_start;
                    CHECK(back_ms == expected_back_ms);
                    CHECK(s_round == R_READY && s_jog_request == request &&
                          s_jog_done == 0u && s_back == 0u && s_active_test == round &&
                          pulse_calls == (int)(2u * n * round) &&
                          host_jog_pulses[axis][dir ^ 1u] == n * round &&
                          strstr(last_message, "status=RETURN_DONE") != NULL &&
                          strstr(last_message, "estimate_only=1 repeat_g=1") != NULL);
                    snprintf(report_line, sizeof report_line, "out=%u back=%u", n, n);
                    CHECK(strstr(last_message, report_line) != NULL);
                    snprintf(report_line, sizeof report_line, "out_ms=%lu back_ms=%lu",
                             (unsigned long)out_ms, (unsigned long)back_ms);
                    CHECK(strstr(last_message, report_line) != NULL && strstr(last_message, "pps=500") != NULL);
                    CHECK(jog_host_service_ms(3000u) == 0);
                    CHECK(s_round == R_READY && pulse_calls == (int)(2u * n * round));
                }
                run_cmd(mode_line);
                CHECK(s_round == R_READY && s_jog_request == 0);
                run_cmd("g");
                CHECK(s_round == R_READY && pulse_calls == (int)(4u * n) &&
                      strstr(last_message, "ERR SET_N") != NULL);
            }
        }
    }

    /* 24/25 retain DONE until g only clears the state; no auto-return. */
    for (int mode = 24; mode <= 25; ++mode) {
        snprintf(mode_line, sizeof mode_line, "%d", mode);
        reset_fixture(); run_cmd(mode_line); run_cmd("nr1000"); run_cmd("g");
        CHECK(jog_host_ticks(1000u) == 0);
        CHECK(s_round == R_DONE && pulse_calls == 1000 &&
              strstr(last_message, "DONE_NO_RETURN") != NULL);
        CHECK(jog_host_service_ms(3000u) == 0);
        CHECK(s_round == R_DONE && pulse_calls == 1000);
        run_cmd("g");
        CHECK(s_round == R_READY && pulse_calls == 1000 && s_active_test == 1u);
        CHECK(jog_host_service_ms(3000u) == 0);
        CHECK(s_round == R_READY && pulse_calls == 1000);
    }
    puts("bounded jog: 24..27 two-direction 1/500/1000/5000/5001/100000 limits, 26/27 READY-g repeat, 32-bit exact return count and unchanged 24/25 clear passed");
    return 0;
}

static int check_jog_cancellation_phases(void)
{
    static const char *const stops[] = { "g", "a", "0" };
    char mode_line[8], request_line[20];
    for (int mode = 26; mode <= 27; ++mode) {
        snprintf(mode_line, sizeof mode_line, "%d", mode);
        for (unsigned dir = 0u; dir < 2u; ++dir) {
            snprintf(request_line, sizeof request_line, "n%c500", dir ? 'r' : 'l');
            for (unsigned phase = 0u; phase < 3u; ++phase) {
                for (unsigned key = 0u; key < sizeof stops / sizeof stops[0]; ++key) {
                    reset_fixture(); run_cmd(mode_line); run_cmd(request_line); run_cmd("g");
                    CHECK(jog_host_ticks(phase == 0u ? 7u : 500u) == 0);
                    if (phase == 1u) {
                        host_tick += 1000u; tick();
                        CHECK(s_round == R_DONE && pulse_calls == 500);
                    } else if (phase == 2u) {
                        host_tick += T_JOG_RETURN_WAIT_MS; tick();
                        CHECK(s_round == R_RET && pulse_calls == 500);
                        CHECK(jog_host_ticks(7u) == 0);
                        CHECK(s_round == R_RET && pulse_calls == 507);
                    } else CHECK(s_round == R_RUN && pulse_calls == 7);
                    int stopped_pulses = pulse_calls;
                    unsigned clock_stops = host_timer_stop_calls;
                    run_cmd(stops[key]);
                    CHECK(s_round == R_READY && s_jog_request == 0 &&
                          pulse_calls == stopped_pulses && !host_abort && !host_timer_active &&
                          host_timer_stop_calls > clock_stops);
                    CHECK(jog_host_service_ms(5000u) == 0);
                    CHECK(s_round == R_READY && pulse_calls == stopped_pulses);
                    run_cmd("g");
                    CHECK(s_round == R_READY && pulse_calls == stopped_pulses &&
                          strstr(last_message, "ERR SET_N") != NULL && s_active_test == 1u);
                }
            }
        }
    }
    puts("bounded jog cancel: 26/27 both directions g/a/0 in OUT/WAIT/RETURN clears request and never resumes passed");
    return 0;
}

static int check_jog_frequency_command(void)
{
    static const unsigned rates[] = { 1u, 50u, 333u, 500u, 1000u, 2000u, 5000u, 10000u, 20000u };
    static const char *const invalid[] = { "f0", "f20001", "f-1", "f", "f50x", "f50.5" };
    char mode_line[8], rate_line[20], report_line[64];
    CHECK(T_JOG_DEFAULT_PPS == 500u && T_JOG_MAX_PPS == 20000u);
    reset_fixture(); CHECK(s_jog_pps == 500u);
    run_cmd("f500"); CHECK(strstr(last_message, "ERR") != NULL);
    for (int mode = 24; mode <= 27; ++mode) {
        snprintf(mode_line, sizeof mode_line, "%d", mode);
        for (unsigned k = 0u; k < sizeof rates / sizeof rates[0]; ++k) {
            unsigned rate = rates[k];
            unsigned count = rate == 20000u ? 100000u : 10u;
            reset_fixture(); run_cmd(mode_line);
            snprintf(rate_line, sizeof rate_line, "f%u", rate); run_cmd(rate_line);
            CHECK(s_jog_pps == rate && s_round == R_READY && pulse_calls == 0 &&
                  strstr(last_message, "ERR") == NULL);
            snprintf(rate_line, sizeof rate_line, "nr%u", count); run_cmd(rate_line); run_cmd("g");
            snprintf(report_line, sizeof report_line, "pps=%u", rate);
            CHECK(strstr(last_message, report_line) != NULL && host_timer_pps == rate && host_timer_active);
            uint32_t start = host_tick;
            uint32_t elapsed = jog_host_expected_ms(count);
            CHECK(jog_host_ticks(count) == 0);
            CHECK(s_round == R_DONE && pulse_calls == (int)count && host_tick - start == elapsed &&
                  s_jog_out_ms == elapsed && strstr(last_message, report_line) != NULL);
            if (mode >= 26) {
                CHECK(jog_host_service_ms(T_JOG_RETURN_WAIT_MS) == 0);
                CHECK(s_round == R_RET && pulse_calls == (int)count && host_timer_active);
                start = host_tick;
                elapsed = jog_host_expected_ms(count);
                CHECK(jog_host_ticks(count) == 0);
                CHECK(s_round == R_READY && pulse_calls == (int)(2u * count) && host_tick - start == elapsed &&
                      s_jog_back_ms == elapsed && s_jog_pps == rate && s_jog_request == (int32_t)count &&
                      !host_timer_active);
                CHECK(strstr(last_message, "estimate_only=1 repeat_g=1") != NULL &&
                      strstr(last_message, report_line) != NULL);
                run_cmd("g");
                CHECK(s_round == R_RUN && s_active_test == 2u && host_timer_pps == rate &&
                      s_jog_request == (int32_t)count && strstr(last_message, report_line) != NULL);
                if (rate == 20000u) {
                    /* The largest count repeats a COMPLETE high-rate pair,
                     * not merely the second g's startup message. */
                    CHECK(jog_host_ticks(count) == 0 && s_round == R_DONE && pulse_calls == (int)(3u * count));
                    CHECK(jog_host_service_ms(T_JOG_RETURN_WAIT_MS) == 0 && s_round == R_RET);
                    CHECK(jog_host_ticks(count) == 0 && s_round == R_READY &&
                          pulse_calls == (int)(4u * count) && s_active_test == 2u &&
                          s_jog_request == (int32_t)count && s_jog_pps == 20000u && !host_timer_active);
                    run_cmd("g"); CHECK(s_round == R_RUN && s_active_test == 3u);
                }
                run_cmd("g"); CHECK(s_round == R_READY && s_jog_pps == rate && s_jog_request == 0);
            } else {
                run_cmd("g"); CHECK(s_round == R_READY && pulse_calls == (int)count && s_jog_pps == rate);
            }
        }
    }

    reset_fixture(); run_cmd("27"); run_cmd("f333");
    for (unsigned i = 0u; i < sizeof invalid / sizeof invalid[0]; ++i) {
        run_cmd(invalid[i]);
        CHECK(strstr(last_message, "ERR") != NULL && s_jog_pps == 333u &&
              s_round == R_READY && pulse_calls == 0);
    }
    run_cmd("v100");
    CHECK(strstr(last_message, "JOG_USE_F_PPS") != NULL && s_jog_pps == 333u);
    for (int mode = 24; mode <= 27; ++mode) {
        snprintf(mode_line, sizeof mode_line, "%d", mode); run_cmd(mode_line);
        CHECK(s_jog_pps == 333u && s_jog_request == 0 && pulse_calls == 0);
    }
    run_cmd("nl1"); run_cmd("g");
    CHECK(host_timer_pps == 333u && strstr(last_message, "pps=333") != NULL);
    run_cmd("g");
    CHECK(s_round == R_READY && s_jog_pps == 333u && s_jog_request == 0);

    for (int mode = 26; mode <= 27; ++mode) {
        snprintf(mode_line, sizeof mode_line, "%d", mode);
        for (unsigned phase = 0u; phase < 3u; ++phase) {
            reset_fixture(); run_cmd(mode_line); run_cmd("f333"); run_cmd("nl20"); run_cmd("g");
            CHECK(jog_host_ticks(phase == 0u ? 7u : 20u) == 0);
            if (phase == 1u) CHECK(s_round == R_DONE);
            else if (phase == 2u) {
                CHECK(jog_host_service_ms(T_JOG_RETURN_WAIT_MS) == 0);
                CHECK(s_round == R_RET && jog_host_ticks(7u) == 0);
            } else CHECK(s_round == R_RUN);
            int before = pulse_calls;
            run_cmd("f50");
            CHECK(strstr(last_message, "ERR") != NULL && s_jog_pps == 333u &&
                  host_timer_pps == 333u && pulse_calls == before);
            run_cmd("v100");
            CHECK(strstr(last_message, "JOG_USE_F_PPS") != NULL && s_jog_pps == 333u);
            run_cmd("g"); CHECK(jog_host_service_ms(1000u) == 0);
            CHECK(s_round == R_READY && pulse_calls == before);
        }
    }
    puts("jog frequency: default500, f1/50/333/500/1000/2000/5000/10000/20000, range20001 rejection, active locks, v-use-f, mode/repeat retention and virtual TIM7 timing passed");
    return 0;
}

static int check_jog_clock_isolation(void)
{
    reset_fixture(); CHECK(jog_host_service_ms(1000u) == 0);
    CHECK(pulse_calls == 0 && s_round == R_READY && s_jog_clock.remaining == 0u &&
          !host_timer_active && host_timer_start_calls == 0u);
    run_cmd("27"); CHECK(jog_host_service_ms(100u) == 0);
    run_cmd("nr100"); CHECK(jog_host_service_ms(100u) == 0);
    CHECK(pulse_calls == 0 && s_round == R_READY);
    run_cmd("f20000"); run_cmd("g");
    CHECK(host_timer_active && host_timer_period_us == 50u && pulse_calls == 0);
    CHECK(jog_host_service_ms(1u) == 0);
    CHECK(pulse_calls == 0); /* All first-millisecond IRQs respect DIR setup. */
    for (unsigned i = 0u; i < 20u; ++i) CHECK(jog_host_timer_event() == 0);
    CHECK(pulse_calls == 1 && s_round == R_RUN);
    uint32_t shared_ms = host_tick;
    for (unsigned i = 0u; i < 10u; ++i) CHECK(jog_host_timer_event() == 0);
    CHECK(pulse_calls == 11 && host_tick == shared_ms); /* Distinct IRQs in one HAL ms are valid. */
    for (unsigned i = 0u; i < 100u; ++i) tick();
    CHECK(pulse_calls == 11 && s_jog_done == 11u); /* DefaultTask never creates STEP. */
    unsigned stops_before = host_timer_stop_calls;
    run_cmd("g");
    CHECK(!host_timer_active && host_timer_stop_calls > stops_before && s_jog_request == 0);
    test_stepper_timer_irq(); /* Even an already-pending stale IRQ is inert after cancellation. */
    CHECK(jog_host_service_ms(1000u) == 0 && pulse_calls == 11);

    /* A delayed DefaultTask callback does not emit pulses. A pending TIM7
     * interrupt handles at most one event; it does not replay missed periods. */
    reset_fixture(); run_cmd("25"); run_cmd("f500"); run_cmd("nr10"); run_cmd("g");
    host_tick += 1000u;
    for (unsigned i = 0u; i < 100u; ++i) tick();
    CHECK(pulse_calls == 0 && s_jog_clock.remaining == 10u && s_round == R_RUN);
    host_timer_sync_hal();
    unsigned uart_before = host_uart_calls;
    unsigned rephase_before = host_timer_rephase_calls;
    test_stepper_timer_irq();
    CHECK(pulse_calls == 1 && s_jog_clock.remaining == 9u && host_uart_calls == uart_before &&
          host_timer_rephase_calls == rephase_before + 1u &&
          host_timer_next_us == host_timer_us + host_timer_period_us);
    CHECK(jog_host_timer_event() == 0 && pulse_calls == 2 &&
          host_timer_next_us == host_timer_us + host_timer_period_us);
    run_cmd("a"); CHECK(jog_host_service_ms(1000u) == 0 && pulse_calls == 2);

    /* Finishing while the UART/report task is delayed cannot add any STEP. */
    reset_fixture(); run_cmd("25"); run_cmd("f1000"); run_cmd("nr3"); run_cmd("g");
    for (unsigned i = 0u; i < 4u; ++i) CHECK(jog_host_timer_event() == 0);
    CHECK(pulse_calls == 3 && s_round == R_RUN && s_jog_clock.completed == 3u &&
          s_jog_clock.remaining == 0u && s_jog_done == 0u && !host_timer_active);
    host_tick += 3000u; host_timer_sync_hal(); test_stepper_timer_irq();
    CHECK(pulse_calls == 3 && s_round == R_RUN);
    tick();
    CHECK(s_round == R_DONE && pulse_calls == 3 && s_jog_done == 3u && s_jog_out_ms == 4u &&
          strstr(last_message, "out_ms=4") != NULL);
    CHECK(jog_host_service_ms(1000u) == 0 && s_round == R_DONE && pulse_calls == 3);

    /* Waiting starts at the last pulse, not at a delayed DONE message. */
    reset_fixture(); run_cmd("27"); run_cmd("f1000"); run_cmd("nr3"); run_cmd("g");
    for (unsigned i = 0u; i < 4u; ++i) CHECK(jog_host_timer_event() == 0);
    host_tick += 3000u; host_timer_sync_hal();
    CHECK(pulse_calls == 3 && s_round == R_RUN);
    tick(); CHECK(s_round == R_DONE && pulse_calls == 3 && s_jog_out_ms == 4u);
    tick(); CHECK(s_round == R_RET && pulse_calls == 3 && host_timer_active);
    CHECK(jog_host_timer_event() == 0 && pulse_calls == 3);
    CHECK(jog_host_timer_event() == 0 && pulse_calls == 4);
    run_cmd("0"); CHECK(jog_host_service_ms(1000u) == 0 && pulse_calls == 4 && s_jog_request == 0);

    /* Millisecond wrap uses unsigned elapsed arithmetic without a pulse burst. */
    reset_fixture(); run_cmd("27"); run_cmd("f1000"); run_cmd("nr3");
    host_tick = UINT32_MAX - 1u; run_cmd("g");
    CHECK(jog_host_timer_event() == 0 && pulse_calls == 0);
    CHECK(jog_host_timer_event() == 0 && host_tick == 0u && pulse_calls == 1);
    CHECK(jog_host_ticks(2u) == 0);
    CHECK(s_round == R_DONE && pulse_calls == 3 && s_jog_out_ms == 4u);
    run_cmd("g");

    /* Clock failure cannot leave a job running forever or initiate a return. */
    reset_fixture(); run_cmd("27"); run_cmd("f20000"); run_cmd("nr1000");
    host_timer_start_fail = 1; run_cmd("g");
    CHECK(s_round == R_READY && !host_timer_active && s_jog_clock.remaining == 0u &&
          pulse_calls == 0 && s_jog_request == 0 && strstr(last_message, "ERR JOG_CLOCK_NOT_STARTED") != NULL);
    CHECK(jog_host_service_ms(3000u) == 0 && pulse_calls == 0 && s_round == R_READY);
    unsigned starts_before = host_timer_start_calls;
    run_cmd("g");
    CHECK(strstr(last_message, "ERR SET_N") != NULL && s_round == R_READY &&
          pulse_calls == 0 && host_timer_start_calls == starts_before);
    host_timer_start_fail = 0; run_cmd("nr3"); run_cmd("g");
    CHECK(jog_host_ticks(3u) == 0 && s_round == R_DONE && pulse_calls == 3);
    host_timer_start_fail = 1;
    CHECK(jog_host_service_ms(T_JOG_RETURN_WAIT_MS) == 0);
    CHECK(s_round == R_READY && !host_timer_active && s_jog_clock.remaining == 0u && pulse_calls == 3 &&
          s_jog_request == 0 && strstr(last_message, "ERR JOG_RETURN_CLOCK_NOT_STARTED") != NULL);
    CHECK(jog_host_service_ms(3000u) == 0 && pulse_calls == 3);
    starts_before = host_timer_start_calls; run_cmd("g");
    CHECK(strstr(last_message, "ERR SET_N") != NULL && s_round == R_READY &&
          pulse_calls == 3 && host_timer_start_calls == starts_before);
    puts("jog TIM7 engine: boot inert, 2ms DIR gate, high-rate same-HAL-ms IRQs, one STEP/no UART per IRQ, delayed report isolation, clock failure and uint32 wrap passed");
    return 0;
}

static int check_jog_no_return_cancellation(void)
{
    static const char *const stops[] = { "g", "a", "0" };
    char mode_line[8], request_line[16];
    for (int mode = 24; mode <= 25; ++mode) {
        snprintf(mode_line, sizeof mode_line, "%d", mode);
        for (unsigned dir = 0u; dir < 2u; ++dir) {
            snprintf(request_line, sizeof request_line, "n%c500", dir ? 'r' : 'l');
            for (unsigned phase = 0u; phase < 2u; ++phase) {
                for (unsigned key = 0u; key < sizeof stops / sizeof stops[0]; ++key) {
                    reset_fixture(); run_cmd(mode_line); run_cmd("f1000"); run_cmd(request_line); run_cmd("g");
                    CHECK(jog_host_ticks(phase ? 500u : 7u) == 0);
                    int before = pulse_calls;
                    unsigned clock_stops = host_timer_stop_calls;
                    run_cmd(stops[key]);
                    CHECK(s_jog_clock.remaining == 0u && pulse_calls == before &&
                          s_round == (key == 0u && phase == 0u ? R_DONE : R_READY) &&
                          !host_timer_active && host_timer_stop_calls > clock_stops);
                    CHECK(jog_host_service_ms(3000u) == 0);
                    CHECK(pulse_calls == before && !host_abort && s_jog_pps == 1000u);
                    if (s_round == R_DONE) {
                        run_cmd("g"); CHECK(jog_host_service_ms(100u) == 0);
                        CHECK(s_round == R_READY && pulse_calls == before);
                    }
                }
            }
        }
    }
    puts("jog 24/25: both directions OUT/DONE g/a/0 stop the TIM7 clock without extra STEP or auto-return passed");
    return 0;
}

static int check_servo_full_range_router(void)
{
    static const unsigned valid[] = { 500u, 501u, 1000u, 1400u, 1800u, 2499u, 2500u };
    static const char *const invalid_suffix[] = {
        "499", "2501", "-1", "-500", "", "500x", "500.5", "2147483648"
    };
    static const char *const stops[] = { "g", "a", "0" };
    static const char *const active_writes[] = {
        "su500", "su2500", "u500", "u2500", "co", "cc", "24", "29"
    };
    char command[32], selection[8];

    /* Immediate su is independent of the mode28/29 target slot. Rejected
     * inputs neither clamp-and-move nor disturb a previously valid command. */
    reset_fixture();
    CHECK(ARM_SERVO_START_US == 1150u && host_servo == ARM_SERVO_START_US && servo_calls == 0);
    for (unsigned k = 0u; k < sizeof valid / sizeof valid[0]; ++k) {
        int calls = servo_calls;
        snprintf(command, sizeof command, "su%u", valid[k]); run_cmd(command);
        CHECK(host_servo == valid[k] && servo_calls == calls + 1 &&
              s_servo_target_us == 0u && strstr(last_message, "OK SERVO us=") != NULL);
    }
    for (unsigned k = 0u; k < sizeof invalid_suffix / sizeof invalid_suffix[0]; ++k) {
        int calls = servo_calls;
        snprintf(command, sizeof command, "su%s", invalid_suffix[k]); run_cmd(command);
        CHECK(host_servo == 2500u && servo_calls == calls && s_servo_target_us == 0u &&
              strstr(last_message, "ERR SERVO_RANGE") != NULL);
    }
    run_cmd("co"); CHECK(host_servo == 1000u);
    run_cmd("cc"); CHECK(ARM_SERVO_GRIP_US == 1700u && host_servo == ARM_SERVO_GRIP_US);

    /* u is only a READY parameter write in either servo mode; it is never
     * an actuator write until g. Both new software endpoints are exercised. */
    for (int mode = 28; mode <= 29; ++mode) {
        snprintf(selection, sizeof selection, "%d", mode);
        reset_fixture(); run_cmd(selection); run_cmd("g");
        CHECK(s_round == R_READY && servo_calls == 0 &&
              strstr(last_message, "ERR SET_U first") != NULL);
        for (unsigned k = 0u; k < sizeof valid / sizeof valid[0]; ++k) {
            snprintf(command, sizeof command, "u%u", valid[k]); run_cmd(command);
            CHECK(s_round == R_READY && s_servo_target_us == valid[k] &&
                  host_servo == ARM_SERVO_START_US && servo_calls == 0 &&
                  strstr(last_message, "OK SERVO_TARGET") != NULL);
        }
        for (unsigned k = 0u; k < sizeof invalid_suffix / sizeof invalid_suffix[0]; ++k) {
            snprintf(command, sizeof command, "u%s", invalid_suffix[k]); run_cmd(command);
            CHECK(s_servo_target_us == 2500u && host_servo == ARM_SERVO_START_US && servo_calls == 0 &&
                  s_round == R_READY && strstr(last_message, "ERR U_RANGE") != NULL);
        }
        run_cmd("g");
        CHECK(host_servo == 2500u && servo_calls == 1 && s_round == R_DONE &&
              s_servo_origin_us == ARM_SERVO_START_US && s_active_test == 1u);
        run_cmd("g"); CHECK(s_round == R_READY && host_servo == 2500u && servo_calls == 1);
        run_cmd(selection); CHECK(s_servo_target_us == 0u && servo_calls == 1);
        run_cmd("g"); CHECK(s_round == R_READY && servo_calls == 1);
    }
    reset_fixture(); run_cmd("u500");
    CHECK(s_msel == R_FREE && host_servo == ARM_SERVO_START_US && servo_calls == 0 &&
          strstr(last_message, "ERR U_SELECT_MODE28_OR29") != NULL);
    for (int mode = 24; mode <= 27; ++mode) {
        snprintf(selection, sizeof selection, "%d", mode);
        reset_fixture(); run_cmd(selection); run_cmd("u2500");
        CHECK(s_servo_target_us == 0u && host_servo == ARM_SERVO_START_US && servo_calls == 0 &&
              strstr(last_message, "ERR U_SELECT_MODE28_OR29") != NULL);
    }

    /* 28 returns to the actual command preceding each g, including endpoints,
     * exactly after the scheduled two-second hold. READY g repeats its slot. */
    for (unsigned endpoint = 0u; endpoint < 2u; ++endpoint) {
        unsigned origin = endpoint ? 500u : 2500u;
        unsigned target = endpoint ? 2500u : 500u;
        reset_fixture(); run_cmd("28");
        snprintf(command, sizeof command, "su%u", origin); run_cmd(command);
        snprintf(command, sizeof command, "u%u", target); run_cmd(command);
        CHECK(host_servo == origin && servo_calls == 1);
        run_cmd("g"); CHECK(host_servo == target && servo_calls == 2 && s_round == R_DONE);
        host_tick += T_SERVO_RETURN_WAIT_MS - 1u; tick();
        CHECK(host_servo == target && servo_calls == 2 && s_round == R_DONE);
        host_tick++; tick();
        CHECK(host_servo == origin && servo_calls == 3 && s_round == R_READY &&
              s_servo_target_us == target && strstr(last_message, "RETURN_COMMAND_SENT") != NULL);
        run_cmd("g"); CHECK(host_servo == target && servo_calls == 4 && s_active_test == 2u);
        host_tick += T_SERVO_RETURN_WAIT_MS; tick();
        CHECK(host_servo == origin && servo_calls == 5 && s_round == R_READY);
    }

    /* 29 has no timed return. All stop keys keep the current pulse rather
     * than command the origin, and suppress mode28's pending automatic move. */
    for (int mode = 28; mode <= 29; ++mode) {
        snprintf(selection, sizeof selection, "%d", mode);
        for (unsigned key = 0u; key < sizeof stops / sizeof stops[0]; ++key) {
            reset_fixture(); run_cmd(selection); run_cmd("u500"); run_cmd("g");
            CHECK(s_round == R_DONE && host_servo == 500u && servo_calls == 1);
            host_tick += mode == 28 ? T_SERVO_RETURN_WAIT_MS - 1u : 10000u; tick();
            CHECK(s_round == R_DONE && host_servo == 500u && servo_calls == 1);
            run_cmd(stops[key]);
            CHECK(s_round == R_READY && s_servo_hold_t0 == 0u && host_servo == 500u &&
                  servo_calls == 1 && strstr(last_message, "return_cancelled=1") != NULL);
            host_tick += 10000u; tick();
            CHECK(s_round == R_READY && host_servo == 500u && servo_calls == 1 && !host_abort);
        }
    }

    /* Active servo return/hold cannot be overwritten by su/u/co/cc or mode
     * selection. Wheel motion and a non-BOOT mission reject servo writes too. */
    for (int mode = 28; mode <= 29; ++mode) {
        snprintf(selection, sizeof selection, "%d", mode);
        reset_fixture(); run_cmd(selection); run_cmd("u2500"); run_cmd("g");
        for (unsigned k = 0u; k < sizeof active_writes / sizeof active_writes[0]; ++k) {
            run_cmd(active_writes[k]);
            CHECK(s_msel == mode && s_round == R_DONE && s_servo_target_us == 2500u &&
                  host_servo == 2500u && servo_calls == 1 && strstr(last_message, "ERR") != NULL);
        }
        run_cmd("g"); CHECK(s_round == R_READY && host_servo == 2500u && servo_calls == 1);
    }
    reset_fixture(); run_cmd("15"); run_cmd("v100"); run_cmd("d100"); run_cmd("g");
    run_cmd("su500"); run_cmd("u2500");
    CHECK(s_round == R_RUN && s_msel == 15 && servo_calls == 0 && host_servo == ARM_SERVO_START_US);
    reset_fixture(); run_cmd("28"); host_state = MS_EOD;
    run_cmd("su500"); run_cmd("u2500"); run_cmd("co"); run_cmd("cc");
    CHECK(s_servo_target_us == 0u && host_servo == ARM_SERVO_START_US && servo_calls == 0);
    puts("servo router: 500..2500 bounds, invalid/no-output, u READY-only/g start, exact 28 return and 29 hold, g/a/0 cancel, active/mission locks passed");
    return 0;
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
        CHECK(host_servo == ARM_SERVO_START_US && pulse_calls == 0 && servo_calls == 0 &&
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
    reset_fixture(); run_cmd("44"); CHECK(s_msel == R_FREE && strstr(last_message, "MODE_RANGE") != NULL);
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
    CHECK(check_jog_range_and_repeat() == 0);
    CHECK(check_jog_cancellation_phases() == 0);
    CHECK(check_jog_frequency_command() == 0);
    CHECK(check_jog_clock_isolation() == 0);
    CHECK(check_jog_no_return_cancellation() == 0);
    CHECK(check_servo_full_range_router() == 0);
    puts("real g router: pending/running/terminal mission stop, a alias, closed gate, CRLF/idle framing, wheel/distance/turn/IMU/jog/servo/return passed");
    return 0;
}
