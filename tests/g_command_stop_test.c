/* Actual Bluetooth command router/state machine with host-only peripheral stubs.
 * A mission-start stub models an accepted future start; production calibration
 * gates are NOT changed. Real gate/start-abort checks live in the other fixture. */
#include <stdio.h>
#include <string.h>
#include "../App/test.c"

static uint32_t host_tick;
static MissionState host_state;
static int host_abort, start_calls, brake_calls, pulse_calls, gate_closed;
static int host_imu_valid;
static float host_yaw, last_x, last_y, last_w;
static float host_fore, host_lateral;
static int32_t host_counts[4];
static int zero_calls, prepare_calls, servo_calls;
static uint16_t host_servo;
static char last_message[256];
static int trial_start_calls, trial_report_calls, trial_gate_closed;
static int trial_alignment_calls, trial_alignment_cls, trial_alignment_cx, trial_alignment_sign;
static int laser_state;

uint32_t HAL_GetTick(void) { return host_tick; }
void bp_debug_send(const char *s)
{
    /* Production send() appends CRLF in a separate UART write. */
    if (strcmp(s, "\r\n") != 0)
        snprintf(last_message, sizeof last_message, "%s", s);
}
void bp_laser_set(int on) { laser_state = on; }
int32_t bp_enc_raw_total(int m) { (void)m; return 0; }
void bp_enc_raw_reset_all(void) { }
void motion_brake(void) { brake_calls++; last_x = last_y = last_w = 0.0f; }
void motion_vel_set(float x, float y, float w) { last_x = x; last_y = y; last_w = w; }
float motion_odo_mm(void) { return host_fore; }
float motion_lateral_odo_mm(void) { return host_lateral; }
void motion_ik(float x, float y, float w, int16_t out[4])
{ (void)x; (void)y; (void)w; memset(out, 0, 4 * sizeof *out); }
void motion_profile_get(MotionProfileTune *out) { memset(out, 0, sizeof *out); }
int motion_profile_set(const MotionProfileTune *in) { (void)in; return 1; }
void motion_linear_ramp_init(MotionRamp *r) { memset(r, 0, sizeof *r); }
float motion_linear_profile_step(MotionRamp *r, float v, float d, float dt)
{ (void)r; (void)d; (void)dt; return v; }
void ctrl_set_speed(int m, int16_t rpm) { (void)m; (void)rpm; }
void ctrl_set_duty_open(int m, int16_t duty) { (void)m; (void)duty; }
void ctrl_coast_all(void) { }
void ctrl_enc_reset_all(void) { host_fore = host_lateral = 0.0f; memset(host_counts, 0, sizeof host_counts); }
int32_t ctrl_enc_total(int m) { return host_counts[m]; }
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
float imu_heading_deg(void) { return 0.0f; }
float imu_leg_heading_deg(void) { return host_yaw; }
float imu_pitch_deg(void) { return 0.0f; }
float imu_roll_deg(void) { return 0.0f; }
uint8_t imu_ok(void) { return (uint8_t)host_imu_valid; }
uint8_t imu_zero_leg_heading(void) { zero_calls++; host_yaw = 0.0f; return (uint8_t)host_imu_valid; }
uint32_t imu_last_valid_age_ms(void) { return 0u; }
void run_abort(void) { host_abort = 1; }
int run_aborted(void) { return host_abort; }
int step_prepare_leg(void) { prepare_calls++; return !host_abort && host_imu_valid; }
float step_heading_hold_w(float heading) { (void)heading; return 0.0f; }
float step_heading_kp_deg(void) { return 0.3f; }
int step_heading_kp_set(float v) { (void)v; return 1; }
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
int mission_trial_set_alignment(int cls, int cx, int sign)
{
    trial_alignment_calls++;
    trial_alignment_cls = cls; trial_alignment_cx = cx; trial_alignment_sign = sign;
    return cls >= -1 && cls <= 3 && cx >= -1 && cx <= 4095 && sign >= -1 && sign <= 1;
}
void robot_diag_report(void) { }

static void reset_fixture(void)
{
    host_tick = 100u; host_state = MS_BOOT;
    host_abort = start_calls = brake_calls = pulse_calls = gate_closed = 0;
    host_imu_valid = 1; host_yaw = last_x = last_y = last_w = 0.0f;
    host_fore = host_lateral = 0.0f; memset(host_counts, 0, sizeof host_counts);
    zero_calls = prepare_calls = servo_calls = 0;
    host_servo = 1400u; last_message[0] = '\0';
    trial_start_calls = trial_report_calls = trial_gate_closed = trial_alignment_calls = 0;
    trial_alignment_cls = trial_alignment_cx = trial_alignment_sign = 0;
    laser_state = 0;
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
    } else {
        host_yaw = turn_target_deg(); test_poll();
        CHECK(s_round == R_BRAKE && strcmp(s_turn_result, "DONE") == 0 && s_seq_state == SQ_RUN);
        host_tick += T_TURN_SETTLE_MS; test_poll();
    }
    return 0;
}

static int check_route_sequence(void)
{
    /* Independent expectation, not copied from the live recipe at runtime. */
    static const int expected_modes[10] = {17,16,30,15,17,15,20,15,20,15};
    static const int expected_commands[10] = {-500,-600,-95,750,-730,830,85,2450,85,2125};
    static const char *const stop_keys[] = {"g", "a", "0"};
    reset_fixture();
    host_fore = 4321.0f; host_lateral = -1234.0f; host_yaw = 47.0f;
    run_cmd("31");
    CHECK(s_seq_state == SQ_READY && s_round == R_READY && s_msel == 31);
    for (int i = 0; i < 5; i++) { host_tick += 100u; test_poll(); }
    CHECK(s_seq_state == SQ_READY && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    run_cmd("g"); CHECK(s_seq_run == 1u && s_seq_stage == 0u);
    for (int i = 0; i < 10; i++) {
        CHECK(s_seq_stage == i && sequence_start_stage() == 0);
        CHECK(s_msel == expected_modes[i] && s_route_leg == i + 1 && s_active_test == (uint32_t)i + 1u);
        if (dist_mode()) {
            CHECK(s_dist_target == (float)expected_commands[i] && s_v == 100.0f);
            CHECK(s_dist_ff_ratio == (s_msel == 15 ? T_FORWARD_FF_SEED : 0.0f));
            CHECK(last_w == 0.0f);
            if (dist_lateral()) CHECK(last_x == 0.0f && last_y == -100.0f);
            else CHECK(last_x == (i == 1 ? -100.0f : 100.0f) &&
                       last_y == (i == 1 ? 0.0f : -1.25f));
        } else {
            CHECK(turn_target_deg() == (float)expected_commands[i] && last_x == 0.0f && last_y == 0.0f);
            CHECK(last_w == (i == 2 ? -T_TURN_MAX_W : T_TURN_MAX_W));
        }
        CHECK(start_calls == 0 && pulse_calls == 0 && servo_calls == 0 && !s_go);
        CHECK(sequence_finish_stage() == 0);
        CHECK(i == 9 ? s_seq_state == SQ_DONE : s_seq_state == SQ_STILL);
    }
    CHECK(zero_calls == 17 && prepare_calls == 0); /* 10 node zeros + 7 distance-report resets */
    CHECK(s_msel == 31 && s_round == R_DONE && strstr(last_message, "status=DONE") != NULL);
    for (int i = 0; i < 10; i++) { host_tick += 1000u; test_poll(); }
    run_cmd("g"); CHECK(s_seq_state == SQ_DONE && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    run_cmd("31"); run_cmd("g"); CHECK(s_seq_run == 2u && s_seq_state == SQ_STILL);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED);

    /* Every stage, every stop key, every phase: 10 * 3 * 4 = 120 cancellations. */
    for (int stage = 0; stage < 10; stage++) {
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

    static const char *const writes[] = {"v300","d1500","ykp4","r1","20","31","cc","co","su1500","n5"};
    for (int phase = 0; phase < 4; phase++) {
        reset_fixture(); run_cmd("31"); run_cmd("g");
        if (phase == 1) { host_tick += T_DIST_STILL_MS; test_poll(); }
        if (phase >= 2) CHECK(sequence_start_stage() == 0);
        if (phase == 3) { host_lateral = s_dist_odo0 + s_dist_target; test_poll(); }
        for (size_t k = 0; k < sizeof writes / sizeof writes[0]; k++) {
            run_cmd(writes[k]);
            CHECK(strstr(last_message, "ERR ROUTE_SEQ_ACTIVE") != NULL && s_v == 100.0f &&
                  s_d == 500.0f && s_seq_stage == 0u && s_msel == 17 && servo_calls == 0);
        }
        CHECK(route_seq_active()); run_cmd("g");
    }
    /* All failed sensor/turn statuses cancel rather than advancing a ready/done submode. */
    for (int stage = 0; stage < 10; stage++) {
        reset_fixture(); run_cmd("31"); run_cmd("g");
        s_seq_stage = (uint8_t)stage; route_seq_prepare(); CHECK(sequence_start_stage() == 0);
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
        if (failure == 1) { host_yaw = -111.0f; test_poll(); }
        if (failure == 2) { turn_begin_settle("STOP"); }
        CHECK(s_round == R_BRAKE);
        host_tick += T_TURN_SETTLE_MS; test_poll();
        CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 2u && last_w == 0.0f);
    }
    reset_fixture(); run_cmd("31"); run_cmd("g"); CHECK(sequence_start_stage() == 0);
    dist_begin_finish(0u); host_tick += T_DIST_STILL_MS; test_poll();
    CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 0u);
    puts("route31: exact 10-step traversal, terminal/no-restart, 120 g/a/0 phase cancellations, write lock, nonblocking prepare, IMU/abort/turn failure passed");
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
            CHECK(s_forward_ff_ratio == 0.0125f);
            snprintf(line, sizeof line, "%d", mode); run_cmd(line);
            snprintf(line, sizeof line, "v%d", speeds[k]); run_cmd(line);
            run_cmd("d1500"); run_cmd("g"); tick();
            CHECK(s_round == R_RUN);
            if (mode == 15 && speeds[k] <= 200) expected_ratio = 0.0125f;
            if (mode == 17 && speeds[k] == 300) expected_ratio = T_LEFT_FF_SEED;
            if (mode == 18 && speeds[k] == 300) expected_ratio = T_RIGHT_FF_SEED;
            CHECK(s_dist_ff_ratio == expected_ratio && last_w == 0.0f);
            if (mode <= 16) {
                CHECK(last_x == (mode == 15 ? (float)speeds[k] : -(float)speeds[k]));
                CHECK(last_y == (mode == 15 ? -expected_ratio * last_x : 0.0f));
            } else {
                CHECK(last_y == (mode == 17 ? -(float)speeds[k] : (float)speeds[k]));
                CHECK(last_x == expected_ratio * last_y);
            }
            run_cmd("g"); CHECK(s_round == R_BRAKE && last_x == 0.0f && last_y == 0.0f);
        }
    }
    reset_fixture(); run_cmd("fff0");
    run_cmd("15"); run_cmd("v100"); run_cmd("d1000"); run_cmd("g"); tick();
    CHECK(s_dist_ff_ratio == 0.0f && last_x == 100.0f && last_y == 0.0f);
    reset_fixture(); run_cmd("fff0.01");
    run_cmd("15"); run_cmd("v100"); run_cmd("d1000"); run_cmd("g"); tick();
    CHECK(s_dist_ff_ratio > 0.009999f && s_dist_ff_ratio < 0.010001f &&
          last_y > -1.0001f && last_y < -0.9999f);
    reset_fixture(); run_cmd("param");
    CHECK(strstr(last_message, "fff=0.0125") != NULL);
    puts("forward FF: default/left sign, v100/v200 gate, reverse/strafe isolation, RAM off/override and PARAM passed");
    return 0;
}

static int check_shared_turn_hold(void)
{
    static const int modes[] = {20, 22, 30};
    /* Literal checks pin the successful 180 settings, not just new aliases. */
    CHECK(T_TURN180_MAX_MS == 12000u && T_TURN_MAX_MS == 12000u);
    CHECK(TURN90_TARGET_DEG == 90.0f && T_TURN90_LEFT_COMP_DEG == 5.0f &&
          T_TURN90_RIGHT_COMP_DEG == -5.0f && T_TURN_LEFT_TARGET_DEG == -95.0f &&
          T_TURN_RIGHT_TARGET_DEG == 85.0f);
    CHECK(T_TURN_KP == 0.15f && T_TURN_MAX_W == 2.0f && T_TURN_MIN_W == 0.18f &&
          T_TURN_TOL_DEG == 0.3f && T_TURN_SETTLE_MS == 700u && TURN90_STILL_DEG == 0.2f);
    for (int k = 0; k < 3; k++) {
        char line[8];
        float target, dir;
        reset_fixture(); snprintf(line, sizeof line, "%d", modes[k]);
        run_cmd(line); run_cmd("g");
        target = turn_target_deg(); dir = target < 0.0f ? -1.0f : 1.0f;
        CHECK(target == (modes[k] == 22 ? 180.0f : (modes[k] == 30 ? -95.0f : 85.0f)));
        CHECK(strstr(last_message, modes[k] == 22 ? "target=+180deg" :
                                  (modes[k] == 30 ? "target=-95deg" : "target=+85deg")) != NULL);
        if (modes[k] != 22) {
            host_yaw = dir * 90.0f; tick();
            CHECK(s_round == R_RUN && last_w < 0.0f); /* old90: right overshot, left short */
            host_yaw = dir * 92.0f; tick();
            CHECK(s_round == R_RUN && last_w < 0.0f); /* old92: right corrects left, left continues left */
            CHECK(strstr(last_message, modes[k] == 20 ? "comp=-5deg" : "comp=5deg") != NULL);
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
    CHECK(test_forward_ff_ratio() == T_FORWARD_FF_SEED);
    run_cmd("fff0.0200");
    CHECK(test_forward_ff_ratio() > 0.019999f && test_forward_ff_ratio() < 0.020001f);
    run_cmd("fff0"); CHECK(test_forward_ff_ratio() == 0.0f);
    reset_fixture();
    run_cmd("32");
    CHECK(s_msel == 32 && s_round == R_READY && !s_go && trial_start_calls == 0);
    CHECK(strcmp(s_mname[32], "no_arm_single_pass_mission") == 0 && trial_report_calls == 1);
    run_cmd("vsg1");
    CHECK(trial_alignment_calls == 1 && trial_alignment_cls == -1 &&
          trial_alignment_cx == -1 && trial_alignment_sign == 1);
    run_cmd("vsg2");
    CHECK(trial_alignment_calls == 2 && trial_alignment_sign == -1);
    for (int cls = 0; cls < 4; ++cls) {
        run_cmd(workpoint_keys[cls]);
        CHECK(trial_alignment_cls == cls && trial_alignment_cx >= 0 && trial_alignment_sign == 0);
    }
    int changed = trial_alignment_calls;
    run_cmd("vsg0"); run_cmd("vsg3"); run_cmd("vsg2junk");
    run_cmd("bcx-3"); run_cmd("hcx"); run_cmd("tcx99x");
    CHECK(trial_alignment_calls == changed);
    run_cmd("trial"); CHECK(trial_report_calls >= 8);
    run_cmd("0"); CHECK(!s_go && !host_abort && trial_start_calls == 0);

    for (unsigned i = 0; i < 3u; ++i) {
        reset_fixture(); run_cmd("32"); run_cmd("g");
        CHECK(s_go && !host_abort && trial_start_calls == 1 && start_calls == 0);
        CHECK(host_state == MS_BOOT && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
        /* Only a queued request: DefaultTask keeps handling Bluetooth. */
        host_tick += 100u; test_poll();
        CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f && pulse_calls == 0);
        run_cmd("bcx199"); run_cmd("vsg1"); run_cmd("24"); run_cmd("co");
        CHECK(trial_alignment_calls == 0 && s_msel == 32 && host_servo == 1400u);
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
    reset_fixture(); run_cmd("7"); run_cmd("p45"); run_cmd("g"); run_cmd("bcx180");
    CHECK(trial_alignment_calls == 0 && s_round == R_RUN);
    puts("trial32 router: selection/RAM workpoints/sign aliases, malformed writes, queued start, pending g/a/0, laser-off, terminal no-restart and write lock passed");
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
    host_yaw = -94.0f; tick(); CHECK(s_round == R_RUN && last_w < 0.0f);
    host_yaw = -96.0f; tick(); CHECK(s_round == R_RUN && last_w > 0.0f);
    host_yaw = -95.0f; tick(); CHECK(s_round == R_BRAKE && last_w == 0.0f);
    host_yaw = -95.8f; tick(); CHECK(s_round == R_RUN); /* inertia reopens correction */
    tick(); CHECK(last_w > 0.0f);
    host_yaw = -95.0f; tick(); host_tick += T_TURN_SETTLE_MS; tick();
    CHECK(s_round == R_DONE && strstr(last_message, "type=TURN90_LEFT") != NULL &&
          strstr(last_message, "status=DONE") != NULL && strstr(last_message, "cmd_deg=-95") != NULL);
    run_cmd("g"); CHECK(s_round == R_READY && last_w == 0.0f); /* clear only, no reverse */

    reset_fixture(); run_cmd("30"); run_cmd("g"); tick(); run_cmd("g");
    CHECK(s_round == R_BRAKE && strcmp(s_turn_result, "STOP") == 0 && last_w == 0.0f);
    host_yaw = -20.0f; host_tick += T_TURN_SETTLE_MS; tick();
    CHECK(s_round == R_DONE && strstr(last_message, "status=STOP") != NULL && last_w == 0.0f);
    reset_fixture(); run_cmd("30"); run_cmd("g"); host_yaw = -95.0f; tick(); run_cmd("g");
    host_yaw = -96.0f; host_tick += T_TURN_SETTLE_MS; tick();
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
    reset_fixture(); run_cmd("30"); run_cmd("g"); host_yaw = -95.0f; tick();
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
    reset_fixture(); run_cmd("33"); CHECK(s_msel == R_FREE && strstr(last_message, "MODE_RANGE") != NULL);
    for (int mode = 20; mode <= 22; mode += 2) {
        reset_fixture(); s_msel = mode; run_cmd("g"); tick();
        CHECK(s_round == R_RUN && last_w == T_TURN_MAX_W);
        host_yaw = turn_target_deg(); tick(); host_tick += T_TURN_SETTLE_MS; tick();
        CHECK(s_round == R_DONE && strstr(last_message, "status=DONE") != NULL &&
              strstr(last_message, mode == 20 ? "cmd_deg=85" : "cmd_deg=180") != NULL);
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
    CHECK(check_trial32_router() == 0);
    puts("real g router: pending/running/terminal mission stop, a alias, closed gate, CRLF/idle framing, wheel/distance/turn/IMU/jog/servo/return passed");
    return 0;
}
