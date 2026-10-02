/* Actual Bluetooth command router/state machine with host-only peripheral stubs.
 * A mission-start stub models an accepted future start; production calibration
 * gates are NOT changed. Real gate/start-abort checks live in the other fixture. */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "../App/test.c"

static uint32_t host_tick;
static MissionState host_state;
static int host_abort, start_calls, brake_calls, pulse_calls, gate_closed;
static uint16_t host_servo;
static float host_fore, host_lateral, host_vx, host_vy, host_w;
static int host_imu_valid, reset_calls;
static int32_t host_counts[4];
static char last_message[512];

uint32_t HAL_GetTick(void) { return host_tick; }
void bp_debug_send(const char *s)
{
    /* Production send() appends CRLF in a separate UART write. */
    if (strcmp(s, "\r\n") != 0)
        snprintf(last_message, sizeof last_message, "%s", s);
}
void bp_laser_set(int on) { (void)on; }
int32_t bp_enc_raw_total(int m) { (void)m; return 0; }
void bp_enc_raw_reset_all(void) { }
void motion_brake(void) { brake_calls++; host_vx = host_vy = host_w = 0.0f; }
void motion_vel_set(float x, float y, float w) { host_vx = x; host_vy = y; host_w = w; }
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
void ctrl_enc_reset_all(void)
{ reset_calls++; host_fore = host_lateral = 0.0f; memset(host_counts, 0, sizeof host_counts); }
int32_t ctrl_enc_total(int m) { return host_counts[m]; }
void ctrl_get_rpm_est_all(int16_t out[4]) { memset(out, 0, 4 * sizeof *out); }
void ctrl_get_rpm_fast_all(float out[4]) { memset(out, 0, 4 * sizeof *out); }
int16_t ctrl_get_rpm_est(int m) { (void)m; return 0; }
void ctrl_tune_get(CtrlTune *out) { memset(out, 0, sizeof *out); }
int ctrl_tune_set(const CtrlTune *in) { (void)in; return 1; }
void arm_claw_set_us(uint16_t us) { host_servo = us; }
uint16_t arm_claw_command_us(void) { return host_servo; }
void arm_claw_open(void) { host_servo = 1000u; }
void arm_claw_close(void) { host_servo = 1800u; }
void arm_stepper_dir(int axis, int dir) { (void)axis; (void)dir; }
void arm_stepper_step(int axis) { (void)axis; pulse_calls++; }
float imu_yaw_deg(void) { return 0.0f; }
float imu_heading_deg(void) { return 0.0f; }
float imu_leg_heading_deg(void) { return 0.0f; }
float imu_pitch_deg(void) { return 0.0f; }
float imu_roll_deg(void) { return 0.0f; }
uint8_t imu_ok(void) { return (uint8_t)host_imu_valid; }
uint8_t imu_zero_leg_heading(void) { return 1u; }
uint32_t imu_last_valid_age_ms(void) { return 0u; }
void run_abort(void) { host_abort = 1; }
int run_aborted(void) { return host_abort; }
int step_prepare_leg(void) { return !host_abort && host_imu_valid; }
float step_heading_hold_w(float heading) { (void)heading; return 0.0f; }
float step_heading_kp_deg(void) { return 0.3f; }
int step_heading_kp_set(float v) { (void)v; return 1; }
float step_orth_kp(void) { return 0.0f; }
int step_orth_kp_set(float v) { (void)v; return 1; }
float step_orth_hold_cmd(int lateral, float ref) { (void)lateral; (void)ref; return 0.0f; }
MissionState mission_state(void) { return host_state; }
const char *mission_config_missing(void) { return gate_closed ? "QR_START_LEFT_MM" : 0; }
int mission_start(void) { start_calls++; return !gate_closed; }
void robot_diag_report(void) { }

static void reset_fixture(void)
{
    host_tick = 100u; host_state = MS_BOOT;
    host_abort = start_calls = brake_calls = pulse_calls = gate_closed = 0;
    host_servo = 1400u; last_message[0] = '\0';
    host_fore = host_lateral = host_vx = host_vy = host_w = 0.0f;
    host_imu_valid = 1; reset_calls = 0;
    memset(host_counts, 0, sizeof host_counts);
    test_init();
}

#define CHECK(expr) do { if (!(expr)) { fprintf(stderr, "g test line %d: %s\n", __LINE__, #expr); return 1; } } while (0)

static void settle(void) { host_tick += T_DIST_STILL_MS; tick(); }

/* Real command state machine with synthetic signed axis odometry, not a physics model. */
static int check_walk(int mode)
{
    char selection[8];
    float sign, goal;
    int lateral, resets_at_start;
    reset_fixture(); snprintf(selection, sizeof selection, "%d", mode);
    host_fore = 50.0f; host_lateral = 90.0f; /* nonzero origin for legacy modes */
    run_cmd(selection); run_cmd("v200"); run_cmd("d500"); run_cmd("g");
    CHECK(s_round == R_RUN && s_walk_origin_valid);
    lateral = dist_lateral(); sign = lateral ? s_rvy : s_rvx;
    resets_at_start = reset_calls;
    goal = s_dist_odo0;
    if (lateral) host_lateral = goal + sign * 120.0f;
    else host_fore = goal + sign * 120.0f;
    run_cmd("g");
    CHECK(s_round == R_BRAKE && !host_vx && !host_vy);
    run_cmd("g"); /* too soon must not queue or launch the reverse */
    CHECK(s_round == R_BRAKE && !host_vx && !host_vy);
    if (lateral) host_lateral += sign * 7.0f;
    else host_fore += sign * 7.0f;
    host_counts[0]++; host_tick += 100u; tick(); /* brake coast counts are retained */
    settle();
    CHECK(s_round == R_DONE && reset_calls == resets_at_start && s_dist_odo0 == goal);
    CHECK(strstr(last_message, "phase=OUT status=STOP") != NULL);
    CHECK(strstr(last_message, "c3=") != NULL); /* the ASCII record remains complete */
    run_cmd("d900"); run_cmd("v500"); /* cannot replace this round's origin/speed */
    run_cmd("g"); tick();
    CHECK(s_round == R_RET && s_walk_returning && s_dist_target == -sign * 127.0f);
    CHECK((lateral ? host_vy : host_vx) == -sign * 200.0f && host_w == 0.0f);
    CHECK((lateral ? host_vx : host_vy) == 0.0f);
    if (lateral) host_lateral = goal + sign * 50.0f;
    else host_fore = goal + sign * 50.0f;
    run_cmd("g"); settle();
    CHECK(s_round == R_DONE && s_walk_origin_valid && !host_vx && !host_vy);
    CHECK(strstr(last_message, "phase=RETURN status=STOP") != NULL);
    run_cmd("g"); tick();
    CHECK(s_round == R_RET && s_dist_target == -sign * 50.0f);
    if (lateral) host_lateral = goal;
    else host_fore = goal;
    tick(); settle();
    CHECK(s_round == R_READY && !s_walk_origin_valid && !host_vx && !host_vy);
    CHECK(strstr(last_message, "phase=RETURN status=DONE") != NULL);
    CHECK(reset_calls == resets_at_start);
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

    for (int mode = 1; mode <= 18; mode++) {
        if ((mode <= 6 || mode >= 15) && check_walk(mode)) return 1;
    }
    reset_fixture(); run_cmd("15"); run_cmd("v200"); run_cmd("d100"); run_cmd("g");
    host_fore = 110.0f; tick(); settle(); /* automatic OUT stop is also a pause */
    CHECK(s_round == R_DONE && s_walk_origin_valid);
    run_cmd("g"); tick();
    CHECK(s_round == R_RET && s_dist_target == -110.0f && host_vx == -200.0f);
    host_fore = -1.0f; tick(); settle(); /* crossing the encoder origin stops, no oscillation */
    CHECK(s_round == R_READY && !host_vx && !s_walk_origin_valid);

    reset_fixture(); run_cmd("5"); run_cmd("v100"); run_cmd("d100"); run_cmd("g");
    host_fore = 41.0f; host_tick += 1000u; tick(); settle();
    CHECK(s_round == R_DONE);
    run_cmd("g"); tick();
    CHECK(s_round == R_RET && s_dist_target == -41.0f && host_vx == -100.0f); /* not 100mm/time */
    run_cmd("a");
    CHECK(s_round == R_READY && !s_walk_origin_valid && !host_vx && !host_vy);

    reset_fixture(); run_cmd("17"); run_cmd("lff0.01"); run_cmd("rff0.04"); run_cmd("g");
    host_lateral = -80.0f; run_cmd("g"); settle(); run_cmd("g"); tick();
    CHECK(host_vy == 300.0f && fabsf(host_vx - 12.0f) < 0.001f);
    run_cmd("0"); CHECK(s_round == R_READY && !s_walk_origin_valid && !host_vx && !host_vy);
    reset_fixture(); run_cmd("18"); run_cmd("lff0.01"); run_cmd("rff0.04"); run_cmd("g");
    host_lateral = 80.0f; run_cmd("g"); settle(); run_cmd("g"); tick();
    CHECK(host_vy == -300.0f && fabsf(host_vx + 3.0f) < 0.001f);
    host_imu_valid = 0; tick(); settle();
    CHECK(s_round == R_READY && !s_walk_origin_valid && !host_vx && !host_vy);
    CHECK(strstr(last_message, "phase=RETURN status=IMUERR") != NULL);

    reset_fixture(); run_cmd("15"); run_cmd("v200"); run_cmd("d200"); run_cmd("g");
    host_fore = 40.0f; run_cmd("g"); settle(); host_imu_valid = 0; run_cmd("g");
    CHECK(s_round == R_DONE && !host_vx && !host_vy && !s_walk_returning);
    host_imu_valid = 1; host_fore = -10.0f; run_cmd("g");
    CHECK(s_round == R_DONE && !host_vx && !host_vy && !s_walk_returning);
    run_cmd("16"); CHECK(!s_walk_origin_valid); /* changing mode discards old origin */

    reset_fixture(); run_cmd("15"); run_cmd("v200"); run_cmd("d200"); run_cmd("g");
    run_cmd("g"); settle(); run_cmd("g"); /* no travel: do not invent a reverse leg */
    CHECK(s_round == R_READY && !host_vx && !host_vy && !s_walk_origin_valid);
    reset_fixture(); run_cmd("r1"); run_cmd("d100"); run_cmd("g");
    host_lateral = -30.0f; run_cmd("g"); settle(); run_cmd("g"); tick();
    CHECK(s_round == R_RET && host_vy == 300.0f && s_route_leg == 1u);
    run_cmd("a");
    run_cmd("r2"); run_cmd("d100"); run_cmd("g");
    host_fore = -30.0f; run_cmd("g"); settle(); run_cmd("g"); tick();
    CHECK(s_round == R_RET && host_vx == 80.0f && s_route_leg == 2u);
    run_cmd("0");
    run_cmd("r3"); run_cmd("d100"); run_cmd("g");
    host_fore = 30.0f; run_cmd("g"); settle(); run_cmd("g"); tick();
    CHECK(s_round == R_RET && host_vx == -80.0f && s_route_leg == 3u);

    puts("walk g cycle: all 10 modes signed return / brake drift / pause-resume / speed snapshot; auto-stop, timed actual distance, directional FF, IMU failure and cancel passed");
    puts("real g router: pending/running/terminal mission stop, a alias, closed gate, CRLF/idle framing, wheel/distance/turn/IMU/jog/servo/return passed");
    return 0;
}
