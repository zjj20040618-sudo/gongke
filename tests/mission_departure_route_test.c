/* Real mission.c with synthetic odometry and QR frames. No calibration gate
 * is enabled, and these checks do not certify physical distances or turns. */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../App/mission.c"

static float fore_mm, lateral_mm, vx, vy;
static uint32_t tick_ms;
static int aborted, leg_number, qr_on_leg, delivered;
static int direction_bad, nav_calls, nav_ok, nav_angle;
static int straight_calls, straight_ok;
static float straight_d, straight_v, prepare_fore_shift;

uint32_t HAL_GetTick(void) { return tick_ms; }
void osDelay(uint32_t ms)
{
    fore_mm += vx * (float)ms / 1000.0f;
    lateral_mm += vy * (float)ms / 1000.0f;
    tick_ms += ms;
    if (tick_ms > 2000u) aborted = 1; /* bound only the host fixture */
}
void run_reset(void) { aborted = 0; }
int run_aborted(void) { return aborted; }
void run_abort(void) { aborted = 1; }
void proto_send_scene(ProtoScene scene) { (void)scene; }
int step_vision_scene(ProtoScene scene) { (void)scene; return !aborted; }
void motion_brake(void) { vx = vy = 0.0f; }
void motion_vel_set(float x, float y, float w)
{
    vx = x; vy = y;
    if (w != 0.0f ||
        (leg_number == 1 && (x != 0.0f || y >= 0.0f)) ||
        ((leg_number == 2 || leg_number == 4) && (x >= 0.0f || y != 0.0f)) ||
        (leg_number == 3 && (x <= 0.0f || y != 0.0f))) direction_bad = 1;
}
float motion_odo_mm(void) { return fore_mm; }
float motion_lateral_odo_mm(void) { return lateral_mm; }
void motion_linear_ramp_init(MotionRamp *r) { memset(r, 0, sizeof *r); }
float motion_linear_profile_step(MotionRamp *r, float cruise, float remain, float dt)
{
    (void)r; (void)remain; (void)dt;
    return cruise;
}
float step_orth_hold_cmd(int lateral, float reference)
{
    (void)lateral; (void)reference;
    return 0.0f;
}
float step_heading_hold_w(float heading) { (void)heading; return 0.0f; }
int step_prepare_leg(void)
{
    leg_number++;
    if (aborted) return 0;
    fore_mm += prepare_fore_shift;
    prepare_fore_shift = 0.0f;
    return 1;
}
int step_straight(float d, float v, uint32_t to)
{
    (void)to;
    straight_calls++; straight_d = d; straight_v = v;
    return straight_ok && !aborted;
}
int step_strafe(float d, float v, uint32_t to)
{
    (void)d; (void)v; (void)to;
    return 1;
}
int step_nav_leg(float turn, float d, float v, uint32_t to)
{
    nav_calls++; nav_angle = (int)turn;
    if (d != 0.0f || v != 0.0f || to != 0u) return 0;
    return nav_ok && !aborted;
}
int wait_qr(int32_t d[3], uint32_t to)
{
    (void)to;
    if (delivered || leg_number != qr_on_leg) return 0;
    d[0] = 1; d[1] = 2; d[2] = 3;
    delivered = 1;
    return 1;
}
uint8_t imu_ok(void) { return 1u; }
int step_cross_obstacle(float x, float y, uint32_t timeout)
{
    (void)x; (void)y; (void)timeout;
    return 1;
}
int task_eod_run(int color) { (void)color; return TASK_OK; }
int task_anti_run(int color) { (void)color; return TASK_OK; }
int task_rescue_run(int shape) { (void)shape; return TASK_OK; }
const char *motion_profile_config_missing(void) { return 0; }
const char *steps_config_missing(void) { return 0; }
const char *task_eod_config_missing(void) { return 0; }
const char *task_rescue_config_missing(void) { return 0; }

static void reset_fixture(int qr_leg)
{
    fore_mm = 500.0f; lateral_mm = 200.0f; vx = vy = 0.0f;
    tick_ms = 0u; aborted = leg_number = delivered = direction_bad = 0;
    nav_calls = 0; nav_angle = 0; nav_ok = 1;
    straight_calls = 0; straight_ok = 1;
    straight_d = straight_v = prepare_fore_shift = 0.0f;
    qr_on_leg = qr_leg;
    mission_init();
}

static int check_endpoints(int qr_leg, int expected_legs)
{
    reset_fixture(qr_leg);
    if (!qr_travel_legs(20.0f, 30.0f, 100.0f, 200.0f) || direction_bad ||
        leg_number != expected_legs || nav_calls != 0 ||
        fabsf(fore_mm - 470.0f) > 0.01f || fabsf(lateral_mm - 180.0f) > 0.01f ||
        vx != 0.0f || vy != 0.0f || !mission_qr_ready() ||
        s_targets.ball_color != 0 || s_targets.target_color != 1 ||
        s_targets.hostage_shape != 5) {
        fprintf(stderr, "QR leg=%d got legs=%d fore=%.2f lateral=%.2f bad=%d\n",
                qr_leg, leg_number, fore_mm, lateral_mm, direction_bad);
        return 0;
    }
    return 1;
}

int main(void)
{
    if (!check_endpoints(1, 2)) return 1; /* early QR still completes both legs */
    if (!check_endpoints(2, 2)) return 1; /* QR during first backward leg */
    if (!check_endpoints(3, 4)) return 1; /* QR on forward rescan still goes back */
    if (!check_endpoints(4, 4)) return 1; /* QR on backward rescan completes endpoint */
    reset_fixture(0);
    if (qr_travel_legs(20.0f, 30.0f, 100.0f, 200.0f) || !aborted ||
        vx != 0.0f || vy != 0.0f || nav_calls != 0 || mission_qr_ready()) return 1;
    reset_fixture(1);
    if (qr_travel_legs(0.0f, 30.0f, 100.0f, 200.0f) || leg_number != 0 ||
        mission_start() || strcmp(mission_config_missing(), "QR_START_LEFT_MM") != 0) return 1;
    reset_fixture(1);
    if (!route_pre_cross_turn() || nav_calls != 1 || nav_angle != -90) return 1;
    nav_ok = 0;
    if (route_pre_cross_turn()) return 1;
    run_abort();
    if (route_pre_cross_turn()) return 1;
    puts("departure route: 4 QR endpoint cases + abort + zero gate + left90 success/fail/abort passed");
    /* Fixture values only: entry 500, next entry 600; scan changed the fore axis. */
    reset_fixture(0); fore_mm = 530.0f;
    if (!route_straight_to(600.0f, 200.0f) || straight_calls != 1 ||
        straight_d != 70.0f || straight_v != 200.0f) return 1;
    reset_fixture(0); fore_mm = 650.0f;
    if (!route_straight_to(600.0f, 200.0f) || straight_calls != 1 ||
        straight_d != -50.0f || straight_v != -200.0f) return 1;
    reset_fixture(0); fore_mm = 600.2f;
    if (!route_straight_to(600.0f, 200.0f) || straight_calls != 0) return 1;
    reset_fixture(0); fore_mm = 530.0f; prepare_fore_shift = 2.0f;
    if (!route_straight_to(600.0f, 200.0f) || straight_calls != 1 || straight_d != 68.0f) return 1;
    reset_fixture(0); run_abort();
    if (route_straight_to(600.0f, 200.0f) || straight_calls != 0) return 1;
    reset_fixture(0); straight_ok = 0;
    if (route_straight_to(600.0f, 200.0f) || straight_calls != 1) return 1;
    puts("task-exit forward remainder: scan offset / overshoot / at goal / settle drift / abort / failure passed");
    return 0;
}
