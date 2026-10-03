/* Host-only test of real mission.c: inject an already accepted start request,
 * then abort before MissionTask wakes. The production configuration gate stays
 * closed; no firmware parameter is changed or bypassed on the robot. */
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../App/mission.c"

static jmp_buf terminal;
static int abort_flag;
static int reset_calls;
static int scene_calls;
static int velocity_calls;
static int brake_calls;
static int trial_gate_closed, trial_run_calls, trial_result;
static int32_t trial_qr[3];

uint32_t HAL_GetTick(void) { return 0u; }
void osDelay(uint32_t ms)
{
    if (ms == 200u && (mission_state() == MS_ABORT || mission_state() == MS_DONE))
        longjmp(terminal, 1);
}
void run_reset(void) { abort_flag = 0; reset_calls++; }
int run_aborted(void) { return abort_flag; }
void run_abort(void) { abort_flag = 1; }
void proto_send_scene(ProtoScene scene) { (void)scene; scene_calls++; }
void motion_brake(void) { brake_calls++; }
void motion_vel_set(float x, float y, float w)
{
    (void)x; (void)y; (void)w;
    velocity_calls++;
}
float motion_odo_mm(void) { return 0.0f; }
float motion_lateral_odo_mm(void) { return 0.0f; }
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
int step_prepare_leg(void) { return 1; }
int step_straight(float d, float v, uint32_t to)
{
    (void)d; (void)v; (void)to;
    return 1;
}
int step_strafe(float d, float v, uint32_t to)
{
    (void)d; (void)v; (void)to;
    return 1;
}
int step_nav_leg(float turn, float d, float v, uint32_t to)
{
    (void)turn; (void)d; (void)v; (void)to;
    return 1;
}
int wait_qr(int32_t d[3], uint32_t to) { (void)d; (void)to; return 0; }
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
const char *mission_trial_config_missing(void)
{
    return trial_gate_closed ? "TRIAL_VISION_UNCONFIRMED" : 0;
}
int mission_trial_run(void) { trial_run_calls++; return trial_result; }
void mission_trial_get_qr(int32_t out[3])
{
    out[0] = trial_qr[0]; out[1] = trial_qr[1]; out[2] = trial_qr[2];
}

static int check_trial_start(void)
{
    int32_t qr[3];
    mission_init();
    scene_calls = velocity_calls = trial_run_calls = 0;
    trial_gate_closed = 1;
    if (mission_start_trial() || s_start_req != 0 || mission_is_trial()) return 0;
    trial_gate_closed = 0;
    if (!mission_start_trial() || s_start_req != 2 || !mission_is_trial()) return 0;
    if (mission_qr_ready()) return 0; /* no QR defaults or partially decoded target */
    trial_qr[0] = 2; trial_qr[1] = 3; trial_qr[2] = 1;
    if (!mission_qr_ready()) return 0;
    trial_qr[2] = 0;
    if (mission_qr_ready()) return 0;
    trial_qr[2] = 4;
    if (mission_qr_ready()) return 0;
    trial_qr[2] = 1;
    mission_get_qr(qr);
    if (qr[0] != 2 || qr[1] != 3 || qr[2] != 1) return 0;
    run_abort(); /* Accepted g, then a/g/0 before MissionTask wakes. */
    if (setjmp(terminal) == 0) mission_main();
    if (mission_state() != MS_ABORT || !abort_flag || trial_run_calls != 0 ||
        scene_calls != 0 || velocity_calls != 0 || mission_start_trial()) return 0;

    mission_init(); trial_result = 1;
    if (!mission_start_trial()) return 0;
    if (setjmp(terminal) == 0) mission_main();
    if (mission_state() != MS_DONE || trial_run_calls != 1 ||
        !mission_is_trial() || mission_start_trial()) return 0;

    mission_init(); trial_result = 0;
    if (!mission_start_trial()) return 0;
    if (setjmp(terminal) == 0) mission_main();
    if (mission_state() != MS_ABORT || trial_run_calls != 2 ||
        !mission_is_trial() || mission_start_trial()) return 0;
    return scene_calls == 0 && velocity_calls == 0;
}

int main(void)
{
    MissionTargets decoded;
    int32_t qr[3];
    static const int32_t bad_qr[][3] = {
        {0, 1, 1}, {1, 0, 1}, {1, 1, 0},
        {4, 1, 1}, {1, 4, 1}, {1, 1, 4}, {-1, 1, 1}
    };
    for (int ball = 1; ball <= 3; ++ball) {
        for (int target = 1; target <= 3; ++target) {
            for (int hostage = 1; hostage <= 3; ++hostage) {
                qr[0] = ball; qr[1] = target; qr[2] = hostage;
                if (!qr_decode_targets(qr, &decoded) ||
                    decoded.ball_color != ball - 1 ||
                    decoded.target_color != target - 1 ||
                    decoded.hostage_shape != hostage + 2) {
                    fputs("Valid QR target mapping failed\n", stderr);
                    return 1;
                }
            }
        }
    }
    for (unsigned i = 0; i < sizeof bad_qr / sizeof bad_qr[0]; ++i) {
        decoded.ball_color = 77;
        decoded.target_color = 78;
        decoded.hostage_shape = 79;
        if (qr_decode_targets(bad_qr[i], &decoded) ||
            decoded.ball_color != 77 || decoded.target_color != 78 ||
            decoded.hostage_shape != 79) {
            fputs("Invalid QR changed locked targets\n", stderr);
            return 1;
        }
    }

    mission_init();
    if (!mission_config_missing() ||
        strcmp(mission_config_missing(), "R_PRE_CROSS_FWD_MM") != 0 ||
        mission_start() != 0 || reset_calls != 1) {
        fputs("Production mission configuration gate unexpectedly opened\n", stderr);
        return 1;
    }

    /* Test-only injection after the real gate check: emulate g being accepted
     * in a calibrated future build, then a arriving before the 10 ms wake. */
    s_start_req = 1;
    run_abort();
    if (setjmp(terminal) == 0) {
        mission_main();
        fputs("MissionTask unexpectedly returned\n", stderr);
        return 1;
    }
    if (mission_state() != MS_ABORT || reset_calls != 1 || !abort_flag ||
        scene_calls != 0 || velocity_calls != 0 || brake_calls < 1) {
        fprintf(stderr, "startup abort failed: state=%d reset=%d flag=%d scene=%d velocity=%d brake=%d\n",
                (int)mission_state(), reset_calls, abort_flag,
                scene_calls, velocity_calls, brake_calls);
        return 1;
    }
    puts("mission QR mapping: 27 valid + 7 invalid; startup abort: 1; production gate closed");
    if (!check_trial_start()) {
        fputs("Trial32 request gate/start-abort/terminal routing failed\n", stderr);
        return 1;
    }
    puts("trial32 mission request: independent gate, pending abort, no formal motion, QR log delegation, DONE/ABORT terminal passed");
    return 0;
}
