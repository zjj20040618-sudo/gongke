/* Actual injected heading helper -> actual fractional IK/target dispatch.
 * Link with real App/steps.c and App/motion.c using gc-sections. The existing
 * precise_velocity_test additionally verifies the real PI/PWM downstream. */
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <float.h>
#include "steps.h"
#include "motion.h"
#include "control.h"

static float host_yaw, delivered[4];
static unsigned delivered_mask, precise_calls, integer_calls;

float imu_leg_heading_deg(void) { return host_yaw; }
uint8_t imu_ok(void) { return 1u; }
float imu_heading_deg(void) { return 0.0f; }
uint32_t HAL_GetTick(void) { return 0u; }
void ctrl_stop_all(void) { }
int32_t ctrl_enc_total(int motor) { (void)motor; return 0; }
void ctrl_get_rpm_fast_all(float out[4])
{ for (int i = 0; i < 4; ++i) out[i] = 0.0f; }
void ctrl_set_speed(int motor, int16_t rpm)
{ (void)motor; (void)rpm; integer_calls++; }
void ctrl_set_speed_precise(int motor, float rpm)
{ delivered[motor] = rpm; delivered_mask |= 1u << (unsigned)motor; precise_calls++; }

/* MinGW PE still resolves dependencies in discarded functions. These inert
 * peripheral stubs satisfy linkage and fail if a heading-only test enters any
 * wait, recognition or actuator path. */
static void unexpected_path(void)
{ fputs("heading-only test entered unrelated task/actuator path\n", stderr); abort(); }
void osDelay(uint32_t ms) { (void)ms; unexpected_path(); }
uint8_t imu_zero_leg_heading(void) { unexpected_path(); return 0u; }
void bp_debug_send(const char *line) { (void)line; unexpected_path(); }
uint16_t arm_claw_command_us(void) { unexpected_path(); return 0u; }
void arm_stepper_dir(int axis, int dir) { (void)axis; (void)dir; unexpected_path(); }
void arm_stepper_step(int axis) { (void)axis; unexpected_path(); }
void arm_claw_open(void) { unexpected_path(); }
void arm_claw_close(void) { unexpected_path(); }
void bp_laser_set(int on) { (void)on; unexpected_path(); }
void proto_send_scene(ProtoScene scene) { (void)scene; unexpected_path(); }
void proto_receive_end(void) { } /* heading-test stop has no camera TX */
void proto_stats_get(ProtoStats *out) { (void)out; unexpected_path(); }
int proto_scene_status(void) { unexpected_path(); return 0; }
int proto_target_filter(ProtoTask task, uint8_t digit, int *cls, int *label)
{ (void)task; (void)digit; (void)cls; (void)label; unexpected_path(); return 0; }
int proto_send_target(ProtoTask task, uint8_t digit)
{ (void)task; (void)digit; unexpected_path(); return 0; }

#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "real speed/yaw line %d: %s\n", __LINE__, #expr); return 1; \
} } while (0)

static int check_helper(void)
{
    host_yaw = 3.0f;
    CHECK(step_heading_kp_set(0.3f));
    CHECK(fabsf(step_heading_hold_w_kp(0.0f, 2.0f) + 6.0f * 0.0174533f) < 0.000001f);
    CHECK(fabsf(step_heading_hold_w(0.0f) + 0.9f * 0.0174533f) < 0.000001f);
    CHECK(step_heading_kp_deg() == 0.3f); /* injection never overwrites global */
    CHECK(!step_heading_kp_set(NAN) && !step_heading_kp_set(INFINITY));
    CHECK(step_heading_hold_w_kp(0.0f, NAN) == 0.0f && step_heading_hold_w_kp(0.0f, INFINITY) == 0.0f);
    CHECK(step_heading_hold_w_kp(0.0f, -0.1f) == 0.0f && step_heading_hold_w_kp(0.0f, 5.1f) == 0.0f);
    CHECK(step_heading_kp_deg() == 0.3f);
    host_yaw = -3.0f; CHECK(step_heading_hold_w_kp(0.0f, 2.0f) > 0.0f);
    CHECK(step_heading_hold_w_kp(0.0f, 0.0f) == 0.0f);
    host_yaw = 120.0f; CHECK(step_heading_hold_w_kp(0.0f, 5.0f) == -2.0f);
    host_yaw = -120.0f; CHECK(step_heading_hold_w_kp(0.0f, 5.0f) == 2.0f);
    host_yaw = 179.0f;
    CHECK(fabsf(step_heading_hold_w_kp(-179.0f, 2.0f) - 4.0f * 0.0174533f) < 0.000001f);
    host_yaw = -179.0f;
    CHECK(fabsf(step_heading_hold_w_kp(179.0f, 2.0f) + 4.0f * 0.0174533f) < 0.000001f);
    return 0;
}

static int check_actual_wheel_correction(void)
{
    static const float speeds[] = { 100.0f, 200.0f, 300.0f };
    static const int yaw_sign[4] = { 1, -1, -1, 1 };
    for (int mode = 15; mode <= 18; ++mode) {
        for (int k = 0; k < 3; ++k) {
            float base[4], low_gain[4], corrected[4];
            float vx = mode == 15 ? speeds[k] : mode == 16 ? -speeds[k] : 0.0f;
            float vy = mode == 17 ? -speeds[k] : mode == 18 ? speeds[k] : 0.0f;
            host_yaw = 3.0f;
            motion_ik_precise(vx, vy, 0.0f, base);
            motion_ik_precise(vx, vy, step_heading_hold_w_kp(0.0f, 0.3f), low_gain);
            motion_ik_precise(vx, vy, step_heading_hold_w_kp(0.0f, 2.0f), corrected);
            delivered_mask = precise_calls = integer_calls = 0u;
            motion_vel_set_precise(vx, vy, step_heading_hold_w_kp(0.0f, 2.0f));
            CHECK(delivered_mask == 15u && precise_calls == 4u && integer_calls == 0u);
            for (int m = 0; m < 4; ++m) {
                CHECK(delivered[m] == corrected[m]);
                CHECK((corrected[m] - base[m]) * (float)yaw_sign[m] < 0.0f);
                CHECK(fabsf(corrected[m] - base[m]) > fabsf(low_gain[m] - base[m]));
            }
        }
    }
    /* Fractional correction that integer legacy IK swallows must still arrive. */
    {
        int16_t base[4], integer[4];
        float precise_base[4];
        host_yaw = 3.0f;
        motion_ik(100.0f, -1.25f, 0.0f, base);
        motion_ik(100.0f, -1.25f, step_heading_hold_w_kp(0.0f, 0.3f), integer);
        motion_ik_precise(100.0f, -1.25f, 0.0f, precise_base);
        motion_vel_set_precise(100.0f, -1.25f, step_heading_hold_w_kp(0.0f, 0.3f));
        for (int m = 0; m < 4; ++m) {
            CHECK(base[m] == integer[m]);
            CHECK(fabsf(delivered[m] - precise_base[m]) > 0.2f);
        }
    }
    return 0;
}

static int check_nonfinite_overflow_and_huge_headings(void)
{
    static const float bad[] = {NAN,INFINITY,-INFINITY};
    for (unsigned n = 0u; n < sizeof bad / sizeof bad[0]; ++n) {
        host_yaw = bad[n];
        CHECK(step_heading_hold_w_kp(0.0f,2.0f) == 0.0f);
        CHECK(step_heading_hold_w(0.0f) == 0.0f);
        host_yaw = 0.0f;
        CHECK(step_heading_hold_w_kp(bad[n],2.0f) == 0.0f);
    }
    /* Both operands are finite, but their subtraction is not. */
    host_yaw = FLT_MAX;
    CHECK(step_heading_hold_w_kp(-FLT_MAX,2.0f) == 0.0f);
    host_yaw = -FLT_MAX;
    CHECK(step_heading_hold_w_kp(FLT_MAX,2.0f) == 0.0f);

    /* These enormous finite differences must return promptly, not iterate once
     * per full revolution; then only bounded finite targets may reach IK. */
    static const float huge[] = {1.0e10f,-1.0e10f,1.0e20f,-1.0e20f,FLT_MAX,-FLT_MAX};
    for (unsigned n = 0u; n < sizeof huge / sizeof huge[0]; ++n) {
        host_yaw = huge[n];
        float w = step_heading_hold_w_kp(0.0f,2.0f);
        CHECK(isfinite(w) && fabsf(w) <= 2.0f);
        delivered_mask = precise_calls = integer_calls = 0u;
        motion_vel_set_precise(0.0f,0.0f,w);
        CHECK(delivered_mask == 15u && precise_calls == 4u && integer_calls == 0u);
        for (unsigned m = 0u; m < 4u; ++m) CHECK(isfinite(delivered[m]));
    }
    host_yaw = 721.0f;
    CHECK(fabsf(step_heading_hold_w_kp(0.0f,2.0f) + 2.0f*0.0174533f) < 0.000001f);
    host_yaw = -721.0f;
    CHECK(fabsf(step_heading_hold_w_kp(0.0f,2.0f) - 2.0f*0.0174533f) < 0.000001f);
    CHECK(step_heading_kp_deg() == 0.3f);
    puts("real heading helper: NaN/Inf inputs and finite subtraction overflow return0; six huge finite differences return bounded targets without revolution loops passed");
    return 0;
}

int main(void)
{
    CHECK(check_helper() == 0);
    CHECK(check_actual_wheel_correction() == 0);
    CHECK(check_nonfinite_overflow_and_huge_headings() == 0);
    puts("real injected yaw/IK: sign, zero, wrapping, clamp, global isolation, 12 direction/velocity targets and fractional propagation passed");
    return 0;
}
