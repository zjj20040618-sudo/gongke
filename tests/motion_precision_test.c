/* Real IK -> real speed-loop target/integrator, with inert board functions.
 * This checks numerical command resolution, not physical tracking accuracy. */
#include <stdio.h>
#include <math.h>
#include "../App/control.c"
#include "motion.h"
static unsigned brakes;
void bp_motor_set(int m, int dir, int duty) { (void)m; (void)dir; (void)duty; }
void bp_motor_brake(int m) { (void)m; ++brakes; }
void bp_motor_stop(int m) { (void)m; }
int32_t bp_enc_delta(int m) { (void)m; return 0; }
void bp_enc_raw_reset_all(void) { }
uint32_t HAL_GetTick(void) { return 0u; }
uint8_t imu_ok(void) { return 1u; }
float imu_heading_deg(void) { return 0.0f; }
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "precision line %d: %s\n", __LINE__, #x); return 1; } } while (0)
int main(void)
{
    float base[4], changed[4];
    const float w = 0.3f * 1.0f * 0.0174533f;
    ctrl_init();
    motion_ik_rpm(200.0f, 0.0f, 0.0f, base);
    motion_ik_rpm(200.0f, 0.0f, w, changed);
    CHECK(changed[0] > base[0] && changed[1] < base[1]);
    CHECK(changed[2] < base[2] && changed[3] > base[3]);
    motion_vel_set(200.0f, 0.0f, w);
    for (int i = 0; i < 4; ++i) CHECK(fabsf(s_target[i] - changed[i]) < 0.00001f);
    ctrl_tick_1ms();
    CHECK(s_ei[0] > s_ei[1] && s_ei[3] > s_ei[2]);
    motion_ik_rpm(0.0f, 300.0f, 0.0f, base);
    motion_ik_rpm(0.0f, 300.0f, w, changed);
    for (int i = 0; i < 4; ++i) CHECK(fabsf(changed[i] - base[i]) > 0.05f);
    motion_ik_rpm(0.0f, -300.0f, -w, base);
    for (int i = 0; i < 4; ++i) CHECK(fabsf(base[i] + changed[i]) < 0.0001f);
    motion_brake();
    for (int i = 0; i < 4; ++i) CHECK(s_target[i] == 0.0f && s_ei[i] == 0.0f);
    CHECK(brakes >= 8u);
    puts("real motion/control: fractional yaw reaches PI; forward/strafe/mirror and brake reset passed");
    return 0;
}
