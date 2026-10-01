/* Host-only direction regression for real step_rotate_deg(±180, ±90).
 * Simulates one degree of IMU drift before first motor command; no physical
 * turn accuracy or loaded mecanum behavior is inferred. */
#include "steps.h"
#include "motion.h"
#include "control.h"
#include "arm.h"
#include "board_pins.h"
#include "imu.h"
#include <stdint.h>
#include <stdio.h>

static uint32_t now_ms;
static float first_heading;
static float later_heading;
static unsigned heading_reads;
static float first_w;
static unsigned drive_calls;
static unsigned brake_calls;

uint32_t HAL_GetTick(void) { return now_ms; }
void osDelay(uint32_t ms) { now_ms += ms; }
uint8_t imu_ok(void) { return 1u; }
float imu_heading_deg(void)
{
    return heading_reads++ == 0u ? first_heading : later_heading;
}
void motion_vel_set(float x, float y, float w)
{
    (void)x; (void)y;
    if (drive_calls++ == 0u) first_w = w;
    run_abort(); /* stop immediately after the first commanded turn direction */
}
void motion_brake(void) { brake_calls++; }
/* Other steps.c functions are linked in this host build; these inert stubs
 * satisfy their dependencies but are never called by this test. */
float motion_odo_mm(void) { return 0.0f; }
float motion_lateral_odo_mm(void) { return 0.0f; }
int32_t ctrl_enc_total(int wheel) { (void)wheel; return 0; }
uint8_t imu_zero_leg_heading(void) { return 1u; }
float imu_leg_heading_deg(void) { return 0.0f; }
void motion_linear_ramp_init(MotionRamp *r) { (void)r; }
float motion_linear_profile_step(MotionRamp *r, float cruise, float remain, float dt)
{
    (void)r; (void)remain; (void)dt;
    return cruise;
}
void bp_debug_send(const char *line) { (void)line; }
uint16_t arm_claw_command_us(void) { return 1500u; }
void arm_stepper_dir(int axis, int dir) { (void)axis; (void)dir; }
void arm_stepper_step(int axis) { (void)axis; }
void arm_claw_open(void) { }
void arm_claw_close(void) { }
void bp_laser_set(int on) { (void)on; }

static int check(int deg, float start, float first_sample, int expected_sign)
{
    now_ms = 0u;
    first_heading = start;
    later_heading = first_sample;
    heading_reads = 0u;
    first_w = 0.0f;
    drive_calls = 0u;
    brake_calls = 0u;
    run_reset();
    if (step_rotate_deg(deg, 0u) != 0 || drive_calls != 1u || brake_calls < 1u ||
        (expected_sign > 0 && first_w <= 0.0f) ||
        (expected_sign < 0 && first_w >= 0.0f)) {
        fprintf(stderr, "deg=%d start=%.1f sample=%.1f first_w=%.3f drives=%u brakes=%u\n",
                deg, start, first_sample, first_w, drive_calls, brake_calls);
        return 0;
    }
    return 1;
}

int main(void)
{
    if (!check(180, 0.0f, -1.0f, 1)) return 1;
    if (!check(180, 359.0f, 358.0f, 1)) return 1;
    if (!check(-180, 0.0f, 1.0f, -1)) return 1;
    if (!check(-90, 0.0f, 1.0f, -1)) return 1;
    if (!check(90, 0.0f, -1.0f, 1)) return 1;
    puts("turn first-command direction: 3 cases at 180 + left90 + right90 passed");
    return 0;
}
