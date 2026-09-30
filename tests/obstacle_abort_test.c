/* Host-only check of the real obstacle step's abort/timeout ordering.
 * Fake IMU and clock values do not validate obstacle thresholds or mechanics. */
#include "auto_steps.h"
#include <stdint.h>
#include <stdio.h>

static uint32_t tick_ms;
static int aborted;
static int abort_on_delay;
static int imu_valid;
static int velocity_commands;
static int brake_commands;

uint32_t HAL_GetTick(void) { return tick_ms; }
void osDelay(uint32_t ms)
{
    tick_ms += ms;
    if (abort_on_delay) aborted = 1;
}
int run_aborted(void) { return aborted; }
uint8_t imu_ok(void) { return (uint8_t)imu_valid; }
float imu_yaw_deg(void) { return 0.0f; }
float imu_pitch_deg(void) { return 0.0f; }
float step_heading_kp_deg(void) { return 0.3f; }
void motion_vel_set(float vx, float vy, float w)
{
    (void)vx; (void)vy; (void)w;
    velocity_commands++;
}
void motion_brake(void) { brake_commands++; }

static void reset_case(void)
{
    tick_ms = 0;
    aborted = 0;
    abort_on_delay = 0;
    imu_valid = 1;
    velocity_commands = 0;
    brake_commands = 0;
}

static int check(const char *name, int commands)
{
    if (velocity_commands != commands || brake_commands < 1) {
        fprintf(stderr, "%s: velocity=%d expected=%d brake=%d\n",
                name, velocity_commands, commands, brake_commands);
        return 0;
    }
    return 1;
}

int main(void)
{
    reset_case();
    aborted = 1;
    if (step_cross_obstacle(150.0f, 0.0f, 8000u) != 0 ||
        !check("abort_before_entry", 0)) return 1;

    reset_case();
    abort_on_delay = 1;
    if (step_cross_obstacle(150.0f, 0.0f, 8000u) != 0 ||
        !check("abort_after_first_command", 1)) return 1;

    reset_case();
    if (step_cross_obstacle(150.0f, 0.0f, 0u) != 0 ||
        !check("deadline_before_entry", 0)) return 1;

    reset_case();
    if (step_cross_obstacle(150.0f, 0.0f, 10u) != 0 ||
        !check("deadline_after_two_ticks", 2)) return 1;

    reset_case();
    imu_valid = 0;
    if (step_cross_obstacle(150.0f, 0.0f, 8000u) != 0 ||
        !check("imu_invalid_before_entry", 0)) return 1;

    puts("obstacle abort/timeout: 5 cases passed");
    return 0;
}
