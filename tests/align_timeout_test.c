/* Host-only regression: a finite OBJ alignment timeout must cover waiting
 * for a matching frame. No camera or chassis behavior is inferred. */
#include "steps.h"
#include "motion.h"
#include "control.h"
#include "arm.h"
#include "board_pins.h"
#include "imu.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static uint32_t now_ms;
static unsigned brake_calls;
static unsigned alignment_receive_end_calls;

uint32_t HAL_GetTick(void) { return now_ms; }
void osDelay(uint32_t ms) { now_ms += ms; }
uint8_t imu_ok(void) { return 1u; }
uint8_t imu_zero_leg_heading(void) { return 1u; }
float imu_heading_deg(void) { return 0.0f; }
float imu_leg_heading_deg(void) { return 0.0f; }
void motion_brake(void) { brake_calls++; }
void motion_vel_set(float x, float y, float w) { (void)x; (void)y; (void)w; }
float motion_odo_mm(void) { return 0.0f; }
float motion_lateral_odo_mm(void) { return 0.0f; }
int32_t ctrl_enc_total(int wheel) { (void)wheel; return 0; }
void motion_linear_ramp_init(MotionRamp *r) { (void)r; }
float motion_linear_profile_step(MotionRamp *r, float cruise, float remain, float dt)
{ (void)r; (void)remain; (void)dt; return cruise; }
void bp_debug_send(const char *line) { (void)line; }
uint16_t arm_claw_command_us(void) { return 1500u; }
void arm_stepper_dir(int axis, int dir) { (void)axis; (void)dir; }
void arm_stepper_step(int axis) { (void)axis; }
void arm_claw_open(void) { }
void arm_claw_close(void) { }
void bp_laser_set(int on) { (void)on; }
void proto_receive_end(void) { alignment_receive_end_calls++; }
/* These legacy alignment cases never request a new recognition scene.
 * Satisfy the newly linked handshake without making a wait falsely succeed. */
void proto_send_scene(ProtoScene scene)
{
    (void)scene;
    fputs("legacy alignment unexpectedly requested a vision scene\n", stderr);
    abort();
}
int proto_scene_status(void)
{
    fputs("legacy alignment unexpectedly polled a vision scene\n", stderr);
    abort();
}
int proto_target_filter(ProtoTask task, uint8_t digit, int *cls, int *label)
{
    (void)task; (void)digit; (void)cls; (void)label;
    fputs("legacy alignment unexpectedly selected a vision target\n", stderr); abort();
}
int proto_send_target(ProtoTask task, uint8_t digit)
{
    (void)task; (void)digit;
    fputs("legacy alignment unexpectedly requested a vision target\n", stderr); abort();
}

int main(void)
{
    now_ms = 0u;
    brake_calls = 0u;
    run_reset();
    /* Node preparation consumes ~1 s; a missing OBJ must return at 2 s. */
    if (step_align(CLS_BALL, LAB_R, 2000u) != 0 ||
        now_ms < 2000u || now_ms > 2010u || brake_calls < 2u) {
        fprintf(stderr, "align timeout failed: ms=%lu brakes=%u\n",
                (unsigned long)now_ms, brake_calls);
        return 1;
    }
    puts("alignment missing-frame timeout: 1 case passed");
    return 0;
}
