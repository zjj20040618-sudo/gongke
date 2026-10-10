#ifndef APP_ARM_H
#define APP_ARM_H

#include <stdint.h>

/* 机械臂：爪子开合 = 舵机(TIM12 CH2, PB15, 50Hz)；两轴步进：
 * axis0 STEP/DIR=PA9/PA10，axis1 STEP/DIR=PA11/PA12（2026-10-06恢复原映射，用户飞线；开漏、低有效）。
 * STEP 低脉宽由 Cortex-M4 DWT 周期计数器产生。 */

#define ARM_STEPPER_NUM 2
/* Software command envelope, NOT calibrated claw/mechanical end stops. */
#define ARM_SERVO_MIN_US 500u
#define ARM_SERVO_MAX_US 2500u
/* User-confirmed initial pulse; independent of uncalibrated open/close values. */
#define ARM_SERVO_START_US 1150u
/* Legacy cc/formal close. Mode31 uses private2100;43/independent42 retain1900. */
#define ARM_SERVO_GRIP_US 1700u

void arm_init(void);
void arm_claw_set_us(uint16_t us);            /* 脉宽限幅到 ARM_SERVO_MIN/MAX_US */
uint16_t arm_claw_command_us(void);            /* 最近一次发出的命令脉宽，不是舵机位置反馈 */
void arm_claw_open(void);                     /* 骨架：发预置的开/闭脉宽 */
void arm_claw_close(void);
void arm_stepper_dir(int axis, int dir);      /* dir 0/1；先设方向再发脉冲 */
void arm_stepper_step(int axis);              /* 单发 1 步(约20us低电平脉宽，DWT计时) */
/* Dedicated TIM7 period IRQ for bench STEP jobs; TIM6 HAL tick is not used.
 * 1..20000 pulses/s, nearest representable period. Returns 1 if started,
 * otherwise 0 with the clock stopped. Caller publishes its job atomically
 * before starting; the IRQ callback does not use RTOS/UART services. */
int arm_stepper_clock_start(uint16_t pps);
void arm_stepper_clock_stop(void);            /* Safe even before arm_init. */
/* Rebase the next period immediately before an actual STEP; discard a stale
 * update rather than catch up with back-to-back pulses. IRQ latency may lower
 * effective pps. Inactive/uninitialized clocks are left untouched. */
void arm_stepper_clock_rephase(void);

#endif /* APP_ARM_H */
