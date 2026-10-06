#ifndef APP_ARM_H
#define APP_ARM_H

#include <stdint.h>

/* 机械臂：爪子开合 = 舵机(TIM12 CH2, PB15, 50Hz)；两轴步进：
 * axis0 STEP/DIR=PA10/PA9，axis1 STEP/DIR=PA12/PA11（2026-10-06实板纠正；开漏、低有效）。
 * STEP 低脉宽由 Cortex-M4 DWT 周期计数器产生。 */

#define ARM_STEPPER_NUM 2

void arm_init(void);
void arm_claw_set_us(uint16_t us);            /* 爪子舵机脉宽 500..2500us */
uint16_t arm_claw_command_us(void);            /* 最近一次发出的命令脉宽，不是舵机位置反馈 */
void arm_claw_open(void);                     /* 骨架：发预置的开/闭脉宽 */
void arm_claw_close(void);
void arm_stepper_dir(int axis, int dir);      /* dir 0/1；先设方向再发脉冲 */
void arm_stepper_step(int axis);              /* 单发 1 步(约20us低电平脉宽，DWT计时) */

#endif /* APP_ARM_H */
