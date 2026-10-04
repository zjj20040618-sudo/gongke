#ifndef APP_BOARD_PINS_H
#define APP_BOARD_PINS_H

#include <stdint.h>

#define MOTOR_NUM         4
#define MOTOR_PWM_PERIOD  199u   /* TIM1 ARR：10 kHz 电机 PWM，duty 上限 = ARR */

#define BP_DIR_FWD  1
#define BP_DIR_REV -1

/* 底层唯一动 HAL 的地方。索引 m=0..3 是"轮号"，电机接线换序只改这里。 */

void     bp_init(void);                              /* 启动编码器/PWM、开 STBY、激光灭 */
void     bp_motor_set(int m, int dir, int duty);     /* dir=±1；duty 0..MOTOR_PWM_PERIOD */
void     bp_motor_stop(int m);                       /* 松刹车减速（duty=0） */
void     bp_motor_brake(int m);                      /* TB6612 两 IN 同高 = 短接刹车 */
int32_t  bp_enc_delta(int m);                        /* 距上次调用新增的编码器脉冲数(带符号) */
int32_t  bp_enc_raw_total(int m);                    /* 自上次清零起的硬件原始计数 */
void     bp_enc_raw_reset_all(void);
void     bp_laser_set(int on);                       /* 激光 高=触发 */
void     bp_debug_send(const char *s);               /* 蓝牙 USART3，中断驱动 TX 队列，不阻塞 RTOS 任务 */
uint32_t bp_debug_tx_dropped(void);                  /* TX 队列满导致的整条消息丢弃次数 */

#endif /* APP_BOARD_PINS_H */
