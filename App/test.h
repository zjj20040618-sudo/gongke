#ifndef APP_TEST_H
#define APP_TEST_H

#include <stdint.h>
#include "proto.h"

/* 台上测试执行器(BT 文本命令)。协议/命令集 = fw/bluetooth-test.md(定稿)。
 * 互斥口径:运动类 bench 命令只在 MS_BOOT(没按 'g')响应;一开跑 bench 锁死,
 * 只剩急停 a。命令逐字节喂入 + 周期 poll 成行(两段式),行 = \r\n 或
 * ~20ms 无新字节。USART3 RX/TX 字节搬运由中断完成，不等待 RTOS 调度；
 * 命令解析与每秒计数格式化仍在 StartDefaultTask(robot_bt_service, 20ms 周期)，
 * 运动本身由 ControlTask(1ms)执行。 */

void test_init(void);           /* 清行缓冲/复位测试态(robot_init 调) */
void test_feed(uint8_t c);      /* 每字节喂入(BT RX 轮询里调,g/a 也走这,统一成行) */
void test_poll(void);           /* 每周期调:收不完整的行按"空闲成行"关线 + 执行命令 */
/* Mode31/34 share the complete 12-leg recipe/local tuning. Only31 gates R2 on
 * QR;34 closes RX locally and needs no camera. g/a/0 cancel with no return. */
void test_vision_feed_frame(const ProtoFrame *f); /* RX ISR: mode33 cache only; startup QR cache is owned by proto. */
float test_forward_ff_ratio(void); /* Current RAM fff; +left, no mutation or save. */

#endif /* APP_TEST_H */
