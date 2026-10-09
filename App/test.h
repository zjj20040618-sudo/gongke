#ifndef APP_TEST_H
#define APP_TEST_H

#include <stdint.h>
#include "proto.h"

/* 台上测试执行器(BT 文本命令)。协议/命令集 = fw/bluetooth-test.md(定稿)。
 * 互斥口径:运动类 bench 命令只在 MS_BOOT(没按 'g')响应;一开跑 bench 锁死,
 * 只剩急停 a。命令逐字节喂入 + 周期 poll 成行(两段式),行 = \r\n 或
 * ~20ms 无新字节。USART3 RX/TX 字节搬运由中断完成，不等待 RTOS 调度；
 * 命令解析与每秒计数格式化仍在 StartDefaultTask(robot_bt_service, 20ms 周期)，
 * 底盘运动由 ControlTask(1ms)执行；24..27、31升降、42抓取的有限步进脉冲
 * 由专用TIM7中断执行。42不参与31/视觉：伸缩->下降->舵机->逆序复位；
 * 任意阶段g/a/0只取消，完整复位后g可重复，取消后须手动恢复起点并重新选号。 */

void test_init(void);           /* 清行缓冲/复位测试态(robot_init 调) */
void test_feed(uint8_t c);      /* 每字节喂入(BT RX 轮询里调,g/a 也走这,统一成行) */
void test_poll(void);           /* 每周期调:收不完整的行按"空闲成行"关线 + 执行命令 */
void test_stepper_timer_irq(void); /* TIM7 only: one shared finite job, at most one STEP per update, no UART/RTOS calls. */
/*31 owns the16-stage QR/ball/bucket/target/hostage trial;43 reuses that
 * recipe with stopped-step g gates (its current专项 is deferred).
 * Legacy34 retains its15-node road/bucket/manual-new-d tail;36/37 are
 * isolated motion-only routes. g/a/0 cancel without auto-return. */
void test_vision_feed_frame(const ProtoFrame *f); /* RX ISR only caches current-phase33/35,31 target and VAT38..41/31 task frames; no TX or motion. */
float test_forward_ff_ratio(void); /* Mode32 RAM fff; bench/route candidates are isolated. */

#endif /* APP_TEST_H */
