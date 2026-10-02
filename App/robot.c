#include "robot.h"
#include "main.h"
#include "usart.h"
#include "board_pins.h"
#include "arm.h"
#include "proto.h"
#include "steps.h"
#include "mission.h"
#include "imu.h"
#include "control.h"
#include "motion.h"
#include "test.h"
#include <string.h>
#include <stdio.h>

static uint8_t s_rx2, s_rx3, s_rx4;   /* 2=视觉(USART2 PD5/6) 3=蓝牙(USART3 PD8/9) 4=IMU(UART4 PC10/11) */

/* BT 输入环形缓冲(huart3,ISR 写 / robot_bt_service 读) */
#define BT_RX_N 64u
#define FW_BUILD_ID "20261002-ROUTE-SAFETY"
static volatile uint8_t s_bt[BT_RX_N];
static volatile uint8_t s_bt_wr, s_bt_rd;
static volatile uint32_t s_bt_drop;
static volatile uint32_t s_uart_err[3];      /* 0=视觉USART2,1=蓝牙USART3,2=IMU UART4 */
static volatile uint32_t s_uart_last_err[3]; /* 最近一次 HAL_UART_ERROR_* 位图 */
static volatile uint32_t s_uart_arm_fail[3]; /* Receive_IT 首挂/重挂失败次数 */

static void uart_rx_arm(UART_HandleTypeDef *huart, uint8_t *rx, int ix)
{
    HAL_StatusTypeDef st;
    if (!huart || !rx || ix < 0 || ix >= 3) return;
    if (huart->RxState != HAL_UART_STATE_READY) return; /* 已挂接时不重复调用 */
    st = HAL_UART_Receive_IT(huart, rx, 1u);
    if (st != HAL_OK) s_uart_arm_fail[ix]++;
}

/* DefaultTask 周期兜底：首次挂接失败、异常回调重挂失败，都能在状态恢复 READY 后补挂。 */
static void uart_rx_ensure_all(void)
{
    uart_rx_arm(&huart2, &s_rx2, 0);
    uart_rx_arm(&huart3, &s_rx3, 1);
    uart_rx_arm(&huart4, &s_rx4, 2);
}

/* RxCplt 里被调:压 1 字节进 BT 环形缓冲(满则丢) */
static void bt_push(uint8_t c)
{
    uint8_t next = (uint8_t)(s_bt_wr + 1u) & (BT_RX_N - 1u);
    if (next != s_bt_rd) {
        s_bt[s_bt_wr] = c;
        s_bt_wr = next;
    } else s_bt_drop++;
}

/* proto 的发送口 = MaixCam 视觉所在 USART2(PD5/PD6)：轮询发字符串
 * (2026-09-12 改：视觉由 USART3 挪到 USART2；蓝牙由 USART1 挪到 USART3) */
static void uart2_tx(const char *s)
{
    if (!s) return;
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 20u);
}
static void uart2_binary_tx(const uint8_t *data, uint16_t length)
{
    if (data && length) HAL_UART_Transmit(&huart2, (uint8_t *)data, length, 20u);
}

/* 开机一次性初始化:底层→控制→运动→臂→IMU→协议→任务→调试,再挂三个串口 RX */
void robot_init(void)
{
    bp_init();
    ctrl_init();
    motion_init();
    arm_init();
    imu_init();

    proto_init();
    proto_set_binary_mode(1); /* camera AA55/CRC16; Bluetooth remains ASCII */
    proto_set_tx(uart2_tx);
    proto_set_binary_tx(uart2_binary_tx);
    proto_set_on_frame(steps_feed_frame);   /* MaixCam 帧 → steps 暂存,wait_* 消费 */
    mission_init();
    test_init();
    proto_send_scene(SCENE_IDLE); /* no camera reports accepted before a requested stage */

    s_bt_wr = s_bt_rd = 0;
    s_bt_drop = 0u;
    for (int i = 0; i < 3; ++i) {
        s_uart_err[i] = 0u; s_uart_last_err[i] = 0u; s_uart_arm_fail[i] = 0u;
    }
    uart_rx_ensure_all();                           /* 三路首次挂接；失败由 DefaultTask 重试 */
    bp_debug_send("\r\nREADY FW=" FW_BUILD_ID " SEND ? OR diag\r\n");
}

/* 三个 FreeRTOS 线程各自的入口:1ms 控制环 / 整场脚本 / IMU 解析(周期见 freertos.c) */
void robot_control_tick_1ms(void)
{
    ctrl_tick_1ms();
    motion_pose_update(); /* 航向/里程状态必须随控制周期更新，日志与后续定位才不是陈旧值 */
}
void robot_mission_main(void)      { mission_main(); }
void robot_imu_tick(void)          { imu_tick_parse(); }

/* LogTask:~100ms 打一帧状态摘要到调试口(BOOT 静默不刷屏) */
void robot_log_tick(void)
{
    if (mission_state() == MS_BOOT) return;   /* bench:静默,别刷屏盖掉测试应答(VOFA 曲线也要独享调试口) */
    int32_t d[3];
    mission_get_qr(d);
    char buf[112];
    int n = snprintf(buf, sizeof buf,
                     "\r\nS:%s OdoF:%d OdoL:%d H:%d Yaw:%d D:%d,%d,%d\n",
                     mission_state_str(mission_state()),
                     (int)motion_odo_mm(), (int)motion_lateral_odo_mm(),
                     (int)imu_heading_deg(),
                     (int)imu_yaw_deg(), (int)d[0], (int)d[1], (int)d[2]);
    if (n > 0) bp_debug_send(buf);
}

/* 蓝牙命令服务:把 BT 环里的字节喂给 test 行解析器并周期推进(BT 遥控/调试) */
void robot_bt_service(void)
{
    /* BT 文本命令 → test 行解析器(命令集见 fw/bluetooth-test.md)。
     * g/a 也走这统一成行;单键无 CR 靠空闲成行(~20ms 轮询粒度)触发,
     * 有 CR 立即成行。安全停最坏延时 ≤ ~1 个轮询周期 + 行尾(≈20~40ms)。 */
    while (s_bt_rd != s_bt_wr) {
        uint8_t c = s_bt[s_bt_rd];
        s_bt_rd = (uint8_t)(s_bt_rd + 1u) & (BT_RX_N - 1u);
        test_feed(c);
    }
    uart_rx_ensure_all();
    test_poll();
    proto_service(); /* sole binary TX owner; never blocks inside RX callback */
}

/* UART RX 完成回调：按句柄分发（唯一强定义，CubeMX 没生成过） */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {                                   /* 视觉 USART2 → proto */
        proto_feed_byte(s_rx2);
        uart_rx_arm(&huart2, &s_rx2, 0);
    } else if (huart == &huart4) {                            /* IMU UART4 → imu */
        imu_feed(s_rx4);
        uart_rx_arm(&huart4, &s_rx4, 2);
    } else if (huart == &huart3) {                            /* 蓝牙 USART3 → 命令 */
        bt_push(s_rx3);
        uart_rx_arm(&huart3, &s_rx3, 1);
    }
}

/* ORE 会终止 HAL 的中断接收；其余 FE/NE/PE 也统一记账并尝试补挂。
 * 若非阻塞错误的接收仍在进行，Receive_IT 返回 BUSY，原接收保持有效。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint8_t *rx = 0;
    int ix = -1;
    uint32_t err = HAL_UART_GetError(huart);

    if      (huart == &huart2) { ix = 0; rx = &s_rx2; }
    else if (huart == &huart3) { ix = 1; rx = &s_rx3; }
    else if (huart == &huart4) { ix = 2; rx = &s_rx4; }
    if (ix < 0) return;

    s_uart_err[ix]++;
    s_uart_last_err[ix] = err;
    if (err & HAL_UART_ERROR_ORE) __HAL_UART_CLEAR_OREFLAG(huart);
    uart_rx_arm(huart, rx, ix);
}

void robot_diag_report(void)
{
    CtrlTune t;
    MotionProfileTune p;
    ProtoStats ps;
    uint32_t sw[5];
    uint32_t age = imu_last_valid_age_ms();
    static char b[160]; /* 诊断长行放静态区，避免占用任务栈 */

    ctrl_tune_get(&t);
    motion_profile_get(&p);
    proto_stats_get(&ps);
    robot_get_stack_watermarks(sw);
    snprintf(b, sizeof b, "\r\nDIAG FW=%s IMU=%s age=%lums\r\n",
             FW_BUILD_ID, imu_ok() ? "OK" : "BAD",
             (unsigned long)(age == UINT32_MAX ? 0xFFFFFFFFu : age));
    bp_debug_send(b);
    snprintf(b, sizeof b,
             "UART err V=%lu/B=%lu/I=%lu last=0x%lX/0x%lX/0x%lX BTdrop=%lu TXdrop=%lu\r\n",
             (unsigned long)s_uart_err[0], (unsigned long)s_uart_err[1],
             (unsigned long)s_uart_err[2], (unsigned long)s_uart_last_err[0],
             (unsigned long)s_uart_last_err[1], (unsigned long)s_uart_last_err[2],
             (unsigned long)s_bt_drop, (unsigned long)bp_debug_tx_dropped());
    bp_debug_send(b);
    snprintf(b, sizeof b, "UART RXarm fail V=%lu/B=%lu/I=%lu\r\n",
             (unsigned long)s_uart_arm_fail[0], (unsigned long)s_uart_arm_fail[1],
             (unsigned long)s_uart_arm_fail[2]);
    bp_debug_send(b);
    snprintf(b, sizeof b,
             "VISION line=%lu ok=%lu qr=%lu obj=%lu reject=%lu over=%lu QRready=%d bad3=%lu\r\n",
             (unsigned long)ps.lines, (unsigned long)ps.accepted,
             (unsigned long)ps.qr, (unsigned long)ps.obj,
             (unsigned long)ps.rejected, (unsigned long)ps.overflow,
             mission_qr_ready(), (unsigned long)mission_qr_invalid_count());
    bp_debug_send(b);
    snprintf(b, sizeof b, "VISION wire=BINARY crc_bad=%lu bad=%lu gap=%lu unmapped=%lu duplicate=%lu\r\n",
             (unsigned long)ps.crc_bad, (unsigned long)ps.binary_bad,
             (unsigned long)ps.binary_gap, (unsigned long)ps.binary_unmapped,
             (unsigned long)ps.duplicate);
    bp_debug_send(b);
    snprintf(b, sizeof b, "STACK word D=%lu C=%lu M=%lu I=%lu L=%lu\r\n",
             (unsigned long)sw[0], (unsigned long)sw[1], (unsigned long)sw[2],
             (unsigned long)sw[3], (unsigned long)sw[4]);
    bp_debug_send(b);
    snprintf(b, sizeof b,
             "PARAM kp=%.4f ki=%.5f lp=%.3f dead=%u ykp=%.3f okp=%.3f acc=%.1f dec=%.1f RAM-only\r\n",
             t.kp, t.ki, t.lp_alpha, (unsigned)t.dead_min, step_heading_kp_deg(),
             step_orth_kp(), p.acc_mms2, p.dec_mms2);
    bp_debug_send(b);
}
