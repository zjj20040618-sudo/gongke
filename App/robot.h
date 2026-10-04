#ifndef APP_ROBOT_H
#define APP_ROBOT_H

#include <stdint.h>

/* 胶水层：让 CubeMX 生成的 main.c / freertos.c 只认识 robot_* 接口，
 * 具体分层(proto/imu/ctrl/…)都藏在 App 里。 */

void robot_init(void);              /* 全部 init + 挂三个串口 RX；osKernelStart 前调一次 */
void robot_control_tick_1ms(void);  /* ControlTask：1ms 速度环 */
void robot_mission_main(void);      /* MissionTask：阻塞式整场脚本(见 mission_main) */
void robot_imu_tick(void);          /* ImuTask：周期解析 IMU */
void robot_log_tick(void);          /* LogTask：~100ms 打一帧状态摘要 */
void robot_bt_service(void);        /* StartDefaultTask：蓝牙遥控/命令解析 */
void robot_diag_report(void);       /* 蓝牙输出固件/UART/栈/调参只读诊断 */
void robot_get_stack_watermarks(uint32_t out[5]); /* Default/Control/Mission/IMU/Log，单位 word */

#endif /* APP_ROBOT_H */
