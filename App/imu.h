#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdint.h>

/* 汇电籽-601 IMU @ UART4(PC10/11, 115200 8N1, 100Hz 主动上报)。
 * 协议已核(2026-09-06 官方手册)：帧 AA55 0x60 CMD LEN DATA CS；角度×100；
 * Yaw=uint16 0~360(0/360 回绕)、Pitch/Roll=int16 ±180。
 * 解析按长度帧同时吃两档(别写死排位)：模式1 仅姿态 LEN=6 DATA=[Yaw,Pitch,Roll] **Yaw 在前**；
 * 模式0 全数据 LEN=18 姿态在尾 [..Pitch,Roll,Yaw]。TX 切仅姿态未接(解析不依赖它)。
 * 全协议+指令速查：gongkesai/memory/imu-601-protocol.md。实现见 imu.c。 */

void    imu_init(void);
void    imu_feed(uint8_t ch);        /* UART4 RX ISR 每字节喂一次(字节式状态机) */
void    imu_tick_parse(void);        /* ImuTask 周期调：链路静默超时清标记/复位解析态 */
float   imu_yaw_deg(void);           /* 航向 0..360(0/360 回绕) */
float   imu_heading_deg(void);       /* 连续航向(度,跨绕已解,不跳 359→0;供里程/保向积分) */
uint8_t imu_zero_leg_heading(void);  /* 停稳后把当前航向记为本段零点；不改全局连续航向 */
float   imu_leg_heading_deg(void);   /* 相对最近一次本段清零的航向，单位度 */
float   imu_pitch_deg(void);         /* 俯仰 ±180 */
float   imu_roll_deg(void);          /* 横滚 ±180 */
uint8_t imu_ok(void);                /* 1 = 最近解析出有效姿态(链路活+帧校验过) */
uint32_t imu_last_valid_age_ms(void);/* 距最近有效姿态帧 ms；从未收到返回 UINT32_MAX */

#endif /* APP_IMU_H */
