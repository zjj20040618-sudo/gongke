/*
 * 初学者导读：越障接口。返回1仅表示当前姿态算法判定通过，实物通过效果仍须验证。
 * .h 相当于接口清单，供 #include 引入；函数实现通常在同名.c中。
 * #ifndef / #define / #endif 是头文件保护，防止同一编译单元重复包含定义。
 */

#ifndef AUTO_STEPS_H
#define AUTO_STEPS_H

#include <stdint.h>

/* 越障步（2026-09-12 重写）：定速匀速直冲 + yaw 锁向 + **姿态判据结束**。
 *
 * 结束判据 = 只看姿态的**变化量**（不数次数、不用零点、不用里程）：
 *   阶段A 确认上坎：滑动窗(1s)内 pitch 峰峰值 > 2°（种子）
 *   阶段B 判定过完：滑动窗(1s)内 pitch 峰峰值 < 0.5°（种子）→ 停
 *   兜底：总时长超过 run_ms → 停（返回 0，防卡死 / IMU 没接）
 * 选择理由：不依赖里程 → 不怕坎面打滑；用变化量 → 绕开零点标定/陀螺零漂/动态误差。
 *   窗口 1s > 间隙期"平段"最长 0.33s → 不会在间隙里误判过完。
 *
 * 参数单位：vx=前进 mm/s（可负=后退）、vy=横移 mm/s（一般 0）、run_ms=兜底超时。
 * 返回：1=判定过完；0=被外部中止(run_abort) 或 超时。
 * 里程/卡死细节与阈值实测项见 auto_steps.c 头注 + fw/实测值清单.md。
 */
int step_cross_obstacle(float vx_mms, float vy_mms, uint32_t run_ms);

#endif
