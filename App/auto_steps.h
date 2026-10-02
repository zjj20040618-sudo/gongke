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
 *   必须用当前障碍/速度实测间隙时长；不能仅凭一秒窗口保证不误判。
 *   姿态平稳不证明所有板/后轮已通过，卡在固定倾角也可能满足判据。
 *
 * 参数单位：vx=前进 mm/s（可负=后退）、vy=横移 mm/s（一般 0）、run_ms=兜底超时。
 * 返回：1=完整一秒姿态窗口满足判据；0=中止、超时、IMU失效或参数非法。
 * 里程/卡死细节与阈值实测项见 auto_steps.c 头注 + fw/实测值清单.md。
 */
int step_cross_obstacle(float vx_mms, float vy_mms, uint32_t run_ms);

/* Single owner: mission wrapper OR BT tick. begin does not drive; tick <=20ms.
 * Do not call the blocking wrapper from the Bluetooth service task. */
typedef enum {
    CROSS_IDLE = 0, CROSS_RUNNING, CROSS_DONE, CROSS_ABORT,
    CROSS_IMUERR, CROSS_TIMEOUT, CROSS_BAD_CONFIG
} CrossStatus;
typedef struct {
    CrossStatus status;
    uint32_t elapsed_ms;
    float pitch_deg, pp_deg, yaw_deg, yaw_error_deg;
    uint8_t rise_seen, window_full;
} CrossSnapshot;
int cross_begin(float vx_mms, float vy_mms, uint32_t run_ms);
CrossStatus cross_tick(void);
void cross_cancel(void);
CrossStatus cross_status(void);
void cross_get(CrossSnapshot *out);
const char *cross_status_name(CrossStatus status);
/* Stopped/RAM only; 0.01 <= flat < rise <=20 degrees. Seeds need real tests. */
int cross_tune_set(float rise_deg, float flat_deg);
void cross_tune_get(float *rise_deg, float *flat_deg);

#endif
