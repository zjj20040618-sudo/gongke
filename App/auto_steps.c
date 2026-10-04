/*
 * 初学者导读：越障步骤，用车身俯仰的变化辅助判断上坎和离坎。
 * 滑动窗是最近一段时间的样本数组；写满后回到下标0，覆盖最老的样本。
 * 峰峰值 = 窗口最大 pitch 减最小 pitch，不是平均值，也不是当前角度。
 * rise 先记录是否观察到上坎变化，再允许“变化变小”作为结束判据。
 * 阈值是待验证种子；传感器失效、中止或总时长用尽都返回失败并刹车。
 */

/* auto_steps.c -- 提前写好、不依赖实测数的步骤(2026-09-09 起累积)。
 * 头文件 auto_steps.h；加入工程编译即可调用。现有 steps.c 保持不动。
 * 原则：现在能写的都先写；只留"要实测才填"的常量为宏/入参，台上改一个数。
 */
#include "auto_steps.h"
#include "main.h"       /* HAL_GetTick */
#include "cmsis_os.h"   /* osDelay */
#include "motion.h"     /* motion_vel_set / motion_brake */
#include "imu.h"        /* imu_yaw_deg / imu_pitch_deg / imu_ok */
#include "steps.h"      /* run_aborted */

/* ==================== 越障(2026-09-12 重写) ====================
 * 障碍：3 条 300×60×10 雪弗板、间隔 50（沿行进方向：板60 / 隙50 / 板60 / 隙50 / 板60）。
 * 车头碰到第一条板 → 车尾离开第三条板，总行程 ≈ 280mm + 轴距(≈200mm) ≈ **480mm**。
 * ⚠️ 旧版"定速走 1500ms"(=225mm) 只够一半 → 会被卡在坎中间。
 *
 * 结束判据（2026-09-12 用户方案 + 查证后的完善）：**只看姿态的变化量**，不数次数、不用零点。
 *   阶段A 确认上坎：滑动窗内 pitch 峰峰值 > X_RISE_DEG
 *   阶段B 判定过完：滑动窗内 pitch 峰峰值 < X_FLAT_DEG  → 停
 *   兜底：总时长 > run_ms → 停(失败)
 *
 * 为什么用"窗口变化量"而不是"回到水平0"（三重好处）：
 *   ① 不用标定零点（不用知道"平地 pitch=几"）
 *   ② 不受陀螺零漂影响（就算慢漂，短窗内变化量仍小）
 *   ③ 不受动态误差影响（过坎时加速度计混入运动加速度→绝对值会偏，但"变不动了"仍成立）
 * 为什么不会在"间隙里"误判（间隙期 pitch 也会短暂回平）：
 *   间隙里那次"平"只持续 50mm/车速 ≈ 0.33s，而**窗口 1s > 0.33s**
 *   → 窗口里必然还含着上坎/下坎的变化 → 峰峰值不会小 ✓
 * 为什么会有 2.9° 这个量级：坎高 10mm、轴距 200mm → atan(10/200) ≈ 2.9°。
 *   （参考：文献里坡道检测用 |pitch|>10° 确认、<5° 判完；我们坎矮，按比例缩小。）
 * 不依赖里程 = 不怕坎面打滑（旧方案用里程判结束，会被空转多记骗到）。
 */
#define X_CTRL_MS      5u      /* 控制节拍,与 steps 层一致 */
#define X_SAMPLE_MS    10u     /* 姿态采样节拍(100Hz,够用又省) */
#define X_WIN_N        100u    /* 窗口样本数 = 100 × 10ms = 1s */
#define X_RISE_DEG     2.0f    /* TODO 实测:上坎时窗口峰峰值应 > 这个(理论上限 ~2.9°) */
#define X_FLAT_DEG     0.5f    /* TODO 实测:过完后窗口峰峰值应 < 这个 */

/* 姿态滑动窗(环形缓冲),测"窗口内峰峰值"用 */
static float    s_pwin[X_WIN_N];
static uint8_t  s_pi;        /* 写指针 */
static uint8_t  s_pfill;     /* 1 = 窗口已填满 */
static uint32_t s_smp_t0;    /* 上次采样时刻 */

/* 窗口内 pitch 峰峰值(窗口未填满时只算已填的部分;全空返回 0) */
static float pitch_pp(void)
{
    uint8_t n = s_pfill ? X_WIN_N : s_pi;
    if (n == 0u) return 0.0f;
    float mn = s_pwin[0], mx = s_pwin[0];
    for (uint8_t i = 1u; i < n; i++) {
        if (s_pwin[i] < mn) mn = s_pwin[i];
        if (s_pwin[i] > mx) mx = s_pwin[i];
    }
    return mx - mn;
}

/* 角度差归一到 [-180,180) */
static float wrap180(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/* 越障：定速直冲 + yaw 锁向 + 姿态判据结束。
 * 参数：vx 前进 mm/s(可负=后退)、vy 横移 mm/s(一般0)、run_ms = 兜底超时(防卡死)。
 * 返回：1=判定过完  0=被中止、IMU失效或物理安全超时。
 * ⚠️ 坎上**别停别降速**（坎面本身滑，靠动量冲），所以全程匀速，只在末端判到"过完"才刹。 */
/**
 * @brief 定速保持航向越障，按俯仰窗口变化结束并刹车。
 * @param vx_mms 前后速度mm/s。
 * @param vy_mms 横移速度mm/s，通常为0。
 * @param run_ms 最长允许执行时间ms。
 * @retval 1=姿态算法判定通过，0=中止、超时或IMU失效。
 */
int step_cross_obstacle(float vx_mms, float vy_mms, uint32_t run_ms)
{
    float heading0;
    if (!imu_ok()) { motion_brake(); return 0; }
    heading0 = imu_yaw_deg();                 /* 进坎记录目标朝向 */

    s_pi = 0u; s_pfill = 0u; s_smp_t0 = 0u;   /* 清窗 */
    int rise = 0;                             /* 阶段标志:0=还没确认上坎 */
    uint32_t t0 = HAL_GetTick();

    while (1) {
        uint32_t now = HAL_GetTick();
        /* 急停/防跑飞必须先于本拍速度命令；否则已中止仍会多发一次驱动。 */
        if (run_aborted()) { motion_brake(); return 0; }
        if ((uint32_t)(now - t0) >= run_ms) { motion_brake(); return 0; }
        if (!imu_ok()) { motion_brake(); return 0; }

        /* 每拍带 yaw 锁向开走(定速) */
        float w = 0.0f;
        float e = wrap180(heading0 - imu_yaw_deg());
        w = step_heading_kp_deg() * e * 0.0174533f; /* 共用 RAM ykp；deg→rad */
        motion_vel_set(vx_mms, vy_mms, w);

        /* 姿态采样(每 X_SAMPLE_MS 存一个 pitch) */
        if (s_smp_t0 == 0u || (uint32_t)(now - s_smp_t0) >= X_SAMPLE_MS) {
            s_smp_t0 = now;
            s_pwin[s_pi] = imu_pitch_deg();
            s_pi = (uint8_t)((s_pi + 1u) % X_WIN_N);
            if (s_pi == 0u) s_pfill = 1u;
        }

        float pp = pitch_pp();

        if (!rise) {
            if (pp > X_RISE_DEG) rise = 1;            /* 阶段A:确认已上坎 */
        } else if (pp < X_FLAT_DEG) {                 /* 阶段B:窗口内 1s 都平 → 过完 */
            motion_brake();
            return 1;
        }

        osDelay(X_CTRL_MS);
    }
}
