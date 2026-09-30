#include "motion.h"
#include "board_pins.h"
#include "control.h"
#include "imu.h"
#include "main.h"      /* HAL_GetTick */
#include <math.h>

#define M_PI_F 3.14159265f

/* 麦轮运动学参数：单位 mm / rad，台上实测后填，别猜 */
#define M_WHEEL_R_MM   33.0f   /* TODO 实测轮半径(不含压扁) */
#define M_GEAR_RATIO   1.0f    /* 恒 1.0：MG513 P30 的 30:1 已折进 control.c 的 CPR(1560=13线×4×30)，
                                * 那里算出的就是"轮 rpm"。⚠️ 别在这再填 30，会把里程再缩 30 倍 */
#define M_A_HALF_MM    50.0f   /* TODO 轮心到车中心的距离占位(纯转向项系数) */

/* 普通路线平移剖面，RAM-only；0 表示尚未标定。越障不走此剖面。 */
static MotionProfileTune s_profile = { 0.0f, 0.0f };

/* 运动层初始化:先全轮刹停(robot_init 调一次) */
void motion_init(void)
{
    ctrl_stop_all();
}

/* ---- 速度斜坡(梯形加减速,参考 TFT-Test Speed_Planner):升速按 acc、降速/刹车按 dec ---- */
void motion_ramp_init(MotionRamp *r, float acc, float dec)
{
    if (!r) return;
    r->cur = 0.0f;
    r->acc = acc;
    r->dec = (dec > 0.0f) ? dec : acc;   /* dec≤0 → 回落也用 acc */
}

/* 朝 target 逼近一步:单步最多走 lim·dt(升速=acc、降速=dec),到位即 clamp。dt 单位 s。 */
float motion_ramp_step(MotionRamp *r, float target, float dt_s)
{
    float err, lim, max_d;
    int reversing;
    if (!r) return 0.0f;
    /* 正负速度换向必须先按减速度回零；同号时看绝对值是在加速还是减速。 */
    reversing = (r->cur > 0.0f && target < 0.0f)
             || (r->cur < 0.0f && target > 0.0f);
    if (reversing) target = 0.0f;
    err = target - r->cur;
    if (err == 0.0f) return r->cur;
    lim = (reversing || fabsf(target) < fabsf(r->cur)) ? r->dec : r->acc;
    if (lim <= 0.0f) { r->cur = target; return r->cur; }
    max_d = lim * ((dt_s > 0.0f) ? dt_s : 0.001f);
    if (err > 0.0f) r->cur += (err < max_d) ? err : max_d;
    else            r->cur -= (-err < max_d) ? -err : max_d;
    return r->cur;
}

void motion_profile_get(MotionProfileTune *out)
{
    if (out) *out = s_profile;
}

int motion_profile_set(const MotionProfileTune *in)
{
    if (!in) return 0;
    if (in->acc_mms2 < 0.0f || in->acc_mms2 > 10000.0f) return 0;
    if (in->dec_mms2 < 0.0f || in->dec_mms2 > 10000.0f) return 0;
    s_profile = *in;
    return 1;
}

const char *motion_profile_config_missing(void)
{
    if (s_profile.acc_mms2 <= 0.0f) return "NAV_ACC_MMS2";
    if (s_profile.dec_mms2 <= 0.0f) return "NAV_DEC_MMS2";
    return 0;
}

void motion_linear_ramp_init(MotionRamp *r)
{
    motion_ramp_init(r, s_profile.acc_mms2, s_profile.dec_mms2);
}

/* 普通定距腿：起点由 ramp 限加速度，终点按 v²=2as 限制可停车速度。
 * acc/dec 未设置时保持恒速，供模式15..18先采原始响应；正式 mission 由闸门阻止。 */
float motion_linear_profile_step(MotionRamp *r, float cruise_mms,
                                 float remaining_mm, float dt_s)
{
    float mag, stop_mag, target;
    if (!r) return 0.0f;
    if (s_profile.acc_mms2 <= 0.0f || s_profile.dec_mms2 <= 0.0f)
        return cruise_mms;

    mag = fabsf(cruise_mms);
    stop_mag = sqrtf(2.0f * s_profile.dec_mms2 * fabsf(remaining_mm));
    if (stop_mag < mag) mag = stop_mag;
    target = (cruise_mms < 0.0f) ? -mag : mag;
    return motion_ramp_step(r, target, dt_s);
}

/* 体坐标 vx,vy(mm/s) + w(rad/s) → 每轮 rpm。
 * w 正向定义为车身顺时针：与 2026-09-24 手转 IMU yaw 正向一致。
 * 轮位按 2026-09-25 后轮单轮复核：m0左后、m1右后、m2右前、m3左前。
 * 后轮正向极性已在 board_pins.c 同步修正；横移仍须整车实测。 */
void motion_ik(float vx, float vy, float w, int16_t rpm[4])
{
    static const int8_t sgn[4][3] = {   /* [轮][vx, vy, w] 系数 */
        {  1, -1,  1 },   /* m0 = 左后 */
        {  1,  1, -1 },   /* m1 = 右后 */
        {  1, -1, -1 },   /* m2 = 右前 */
        {  1,  1,  1 },   /* m3 = 左前 */
    };                    /* 右侧 m1/m2 同号自转；左侧 m0/m3 反号；
                           * 横移对角 m0/m2 与 m1/m3 分别同号。 */
    for (int i = 0; i < MOTOR_NUM; i++) {
        float lin = sgn[i][0] * vx + sgn[i][1] * vy + sgn[i][2] * M_A_HALF_MM * w;
        float rps = lin / (M_WHEEL_R_MM * 2.0f * 3.14159f);      /* 轮转/s */
        rpm[i] = (int16_t)(rps * M_GEAR_RATIO * 60.0f);          /* 轴端 rpm */
    }
}

/* 体坐标速度(vx前/vy横/w转 mm·rad/s)→ IK 算四轮 rpm → 下发 control。
 * 普通路线的加减速在调用方按剩余距离生成 vx/vy；越障可继续直接给恒速。 */
void motion_vel_set(float vx, float vy, float w)
{
    int16_t rpm[4];
    motion_ik(vx, vy, w, rpm);
    for (int i = 0; i < MOTOR_NUM; i++) ctrl_set_speed(i, rpm[i]);
}

/* 刹停全轮(即 ctrl_stop_all):步骤收尾/超时/中止的统一停车出口 */
void motion_brake(void)
{
    ctrl_stop_all();
}

static Pose s_pose = { 0.0f, 0.0f, 0.0f };

/* 读里程位姿指针(x,y,th;里程未接前恒零点,日志/判段用) */
const Pose *motion_pose(void)
{
    return &s_pose;
}

/* 四轮平均累计里程 mm（带符号，前进为正）——给"走一段固定距离"用（step_straight）。
 * 换算：mm = 计数 × (2πR / CPR)，四轮取平均。
 * ⚠️ 只反映"轮子转了多少"：打滑/坎上时不准（所有轮式里程的通病）。
 * ⚠️ 精度依赖 CTRL_ENCODER_CPR(control.h) 与 M_WHEEL_R_MM 都实测准。
 * 用法：起点记 odo0，走中读 odo，差值就是走过的距离（自上次 ctrl_enc_reset_all 起算）。 */
float motion_odo_mm(void)
{
    float sum = 0.0f;
    for (int i = 0; i < MOTOR_NUM; i++) sum += (float)ctrl_enc_total(i);
    return sum * 0.25f * (2.0f * M_PI_F * M_WHEEL_R_MM) / (float)CTRL_ENCODER_CPR;
}

/* 与 motion_ik 的 vy 符号表一致；用于 QR 左右补扫的两端点计数。
 * 这是轮式里程而非绝对定位，打滑与轮位/CPR 未校准会造成端点误差。 */
float motion_lateral_odo_mm(void)
{
    float cnt = -(float)ctrl_enc_total(0) + (float)ctrl_enc_total(1)
              - (float)ctrl_enc_total(2) + (float)ctrl_enc_total(3);
    return cnt * 0.25f * (2.0f * M_PI_F * M_WHEEL_R_MM) / (float)CTRL_ENCODER_CPR;
}

/* 里程融合积分(真车在 1ms 环里调):拿四轮快 rpm + IMU 航向对 s_pose 积分。
 * 前向运动学与 motion_ik 同一符号表自洽:
 *   vx(体前向)=(u0+u1+u2+u3)/4, vy(横移)=(−u0+u1−u2+u3)/4,
 *   w=(u0−u1−u2+u3)/(4·A)（航向改用 IMU,此处 w 不用于积分）。
 * 体↔世界方向约定:th=0 朝 +y(前进)、th 正向朝 +x(右);u_i = 轮线速度 mm/s。
 * ⚠️ 世界 x/y 走向符号、vx 正负 与运动学符号表仍需整车上板校准。 */
#define POSE_DT_GUARD_S  0.05f   /* 积分步长保护:掉拍>50ms 不积分(防卡顿跳变) */
#define POSE_DT_FALLBACK 0.001f

static uint32_t s_last_ms;
static int      s_first = 1;

void motion_pose_update(void)
{
    uint32_t now = HAL_GetTick();
    if (s_first) { s_first = 0; s_last_ms = now; return; }
    float dt = (float)(int32_t)(now - s_last_ms) / 1000.0f;
    s_last_ms = now;
    if (dt <= 0.0f || dt > POSE_DT_GUARD_S) dt = POSE_DT_FALLBACK;

    if (!imu_ok()) return;                 /* 无航向不可信,不动 pose */

    float rpm[4];
    ctrl_get_rpm_fast_all(rpm);

    /* 轮线速度 mm/s:rpm(电机轴)÷减速比 → 轮转/s ×2πR */
    float u[4];
    for (int i = 0; i < MOTOR_NUM; i++)
        u[i] = rpm[i] / M_GEAR_RATIO / 60.0f * 2.0f * M_PI_F * M_WHEEL_R_MM;

    float vx = (u[0] + u[1] + u[2] + u[3]) * 0.25f;          /* 体前向 mm/s */
    float vy = (-u[0] + u[1] - u[2] + u[3]) * 0.25f;         /* 体横移 mm/s(与 IK 符号表同源) */
    (void)vx; (void)vy;                                       /* 符号待台上校准后放开 */

    float th = s_pose.th;                                      /* 用上一拍航向做本拍旋转 */

    /* TODO 校准后启用:s_pose.x += (vx*sinf(th) + vy*cosf(th))*dt;  体→世界 */
    /*       s_pose.y += (vx*cosf(th) - vy*sinf(th))*dt; */
    s_pose.th = imu_heading_deg() * M_PI_F / 180.0f;           /* 航向跟 IMU(连续不回绕) */
}
