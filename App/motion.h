/*
 * 初学者导读：车身运动接口。车身速度、四轮转速、加减速参数和里程信息属于不同量，先核对单位。
 * .h 相当于接口清单，供 #include 引入；函数实现通常在同名.c中。
 * #ifndef / #define / #endif 是头文件保护，防止同一编译单元重复包含定义。
 */

#ifndef APP_MOTION_H
#define APP_MOTION_H

#include <stdint.h>

/* 运动层：麦轮 IK、里程 pose、定点/到点。把"想怎么走"翻译成四轮 rpm 交给 control。 */

typedef struct {
    float x;    /* mm，全局 X */
    float y;    /* mm，全局 Y */
    float th;   /* rad；按motion.c约定，0朝全局+y，正向朝+x；当前由连续IMU航向更新 */
} Pose;

/* 普通平移速度斜坡：调用方按固定周期推进，并结合剩余距离限制停车速度。
 * 越障保持独立匀速，不走普通路线剖面。 */
typedef struct {
    float cur;      /* 当前输出速度 */
    float acc;      /* 加速度(升速,每次最多走 acc·dt) */
    float dec;      /* 减速度(降速/刹车;≤0 则回落用 acc) */
} MotionRamp;

typedef struct {
    float acc_mms2;   /* 普通平移起步加速度 */
    float dec_mms2;   /* 普通平移终点减速度 */
} MotionProfileTune;

void  motion_ramp_init(MotionRamp *r, float acc, float dec);           /* 归零 + 定 acc/dec */
float motion_ramp_step(MotionRamp *r, float target, float dt_s);       /* 向 target 逼近一步,回当前速度 */
void  motion_profile_get(MotionProfileTune *out);
int   motion_profile_set(const MotionProfileTune *in); /* 0=暂禁用；范围0..10000，RAM-only */
const char *motion_profile_config_missing(void);
void  motion_linear_ramp_init(MotionRamp *r);
float motion_linear_profile_step(MotionRamp *r, float cruise_mms,
                                 float remaining_mm, float dt_s);

void     motion_init(void);
void     motion_ik(float vx, float vy, float w, int16_t rpm[4]); /* 体坐标速度 → 四轮 rpm */
void     motion_vel_set(float vx, float vy, float w);             /* 直接按体坐标速度走 */
void     motion_ik_precise(float vx, float vy, float w, float rpm[4]); /* 独立小数 IK；旧 IK 保持整数截断 */
void     motion_vel_set_precise(float vx, float vy, float w);    /* 仅显式选择的新调用者保留小数轮速 */
void     motion_brake(void);
void     motion_pose_update(void);          /* 1ms 更新 IMU 航向；x/y 积分待轮位/符号台校后启用 */
const Pose *motion_pose(void);
float motion_odo_mm(void);   /* 四轮平均累计里程 mm(带符号,前进为正);打滑时不准 */
float motion_lateral_odo_mm(void); /* 四轮编码器投影出的累计横移，右为正；待轮位/CPR台校 */
#endif /* APP_MOTION_H */
