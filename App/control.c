#include "control.h"
#include "board_pins.h"
#include <math.h>

#define CTRL_RPM_EST_PERIOD  10u      /* 转速估计窗口：每 10ms 结算一次(日志/整定用) */
/* CTRL_ENCODER_CPR 已挪到 control.h（motion.c 做里程换算也要用） */
#define CTRL_KD              0.0f     /* 先0,振荡再加 */
#define CTRL_INTEG_LIM       20000.0f /* 积分限幅,防 windup */

/* 上电种子值；蓝牙可在 RAM 内临时修改，断电恢复这些默认值。 */
static float    s_kp = 0.05f;       /* rpm 误差→duty */
static float    s_ki = 0.004f;      /* 消静差，太大易振荡 */
static float    s_lp_alpha = 0.2f;  /* 0~1，小=更平滑 */
static uint16_t s_dead_min = 25u;   /* 静摩擦最小 duty */

static float    s_target[MOTOR_NUM]; /* 旧入口保存同值整数；precise 入口保留小数 */
static int32_t  s_acc[MOTOR_NUM];
static uint32_t s_n[MOTOR_NUM];
static int16_t  s_rpm_est[MOTOR_NUM];
volatile int32_t g_enc_total[MOTOR_NUM];  /* 每轮累计编码器计数(带符号)。⚠️ 去 static 是故意的:
                                           * 供 Keil 调试 Watch(static 的函数外看不到)。ctrl_enc_reset_all 清零 */
static uint32_t s_tick;

/* 开环直通(duty)模式:smoke 验每路驱动/方向,不依赖编码器/CPR/PID */
static uint8_t  s_open[MOTOR_NUM];
static int16_t  s_raw_duty[MOTOR_NUM];
static uint8_t  s_coast[MOTOR_NUM];
static uint8_t  s_creep[MOTOR_NUM]; /* Opt-in visual fine translation, never the default control path. */

/* 速度环 1ms 状态(每轮) */
static float s_rpm_lp[MOTOR_NUM];   /* 低通后的快速 rpm(喂 PID,不喂 10ms 慢估) */
static float s_e_prev[MOTOR_NUM];
static float s_ei[MOTOR_NUM];

void ctrl_init(void)
{
    for (int m = 0; m < MOTOR_NUM; m++) {
        s_target[m] = 0;
        s_acc[m] = 0;   s_n[m] = 0;
        s_rpm_est[m] = 0;
        g_enc_total[m] = 0;
        s_open[m] = 0;
        s_raw_duty[m] = 0;
        s_coast[m] = 0;
        s_creep[m] = 0;
        s_rpm_lp[m] = 0.0f;
        s_e_prev[m] = 0.0f;
        s_ei[m] = 0.0f;
    }
    s_tick = 0;
    ctrl_stop_all();
}

void ctrl_set_speed(int m, int16_t rpm)
{
    ctrl_set_speed_precise(m, (float)rpm);
}

void ctrl_set_speed_precise(int m, float rpm)
{
    if (m < 0 || m >= MOTOR_NUM) return;
    s_target[m] = rpm;
    s_open[m] = 0;                 /* 命令转速 → 回到闭环 */
    s_coast[m] = 0;
    s_creep[m] = 0;
}

void ctrl_set_speed_creep(int m, float rpm)
{
    if (m < 0 || m >= MOTOR_NUM) return;
    if (!isfinite(rpm)) { ctrl_stop_all(); return; }
    if (!s_creep[m] || rpm == 0.0f ||
        (rpm > 0.0f && s_target[m] <= 0.0f) ||
        (rpm < 0.0f && s_target[m] >= 0.0f)) {
        s_ei[m] = 0.0f; s_e_prev[m] = 0.0f;
    }
    s_target[m] = rpm;
    s_open[m] = s_coast[m] = 0u;
    s_creep[m] = 1u;
}

/* 开环直通 duty(带符号,±199 内;验单路驱动/方向用,不走 PID/不依赖编码器) */
void ctrl_set_duty_open(int m, int16_t duty)
{
    if (m < 0 || m >= MOTOR_NUM) return;
    s_raw_duty[m] = duty;
    s_open[m] = 1;
    s_coast[m] = 0;
    s_creep[m] = 0;
}

void ctrl_stop_all(void)
{
    for (int m = 0; m < MOTOR_NUM; m++) {
        s_target[m] = 0;
        s_open[m] = 0;        /* ⚠️ 开环直通也要收:不清 s_open 的话刹车后 1ms 环
                               * 仍按 s_raw_duty 给 PWM,轮子停不下来 */
        s_raw_duty[m] = 0;
        s_coast[m] = 0;
        s_creep[m] = 0;
        s_ei[m] = 0.0f;
        s_e_prev[m] = 0.0f;
        bp_motor_brake(m);
    }
}

/* 手转编码器时不能用短接刹车；保持控制环读计数，但 PWM 自由滑行。
 * 与 stop 一样清 PI 历史，防止退出手转模式后带旧积分起步。 */
void ctrl_coast_all(void)
{
    for (int m = 0; m < MOTOR_NUM; m++) {
        s_target[m] = 0;
        s_open[m] = 0;
        s_raw_duty[m] = 0;
        s_coast[m] = 1;
        s_creep[m] = 0;
        s_ei[m] = 0.0f;
        s_e_prev[m] = 0.0f;
        bp_motor_stop(m);
    }
}

/* 1ms 每轮:目标 rpm vs 快速实际 rpm → 位置式 PID → 方向+duty。
 * feedback 用低通快速 rpm(不是 10ms 慢估),慢估只留日志/报告。 */
static void ctrl_run_wheel(int m)
{
    float t = s_target[m];

    if (s_coast[m]) { bp_motor_stop(m); return; }

    if (s_open[m]) {                                  /* 开环直通:验驱动/方向 */
        if (s_raw_duty[m] == 0) { bp_motor_brake(m); return; }
        int duty = s_raw_duty[m];
        int dir  = (duty >= 0) ? BP_DIR_FWD : BP_DIR_REV;
        if (duty < 0) duty = -duty;
        if (duty > (int)MOTOR_PWM_PERIOD) duty = (int)MOTOR_PWM_PERIOD;
        bp_motor_set(m, dir, duty);
        return;
    }

    if (t == 0) { bp_motor_brake(m); return; }

    float e = (float)t - s_rpm_lp[m];
    s_ei[m] += e;
    if (s_ei[m] >  CTRL_INTEG_LIM) s_ei[m] =  CTRL_INTEG_LIM;
    if (s_ei[m] < -CTRL_INTEG_LIM) s_ei[m] = -CTRL_INTEG_LIM;
    float dout = s_kp * e + s_ki * s_ei[m]
               + CTRL_KD * (e - s_e_prev[m]);
    s_e_prev[m] = e;

    if (s_creep[m]) {
        /* A positive target does not authorize a negative torque pulse when
         * a sparse encoder sample temporarily reports overspeed (and vice
         * versa). Reduce drive to zero, preserving explicit motor direction;
         * do not carry a wrong-sign integral into the next low-speed start. */
        if ((t > 0.0f && dout <= 0.0f) || (t < 0.0f && dout >= 0.0f)) {
            s_ei[m] = 0.0f;
            bp_motor_set(m, t > 0.0f ? BP_DIR_FWD : BP_DIR_REV, 0);
            return;
        }
        float mag = fabsf(dout);
        if (mag > 0.0f && mag < (float)s_dead_min) mag = (float)s_dead_min;
        if (mag > (float)MOTOR_PWM_PERIOD) mag = (float)MOTOR_PWM_PERIOD;
        bp_motor_set(m, t > 0.0f ? BP_DIR_FWD : BP_DIR_REV, (int)mag);
        return;
    }

    int duty = (int)dout;
    int dir  = (duty >= 0) ? BP_DIR_FWD : BP_DIR_REV;
    if (duty < 0) duty = -duty;
    if (duty > (int)MOTOR_PWM_PERIOD) duty = (int)MOTOR_PWM_PERIOD;
    if (duty > 0 && duty < (int)s_dead_min) duty = (int)s_dead_min; /* 死区补偿 */
    bp_motor_set(m, dir, duty);
}

void ctrl_tick_1ms(void)
{
    for (int m = 0; m < MOTOR_NUM; m++) {
        int d = bp_enc_delta(m);
        s_acc[m] += d;   s_n[m]++;
        g_enc_total[m] += d;

        /* 快速 rpm:脉冲/1ms *1000 /CPR *60,低通 */
        float inst = (float)d * 1000.0f / (float)CTRL_ENCODER_CPR * 60.0f;
        s_rpm_lp[m] += (inst - s_rpm_lp[m]) * s_lp_alpha;

        ctrl_run_wheel(m);
    }

    if (++s_tick >= CTRL_RPM_EST_PERIOD) {
        s_tick = 0;
        for (int m = 0; m < MOTOR_NUM; m++) {
            if (s_n[m]) {
                s_rpm_est[m] = (int16_t)((float)s_acc[m] * 1000.0f
                                         / (float)s_n[m] / (float)CTRL_ENCODER_CPR * 60.0f);
            } else {
                s_rpm_est[m] = 0;
            }
            s_acc[m] = 0;   s_n[m] = 0;
        }
    }
}

void ctrl_tune_get(CtrlTune *out)
{
    if (!out) return;
    out->kp = s_kp;
    out->ki = s_ki;
    out->lp_alpha = s_lp_alpha;
    out->dead_min = s_dead_min;
}

int ctrl_tune_set(const CtrlTune *in)
{
    int m;
    if (!in) return 0;
    if (in->kp < 0.0f || in->kp > 2.0f) return 0;
    if (in->ki < 0.0f || in->ki > 0.2f) return 0;
    if (in->lp_alpha < 0.01f || in->lp_alpha > 1.0f) return 0;
    if (in->dead_min > MOTOR_PWM_PERIOD) return 0;

    s_kp = in->kp;
    s_ki = in->ki;
    s_lp_alpha = in->lp_alpha;
    s_dead_min = in->dead_min;
    /* 调参后不带旧积分继续跑；下一次 g 从干净状态起步。 */
    for (m = 0; m < MOTOR_NUM; ++m) {
        s_ei[m] = 0.0f;
        s_e_prev[m] = 0.0f;
    }
    return 1;
}

int16_t ctrl_get_rpm_est(int m)
{
    if (m < 0 || m >= MOTOR_NUM) return 0;
    return s_rpm_est[m];
}

void ctrl_get_rpm_est_all(int16_t out[4])
{
    for (int m = 0; m < MOTOR_NUM; m++) out[m] = s_rpm_est[m];
}

/* 1ms 低通快速 rpm 输出(供里程积分等需要实时轮速处;别拿 10ms 慢估积分) */
void ctrl_get_rpm_fast_all(float out[4])
{
    for (int m = 0; m < MOTOR_NUM; m++) out[m] = s_rpm_lp[m];
}

/* 每轮累计编码器计数(带符号;自上次 reset 起,模式7-10 raw readout 用) */
int32_t ctrl_enc_total(int m)
{
    if (m < 0 || m >= MOTOR_NUM) return 0;
    return g_enc_total[m];
}

/* 清零四轮累计计数(每轮 g1 起测/reset 时调,测量卫生) */
void ctrl_enc_reset_all(void)
{
    for (int m = 0; m < MOTOR_NUM; m++) g_enc_total[m] = 0;
    bp_enc_raw_reset_all();
}
