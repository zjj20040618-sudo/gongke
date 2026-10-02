#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>

/* 每轮每圈编码器计数：MG513 P30 = 13线霍尔 ×4 ×30 = 1560。
 * ⚠️ 台核准：手推某轮转整整一圈，读 ctrl_enc_total 应 ≈ 这个值；不符按实测改（见 fw/实测值清单.md A1）。
 * 放头文件是为了 motion.c 也能用它做「计数 → mm」换算（里程）。 */
#define CTRL_ENCODER_CPR  1560u

typedef struct {
    float kp;
    float ki;
    float lp_alpha;
    uint16_t dead_min;
} CtrlTune;

/* 1ms 轮速控制层。ControlTask(High) 每 1ms 调 ctrl_tick_1ms()。
 * 已实现：速度环 PID(位置式,1ms,低通快速rpm反馈)；只留 CPR/增益做实测微调。 */

void     ctrl_init(void);
void     ctrl_set_speed(int m, float rpm);     /* 保留小数 RPM，避免小角度修正被截断 */
void     ctrl_set_duty_open(int m, int16_t duty); /* 开环直通 duty(±199,验单路驱动/方向) */
void     ctrl_tick_1ms(void);                  /* 1ms：读编码器→PID→写 PWM */
void     ctrl_stop_all(void);                  /* 四轮刹车，并清 PI 历史量 */
void     ctrl_coast_all(void);                 /* 四轮自由滑行，并清 PI 历史量（手转编码器用） */
int16_t  ctrl_get_rpm_est(int m);              /* 最近 ~10ms 编码器转速估计(rpm)，仅日志/整定 */
void     ctrl_get_rpm_est_all(int16_t out[4]);
void     ctrl_get_rpm_fast_all(float out[4]);  /* 1ms 低通快速 rpm(喂里程积分用) */
void     ctrl_get_target_rpm_all(float out[4]); /* 实际下发速度环的小数RPM目标，用于BLE核对 */
int32_t  ctrl_enc_total(int m);                /* 每轮累计编码器计数(带符号,自上次 reset) */
void     ctrl_enc_reset_all(void);             /* 清零四轮累计计数 */
void     ctrl_tune_get(CtrlTune *out);          /* 读取当前 RAM 参数（不涉及 Flash） */
int      ctrl_tune_set(const CtrlTune *in);     /* 校验并应用 RAM 参数；成功回 1 */

#endif /* APP_CONTROL_H */
