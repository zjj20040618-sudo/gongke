/*
 * 初学者导读：模式32接口：等待和动作在 MissionTask，道路进度更新在1ms ControlTask。
 * .h 相当于接口清单，供 #include 引入；函数实现通常在同名.c中。
 * #ifndef / #define / #endif 是头文件保护，防止同一编译单元重复包含定义。
 */

#ifndef APP_MISSION_TRIAL_H
#define APP_MISSION_TRIAL_H
#include <stdint.h>

/* 模式32：QR/视觉/激光联调，不调用机械臂，不做往返补扫。
 * 距离是用户登记的轮式命令值，不是已测得的绝对赛场坐标。 */
void mission_trial_init(void);
const char *mission_trial_config_missing(void);
int mission_trial_run(void);
void mission_trial_tick_1ms(void); /* ControlTask integrates even while parked/aligning/turning */
void mission_trial_report(void);
const char *mission_trial_phase(void);
void mission_trial_get_progress(float *done, float *total);
void mission_trial_get_qr(int32_t out[3]);
/* 仅改RAM。cls=-1不改类别工作点；sign=0保留方向；cx=-1保留工作点。
 * 非零sign只能为+1/-1；cx接口范围0..479，且必须处于现场实际图像宽度内，不能假定总是480px。 */
int mission_trial_set_alignment(int cls, int cx, int sign);
/* RAM-only entry-to-common-bucket leg, 1..2449 wheel-command mm;
 * the bucket-based second leg is 2450-first, not a ball search limit.
 * Unset after init; writes are rejected while the trial is running. */
int mission_trial_set_first_leg(uint16_t mm);
#endif
