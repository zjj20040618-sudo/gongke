#ifndef APP_MISSION_TRIAL_H
#define APP_MISSION_TRIAL_H
#include <stdint.h>

/* Independent mode32: QR/vision/laser, no arm calls and no retry scans.
 * Distances are the user's wheel-command totals, NOT measured field pose. */
void mission_trial_init(void);
const char *mission_trial_config_missing(void);
int mission_trial_run(void);
void mission_trial_tick_1ms(void); /* ControlTask integrates even while parked/aligning/turning */
void mission_trial_report(void);
const char *mission_trial_phase(void);
void mission_trial_get_progress(float *done, float *total);
void mission_trial_get_qr(int32_t out[3]);
/* RAM-only. cls=-1: sign only; sign=0: retain sign; cx=-1: retain cx.
 * Nonzero sign must be +1/-1, cx must be measured on the actual 480px image. */
int mission_trial_set_alignment(int cls, int cx, int sign);
#endif
