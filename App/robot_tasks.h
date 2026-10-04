/*
 * 初学者导读：三个任务的接口。注意 TASK_OK 是0，不能用 if (!task_xxx_run(...)) 判断失败。
 * .h 相当于接口清单，供 #include 引入；函数实现通常在同名.c中。
 * #ifndef / #define / #endif 是头文件保护，防止同一编译单元重复包含定义。
 */

#ifndef APP_ROBOT_TASKS_H
#define APP_ROBOT_TASKS_H

/* 三个大任务各一个文件;每个任务 = 顺序调 steps 库里的步骤函数。
 * 返回 TASK_OK 该任务步骤全走完;TASK_ABORT = 步骤失败或被外部中止。
 * 找目标/对准不主动设任务时限；但 IMU 失效等安全失败会返回 TASK_ABORT，
 * mission 必须据此停整场，不得忽略任务返回值。
 * 三个任务各收一个选择子 = QR 三位解码出来:抓/打/救前选对目标
 * (官方命题 d1 排爆球色 / d2 反恐靶色 / d3 救援人形)。
 *
 * ⚠️ 文件名说明(2026-09-13)：本文件**原来叫 `task.h`**，和 **FreeRTOS 的
 *    `Middlewares/.../FreeRTOS/Source/include/task.h` 同名**。因为 `../App` 在
 *    Keil 的 include 路径里排在 FreeRTOS 前面 → FreeRTOS 自己的 `stream_buffer.c`
 *    等文件 `#include "task.h"` 会**误包含我们这个** → 报 `unknown type name
 *    'TaskHandle_t'` 等一堆错（只在**全量重编/ReBuild** 时暴露，增量编译复用旧
 *    .obj 看不出来）。改名成 `robot_tasks.h` 彻底避开，别再改回去。 */

#define TASK_OK    0
#define TASK_ABORT 1

int task_eod_run(int ball_color);       /* 排爆:视觉锁 d1 色球→抓→转180→锁桶放→转回 */
int task_anti_run(int target_color);    /* 反恐:视觉锁 d2 色靶→射击位稳住→激光 */
int task_rescue_run(int hostage_shape); /* 救援:视觉锁 d3 形人形→抓并抬起；返回区移动由 mission 负责，不放下 */
const char *task_eod_config_missing(void);
const char *task_rescue_config_missing(void);

#endif /* APP_ROBOT_TASKS_H */
