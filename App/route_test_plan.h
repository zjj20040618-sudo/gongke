/*
 * 初学者导读：模式31固定路线表。每一行是一段；仅有路线，没有QR识别或抓放任务。
 * .h 相当于接口清单，供 #include 引入；函数实现通常在同名.c中。
 * #ifndef / #define / #endif 是头文件保护，防止同一编译单元重复包含定义。
 */

#ifndef APP_ROUTE_TEST_PLAN_H
#define APP_ROUTE_TEST_PLAN_H

#include <stdint.h>

/* 2026-10-03 user-provided command distances, no-task trial only.
 * 750 includes the entire obstacle road. Do not add a second crossing leg.
 * 2450 and 2125 are whole corridors, not task-to-task station offsets.
 * Modes reuse the stable distance/turn machines. Forward v100 alone now
 * uses the small T_FORWARD_FF_SEED correction; reverse/strafe unchanged. */
#define ROUTE_TEST_MODE 31
#define ROUTE_TEST_V_MMS 100.0f
#define ROUTE_TEST_STAGES 10u

typedef struct {
    uint8_t mode;
    uint16_t distance_mm; /* positive slot value; direction comes from mode */
    const char *name;
} RouteTestLeg;

/* static只让当前包含它的编译单元使用，const禁止修改表；每行按成员声明顺序填写。 */
static const RouteTestLeg s_route_test_plan[ROUTE_TEST_STAGES] = {
    { 17u,  500u, "START_LEFT" },
    { 16u,  600u, "BACK_TO_3RD" },
    { 30u,    0u, "LEFT95" },
    { 15u,  750u, "FULL_OBSTACLE_ROAD" },
    { 17u,  730u, "EXIT_LEFT" },
    { 15u,  830u, "FWD_TO_TASK_CORNER" },
    { 20u,    0u, "RIGHT85_TO_TASKS" },
    { 15u, 2450u, "TASK_CORRIDOR_NO_TASKS" },
    { 20u,    0u, "RIGHT85_TO_RESCUE" },
    { 15u, 2125u, "RESCUE_TO_FINAL_NO_TASK" }
};

#endif
