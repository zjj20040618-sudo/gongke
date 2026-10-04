/*
 * 初学者导读：转身保持参数。KP 把角度误差换成角速度，MIN/MAX 限速，SETTLE 是停后观察时长。
 * .h 相当于接口清单，供 #include 引入；函数实现通常在同名.c中。
 * #ifndef / #define / #endif 是头文件保护，防止同一编译单元重复包含定义。
 */

#ifndef APP_TURN_PROFILE_H
#define APP_TURN_PROFILE_H

/* Shared bench hold profile: mode20 +90, mode22 +180, mode30 -90.
 * 2026-10-03: preserve the successful mode22 gains/speed/tolerance/settle;
 * give +/-90 the same 12s correction budget, not a guessed angle offset.
 * Host equivalence is not physical acceptance of a new angle/load.
 * TURN90 aliases keep the existing formal +90 caller source-compatible;
 * formal -90/180 still use their legacy controller, outside route31. */
#define TURN_HOLD_MAX_MS       12000u
#define TURN_HOLD_SETTLE_MS    700u
#define TURN_HOLD_TOL_DEG      0.3f
#define TURN_HOLD_LIMIT_DEG    15.0f
#define TURN_HOLD_KP_RADS_DEG  0.15f
#define TURN_HOLD_MAX_W_RADS   2.0f
#define TURN_HOLD_MIN_W_RADS   0.18f
#define TURN_HOLD_STILL_DEG    0.2f

/* 下面是宏别名，预处理时替换成共享值；不是运行时变量赋值。 */
#define TURN90_MAX_MS          TURN_HOLD_MAX_MS
#define TURN90_SETTLE_MS       TURN_HOLD_SETTLE_MS
#define TURN90_TARGET_DEG      90.0f
#define TURN90_TOL_DEG         TURN_HOLD_TOL_DEG
#define TURN90_LIMIT_DEG       TURN_HOLD_LIMIT_DEG
#define TURN90_KP_RADS_DEG     TURN_HOLD_KP_RADS_DEG
#define TURN90_MAX_W_RADS      TURN_HOLD_MAX_W_RADS
#define TURN90_MIN_W_RADS      TURN_HOLD_MIN_W_RADS
#define TURN90_STILL_DEG       TURN_HOLD_STILL_DEG

#endif
