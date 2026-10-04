/*
 * 初学者导读：正式流程的状态和启动接口；模式32也借用这里申请 MissionTask 执行。
 * .h 相当于接口清单，供 #include 引入；函数实现通常在同名.c中。
 * #ifndef / #define / #endif 是头文件保护，防止同一编译单元重复包含定义。
 */

#ifndef APP_MISSION_H
#define APP_MISSION_H

#include <stdint.h>

/* 正式整场由mission_main顺序编排，具体动作在steps.c与task_*.c。
 * 当前QR出发段是左平移、倒退；未读到合法三位码时在倒退段两端前后补扫。
 * 然后左转、越障、排爆、反恐、救援、返回；关键步骤失败统一停车。
 * 节点会停稳、记录本段航向并等待；不要沿用旧版“前进后左移/左右补扫”的说明。
 * 普通任务可不限时等待目标，仍响应外部中止和传感器失败；未标定参数拒绝启动。
 * MissionTask执行时通过osDelay让出CPU；MS_DONE/MS_ABORT后驻留，须人工复位。
 * 模式32是独立分支：R1左移后停车等合法QR，其流程见mission_trial.c。 */

typedef enum {
    MS_BOOT = 0,        /* 开机待命:等启动指令(BT 'g') */
    MS_READ_QR,         /* 正式QR出发左移、倒退与前后补扫；模式32也借此状态进入独立联调 */
    MS_CROSS_OBSTACLE,  /* 越障(驶过减速带/坡) */
    MS_EOD,             /* 排爆 */
    MS_ANTI,            /* 反恐 */
    MS_RESCUE,          /* 救援 */
    MS_DONE,            /* 跑完全部任务,驻留 */
    MS_ABORT,           /* 外部安全停机(run_abort);停住等人工复位 */
    MS_COUNT
} MissionState;

void        mission_init(void);                    /* 开机置 BOOT 待命(robot_init 调) */
int         mission_start(void);                   /* 参数齐才请求开跑；成功=1，缺参数=0 */
int         mission_start_trial(void);             /* 独立模式32请求；仅无臂联调参数闸门 */
int         mission_is_trial(void);                /* 32请求/运行/终端均为1，不能据此重启 */
const char *mission_config_missing(void);          /* 首个未标定关键参数名；齐全返回 NULL */
void        mission_main(void);                    /* 阻塞式整场;MissionTask 调 */
MissionState mission_state(void);
const char *mission_state_str(MissionState s);
void        mission_get_qr(int32_t out[3]);        /* 读到的二维码 d1/d2/d3(供 Log) */
int         mission_qr_ready(void);                 /* 三个任务目标已同时校验并锁存 */
uint32_t    mission_qr_invalid_count(void);         /* 格式正确但三目标值非法的帧数 */

#endif /* APP_MISSION_H */
