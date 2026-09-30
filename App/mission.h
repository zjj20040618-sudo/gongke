#ifndef APP_MISSION_H
#define APP_MISSION_H

#include <stdint.h>

/* 整场任务编排。细粒度动作全在 steps 库 + task_*.c 里顺序调；
 * mission 只管顶层:等启动 → QR获取=驶向越障(前进→左移一路扫,扫到才放行越障) →
 * 越障 → EOD → ANTI → RESCUE → 终端。
 * 运行口径:一次性按顺序完成；关键步骤失败必须向上返回并停车，不假报成功、不跳站保分。
 * 普通任务不设"倒计时到就放弃"的兜底；外部急停、关键传感器失效或未标定参数拒绝启动。
 * QR 读不到**不停车**:没读到有效 QR 就在出发横移起点与第三段直线起点之间左右补扫、一直
 * 走(刹停=丢分),靠外部 stop / 比赛时限兜底。MissionTask 调 mission_main()
 * (阻塞式,脚本内 osDelay 让出 CPU),跑到 MS_DONE / MS_ABORT 后驻留,等人工复位/重启。 */

typedef enum {
    MS_BOOT = 0,        /* 开机待命:等启动指令(BT 'g') */
    MS_READ_QR,         /* QR获取=驶向越障(前进→左移)一路扫;扫到置标志才放行越障;没扫到回补扫,一直走不停(刹停=丢分) */
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
const char *mission_config_missing(void);          /* 首个未标定关键参数名；齐全返回 NULL */
void        mission_main(void);                    /* 阻塞式整场;MissionTask 调 */
MissionState mission_state(void);
const char *mission_state_str(MissionState s);
void        mission_get_qr(int32_t out[3]);        /* 读到的二维码 d1/d2/d3(供 Log) */
int         mission_qr_ready(void);                 /* 三个任务目标已同时校验并锁存 */
uint32_t    mission_qr_invalid_count(void);         /* 格式正确但三目标值非法的帧数 */

#endif /* APP_MISSION_H */
