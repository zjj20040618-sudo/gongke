#ifndef APP_STEPS_H
#define APP_STEPS_H

#include <stdint.h>
#include "proto.h"

/* 通用「步骤」库:任务文件只从上往下调这些,自己不写等待/超时。
 * 每个等待类步骤内部 osDelay 让出 CPU,超时或被中止(run_abort)返回失败。 */

/* ---- 运行控制(整场一条) ---- */
void run_reset(void);       /* 清中止标记/帧暂存；仅初始化或接受启动请求前调用 */
int  run_aborted(void);     /* 1=外部安全停机(BT 'a'/'s' 或安全 sensor);内部超时不算 */
void run_abort(void);       /* 请求外部安全停机:正在等待的步骤尽快返回失败 */

/* ---- 通用等待 ---- */
void wait_ms(uint32_t ms);                 /* 睡 ms(让出 CPU,期间可被中止) */
int  wait_qr(int32_t d[3], uint32_t to);   /* 等二维码帧,存 d1/d2/d3;to==0→不限时;超时/中止→0 */
int  step_vision_scene(ProtoScene scene); /* stopped command/ACK/fresh-result handshake */
int  step_vision_target(ProtoTask task, uint8_t digit); /* selected task; stopped handshake */
void step_vision_receive_end(void); /* local RX stage end + clear slots; camera keeps recognizing */
void step_object_select(int cls, int label); /* new phase: clear slot and filter ISR inputs */
int  step_object_take(ProtoFrame *out);      /* consume the latest matching fresh object */

/* Left-facing camera grab alignment: image X -> body forward/backward,
 * image Y -> body right/left. Workpoints are measured, never image center.
 * Signs: +1 means positive pixel error requests positive body-axis speed.
 * Inclusive tolerances are pixels; default trial seeds are 8px on each axis. */
typedef struct {
    int cx, cy, x_sign, y_sign;
    float x_tol_px, y_tol_px;
} GrabAlignConfig;
#define GRAB_XY_TOL_PX 8.0f
#define GRAB_XY_GOOD_FRAMES 5u
#define GRAB_XY_FRESH_MS 300u
#define GRAB_XY_STILL_MS 250u
/* Caller prepares/stops the leg and captures an absolute heading FIRST.
 * Does not clear heading/odom. Rechecks BOTH pixels on every new frame,
 * X priority; brakes and settles before changing axis. Only BALL/HOSTAGE.
 * Missing/empty/stale coordinates brake and wait, never release the gate. */
int step_align_xy(int cls, int label, const GrabAlignConfig *cfg,
                  float absolute_heading_deg, uint32_t to);
/* Formal grab calibration, RAM-only: -1 keeps cx/cy, 0 keeps each sign.
 * No defaults for workpoints/signs; false while XY alignment is running. */
int step_grab_alignment_set(int cls, int cx, int cy, int x_sign, int y_sign);
const char *step_grab_alignment_config_missing(void);

/* ---- 视觉对准 ----
 * BALL/HOSTAGE: step_grab_alignment_set先给实测cx/cy/两个符号，
 * 再调用共享step_align_xy反复X(前后)/Y(左右)单轴闭环；同一新帧两轴
 * 同时达标，连续5个新seq且四轮静止250ms才成功。未标定不驱动。
 * TARGET/BUCKET保留以下历史两步路径，35独立打靶仍只看X。
 * 左侧相机阶段1：用目标像素高估距离，再左平移靠近/右平移离开（最多3次）。
 *        `d_站` 填0可关闭粗调；它不是沿车头方向移动。
 * 阶段2：沿车身前后轴把cx拉到每类标定值，不是画面中心；容差外最小速度+限幅。
 *        连续N帧偏差均达标才对准。VISION_CX_FWD_SIGN须实测为±1，0拒绝驱动。
 * 分时做先纵深后画面左右，两轴不能按旧朝前相机的车体系混用。
 * 语义：三个要用视觉的任务都是「**等到对齐为止**」——调用方传 to=0（不设超时兜底）。
 * 靶/桶历史标定值（站距 / 站位处像素高 / cx）在 steps.c 的 `s_stand[4]` 表里，按 cls 查：
 *   0球 / 1靶 / 2人质 / 3桶。**把车摆到满意站位时，这三个一次量齐。**
 * label = 在 3 个同色/同形并列里挑 QR 选中的那个；label<0 → 该类全场只此一只
 *   （如排爆桶，无色可挑），不挑 label。to==0 → 不限时。
 * ⚠️ 阈值/增益/站表全是种子值（在 steps.c），待实测标定；方向符号要台校。 */
int  step_align(int cls, int label, uint32_t to);

/* ---- 左侧相机沿车身前后轴扫目标（align=出现后锁；sweep=没出现时扫）----
 * 目标没在画面里时,车就在它那条带里来回走、边走边扫,目标一出现就停、回 1(交 step_align);
 * 一直读不到就持续在两个实测端点之间往返；to 超时/被中止则回 0，整场中止。
 * 各处共用这一份前后往返代码,只差"找谁":
 * 当前只接受 want=PF_OBJ，按 cls+label 匹配；label<0 表示该类单只不挑 label。
 * want=PF_QR 会拒绝，d 参数为兼容旧接口而保留但不写入。
 * QR 扫描和三位合法性检查属于 mission.c；to==0 表示不限时。
 * 扫描用前后编码器里程在入口/远端间换向；SWEEP_FWD_MMS与每类远端距离待实测。 */
int  step_sweep(int want, int cls, int label, int32_t d[3], uint32_t to);
const char *steps_config_missing(void); /* 主流程启动前检查仍为占位的关键步骤参数 */

/* ---- 跨区导航腿(相对分段：转向 + 定距直行；不依赖视觉/绝对定位) ----
 * 节点先刹车、确认四轮编码器静止、记录软件航向零点，等待 0.75s 再走。
 * 此零点不改变 IMU 全局连续航向，也不清累计编码器计数。
 * 视情况可加"视觉重锚"(识别车道线/靶心矫正)作为加分项；没有视觉也能走。 */
#define NAV_SETTLE_MS  750u    /* 清本段航向零点后等待 0.75s，再启动下一段 */
int  step_prepare_leg(void);      /* 刹车→四轮编码器连续静止→航向软件清零→等待；失败/中止回 0 */
float step_heading_hold_w(float heading0_deg); /* 当前 IMU 相对航向→yaw 保持角速度(rad/s) */
float step_heading_hold_w_kp(float heading0_deg, float kp); /* 显式0..5增益，不修改全局；单段调试用 */
float step_heading_kp_deg(void);                /* 当前 RAM 航向增益 */
int   step_heading_kp_set(float kp);            /* 0..5，RAM-only；成功回 1 */
float step_orth_kp(void);                       /* 当前局部正交串动增益 */
int   step_orth_kp_set(float kp);               /* 0..5，0=关闭，RAM-only */
float step_orth_hold_cmd(int lateral_motion, float orth_reference_mm);
int  step_straight(float dist_mm, float v_mms, uint32_t to); /* 定距直行(yaw锁向):走够 dist 刹停;to==0不限时 */
int  step_strafe(float dist_mm, float v_mms, uint32_t to);   /* 定距横移:正=右/负=左；依赖横向编码器里程台校 */
int  step_return_forward_odo(float target_mm, uint32_t to);  /* 用扫描速度回到指定前后里程基准 */
int  step_nav_leg(float turn_deg, float dist_mm, float v_mms, uint32_t to); /* 停稳/清本段航向→转向→直行 */

/* ---- 动作步骤(EOD / RESCUE 通用) ---- */
int  step_rotate_deg(int deg, uint32_t to);         /* 原地转 deg°:纯 w 自转 + IMU yaw 判到位(判据见函数内;EOD 转 180 用) */
int  step_arm_prepare(const char *task, const char *stage); /* 抓放前停稳/清本段航向/等待，蓝牙记阶段 */
int  step_arm_run(const char *task, const char *stage, uint32_t steps,
                  int (*action)(uint32_t)); /* 任务阶段记录：只证明命令发送，绝非物理到位 */
int  step_arm_release(const char *task, const char *stage); /* 带阶段回传的开爪 */
int  step_grasp(uint32_t extend_steps);              /* 开爪→axis0伸指定步数→闭爪；球/人质各自标定 */
int  step_release(void);                              /* 松爪放下；中止前不再新发舵机命令 */
int  step_rack_retract(uint32_t steps);              /* axis0按已伸出的步数反向回程；不是撞挡块/传感器回零 */
int  step_arm_lift(uint32_t steps);   /* axis1(竖直丝杆)抬 steps 步:抓后抬离球座/平台或放后出桶 */
int  step_arm_lower(uint32_t steps);  /* axis1(竖直丝杆)降 steps 步:抓前到目标高度或放桶前入桶 */

/* ---- 动作步骤(ANTI) ---- */
#define TARGET_AIM_SETTLE_MS 1000u /* 对准后保持刹车、激光关闭，静置1秒 */
#define TARGET_LASER_ON_MS   2000u /* 靶射击保持2秒，结束/中止均关闭 */
int  step_target_settle(void);                      /* 对准后静置；可中止，中止不得开光 */
int  step_fire(uint32_t hold_ms);                    /* 激光亮 hold_ms 后自动灭 */

/* ---- 帧喂入(MaixCam 帧解析回调里调,ISR 上下文) ---- */
void steps_feed_frame(const ProtoFrame *f);

#endif /* APP_STEPS_H */
