#include "mission.h"
#include "robot_tasks.h"
#include "steps.h"
#include "auto_steps.h"  /* step_cross_obstacle */
#include "proto.h"
#include "motion.h"
#include "imu.h"
#include "main.h"        /* HAL_GetTick */
#include "cmsis_os.h"

/* ========== 整场编排:阻塞式顺序脚本 ==========
 * 任务文件 = 顺序调 steps 库函数,每步内部"做完动作→等判据"。
 * 运行口径(2026-09-06):别给自己限任务时间、先完成再完美——步骤内部不设
 * "倒计时到就放弃"的跳站/丢球兜底,找目标扫到出现、对准对到成、做完才算过;
 * 外部急停、未标定关键路线、IMU失效、越障物理安全兜底均进入 ABORT；
 * 跑到终端 DONE/ABORT 后驻留。
 * QR 规矩:① QR 是越障的放行门槛——没读到**有效** QR(三位都∈[1,3])就不越障,
 * 在倒退段两个已知端点之间前后补扫；最终回到第三段起点，左转90°后前进到越障点。
 * 节点会按最新口径停车、清本段航向零点、等待再开。 */

/* ===== QR 获取 =====
 * 2026-10-01 用户纠正：机械臂朝车身左侧，出发先左平移、再沿车尾倒退。
 * 2026-10-02相机同朝左侧已确认；激光是否同侧/共轴仍待单独确认。
 * 倒退到位=第三段起点；在该节点车身左转90°，再沿新的车头前进到减速带前。
 * motion_vel_set 使用车体系：参数1=前后、参数2=横移；不把它们当固定地图坐标。
 * 前两段及补扫期间不转车身，累计编码器不清零。**有效 QR** = 三位都
 * ∈[1,3](d1 排爆球色 / d2 反恐靶色 1红2绿3蓝;d3 救援人形 1圆柱2圆锥3腰鼓);
 * 读花(某位出界)=还没读到，继续扫。第三段起点没读到有效码时，在倒退段起点 b0 与
 * 终点 b1 之间前后补扫：后退扫到→继续到 b1；前进扫到→先到 b0，再完整倒退到 b1。
 * 相机朝向改变后的实际扫码可见范围仍待联调；不据此假报已扫码或中途改节点。
 * 几何、CPR/轮位全待实测；参数为 0 时必须拒绝启动，不能假报到位。 */
#define QR_START_LEFT_MM 0u    /* TODO 实测：出发→倒退起点的左平移距离；旧直走d520不直接沿用 */
#define QR_BACK_MM       0u    /* TODO 实测：倒退起点b0→第三段起点b1；旧左移d560不直接沿用 */
#define QR_BACK_V_MMS    0     /* TODO 实测：倒退及前后补扫速度 mm/s */
#define QR_STR_V_MMS     0     /* TODO 实测：第一段左平移速度 mm/s */

/* ===== 与赛道一致的路线段 =====
 * 2026-10-01/02已逐段确认左装机构路线，救援前右90°后直走进区、抓后继续直走返回。
 * 所有必须上车确定的距离/速度先置 0。
 * QR 的 0 值会阻止整场启动；后段 0 值仍为待实测占位，不能视作已验证到位。
 * 横移距离正=右、负=左；旋转正负最终按 IMU/底盘方向台校。 */
#define ROUTE_FWD_V_MMS             0.0f /* TODO 实测：普通路线直行速度，独立于横移 */
#define ROUTE_STRAFE_V_MMS          0.0f /* TODO 实测：普通路线横移速度，独立于直行 */
#define R_PRE_CROSS_FWD_MM          0.0f /* TODO 实测：第三段起点→减速带前安全起冲点 */
#define R_CROSS_REST_FWD_MM         0.0f /* TODO 实测：越障结束→第三段直线末端 */
#define R_CROSS_EXIT_LEFT_MM        0.0f /* TODO 实测：到第三段末端后左横移 */
#define R_CROSS_EXIT_FWD_MM         0.0f /* TODO 实测：横移后向前到排爆高度 */
#define R_EOD_ENTRY_RIGHT_TURN_DEG 90.0f /* 用户2026-10-02：原地右90°，准备进入任务区；不是右横移 */
#define R_EOD_ENTRY_FWD_MM          0.0f /* TODO 实测：右90°后沿新车头直走进入任务区的距离 */
#define R_EOD_TO_ANTI_FWD_MM        0.0f /* TODO 实测：排爆转回原朝向后，排爆入口→反恐入口的前后里程差 */
#define R_ANTI_EXIT_FWD_MM          0.0f /* TODO 实测：反恐带入口→救援前拐点的前后里程差 */
#define R_RESCUE_RIGHT_TURN_DEG    90.0f /* 用户2026-10-02：直走到拐点后原地右转90° */
#define R_RESCUE_ENTRY_FWD_MM       0.0f /* TODO 实测：救援前右90°后沿新车头直走进入救援带 */
#define R_RESCUE_TO_HOME_FWD_MM     0.0f /* TODO 实测：从救援入口基准到返回区的前后里程差 */

#define CROSS_V_MMS   150.0f   /* 越障速度 mm/s(匀速冲,坎上别停别降速——坎面本身滑) */
#define CROSS_TMO_MS  8000u    /* 越障兜底超时(防卡死);正常由姿态判据提前结束。
                                * ⚠️ 实际要走的行程 ≈480mm(带280+轴距200),150mm/s 约需 3.2s */

/* 正式整场最终确认闸门：数值填完后，还必须按对应台测清单逐项确认再改 1。
 * 这不是参数本身，目的是防止某个非零种子值被误当成已经实测。 */
#define CAL_DRIVE_READY    0u /* 轮位/方向/CPR/有效轮径/速度环/acc/dec/ykp/okp */
#define CAL_VISION_READY   0u /* QR三元组、OBJ类/标签/cx方向与四类站位联调 */
#define CAL_ARM_READY      0u /* 爪脉宽、两轴方向、伸/抬/降步数 */
#define CAL_OBSTACLE_READY 0u /* 越障速度、pitch窗口和上坎/平稳阈值 */

static volatile MissionState s_state = MS_BOOT;
static volatile int  s_start_req;   /* mission_start() 置位 */
static volatile int  s_qr_ok;       /* 三个任务目标已同时校验并锁存；越障只在此后放行 */
static int32_t s_qr[3];
static volatile uint32_t s_qr_invalid;

typedef struct {
    int ball_color;       /* LAB_R..LAB_B */
    int target_color;     /* LAB_R..LAB_B */
    int hostage_shape;    /* LAB_CYL..LAB_WAIST */
} MissionTargets;

static MissionTargets s_targets;

static const char *const s_name[MS_COUNT] = {
    [MS_BOOT] = "BOOT", [MS_READ_QR] = "READ_QR",
    [MS_CROSS_OBSTACLE] = "CROSS_OBSTACLE",
    [MS_EOD] = "EOD", [MS_ANTI] = "ANTI", [MS_RESCUE] = "RESCUE",
    [MS_DONE] = "DONE", [MS_ABORT] = "ABORT",
};

/* 开机清零全态、置 MS_BOOT(robot_init 调一次) */
void mission_init(void)
{
    run_reset();   /* 仅初始化时清旧中止/视觉帧；启动请求之后绝不清急停 */
    s_state = MS_BOOT;
    s_start_req = 0;
    s_qr_ok = 0;
    s_qr[0] = s_qr[1] = s_qr[2] = 0;
    s_qr_invalid = 0u;
    s_targets.ball_color = -1;
    s_targets.target_color = -1;
    s_targets.hostage_shape = -1;
}

/* ---- 状态/启动/QR 读取的对外小接口(日志、BT 'g'、test 用) ---- */
const char *mission_config_missing(void)
{
    const char *missing;
    if (QR_START_LEFT_MM == 0u) return "QR_START_LEFT_MM";
    if (QR_BACK_MM == 0u) return "QR_BACK_MM";
    if (QR_BACK_V_MMS <= 0) return "QR_BACK_V_MMS";
    if (QR_STR_V_MMS <= 0) return "QR_STR_V_MMS";
    if (ROUTE_FWD_V_MMS <= 0.0f) return "ROUTE_FWD_V_MMS";
    if (ROUTE_STRAFE_V_MMS <= 0.0f) return "ROUTE_STRAFE_V_MMS";
    if (R_PRE_CROSS_FWD_MM <= 0.0f) return "R_PRE_CROSS_FWD_MM";
    if (R_CROSS_REST_FWD_MM <= 0.0f) return "R_CROSS_REST_FWD_MM";
    if (R_CROSS_EXIT_LEFT_MM <= 0.0f) return "R_CROSS_EXIT_LEFT_MM";
    if (R_CROSS_EXIT_FWD_MM <= 0.0f) return "R_CROSS_EXIT_FWD_MM";
    if (R_EOD_ENTRY_RIGHT_TURN_DEG != 90.0f) return "R_EOD_ENTRY_RIGHT_TURN_DEG";
    if (R_EOD_ENTRY_FWD_MM <= 0.0f) return "R_EOD_ENTRY_FWD_MM";
    if (R_EOD_TO_ANTI_FWD_MM <= 0.0f) return "R_EOD_TO_ANTI_FWD_MM";
    if (R_ANTI_EXIT_FWD_MM <= 0.0f) return "R_ANTI_EXIT_FWD_MM";
    if (R_RESCUE_RIGHT_TURN_DEG != 90.0f) return "R_RESCUE_RIGHT_TURN_DEG";
    if (R_RESCUE_ENTRY_FWD_MM <= 0.0f) return "R_RESCUE_ENTRY_FWD_MM";
    if (R_RESCUE_TO_HOME_FWD_MM <= 0.0f) return "R_RESCUE_TO_HOME_FWD_MM";
    missing = motion_profile_config_missing();
    if (missing) return missing;
    missing = steps_config_missing();
    if (missing) return missing;
    missing = task_eod_config_missing();
    if (missing) return missing;
    missing = task_rescue_config_missing();
    if (missing) return missing;
    if (!CAL_DRIVE_READY) return "CAL_DRIVE_READY";
    if (!CAL_VISION_READY) return "CAL_VISION_READY";
    if (!CAL_ARM_READY) return "CAL_ARM_READY";
    if (!CAL_OBSTACLE_READY) return "CAL_OBSTACLE_READY";
    return 0;
}

int mission_start(void)
{
    if (s_state != MS_BOOT || s_start_req || mission_config_missing()) return 0;
    run_reset();   /* 接受 g 之前清 BOOT 旧帧；随后的 a 必须一直有效 */
    s_start_req = 1;
    return 1;
}
MissionState mission_state(void) { return s_state; }
const char  *mission_state_str(MissionState s) { return (s < MS_COUNT) ? s_name[s] : "?"; }
void mission_get_qr(int32_t out[3]) { out[0] = s_qr[0]; out[1] = s_qr[1]; out[2] = s_qr[2]; }
int mission_qr_ready(void) { return s_qr_ok; }
uint32_t mission_qr_invalid_count(void) { return s_qr_invalid; }

/* 内部切状态(只设 s_state) */
static void to_state(MissionState ns) { s_state = ns; }

/* 将视觉返回的完整三元组一次性解码为三个任务目标。
 * 只有三项同时合法才写 out 并返回 1；部分值、默认 0、越界值都不改变已锁存目标。
 * 官方编码:d1/d2 1红2绿3蓝；d3 1圆柱2圆锥3腰鼓。 */
static int qr_decode_targets(const int32_t d[3], MissionTargets *out)
{
    MissionTargets decoded;
    if (!d || !out
        || d[0] < 1 || d[0] > 3
        || d[1] < 1 || d[1] > 3
        || d[2] < 1 || d[2] > 3) return 0;

    decoded.ball_color = (int)d[0] - 1;
    decoded.target_color = (int)d[1] - 1;
    decoded.hostage_shape = (int)d[2] + 2;
    *out = decoded;
    return 1;
}

/* 只在端点切换目标。采到有效 QR 仅置标志，不中途刹车/改向。 */
static int qr_leg_to(float target_mm, int lateral, float speed_mms)
{
    float now, dir, orth0;
    MotionRamp ramp;
    uint32_t ramp_ms;
    if (speed_mms <= 0.0f || !step_prepare_leg()) return 0;
    motion_linear_ramp_init(&ramp);
    now = lateral ? motion_lateral_odo_mm() : motion_odo_mm();
    orth0 = lateral ? motion_odo_mm() : motion_lateral_odo_mm();
    dir = (target_mm >= now) ? 1.0f : -1.0f;
    ramp_ms = HAL_GetTick();
    while (!run_aborted()) {
        float err;
        int32_t candidate[3];
        MissionTargets decoded;
        if (!imu_ok()) break;
        now = lateral ? motion_lateral_odo_mm() : motion_odo_mm();
        err = target_mm - now;
        if (dir * err <= 0.0f) { motion_brake(); return 1; }
        {
            uint32_t tick = HAL_GetTick();
            float dt = (float)(uint32_t)(tick - ramp_ms) / 1000.0f;
            float cmd;
            ramp_ms = tick;
            cmd = motion_linear_profile_step(&ramp, dir * speed_mms, err, dt);
            float orth_cmd = step_orth_hold_cmd(lateral, orth0);
            motion_vel_set(lateral ? orth_cmd : cmd,
                       lateral ? cmd : orth_cmd,
                       step_heading_hold_w(0.0f));
        }
        if (!s_qr_ok && wait_qr(candidate, 10)) {
            if (qr_decode_targets(candidate, &decoded)) {
                s_qr[0] = candidate[0]; s_qr[1] = candidate[1]; s_qr[2] = candidate[2];
                s_targets = decoded; /* 三个后续任务选择值一次性锁存，不接受部分更新 */
                s_qr_ok = 1;
            } else {
                s_qr_invalid++;
            }
        }
        osDelay(5);
    }
    motion_brake();
    return 0;
}

/* 只包含转身前的两个平移段。参数化供主机回放使用，正式入口仍取0占位并拒绝开跑。
 * 先左移，再倒退至b1；补扫前进发现码也必须回b1，转身后不再复用这条旧里程轴。 */
static int qr_travel_legs(float start_left_mm, float back_mm,
                          float strafe_v_mms, float back_v_mms)
{
    float b0, b1;
    if (start_left_mm <= 0.0f || back_mm <= 0.0f
        || strafe_v_mms <= 0.0f || back_v_mms <= 0.0f) return 0;
    if (!qr_leg_to(motion_lateral_odo_mm() - start_left_mm, 1, strafe_v_mms)) return 0;
    b0 = motion_odo_mm();
    b1 = b0 - back_mm;
    if (!qr_leg_to(b1, 0, back_v_mms)) return 0;
    while (!s_qr_ok && !run_aborted()) {
        if (!qr_leg_to(b0, 0, back_v_mms)) return 0;
        if (!qr_leg_to(b1, 0, back_v_mms)) return 0;
    }
    /* 前进补扫发现也先走完前进腿，再倒退到b1。 */
    return s_qr_ok && !run_aborted();
}

static int qr_travel(void)
{
    return qr_travel_legs((float)QR_START_LEFT_MM, (float)QR_BACK_MM,
                          (float)QR_STR_V_MMS, (float)QR_BACK_V_MMS);
}

/* 第三段前新增的左90°：负角度=左转，通用IMU闭环，尚非实车验收参数组。
 * 必须先停稳并转到位；中止/IMU失效不得继续第三段或越障。 */
static int route_pre_cross_turn(void)
{
    return step_nav_leg(-90.0f, 0.0f, 0.0f, 0);
}

/* 普通路线节点也沿用停车确认/本段航向零点/等待，再启动定距段。 */
static int route_straight(float dist_mm, float v_mms)
{
    if (dist_mm == 0.0f) return !run_aborted();
    return step_prepare_leg() && step_straight(dist_mm, v_mms, 0);
}

static int route_strafe(float dist_mm, float v_mms)
{
    if (dist_mm == 0.0f) return !run_aborted();
    return step_prepare_leg() && step_strafe(dist_mm, v_mms, 0);
}

/* 视觉扫描/对位会改变车在工作带内的位置；后续路段必须回到相对“带入口基准”定义的
 * 目标节点，而不是从识别到的任意中间点再盲走一个固定距离。 */
static int route_straight_to(float target_odo_mm, float speed_mms)
{
    float remain;
    if (!step_prepare_leg()) return 0;
    /* 当前车头须与入口基准一致（排爆须先转回）；停稳后扣除前后轴已走里程。 */
    remain = target_odo_mm - motion_odo_mm();
    if (remain > -0.5f && remain < 0.5f) return !run_aborted();
    return step_straight(remain, (remain > 0.0f) ? speed_mms : -speed_mms, 0);
}

void mission_main(void)
{
    float eod_entry_fwd_odo, anti_entry_fwd_odo, rescue_entry_fwd_odo;

    /* BOOT:等启动指令(BT 'g')。此后不得再清 run_abort 标志：
     * g 后立即再g或a可能先于本任务苏醒到达，清标志会吞掉这次急停。 */
    while (!s_start_req) osDelay(10);
    s_start_req = 0;
    if (run_aborted()) goto failed;
    s_qr_ok = 0;
    s_qr[0] = s_qr[1] = s_qr[2] = 0;
    s_qr_invalid = 0u;
    s_targets.ball_color = -1;
    s_targets.target_color = -1;
    s_targets.hostage_shape = -1;

    /* ① QR 获取：左平移→倒退到第三段起点；无效则在倒退段两端前后有界补扫。
     * 节点按最新口径停车确认/清本段航向/等待；未标定路线直接 ABORT。 */
    to_state(MS_READ_QR);
    proto_send_scene(SCENE_QR);   /* 让视觉切 QR 上报;MaixCam 常开也照吃 */
    if (!qr_travel()) { to_state(MS_ABORT); motion_brake(); goto terminal; }

    /* 防御性二次门：只有三个任务目标已同时解码并锁存，才允许切到越障。 */
    if (!s_qr_ok) goto failed;

    /* ② 第三段前车身左转90°，再沿新车头前进到减速带前安全起冲点，再越障。
     * 倒退端点不等于越障入口；距离须按车中心/车头安全间隙实测。
     * 路线已按用户逐项确认换轴；各段距离及侧装相机标定仍待实测，闸门保持关闭。 */
    if (!route_pre_cross_turn()) goto failed;
    if (!route_straight(R_PRE_CROSS_FWD_MM, ROUTE_FWD_V_MMS)) goto failed;

    /* 越障:车已在越障正前,匀速直冲减速带(只有有效 QR 才走得到这)。
     * 2026-09-12 改用 step_cross_obstacle:定速 + yaw 锁向 + **姿态判据结束**
     * (先确认上坎、再等 pitch 窗口平稳)。不依赖里程 → 不怕坎面打滑。详见 auto_steps.c。 */
    if (run_aborted() || !step_prepare_leg()) goto failed;
    to_state(MS_CROSS_OBSTACLE);
    if (!step_cross_obstacle(CROSS_V_MMS, 0.0f, CROSS_TMO_MS)) goto failed;

    /* ③ 用户所说的越障路=完整第三段：通过障碍后仍须走完剩余直线。
     * 再左平移、直走，停稳原地右转90°后，沿新车头直走进入任务区。
     * 进区距离待实测；侧装相机对位方向符号/工作点仍须装车标定。 */
    if (!route_straight(R_CROSS_REST_FWD_MM, ROUTE_FWD_V_MMS)) goto failed;
    if (!route_strafe(-R_CROSS_EXIT_LEFT_MM, -ROUTE_STRAFE_V_MMS)) goto failed;
    if (!route_straight(R_CROSS_EXIT_FWD_MM, ROUTE_FWD_V_MMS)) goto failed;
    if (!step_nav_leg(R_EOD_ENTRY_RIGHT_TURN_DEG, 0.0f, 0.0f, 0)) goto failed;
    if (!route_straight(R_EOD_ENTRY_FWD_MM, ROUTE_FWD_V_MMS)) goto failed;

    /* ④ 排爆：进入工作带后由视觉找球/桶并完成抓放。 */
    eod_entry_fwd_odo = motion_odo_mm();
    if (run_aborted()) goto failed;
    to_state(MS_EOD);
    if (task_eod_run(s_targets.ball_color) != TASK_OK) goto failed;

    /* ⑤ 用户2026-10-02确认：排爆放桶后旋回原车头，直走去反恐区。
     * 入口基准同用前后轴；扫描已走的部分不重复走。距离仍待实测。 */
    if (!route_straight_to(eod_entry_fwd_odo + R_EOD_TO_ANTI_FWD_MM, ROUTE_FWD_V_MMS)) goto failed;
    if (run_aborted()) goto failed;
    anti_entry_fwd_odo = motion_odo_mm();
    to_state(MS_ANTI);
    if (task_anti_run(s_targets.target_color) != TASK_OK) goto failed;

    /* ⑥ 用户2026-10-02确认：反恐完成后继续直走到拐点，再原地右转90°。
     * 以反恐入口前后基准扣除找靶/对位已走距离，先到拐点再转身。
     * 右转后沿新的车头直走进入救援工作带；转身前后的里程轴不混用。 */
    if (!route_straight_to(anti_entry_fwd_odo + R_ANTI_EXIT_FWD_MM, ROUTE_FWD_V_MMS)) goto failed;
    if (!step_nav_leg(R_RESCUE_RIGHT_TURN_DEG, 0.0f, 0.0f, 0)) goto failed;
    if (!route_straight(R_RESCUE_ENTRY_FWD_MM, ROUTE_FWD_V_MMS)) goto failed;
    if (run_aborted()) goto failed;
    rescue_entry_fwd_odo = motion_odo_mm();
    to_state(MS_RESCUE);
    if (task_rescue_run(s_targets.hostage_shape) != TASK_OK) goto failed;

    /* ⑦ 用户2026-10-02确认：抓住并抬离人质后保持夹持，不转身，继续直走返回。
     * 以右转后救援入口的前后基准扣除扫描/对位位移，不从抓取停点重复走整段。 */
    if (!route_straight_to(rescue_entry_fwd_odo + R_RESCUE_TO_HOME_FWD_MM, ROUTE_FWD_V_MMS)) goto failed;

    to_state(run_aborted() ? MS_ABORT : MS_DONE);
    goto terminal;
failed:
    to_state(MS_ABORT);
terminal:
    motion_brake();
    for (;;) osDelay(200);   /* 终端驻留:停着等人工复位/重启 */
}
