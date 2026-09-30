#include "steps.h"
#include "main.h"       /* HAL_GetTick */
#include "cmsis_os.h"   /* osDelay */
#include "arm.h"
#include "motion.h"
#include "imu.h"
#include "board_pins.h"
#include "control.h"
#include "turn_profile.h"
#include <math.h>
#include <stdio.h>

/* ---- 分类型帧暂存:回调(ISR)写 / wait_* 读 ----
 * QR 与 OBJ 分仓，避免 READY/PONG 或另一类业务帧覆盖刚收到的有效结果。 */
static volatile int s_qr_pending, s_obj_pending;
static ProtoFrame  s_qr_frame, s_obj_frame;
static volatile int s_abort;
static volatile int s_obj_want_cls = -1, s_obj_want_label = -1;

/* Drop old frames at a new OBJ phase; filter before overwriting the latest slot. */
static void select_object(int cls, int label)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    s_obj_want_cls = cls; s_obj_want_label = label; s_obj_pending = 0;
    __set_PRIMASK(pm);
}

/* 视觉帧写入口(RxCplt 回调里被调):把整帧压进单槽并置 pending(关中断防与 take_frame 竞争) */
void steps_feed_frame(const ProtoFrame *f)
{
    uint32_t pm = __get_PRIMASK();
    if (!f || (f->type != PF_QR && f->type != PF_OBJ)) return;
    __disable_irq();
    if (f->type == PF_QR) {
        s_qr_frame = *f;
        s_qr_pending = 1;
    } else {
        if ((s_obj_want_cls >= 0 && f->cls != s_obj_want_cls)
            || (s_obj_want_label >= 0 && f->label != s_obj_want_label)) {
            __set_PRIMASK(pm);
            return;
        }
        s_obj_frame = *f;
        s_obj_pending = 1;
    }
    __set_PRIMASK(pm);
}

/* 消费侧取走指定类型的最新帧；同类高帧率只保留最新值，控制不追过时目标。 */
static int take_frame(ProtoType type, ProtoFrame *out)
{
    uint32_t pm = __get_PRIMASK();
    int got = 0;
    __disable_irq();
    if (type == PF_QR && s_qr_pending) {
        *out = s_qr_frame; s_qr_pending = 0; got = 1;
    } else if (type == PF_OBJ && s_obj_pending) {
        *out = s_obj_frame; s_obj_pending = 0; got = 1;
    }
    __set_PRIMASK(pm);
    return got;
}

/* 整场中止标三连:run_abort() 置位 / run_aborted() 阻塞步每 ~5ms 轮询、见标即退 /
 * run_reset() 只能在初始化或接受启动请求前清中止标与暂存帧；g 后立即 a 的中止
 * 不得被 MissionTask 醒来时重置。当前整场中止命令为 BT 'a'。 */
void run_reset(void)
{
    s_abort = 0; s_qr_pending = 0;
    select_object(-1, -1);
}
int  run_aborted(void) { return s_abort; }
void run_abort(void)   { s_abort = 1; }

/* 阻塞等 ms(被 run_abort 打断即提前回):机械/视觉动作后"停一拍"的通用延时 */
void wait_ms(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    while (!s_abort && (uint32_t)(HAL_GetTick() - t0) < ms)
        osDelay(2);
}

/* 等一帧 PF_QR,读到把 a/b/c 写进 d 回 1;to 超时或被中止回 0(to==0 无限等) */
int wait_qr(int32_t d[3], uint32_t to)   /* to==0 → 不限时:读到 / 被中止才回 */
{
    uint32_t t0 = HAL_GetTick();
    while (!s_abort) {
        if (to && (uint32_t)(HAL_GetTick() - t0) >= to) return 0;   /* 限时到,超时 */
        ProtoFrame f;
        if (take_frame(PF_QR, &f)) {
            if (d) { d[0] = f.a; d[1] = f.b; d[2] = f.c; }
            return 1;
        }
        osDelay(5);
    }
    return 0;   /* 被中止 */
}

/* ---- 视觉对准闭环(2026-09-13 真正实现,替代原来"看到就返回 1") ----
 * 语义(用户口径)：三个要用视觉的任务都**等到对齐为止**（调用方传 to=0，不设兜底）。
 * 横向闭环把目标 cx 拉到每类实测的抓取/射击站位像素 cx_stand_px，
 * 它不等于画面中心；纵向是可关闭的像素高估距开环粗调，不是实时视觉闭环。
 * 判据：**连续 X_ALIGN_N 帧**偏差都在容差内 → 才算对准（防抖、防一帧误判）。
 * 控制律：偏差(像素) → 横移速度 vy，带死区 + 限幅（慢慢对，别猛冲过冲）。
 * ⚠️ 待实测/标定：画面宽、像素↔mm、增益/容差/帧数；**方向符号（偏右→往哪走）要台校**。 */
#define X_ALIGN_TOL_PX  8.0f     /* TODO 台校：横向偏差 ≤ 这么多像素算"对上了" */
#define X_ALIGN_N       5u       /* TODO 台校：连续这么多**有效帧**都小才算对准 */
#define X_ALIGN_KP      0.6f     /* TODO 台校：偏差(px) → 横移速度(mm/s) */
#define X_ALIGN_VMAX    80.0f    /* 横移最大速度 mm/s */
#define X_ALIGN_VMIN    12.0f    /* 死区：算出来的速度小于它就别动（防抖）*/
#define X_ALIGN_LOST_MS 300u     /* 多久没收到匹配帧算"目标丢了"→ 清计数 */

/* ---- 纵向粗调（开环算距离，见 step_align 阶段1）----
 * 针孔模型:距离 d ∝ 1/目标像素高。只要标定「站距 d_站」+「站位正确时的像素高 h_站」：
 *   d_now = d_站 × h_站 / h_现在 ;  该走多少 = d_now − d_站。
 * ⚠️ 每个任务（球/靶/人质/桶）一套，全是占位值，**待实测标定**；d_站 填 0 = 关闭纵向这一段。 */
#define X_DEPTH_TOL_MM  20.0f    /* TODO 台校：纵向差 ≤ 这么多 mm 就不动 */
#define X_DEPTH_MAX_MM  150.0f   /* 单次最多走这么多 mm（限幅，别一次冲太远）*/
#define X_DEPTH_V       100.0f   /* 纵向微调速度 mm/s（慢）*/
#define X_DEPTH_N       3u       /* 最多迭代几次（防振荡）*/

/* 站位标定表：一次标定三件事（都在"把车摆到满意站位"时量）。
 *   d_stand_mm = 目标到相机距离(mm)；h_stand_px = 那时目标像素高；cx_stand_px = 那时目标 cx。
 * ⚠️ 全占位，**待实测标定**。d_stand 填 0 = 关掉纵向粗调（只闭横向）。 */
typedef struct { float d_stand_mm; float h_stand_px; float cx_stand_px; } AlignStand;
static const AlignStand s_stand[4] = {      /* 下标 = cls：0球 1靶 2人质 3桶（proto.h）*/
    {   0.0f,   0.0f, 160.0f },   /* CLS_BALL     TODO 实测；0=先禁用纵深粗调 */
    {   0.0f,   0.0f, 160.0f },   /* CLS_TARGET   TODO 实测；0=先禁用纵深粗调 */
    {   0.0f,   0.0f, 160.0f },   /* CLS_HOSTAGE  TODO 实测；0=先禁用纵深粗调 */
    {   0.0f,   0.0f, 160.0f },   /* CLS_BUCKET   TODO 实测；0=先禁用纵深粗调 */
};

/* 等一帧 cls+label 匹配的目标帧（阻塞，被中止回 0）。纵向/横向两段都用它。 */
static int align_wait_frame(int cls, int label, ProtoFrame *out,
                            uint32_t started_ms, uint32_t timeout_ms)
{
    while (!s_abort) {
        ProtoFrame f;
        if (timeout_ms && (uint32_t)(HAL_GetTick() - started_ms) >= timeout_ms)
            return 0;
        if (take_frame(PF_OBJ, &f) && f.cls == cls
            && (label < 0 || f.label == label)) {
            *out = f;
            return 1;
        }
        osDelay(5);
    }
    return 0;
}

int step_align(int cls, int label, uint32_t to)
{
    /* 目标该落在画面哪个 cx —— 直接取标定值（**不是**画面中心，也不用知道画面宽） */
    const float cx_tgt = (cls >= 0 && cls < 4) ? s_stand[cls].cx_stand_px : 0.0f;
    uint32_t t0  = HAL_GetTick();
    uint32_t last_ok = t0;      /* 最近一次收到"匹配帧"的时刻 */
    uint8_t  ok_n = 0u;         /* 连续在容差内的有效帧数 */
    float orth0;

    if (!step_prepare_leg()) return 0;

    select_object(cls, label);

    /* ===== 阶段1:纵向粗调（开环算距离，一次走到位；最多 X_DEPTH_N 次防万一）=====
     * 针孔模型：距离 d ∝ 1/目标像素高 → d_now = d_站 × (h_站 / h_现在)。
     * 只要标定两个值：本任务的站距 d_站、站位正确时目标像素高 h_站。
     * 空调用（表里 d_站=0）就跳过这一段 → 退回"只闭横向"。 */
    {
        float d_stand = 0.0f, h_stand = 0.0f;
        if (cls >= 0 && cls < 4) {
            d_stand = s_stand[cls].d_stand_mm;
            h_stand = s_stand[cls].h_stand_px;
        }
        for (uint8_t it = 0u; (d_stand > 0.0f && h_stand > 0.0f)
                              && it < X_DEPTH_N && !s_abort; it++) {
            if (to && (uint32_t)(HAL_GetTick() - t0) >= to) { motion_brake(); return 0; }
            ProtoFrame f;
            if (!align_wait_frame(cls, label, &f, t0, to)) {
                motion_brake();
                return 0; /* 超时或被中止，不在等待匹配帧时卡死 */
            }
            if (f.h <= 0) continue;                            /* 视觉没给高度 → 这次跳过 */
            float d_now = d_stand * h_stand / (float)f.h;
            float err   = d_now - d_stand;                     /* >0 = 比站位远 → 往前走 */
            if (err > -X_DEPTH_TOL_MM && err < X_DEPTH_TOL_MM) break;   /* 纵向够了 */
            if (err >  X_DEPTH_MAX_MM) err =  X_DEPTH_MAX_MM;  /* 限幅：别一次冲太远 */
            if (err < -X_DEPTH_MAX_MM) err = -X_DEPTH_MAX_MM;
            uint32_t remain_ms = 0u;
            if (to) {
                uint32_t elapsed_ms = (uint32_t)(HAL_GetTick() - t0);
                if (elapsed_ms >= to) { motion_brake(); return 0; }
                remain_ms = to - elapsed_ms;
            }
            if (!step_straight(err, (err > 0.0f) ? X_DEPTH_V : -X_DEPTH_V,
                               remain_ms)) {
                motion_brake();
                return 0;
            }
        }
    }

    /* 纵向粗调若动过车，横向锁定前重新停稳并重设本段航向零点。 */
    if (!step_prepare_leg()) return 0;
    select_object(cls, label);
    orth0 = motion_odo_mm();

    /* ===== 阶段2:横向闭环（把 cx 拉到该类别标定的 cx_stand_px）===== */
    while (!s_abort) {
        uint32_t now = HAL_GetTick();
        if (to && (uint32_t)(now - t0) >= to) { motion_brake(); return 0; }

        ProtoFrame f;
        if (take_frame(PF_OBJ, &f) && f.cls == cls
            && (label < 0 || f.label == label)) {
            last_ok = now;
            float e = (float)f.cx - cx_tgt;          /* >0 = 目标在目标点右边 */

            if (e > -X_ALIGN_TOL_PX && e < X_ALIGN_TOL_PX) {
                motion_brake();                      /* 对上了:停住别动(别抖着过冲) */
                if (++ok_n >= X_ALIGN_N) return 1;   /* 连续 N 帧都小 → 对准完成 */
            } else {
                ok_n = 0;
                float v = -X_ALIGN_KP * e;           /* 像素误差→车体 vy，方向符号待台校 */
                if (v >  X_ALIGN_VMAX) v =  X_ALIGN_VMAX;
                if (v < -X_ALIGN_VMAX) v = -X_ALIGN_VMAX;
                if (v > -X_ALIGN_VMIN && v < X_ALIGN_VMIN) {
                    motion_brake();                  /* 死区:差一点点就别动,防抖 */
                } else {
                    if (!imu_ok()) { motion_brake(); return 0; }
                    motion_vel_set(step_orth_hold_cmd(1, orth0), v,
                                   step_heading_hold_w(0.0f));
                }
            }
        } else if ((uint32_t)(now - last_ok) > X_ALIGN_LOST_MS) {
            ok_n = 0u;
            motion_brake();                          /* 丢目标必须停，不能沿用上一拍速度盲走 */
        }
        osDelay(5);
    }
    motion_brake();
    return 0;   /* 被中止 */
}

/* 工作带内左右扫描：入口点 x0 与远端 x1 均用横向编码器里程限定，绝不按时间盲走。
 * DELTA 是“从入口到远端”的有符号距离；符号与实际左右方向一起落地测试。
 * 扫到目标立即停车交给 step_align。任务结束后 mission 会按入口基准去下一节点。 */
#define SWEEP_STR_MMS          0.0f  /* TODO 实测：扫描横移速度 mm/s */
#define SWEEP_BALL_DELTA_MM    0.0f  /* TODO 实测：排爆入口→扫描远端 */
#define SWEEP_TARGET_DELTA_MM  0.0f  /* TODO 实测：反恐入口→扫描远端 */
#define SWEEP_HOSTAGE_DELTA_MM 0.0f  /* TODO 实测：救援入口→扫描远端 */
#define SWEEP_BUCKET_DELTA_MM  0.0f  /* TODO 实测：放球点入口→扫描远端 */

static float sweep_delta_mm(int cls)
{
    switch (cls) {
        case CLS_BALL:    return SWEEP_BALL_DELTA_MM;
        case CLS_TARGET:  return SWEEP_TARGET_DELTA_MM;
        case CLS_HOSTAGE: return SWEEP_HOSTAGE_DELTA_MM;
        case CLS_BUCKET:  return SWEEP_BUCKET_DELTA_MM;
        default:          return 0.0f;
    }
}

const char *steps_config_missing(void)
{
    if (SWEEP_STR_MMS <= 0.0f) return "SWEEP_STR_MMS";
    if (SWEEP_BALL_DELTA_MM == 0.0f) return "SWEEP_BALL_DELTA_MM";
    if (SWEEP_TARGET_DELTA_MM == 0.0f) return "SWEEP_TARGET_DELTA_MM";
    if (SWEEP_HOSTAGE_DELTA_MM == 0.0f) return "SWEEP_HOSTAGE_DELTA_MM";
    if (SWEEP_BUCKET_DELTA_MM == 0.0f) return "SWEEP_BUCKET_DELTA_MM";
    return 0;
}

int step_sweep(int want, int cls, int label, int32_t d[3], uint32_t to)
{
    float x0, x1, target, orth0;
    float delta;
    uint32_t t0 = HAL_GetTick();

    /* QR 有独立的有界扫描流程；本函数只负责四类 OBJ，防止接口被误用。 */
    (void)d;
    if (want != PF_OBJ || cls < CLS_BALL || cls > CLS_BUCKET) return 0;
    delta = sweep_delta_mm(cls);
    if (SWEEP_STR_MMS <= 0.0f || delta == 0.0f) return 0;
    if (!step_prepare_leg()) return 0;
    x0 = motion_lateral_odo_mm();
    select_object(cls, label);
    orth0 = motion_odo_mm();
    x1 = x0 + delta;
    target = x1;

    while (!s_abort) {
        float here = motion_lateral_odo_mm();
        float remain = target - here;
        float dir = (remain > 0.0f) ? 1.0f : -1.0f;
        MotionRamp ramp;
        motion_linear_ramp_init(&ramp);

        while (!s_abort) {
            if (to && (uint32_t)(HAL_GetTick() - t0) >= to) { motion_brake(); return 0; }
            ProtoFrame f;
            if (take_frame(PF_OBJ, &f) && f.cls == cls
                && (label < 0 || f.label == label)) {
                motion_brake(); return 1;       /* 扫到目标,交 step_align 锁 */
            }
            if (!imu_ok()) { motion_brake(); return 0; }
            here = motion_lateral_odo_mm();
            if ((dir > 0.0f && here >= target) || (dir < 0.0f && here <= target)) break;
            remain = target - here;
            motion_vel_set(step_orth_hold_cmd(1, orth0),
                           motion_linear_profile_step(&ramp, dir * SWEEP_STR_MMS,
                                                      remain, 0.005f),
                           step_heading_hold_w(0.0f));
            osDelay(5);
        }
        motion_brake();
        if (!step_prepare_leg()) return 0;
        target = (target == x1) ? x0 : x1;
    }
    return 0;   /* 被中止 */
}

/* ---- 跨区导航腿:定距直行 + 拐点 ---- */
static float s_nav_w_kp_deg = 0.3f; /* RAM-only 航向增益种子；蓝牙 ykp 可改 */
static float s_orth_kp = 0.0f;      /* RAM-only 正交串动增益；0=台校前关闭 */
#define ORTH_VMAX_MMS 80.0f         /* 正交修正限速，防参数误设造成横冲 */
#define NAV_W_MAX_RADS 2.0f         /* 航向修正角速度限幅，防大误差/误参数瞬间打满 */
#define NAV_STILL_MS  250u   /* 四轮累计计数连续不变多久才认作停稳；待上车验证 */

/* 节点准备：不是给传感器发硬清零命令，而是记录当前连续航向作本段零点。
 * 不能清 ctrl_enc_total 或 imu_heading_deg：那会破坏累计位置/全局航向。
 * 编码器无变化只证明轮子没继续转；打滑、编码器掉线不能靠此判出，需上车核验。 */
int step_prepare_leg(void)
{
    int32_t last[4];
    uint32_t still_from = HAL_GetTick();
    motion_brake();
    for (int i = 0; i < 4; ++i) last[i] = ctrl_enc_total(i);
    while (!s_abort) {
        int changed = 0;
        for (int i = 0; i < 4; ++i) {
            int32_t now = ctrl_enc_total(i);
            if (now != last[i]) changed = 1;
            last[i] = now;
        }
        if (changed) still_from = HAL_GetTick();
        if ((uint32_t)(HAL_GetTick() - still_from) >= NAV_STILL_MS) break;
        osDelay(5);
    }
    if (s_abort || !imu_zero_leg_heading()) return 0;
    wait_ms(NAV_SETTLE_MS);
    return !s_abort && imu_ok();
}

/* 角度差归一到 [-180,180)（同 step_rotate_deg 里那段,抽出来复用） */
static float wrap180f(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/* 普通路线与落地测试共用同一套 yaw 保持，避免测试代码另复制一份增益。 */
float step_heading_hold_w(float heading0_deg)
{
    float e = wrap180f(heading0_deg - imu_leg_heading_deg());
    float w = s_nav_w_kp_deg * e * 0.0174533f;
    if (w >  NAV_W_MAX_RADS) w =  NAV_W_MAX_RADS;
    if (w < -NAV_W_MAX_RADS) w = -NAV_W_MAX_RADS;
    return w;
}

float step_heading_kp_deg(void)
{
    return s_nav_w_kp_deg;
}

int step_heading_kp_set(float kp)
{
    if (kp < 0.0f || kp > 5.0f) return 0;
    s_nav_w_kp_deg = kp;
    return 1;
}

float step_orth_kp(void)
{
    return s_orth_kp;
}

int step_orth_kp_set(float kp)
{
    if (kp < 0.0f || kp > 5.0f) return 0;
    s_orth_kp = kp;
    return 1;
}

/* 直行时约束横向里程，横移时约束前向里程；只闭本段正交误差，不需要全局 x/y。 */
float step_orth_hold_cmd(int lateral_motion, float orth_reference_mm)
{
    float now = lateral_motion ? motion_odo_mm() : motion_lateral_odo_mm();
    float cmd = -s_orth_kp * (now - orth_reference_mm);
    if (cmd >  ORTH_VMAX_MMS) cmd =  ORTH_VMAX_MMS;
    if (cmd < -ORTH_VMAX_MMS) cmd = -ORTH_VMAX_MMS;
    return cmd;
}

/* 定距直行(带 yaw 锁向):走够 dist_mm 就刹停回 1。
 * dist_mm 正=前进 / 负=后退（v_mms 符号应与 dist 同向）。
 * 里程用 motion_odo_mm()（四轮平均）—— ⚠️ 打滑时不准（正常直道还行）。
 * 每拍记进段时的朝向当基准、按 yaw 误差微调 w 防歪（打滑不骗 IMU）。
 * to==0 不限时；被 run_abort 中止回 0。 */
int step_straight(float dist_mm, float v_mms, uint32_t to)
{
    MotionRamp ramp;
    float orth0;
    if (s_abort || dist_mm == 0.0f) return !s_abort;
    if (dist_mm * v_mms <= 0.0f) return 0; /* 未填速度或方向反了，拒绝空转等待 */

    float odo0 = motion_odo_mm();
    float heading0 = 0.0f;
    if (imu_ok()) heading0 = imu_leg_heading_deg();
    uint32_t t0 = HAL_GetTick();
    orth0 = motion_lateral_odo_mm();
    motion_linear_ramp_init(&ramp);

    while (!s_abort) {
        float w = 0.0f, cmd, remain;
        float d = motion_odo_mm() - odo0;
        if (dist_mm > 0.0f ? (d >= dist_mm) : (d <= dist_mm)) {
            motion_brake();
            return 1;
        }
        if (!imu_ok()) { motion_brake(); return 0; }
        w = step_heading_hold_w(heading0);
        remain = dist_mm - d;
        cmd = motion_linear_profile_step(&ramp, v_mms, remain, 0.005f);
        motion_vel_set(cmd, step_orth_hold_cmd(0, orth0), w);
        if (to && (uint32_t)(HAL_GetTick() - t0) >= to) { motion_brake(); return 0; }
        osDelay(5);
    }
    motion_brake();
    return 0;
}

/* 定距横移：车头方向不变，正距离=右移、负距离=左移。
 * 按四轮编码器的横向投影判本段位移；不依赖尚未放开的 pose x/y。
 * 打滑、轮位/方向/CPR 未台校会带来误差，不等于绝对位置确认。 */
int step_strafe(float dist_mm, float v_mms, uint32_t to)
{
    MotionRamp ramp;
    float orth0;
    float odo0;
    float heading0 = 0.0f;
    uint32_t t0;

    if (s_abort || dist_mm == 0.0f) return !s_abort;
    if (dist_mm * v_mms <= 0.0f) return 0;

    odo0 = motion_lateral_odo_mm();
    if (imu_ok()) heading0 = imu_leg_heading_deg();
    t0 = HAL_GetTick();
    orth0 = motion_odo_mm();
    motion_linear_ramp_init(&ramp);

    while (!s_abort) {
        float w = 0.0f, d, cmd, remain;
        d = motion_lateral_odo_mm() - odo0;
        if (dist_mm > 0.0f ? (d >= dist_mm) : (d <= dist_mm)) {
            motion_brake();
            return 1;
        }
        if (!imu_ok()) { motion_brake(); return 0; }
        w = step_heading_hold_w(heading0);
        remain = dist_mm - d;
        cmd = motion_linear_profile_step(&ramp, v_mms, remain, 0.005f);
        motion_vel_set(step_orth_hold_cmd(1, orth0), cmd, w);
        if (to && (uint32_t)(HAL_GetTick() - t0) >= to) { motion_brake(); return 0; }
        osDelay(5);
    }
    motion_brake();
    return 0;
}

/* 任务区内恢复扫描基准：先停稳再计算余量，避免把制动余动漏到账外。
 * 使用同一个 SWEEP_STR_MMS，确保任务层不再私藏另一套未标定速度。 */
int step_return_lateral_odo(float target_mm, uint32_t to)
{
    float remain;
    if (SWEEP_STR_MMS <= 0.0f || !step_prepare_leg()) return 0;
    remain = target_mm - motion_lateral_odo_mm();
    if (remain > -0.5f && remain < 0.5f) return !s_abort;
    return step_strafe(remain,
                       (remain > 0.0f) ? SWEEP_STR_MMS : -SWEEP_STR_MMS,
                       to);
}

/* 拐点的一条导航腿：停车稳 NAV_SETTLE_MS → 相对当前朝向转 turn_deg° → 直行 dist_mm。
 * turn_deg / dist_mm 传 0 就跳过那一步（允许"纯直行"或"只转向"）。
 * 基准不用显式置零：step_rotate_deg 本身就是相对当前、step_straight 自己记 heading。 */
int step_nav_leg(float turn_deg, float dist_mm, float v_mms, uint32_t to)
{
    if (!step_prepare_leg()) return 0;
    if (turn_deg != 0.0f && !step_rotate_deg((int)turn_deg, to)) return 0;
    if (turn_deg != 0.0f && dist_mm != 0.0f && !step_prepare_leg()) return 0;
    if (dist_mm  != 0.0f && !step_straight(dist_mm, v_mms, to)) return 0;
    return !s_abort;
}

/* ---- 原地旋转(麦轮绕自身中心自转,EOD 抓球放桶转 180° 用):转的判据 = IMU yaw ---- */
#define ROT_SPIN_RADS  2.0f   /* 最大角速度 rad/s，待实测 */
#define ROT_MIN_RADS   0.12f  /* 克服静摩擦的最小角速度，待实测 */
#define ROT_KP_RADS_DEG 0.02f /* 航向误差(deg)→角速度(rad/s)，待实测 */
#define ROT_TOL_DEG    1.0f   /* 用户 2026-09-24：停稳后误差必须≤1° */
#define ROT_SETTLE_MS  700u   /* 抱闸后稳定观察；若惯性越界则继续修正 */
#define ROT_STILL_DEG  0.2f   /* 观察窗内若仍变化>0.2°，重新计稳定时间 */

/* Only +90 deg uses the on-ground validated mode-20 profile. The caller
 * prepares/stops/zeros the leg first; continuous heading avoids wraparound.
 * Do not silently apply this tune to untested left turns or EOD 180 deg. */
static int step_rotate_right90_validated(uint32_t to)
{
    float heading0 = imu_heading_deg();
    uint32_t t0 = HAL_GetTick();
    uint32_t settle_t0 = 0u;
    float settle_yaw = 0.0f;
    int settling = 0;
    while (!s_abort) {
        uint32_t now = HAL_GetTick();
        float yaw, e, w;
        if (!imu_ok() || (to && (uint32_t)(now - t0) >= to)) {
            motion_brake();
            return 0;
        }
        yaw = imu_heading_deg() - heading0;
        if (yaw > TURN90_TARGET_DEG + TURN90_LIMIT_DEG || yaw < -TURN90_LIMIT_DEG) {
            motion_brake();
            return 0;
        }
        e = TURN90_TARGET_DEG - yaw;
        if (fabsf(e) <= TURN90_TOL_DEG) {
            if (!settling) {
                motion_brake();
                settle_t0 = now;
                settle_yaw = yaw;
                settling = 1;
            } else {
                if (fabsf(yaw - settle_yaw) > TURN90_STILL_DEG) {
                    settle_t0 = now;
                    settle_yaw = yaw;
                }
                if ((uint32_t)(now - settle_t0) >= TURN90_SETTLE_MS) return 1;
            }
            osDelay(5);
            continue;
        }
        settling = 0; /* brake drifted outside tolerance: correct and settle again */
        if ((uint32_t)(now - t0) >= TURN90_MAX_MS) {
            motion_brake();
            return 0;
        }
        w = TURN90_KP_RADS_DEG * e;
        if (w > TURN90_MAX_W_RADS) w = TURN90_MAX_W_RADS;
        if (w < -TURN90_MAX_W_RADS) w = -TURN90_MAX_W_RADS;
        if (w > 0.0f && w < TURN90_MIN_W_RADS) w = TURN90_MIN_W_RADS;
        if (w < 0.0f && w > -TURN90_MIN_W_RADS) w = -TURN90_MIN_W_RADS;
        motion_vel_set(0.0f, 0.0f, w);
        osDelay(5);
    }
    motion_brake();
    return 0;
}

/* 相对当前航向原地转 deg°(有符号,纯 w 自转)→ 抱闸后稳定在 target±1° 回 1,
 * 超时/中止回 0。真判据 = IMU yaw(闭环)；IMU 未接/无有效帧回 0，不假报转完。
 * 非 +90 分支也用连续航向的相对增量；±180°若用 0..360 yaw 的最短角差，
 * 起转前仅 -1° 的噪声就可能把 +180°判成 -179°，从第一拍反向转。
 * 每拍按剩余误差重算正负方向；即使过冲也会反向收敛。 */
int step_rotate_deg(int deg, uint32_t to)
{
    if (s_abort) return 0;
    if (deg == 0) return 1;
    if (!imu_ok()) { motion_brake(); return 0; }   /* 无 IMU 不得假报已转到 */
    if (deg == 90) return step_rotate_right90_validated(to);
    float heading0 = imu_heading_deg();
    float target = (float)deg;
    uint32_t t0 = HAL_GetTick();
    uint32_t settle_t0 = 0u;
    float settle_yaw = 0.0f;
    int settling = 0;
    while (!s_abort) {
        float e, w, yaw;
        if (!imu_ok()) { motion_brake(); return 0; }
        if (to && (uint32_t)(HAL_GetTick() - t0) >= to) { motion_brake(); return 0; }
        yaw = imu_heading_deg() - heading0;
        e = target - yaw;
        if (e <= ROT_TOL_DEG && e >= -ROT_TOL_DEG) {
            if (!settling) {
                motion_brake();
                settle_t0 = HAL_GetTick();
                settle_yaw = yaw;
                settling = 1;
            } else {
                float dy = yaw - settle_yaw;
                if (dy > ROT_STILL_DEG || dy < -ROT_STILL_DEG) {
                    settle_t0 = HAL_GetTick();
                    settle_yaw = yaw;
                }
                if ((uint32_t)(HAL_GetTick() - settle_t0) >= ROT_SETTLE_MS)
                    return 1;
            }
            osDelay(5);
            continue;
        }
        settling = 0; /* 抱闸后越界，继续按误差正负修正 */
        w = ROT_KP_RADS_DEG * e;
        if (w >  ROT_SPIN_RADS) w =  ROT_SPIN_RADS;
        if (w < -ROT_SPIN_RADS) w = -ROT_SPIN_RADS;
        if (w > 0.0f && w <  ROT_MIN_RADS) w =  ROT_MIN_RADS;
        if (w < 0.0f && w > -ROT_MIN_RADS) w = -ROT_MIN_RADS;
        motion_vel_set(0.0f, 0.0f, w);
        osDelay(5);
    }
    motion_brake();
    return 0;   /* 被中止 */
}

/* ---- 抓取/松爪：机械轴驱动共用，球/人质的行程在各自任务中标定 ---- */
#define ARM_STEP_INTERVAL_MS 20u /* 安全种子=约50 step/s；实测不丢步后再提高 */

/* 只记录软件命令，不把 STEP 脉冲数或 PWM 指令冒充机构实际到位。由 MissionTask
 * 单线程调用；静态缓冲避免占用其 1KB 栈，bp_debug_send 会原子复制整行到蓝牙队列。 */
static void arm_stage_report(const char *task, const char *stage,
                             const char *phase, uint32_t steps)
{
    static char b[144];
    snprintf(b, sizeof b,
             "REC type=ARM task=%s stage=%s phase=%s req_steps=%lu physical_unverified=1\r\n",
             task, stage, phase, (unsigned long)steps);
    bp_debug_send(b);
}

static void arm_pulse_report(int axis, int dir, uint32_t requested,
                             uint32_t sent, const char *status)
{
    static char b[144];
    snprintf(b, sizeof b,
             "REC type=ARM_PULSE axis=%d dir=%d req=%lu sent=%lu status=%s physical_unverified=1\r\n",
             axis, dir, (unsigned long)requested, (unsigned long)sent, status);
    bp_debug_send(b);
}

static void arm_claw_report(const char *command)
{
    static char b[112];
    snprintf(b, sizeof b,
             "REC type=ARM_CLAW cmd=%s us=%u physical_unverified=1\r\n",
             command, (unsigned)arm_claw_command_us());
    bp_debug_send(b);
}

int step_arm_prepare(const char *task, const char *stage)
{
    arm_stage_report(task, stage, "STILL_START", 0u);
    if (!step_prepare_leg()) {
        arm_stage_report(task, stage, "STILL_FAIL", 0u);
        return 0;
    }
    arm_stage_report(task, stage, "STILL_READY", 0u);
    return 1;
}

int step_arm_run(const char *task, const char *stage, uint32_t steps,
                 int (*action)(uint32_t))
{
    if (!action || steps == 0u || s_abort) {
        arm_stage_report(task, stage, "REJECTED", steps);
        return 0;
    }
    arm_stage_report(task, stage, "START", steps);
    if (!action(steps)) {
        arm_stage_report(task, stage, "STOP_POS_UNCERTAIN", steps);
        return 0;
    }
    arm_stage_report(task, stage, "COMMAND_SENT", steps);
    return 1;
}

int step_arm_release(const char *task, const char *stage)
{
    if (s_abort) {
        arm_stage_report(task, stage, "REJECTED", 0u);
        return 0;
    }
    arm_stage_report(task, stage, "START", 0u);
    if (!step_release()) {
        arm_stage_report(task, stage, "STOP_POS_UNCERTAIN", 0u);
        return 0;
    }
    arm_stage_report(task, stage, "COMMAND_SENT", 0u);
    return 1;
}

/* mission 侧步进必须能被蓝牙 a 中止；每步后显式留高电平间隔并让出 CPU。 */
static int stepper_move_safe(int axis, int dir, uint32_t steps)
{
    arm_stepper_dir(axis, dir);
    for (uint32_t i = 0; i < steps; ++i) {
        if (s_abort) {
            arm_pulse_report(axis, dir, steps, i, "STOP_PARTIAL");
            return 0;
        }
        arm_stepper_step(axis);
        osDelay(ARM_STEP_INTERVAL_MS);
    }
    arm_pulse_report(axis, dir, steps, steps,
                     s_abort ? "STOP_AFTER_LAST_PULSE" : "ALL_PULSES_SENT");
    return !s_abort;
}

/* 抓取时序:开爪→axis0伸入该任务的实测行程→闭爪咬合即停,不自动收。
 * 咬住后的升降由各任务分别标定，本函数只负责咬住;
 * 任一步被中止即失败。 */
int step_grasp(uint32_t extend_steps)
{
    if (s_abort || extend_steps == 0u) return 0;
    arm_claw_open();
    arm_claw_report("OPEN");
    wait_ms(300);
    if (s_abort) return 0;
    if (!stepper_move_safe(0, 1, extend_steps)) return 0;
    wait_ms(200);
    if (s_abort) return 0;
    arm_claw_close();
    arm_claw_report("CLOSE");
    wait_ms(300);
    return !s_abort;
}

/* 松爪:仅开爪等一拍(现只 EOD 放桶用——降进桶口后开爪;救援不放了,人质咬着带回) */
int step_release(void)
{
    if (s_abort) return 0;
    arm_claw_open();
    arm_claw_report("OPEN");
    wait_ms(200);
    return !s_abort;
}

/* 已知完整伸出行程后的反向回程。机械挡块没有电信号，丢步、中止或断电后
 * 不能仅凭这次步数认定到原点，须重新人工确认起点。 */
int step_rack_retract(uint32_t steps)
{
    if (s_abort || steps == 0u) return 0;
    if (!stepper_move_safe(0, 0, steps)) return 0;
    wait_ms(200);
    return !s_abort;
}

/* ---- axis1 升降铰链(垂直):先抬后转、降进桶口放、救援抬离台,共用这一对 ----
 * 方向位 dir 0/1 ↔ 升/降 的对应机械方向台上校(和 test 11/12 伸向一致,翻了翻这里);
 * steps 为要走的步数(种子在调用方/实测表,见 fw/实测值清单.md)。 */
#define AXIS1_UP_DIR   1u   /* axis1 dir:伸出(升)方向位——机械方向台上校 */
#define AXIS1_DOWN_DIR 0u   /* axis1 dir:收回(降)方向位,与 UP 反向 */

int step_arm_lift(uint32_t steps)
{
    if (s_abort) return 0;
    if (!stepper_move_safe(1, (int)AXIS1_UP_DIR, steps)) return 0;
    wait_ms(200);                 /* 抬完停一拍,让机械稳(也防马上转体带起的回弹) */
    return !s_abort;
}

int step_arm_lower(uint32_t steps)
{
    if (s_abort) return 0;
    if (!stepper_move_safe(1, (int)AXIS1_DOWN_DIR, steps)) return 0;
    wait_ms(200);
    return !s_abort;
}

/* ---- 激光触发 ---- */
/* 激光开枪:点亮 hold_ms 后自动灭;期间被中止则不点亮直接失败 */
int step_fire(uint32_t hold_ms)
{
    if (s_abort) return 0;
    bp_laser_set(1);
    wait_ms(hold_ms);
    bp_laser_set(0);
    return !s_abort;
}
