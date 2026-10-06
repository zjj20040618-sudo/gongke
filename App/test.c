#include "test.h"
#include "board_pins.h"    /* bp_debug_send, MOTOR_PWM_PERIOD */
#include "control.h"       /* ctrl_set_speed */
#include "motion.h"        /* motion_vel_set / motion_brake */
#include "arm.h"           /* arm_claw_open/close, arm_stepper_dir/step */
#include "imu.h"           /* imu_yaw/pitch/roll_deg, imu_ok */
#include "steps.h"         /* run_abort */
#include "test_config.h"   /* bench-only defaults; 90/180 hold profile is shared */
#include "route_test_plan.h" /* private31 direct route; legacy34 bucket route */
#include "bucket36_plan.h"   /* route34 bucket alignment plus isolated36/37 plans */
#include "mission.h"       /* mission_state / mission_start / MS_BOOT */
#include "mission_trial.h" /* mode32 runs only in MissionTask */
#include "mission_trial_plan.h"
#include "vision_align_test.h" /* isolated38..41: nonblocking XY and heading recheck */
#include "robot.h"         /* robot_diag_report */
#include "main.h"          /* HAL_GetTick */
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ===== 台上调试执行器(v3,见 fw/bluetooth-test.md) =====
 * 主程序 = 默认;调试 = 数字选号进入。g 是唯一的走/停/复位键,按一圈 = 一轮:
 *   g1 启动 → g2 暂停+自动补终点动作(刹停/爪合)+报 done,然后车停住不动
 *   (这一段就是给你"看暂停得准不准"的)→ g3 回原点/清态,可再下一轮。
 * 1-4 持续走没有回原点那拍:g2 停完直接回就绪,g 再按 = 继续走(v2 f/x 手感)。
 * 0 / a = 中途放弃或停机；模式15/16绝不因0/a自行倒车。空闲态(没选号,MS_BOOT)
 * 按 g = 开跑主程序整场;整场运行中再次g与a一样中止，不能再g续跑。
 * 数字/参数/爪只在 MS_BOOT 服务；测试回程中g取消剩余回程。
 * 参数槽全局一份；仅模式17/18选号时装入 v300/d1500 测试默认值，
 * 其他依赖参数的模式没设槽 → ERR 先设,不瞎跑。
 * 所有动作都 ~20ms 可中断；步进每 tick 单发一脉冲：
 *   段行走= 时间折算(mm/v),步进= 每 tick 单发一脉冲自己数步,保证 g2 随时抓得住。 */

#if BENCH_AUTO
volatile int32_t g_bench_cnt[4];     /* 编码器校正后计数 */
volatile int32_t g_bench_raw_cnt[4]; /* 编码器硬件原始计数 */
volatile int     g_bench_wheel;      /* 当前轮 0..3 */
static uint32_t  s_bench_t0;
static int       s_bench_phase;      /* 0=起测 1=转中 2=刹停等待 3=完成 */
static uint32_t  s_bench_boot;
static uint8_t   s_bench_boot_ok;
#endif

/* 一轮状态机 */
enum { R_READY = 0, R_RUN, R_DONE, R_RET, R_BRAKE, R_ALIGN };
#define R_FREE 0   /* s_msel=0:没进任何模式,空闲态 */

/* 模式 11/12 的轴"伸"方向位(dir 0/1 ↔ 机械伸缩,台上校,翻这里;0 复位会反向收) */
static const uint8_t s_ext_dir[ARM_STEPPER_NUM] = { 1u, 1u };
static const char *const s_mname[T_MODE_MAX + 1u] = {
    "", "forward", "backward", "left", "right", "timed_forward", "timed_left",
    "wheel0", "wheel1", "wheel2", "wheel3", "stepper0", "stepper1",
    "encoder_manual", "imu_stats",
    "dist_forward", "dist_backward", "dist_left", "dist_right",
    "turn_sign", "turn_90_hold", "turn_speed_probe", "turn_180_hold",
    "forward_sign_probe", "jog_axis0", "jog_axis1",
    "jog_return_axis0", "jog_return_axis1", "servo_return", "servo_hold",
    "turn_left_90_hold", "route_only_sequence", "no_arm_single_pass_mission",
    "vision_receive_only", "route_only_no_qr_sequence", "qr_target_aim_fire",
    "bucket_anchor_route", "cross_reverse_only",
    "qr_ball_xy", "qr_bucket_xy", "qr_hostage_xy", "qr_ball_turn_bucket_xy"
};

static char     s_line[T_LINE_MAX + 1u];
static uint8_t  s_len;
static uint8_t  s_over;
static uint32_t s_last;

static float    s_v = -1.0f;   /* 槽:巡航 mm/s(模式1-6);-1=未设 */
static float    s_left_ff_ratio = T_LEFT_FF_SEED; /* RAM-only: 正=向车尾补偿，负=向车头 */
static float    s_right_ff_ratio = T_RIGHT_FF_SEED; /* RAM-only: 正=右移时向车头补偿 */
static float    s_forward_ff_ratio = T_FORWARD_FF_SEED; /* RAM-only: 正=前进向车头左侧补偿 */
static float    s_backward_ff_ratio = T_BACKWARD_FF_SEED; /* 正=后退向车身左侧补偿，不按面向车尾判断 */
static float    s_mission_forward_ff_ratio = T_MISSION_FORWARD_FF_SEED; /* mode32保持旧槽，不暗中继承台测候选 */
static float    s_route_forward_ff_ratio = ROUTE_TEST_FORWARD_FF_SEED; /* mode31独立槽，fff直接写实际系数 */
static float    s_route_backward_ff_ratio = T_BACKWARD_FF_SEED;
static float    s_route_left_ff_ratio = ROUTE_TEST_LEFT_FF_SEED;
static float    s_route_right_ff_ratio = ROUTE_TEST_RIGHT_FF_SEED;
static uint8_t  s_dist_align_on = 1u; /* yfix0/1: normal DONE only, RAM-only */
static float    s_route_heading_kp = ROUTE_TEST_HEADING_KP_SEED; /* mode31独立ykp，不覆盖单测/32全局值 */
/* Independent manual distance trials only. Empty speed=0; kp=0 is valid.
 * No guessed gains/interpolation, no Flash writes, no changes to global yaw. */
typedef struct { uint16_t speed; uint8_t mode; float kp; } HeadingKpProfile;
static HeadingKpProfile s_heading_profiles[T_YKP_PROFILE_SLOTS];
static int32_t  s_jog_request; /* 24..27：有符号有限步数，选模式时归零；无电气限位 */
static uint32_t s_jog_done;
static uint32_t s_jog_hold_t0; /* 26/27：正向到点后2s等待起点；等待中g只停不回 */
static uint16_t s_servo_target_us; /* 28/29: u<us>设槽，选号时清零 */
static uint16_t s_servo_origin_us; /* 28:本轮动作前命令脉宽，不是实际舵机角度 */
static uint32_t s_servo_hold_t0;
static float    s_d = -1.0f;   /* 槽:段长 mm(模式5/6) */
static float    s_p = -1.0f;   /* 槽:单轮直通 duty(模式7-10) */
static uint8_t  s_p_valid;     /* p 可为负数，不能再用 -1 代表未设 */
static int      s_go;          /* 按过 g(空闲态)开跑主程序 = 调试全锁 */

static int      s_msel;        /* 当前调试模式 0=空闲 */
static int      s_round;       /* R_* */
static uint8_t  s_route_leg;   /* 0=普通台测；1..3=无障碍路线逐段台测 */
static float    s_rvx, s_rvy;  /* 1-6 走行单位方向(±1) */
static uint32_t s_leg_start;   /* 段走/回程开始 tick */
static uint32_t s_leg_ms;      /* 段走全时长 ms(到点自动停用) */
static uint32_t s_leg_run;     /* 本轮实际走行 ms(g2/到点停/急停时记;回程按它等长返;回完清零) */
static int      s_wheel;       /* 7-10 当前轮 */
static int      s_axis;        /* 11/12 当前轴 */
static int      s_axis_dir;    /* 该轴伸出方向位(g1 定,回程取反) */
static uint32_t s_steps;       /* 本轮已伸步数(回完清零) */
static uint32_t s_back;        /* 回程已收步数 */
static uint8_t  s_imu_mon;     /* 'imu' 开:BOOT 空闲连续刷 IMU 读数 */
static uint32_t s_mon_last;    /* 上帧 IMU 读数时刻 */
static uint32_t s_meas_t0;     /* 模式13/14 本轮测量起点 */
static uint32_t s_enc_report_t0;
static uint32_t s_enc_report_seq;
static uint32_t s_test_seq;     /* 关键测量轮次：每次上电从 1 递增，REC 统一携带 */
static uint32_t s_active_test;  /* 当前正在执行并将写入 REC 的 test 编号 */
static uint32_t s_imu_n;       /* 模式14 有效采样数(test_poll 约50Hz) */
static uint32_t s_imu_bad;     /* 模式14 imu_ok=0 的轮询次数 */
static float    s_imu_y_min, s_imu_y_max;
static float    s_imu_p_min, s_imu_p_max;
static float    s_imu_r_min, s_imu_r_max;
static float    s_dist_odo0;     /* 15..18 本轮对应轴编码器里程起点 */
static float    s_dist_heading0; /* 15..18 本轮航向保持目标 */
static float    s_dist_orth0;    /* 15..18 本轮正交轴里程基准 */
static float    s_dist_target;   /* 带符号目标距离 */
static float    s_dist_ff_ratio; /* 本轮独立方向前馈快照，直走不限v100/v200 */
static float    s_dist_heading_kp; /* 本轮方向/巡航速度档的增益快照 */
static uint8_t  s_dist_heading_profile, s_dist_precise;
static uint32_t s_dist_run_ms;   /* 发出刹车命令前的运行时间 */
static uint32_t s_dist_trace_t0;
static float s_dist_brake_mm;    /* 刹车瞬间编码器段里程，用于分离制动过冲与尺度误差 */
static float s_dist_brake_yaw;   /* 刹车瞬间本段航向 */
static uint32_t s_dist_brake_t0;
static uint32_t s_dist_still_t0;
static int32_t  s_dist_last[4];
static uint8_t  s_dist_reason;   /* 0=人工停止，1=到目标，2=IMU失效，3=纠角超时，4=中止 */
static uint8_t  s_dist_align_enabled, s_dist_align_started, s_dist_align_hold, s_dist_end_saved;
static uint32_t s_dist_align_t0, s_dist_align_stable_t0;
static float    s_dist_align_stable_yaw, s_dist_pre_align_yaw, s_dist_end_mm;
static int32_t  s_dist_end_counts[4]; /* translation counts BEFORE in-place yaw correction */
static char     s_dist_report[512];
static MotionRamp s_dist_ramp;
static uint32_t s_turn_settle_t0;
static float s_turn_settle_yaw;
static const char *s_turn_result;
static uint32_t s_turn_trace_t0;
static uint32_t s_speed_probe_trace_t0;
static uint32_t s_fwd_phase_t0, s_fwd_trace_t0;
static uint8_t s_fwd_phase;       /* 0=open-loop, 1=braked gap, 2=closed-loop */
static int32_t s_fwd_open_count[4];
/* Routes share the executor/local tuning, not31's private recipe.
 * Only mode31 waits for QR; only legacy34 has bucket/manual-d stages.
 * The owner stays stable while s_msel selects a distance/turn submode.
 * Distance legs use fractional
 * RPM so the low-speed tail is not truncated to zero. Only31 overrides
 * right-angle targets to90; other bench turn profiles stay unchanged.
 * Preparation is non-blocking so g can cancel even between legs. */
enum { SQ_OFF = 0, SQ_READY, SQ_STILL, SQ_WAIT, SQ_RUN, SQ_DONE, SQ_STOPPED,
       SQ_QR_WAIT, SQ_BUCKET_ALIGN, SQ_MANUAL_D_WAIT };
static volatile uint8_t s_seq_state; /* DefaultTask owns route progression; proto owns the QR cache. */
static uint8_t s_seq_stage, s_seq_prepared;
#define ROUTE_NO_QR_MODE 34
static uint8_t s_seq_mode; /* owner: ROUTE_TEST_MODE or ROUTE_NO_QR_MODE */
static uint32_t s_seq_run, s_seq_still_t0, s_seq_wait_t0;
static int32_t s_seq_last[4];
static int32_t s_seq_qr[3];
static uint32_t s_seq_qr_report_t0;

/* Mode33 is receive-only. The ISR caches each selected class separately;
 * packet_index distinguishes a missing target in the latest (even empty) packet. */
#define VISION_DIAG_MODE 33
enum { VD_OFF = 0, VD_QR_WAIT, VD_OBJECT_WAIT, VD_OBJECT };
typedef struct {
    ProtoFrame frame;
    uint32_t tick, seen, packet_index;
} VisionDiagSample;
static volatile uint8_t s_vdiag_phase, s_vdiag_qr_pending;
static int32_t s_vdiag_qr[3];
static VisionDiagSample s_vdiag_sample[4];
static uint32_t s_vdiag_report_t0;
static ProtoStats s_vdiag_base;

/* Mode35 owns wheels/laser in DefaultTask, never blocking Bluetooth stop.
 * RX only caches the selected fresh target; a new empty packet invalidates it.
 * No QR => no motion, even if a target happened to arrive earlier. */
#define TARGET_TRIAL_MODE 35
enum { TA_OFF = 0, TA_READY, TA_QR_WAIT, TA_TASK_WAIT, TA_STILL,
       TA_PREP_WAIT, TA_SEEK, TA_SETTLE, TA_FIRE, TA_DONE, TA_STOPPED };
static volatile uint8_t s_target35_phase;
static int32_t s_target35_qr[3], s_target35_last_counts[4];
static VisionDiagSample s_target35_sample;
static uint32_t s_target35_phase_t0, s_target35_report_t0, s_target35_packet;
static uint16_t s_target35_sequence;
static uint8_t s_target35_good, s_target35_seen;
static float s_target35_heading, s_target35_kp, s_target35_cmd;

/* Independent38..41 retain their owner while mode22 runs the shared +180.
 * QR and all waits stay in DefaultTask; never call a blocking step here. */
enum { XT_OFF = 0, XT_READY, XT_QR_WAIT, XT_ACTIVE, XT_TURN_STILL,
       XT_TURN_WAIT, XT_TURN_RUN, XT_DONE, XT_STOPPED };
static volatile uint8_t s_xy_state, s_xy_owner; /* RX callback observes ownership. */
static int32_t s_xy_qr[3], s_xy_last_counts[4];
static uint32_t s_xy_phase_t0, s_xy_report_t0, s_xy_test;
static VisionAlignTestStatus s_xy_snapshot; /* keep snapshots off the task stack */

/* Mode36 owns an independent bucket sample. No QR, arm or laser action.
 * Missing/empty/stale coordinates stop, never authorize the manual back leg. */
enum { BA_OFF = 0, BA_STILL, BA_PREP_WAIT, BA_SEEK, BA_SETTLE };
static volatile uint8_t s_bucket36_phase;
static VisionDiagSample s_bucket36_sample;
static uint32_t s_bucket36_t0, s_bucket36_report_t0, s_bucket36_packet;
static uint16_t s_bucket36_sequence, s_bucket36_back_mm;
static uint8_t s_bucket36_good, s_bucket36_seen;
static float s_bucket36_cmd;

static void run_cmd(const char *ln);
static void flush_line(void);
static void tick(void);
static void send(const char *s);
static void format_deg2(char out[20], float deg);
static void jog_stop(const char *phase);
static void servo_stop(void);
static void cmd_abort(void);
static void mode_start(void);
static void cmd_param_report(void);
static int bench_ok(void);
static int route_seq_active(void);
static int route_seq_selected(void);
static void route_seq_poll(void);
static void route_seq_g(void);
static void route_seq_end(const char *status);
static void route_seq_next(void);
static unsigned route_seq_stage_count(void);
static const RouteTestLeg *route_seq_leg(void);
static int route_seq_bucket_enabled(void);
static unsigned route_seq_bucket_align_stage(void);
static unsigned route_seq_bucket_back_stage(void);
static void bucket36_begin(void);
static void bucket36_poll(void);
static void bucket36_report(void);
static void vision_diag_start(void);
static void vision_diag_stop(void);
static void vision_diag_poll(void);
static void vision_diag_report(void);
static void vision_wire_report(void);
static int target35_active(void);
static void target35_g(void);
static void target35_poll(void);
static void target35_stop(const char *status);
static void target35_report(void);
static int xy_trial_selected(void);
static int xy_trial_active(void);
static void xy_trial_g(void);
static void xy_trial_stop(const char *reason);
static void xy_trial_poll(void);
static void xy_trial_report(void);
static void xy_trial_turn_done(int success);

static void begin_recorded_test(void)
{
    s_test_seq++;
    if (s_test_seq == 0u) s_test_seq = 1u; /* 理论回绕时也不使用 0 */
    s_active_test = s_test_seq;
}

void test_vision_feed_frame(const ProtoFrame *f)
{
    ProtoStats stats;
    VisionDiagSample *sample;
    if (!f) return;
    if (xy_trial_active()) vision_align_test_feed_frame(f);
    if (route_seq_bucket_enabled() && s_seq_state == SQ_BUCKET_ALIGN &&
        s_bucket36_phase != BA_OFF && f->type == PF_OBJ && f->cls == CLS_BUCKET) {
        proto_stats_get(&stats);
        s_bucket36_sample.frame = *f;
        s_bucket36_sample.tick = HAL_GetTick();
        s_bucket36_sample.packet_index = stats.obj;
        s_bucket36_sample.seen++;
    }
    if (s_target35_phase >= TA_TASK_WAIT && s_target35_phase <= TA_SETTLE &&
        f->type == PF_OBJ && f->cls == CLS_TARGET && f->label == s_target35_qr[1] - 1) {
        proto_stats_get(&stats);
        s_target35_sample.frame = *f;
        s_target35_sample.tick = HAL_GetTick();
        s_target35_sample.packet_index = stats.obj;
        s_target35_sample.seen++;
    }
    /* The parser independently caches the startup QR for route31/32. This
     * callback remains a receive-only mode33 diagnostic cache. */
    if (s_vdiag_phase == VD_OFF) return;
    if (s_vdiag_phase == VD_QR_WAIT) {
        if (f->type == PF_QR && !s_vdiag_qr_pending &&
            f->a >= 1 && f->a <= 3 && f->b >= 1 && f->b <= 3 &&
            f->c >= 1 && f->c <= 3) {
            s_vdiag_qr[0] = f->a; s_vdiag_qr[1] = f->b; s_vdiag_qr[2] = f->c;
            s_vdiag_qr_pending = 1u;
        }
        return;
    }
    if (f->type != PF_OBJ || f->cls < CLS_BALL || f->cls > CLS_BUCKET) return;
    if ((f->cls == CLS_BALL && f->label != s_vdiag_qr[0] - 1) ||
        (f->cls == CLS_TARGET && f->label != s_vdiag_qr[1] - 1) ||
        (f->cls == CLS_HOSTAGE && f->label != s_vdiag_qr[2] + 2)) return;
    proto_stats_get(&stats);
    sample = &s_vdiag_sample[f->cls];
    sample->frame = *f;
    sample->tick = HAL_GetTick();
    sample->packet_index = stats.obj;
    sample->seen++;
}

static void vision_diag_start(void)
{
    uint32_t pm;
    motion_brake(); bp_laser_set(0);
    begin_recorded_test();
    pm = __get_PRIMASK(); __disable_irq();
    memset(s_vdiag_sample, 0, sizeof s_vdiag_sample);
    memset(s_vdiag_qr, 0, sizeof s_vdiag_qr);
    s_vdiag_qr_pending = 0u;
    proto_stats_get(&s_vdiag_base);
    proto_send_scene(SCENE_QR); /* Queue only; DefaultTask owns actual binary TX. */
    s_vdiag_phase = VD_QR_WAIT;
    __set_PRIMASK(pm);
    s_round = R_RUN;
    s_vdiag_report_t0 = HAL_GetTick();
    robot_diag_report(); /* automatic FW/UART/parameter provenance */
    static char b[112];
    snprintf(b, sizeof b, "VD33 test=%lu START QR_WAIT no_motion=1 laser=0; next_g/a/0 stops",
             (unsigned long)s_active_test);
    send(b);
}

static void vision_diag_stop(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    s_vdiag_phase = VD_OFF; s_vdiag_qr_pending = 0u;
    memset(s_vdiag_sample, 0, sizeof s_vdiag_sample);
    memset(s_vdiag_qr, 0, sizeof s_vdiag_qr);
    step_vision_receive_end(); /* Ignore further coordinates locally; camera keeps running. */
    proto_qr_cancel(); /* Manual stop also clears a pending one-shot QR notice. */
    __set_PRIMASK(pm);
    motion_brake(); bp_laser_set(0);
    s_round = R_READY; /* Keep mode33 selected: next g cannot launch the mission. */
    static char b[100];
    snprintf(b, sizeof b, "VD33 test=%lu LOCAL_RX_STOP no_motion=1; g starts_new_QR_test",
             (unsigned long)s_active_test);
    send(b);
}

/* Same low-rate evidence for receive-only mode33 and stationary route31 wait.
 * No UART formatting or transmission in the RX callback. */
static void vision_wire_report(void)
{
    static ProtoWireDiag wire;
    static char b[320], hex[2u * PROTO_WIRE_PREFIX_LEN + 1u];
    static const char digits[] = "0123456789ABCDEF";
    proto_wire_diag_get(&wire);
    snprintf(b, sizeof b,
             "VW req=%u mode=%u task=%u sel=%u ack=%u fresh=%u rx_on=%u ignored=%lu txtry=%lu rxB=%lu ackN=%lu oldAck=%lu echoCmd=%lu legacyQR=%lu legacyOBJ=%lu preAck=%lu oldRes=%lu",
             (unsigned)wire.request, (unsigned)wire.mode,
             (unsigned)wire.task, (unsigned)wire.selection, (unsigned)wire.ack,
             (unsigned)wire.fresh, (unsigned)wire.receiving,
             (unsigned long)wire.outside_phase, (unsigned long)wire.tx_attempts,
             (unsigned long)wire.rx_bytes, (unsigned long)wire.ack_packets,
             (unsigned long)wire.ack_mismatch, (unsigned long)wire.command_echo,
             (unsigned long)wire.legacy_qr, (unsigned long)wire.legacy_obj,
             (unsigned long)wire.result_preack, (unsigned long)wire.result_mismatch);
    send(b);
    for (unsigned i = 0u; i < wire.prefix_len; i++) {
        hex[2u * i] = digits[wire.prefix[i] >> 4];
        hex[2u * i + 1u] = digits[wire.prefix[i] & 15u];
    }
    hex[2u * wire.prefix_len] = '\0';
    snprintf(b, sizeof b, "VR type=0x%02X reject=0x%02X len=%u hex=%s failed=%u",
             (unsigned)wire.last_type, (unsigned)wire.last_reject_type,
             (unsigned)wire.last_reject_len, hex, (unsigned)wire.failed);
    send(b);
}

static void vision_diag_report(void)
{
    static VisionDiagSample snapshot[4];
    static char b[192];
    static const char *const names[] = { "BALL", "TARGET", "HOSTAGE", "BUCKET" };
    static const char *const phases[] = { "OFF", "QR_WAIT", "OBJECT_WAIT", "OBJECT" };
    ProtoStats stats;
    uint32_t now = HAL_GetTick(), pm = __get_PRIMASK();
    __disable_irq();
    memcpy(snapshot, s_vdiag_sample, sizeof snapshot);
    proto_stats_get(&stats);
    __set_PRIMASK(pm);
    snprintf(b, sizeof b,
             "VD33 test=%lu phase=%s link=%d QR=%ld%ld%ld pkt=%lu bad=%lu crc=%lu unmapped=%lu",
             (unsigned long)s_active_test, phases[s_vdiag_phase], proto_scene_status(),
             (long)s_vdiag_qr[0], (long)s_vdiag_qr[1], (long)s_vdiag_qr[2],
             (unsigned long)(stats.obj - s_vdiag_base.obj),
             (unsigned long)(stats.binary_bad - s_vdiag_base.binary_bad),
             (unsigned long)(stats.crc_bad - s_vdiag_base.crc_bad),
             (unsigned long)(stats.binary_unmapped - s_vdiag_base.binary_unmapped));
    send(b);
    if (s_vdiag_phase == VD_QR_WAIT || s_vdiag_phase == VD_OBJECT_WAIT)
        vision_wire_report();
    if (s_vdiag_phase != VD_OBJECT_WAIT && s_vdiag_phase != VD_OBJECT) return;
    for (int i = 0; i < 4; i++) {
        const VisionDiagSample *sample = &snapshot[i];
        const ProtoFrame *f = &sample->frame;
        if (!sample->seen) {
            snprintf(b, sizeof b, "VD33 test=%lu cls=%s seen=0 latest=0 no_coordinate=1",
                     (unsigned long)s_active_test, names[i]);
        } else {
            snprintf(b, sizeof b,
                     "VD33 test=%lu cls=%s lab=%d seen=%lu latest=%d age=%lu seq=%u lastcx=%d lastcy=%d w=%d h=%d conf=%d img=%ux%u",
                     (unsigned long)s_active_test, names[i], f->label,
                     (unsigned long)sample->seen, sample->packet_index == stats.obj,
                     (unsigned long)(now - sample->tick), (unsigned)f->sequence,
                     f->cx, f->cy, f->w, f->h, f->conf,
                     (unsigned)f->img_w, (unsigned)f->img_h);
        }
        send(b);
    }
}

static void vision_diag_poll(void)
{
    if (s_vdiag_phase == VD_OFF) return;
    if (s_vdiag_phase == VD_QR_WAIT && s_vdiag_qr_pending && proto_scene_status() == 1) {
        uint32_t pm = __get_PRIMASK();
        static char b[112];
        __disable_irq();
        s_vdiag_qr_pending = 0u;
        proto_send_scene(SCENE_EOD); /* OBJECT algorithm; target tuple stays latched in camera. */
        s_vdiag_phase = VD_OBJECT_WAIT;
        __set_PRIMASK(pm);
        snprintf(b, sizeof b, "VD33 test=%lu QR_VALID=%ld%ld%ld OBJECT_requested no_motion=1",
                 (unsigned long)s_active_test,
                 (long)s_vdiag_qr[0], (long)s_vdiag_qr[1], (long)s_vdiag_qr[2]);
        send(b);
    } else if (s_vdiag_phase == VD_OBJECT_WAIT && proto_scene_status() == 1) {
        s_vdiag_phase = VD_OBJECT;
    }
    if ((uint32_t)(HAL_GetTick() - s_vdiag_report_t0) >= 1000u) {
        s_vdiag_report_t0 = HAL_GetTick();
        vision_diag_report(); /* <=1 Hz snapshots, not a claim to log every camera frame. */
    }
}

static int dist_mode(void)
{
    return s_msel >= 15 && s_msel <= 18;
}

static int jog_mode(void) { return s_msel >= 24 && s_msel <= 27; }
static int jog_return_mode(void) { return s_msel == 26 || s_msel == 27; }
static int jog_axis(void) { return (s_msel - 24) & 1; }
static int servo_mode(void) { return s_msel == 28 || s_msel == 29; }

/* 自动回程模式的任意阶段 g/a/0 只停脉冲并取消未发生的回程。
 * 没有原点传感器，不能将发出的步数当成实际位置。下次须重新发 n。 */
static void jog_stop(const char *phase)
{
    static char b[160];
    snprintf(b, sizeof b,
             "REC type=JOG test=%lu mode=%d axis=%d out=%lu back=%lu status=STOP phase=%s no_auto_return pos_uncertain=1",
             (unsigned long)s_active_test, s_msel, jog_axis(),
             (unsigned long)s_jog_done, (unsigned long)s_back, phase);
    s_round = R_READY;
    s_jog_request = 0; s_jog_hold_t0 = 0u;
    send(b);
}

/* g/a/0 取消尚未发生的舵机自动回程。继续输出当前 PWM 以保持夹持，
 * 不切断供电，也无法根据命令脉宽推断物理角度。 */
static void servo_stop(void)
{
    static char b[112];
    snprintf(b, sizeof b, "REC type=SERVO test=%lu mode=%d status=HOLD us=%u return_cancelled=1 no_feedback=1",
             (unsigned long)s_active_test, s_msel, (unsigned)arm_claw_command_us());
    s_round = R_READY;
    s_servo_hold_t0 = 0u;
    send(b);
}

static int dist_lateral(void)
{
    return s_msel == 17 || s_msel == 18;
}

static float dist_odo(void)
{
    return dist_lateral() ? motion_lateral_odo_mm() : motion_odo_mm();
}

static float dist_heading_kp_get(int *profile)
{
    if (profile) *profile = 0;
    if (s_seq_state != SQ_OFF) {
        /* Crossing/offset disturbance is not an alignment target.
         * The board leg squares mechanically, without yaw fighting contact.
         * The clearance leg restores the RAM gain after stopped heading reset.
         * Mode36 mirrors these directions; its per-leg hold flags stay the same. */
        if (s_seq_stage < route_seq_stage_count() && !route_seq_leg()->heading_hold)
            return 0.0f;
        return s_route_heading_kp;
    }
    if (dist_mode() && s_seq_state == SQ_OFF && s_v >= 1.0f && s_v <= T_V_MAX) {
        for (unsigned i = 0; i < T_YKP_PROFILE_SLOTS; ++i) {
            if (s_heading_profiles[i].mode == s_msel && s_heading_profiles[i].speed == s_v) {
                if (profile) *profile = 1;
                return s_heading_profiles[i].kp;
            }
        }
    }
    return step_heading_kp_deg();
}

/* Caller has validated selected manual mode and integer v. -1 means full;
 * do not silently discard another direction/speed's measured value. */
static int dist_heading_kp_set(float kp)
{
    int empty = -1;
    if (!dist_mode() || s_seq_state != SQ_OFF || !(s_v >= 1.0f && s_v <= T_V_MAX)
        || s_v != (float)(uint16_t)s_v) return 0;
    if (!(kp >= 0.0f && kp <= 5.0f)) return 0;
    for (unsigned i = 0; i < T_YKP_PROFILE_SLOTS; ++i) {
        if (s_heading_profiles[i].mode == s_msel && s_heading_profiles[i].speed == s_v) {
            s_heading_profiles[i].kp = kp;
            return 1;
        }
        if (!s_heading_profiles[i].speed && empty < 0) empty = (int)i;
    }
    if (empty < 0) return -1;
    s_heading_profiles[empty].mode = (uint8_t)s_msel;
    s_heading_profiles[empty].speed = (uint16_t)s_v;
    s_heading_profiles[empty].kp = kp;
    return 1;
}

static int dist_ordinary_leg(void)
{
    return s_seq_state == SQ_OFF ||
           (s_seq_stage < route_seq_stage_count() && route_seq_leg()->heading_hold);
}

static void dist_save_translation_end(void)
{
    if (s_dist_end_saved) return;
    s_dist_end_mm = dist_odo() - s_dist_odo0;
    s_dist_pre_align_yaw = imu_ok() ? imu_leg_heading_deg() : 9999.0f;
    for (int i = 0; i < 4; ++i) s_dist_end_counts[i] = ctrl_enc_total(i);
    s_dist_end_saved = 1u;
}

static float dist_yaw_error(float yaw)
{
    float e = s_dist_heading0 - yaw;
    if (!isfinite(e)) return e;
    e = fmodf(e, 360.0f); /* bounded even for a corrupt but finite large value */
    if (e > 180.0f) e -= 360.0f;
    if (e < -180.0f) e += 360.0f;
    return e;
}

/* 到目标或人工暂停后先刹车，等待真实轮子不再产生计数，再报告最终值。 */
static void dist_begin_finish(uint8_t reason)
{
    s_dist_brake_mm = dist_odo() - s_dist_odo0;
    s_dist_brake_yaw = imu_ok() ? imu_leg_heading_deg() : 9999.0f;
    motion_brake();
    s_dist_reason = reason;
    s_dist_run_ms = (uint32_t)(HAL_GetTick() - s_meas_t0);
    s_dist_brake_t0 = HAL_GetTick();
    s_dist_still_t0 = s_dist_brake_t0;
    for (int i = 0; i < 4; i++) s_dist_last[i] = ctrl_enc_total(i);
    s_round = R_BRAKE;
}

static void dist_finish_report(void)
{
    float yaw = imu_ok() ? imu_leg_heading_deg() : 9999.0f;
    uint32_t settle_ms = (uint32_t)(HAL_GetTick() - s_dist_brake_t0);
    uint32_t align_ms = s_dist_align_started ? HAL_GetTick() - s_dist_align_t0 : 0u;
    dist_save_translation_end();
    /* Never erase a failed/stopped heading and never continue a failed chain. */
    if (s_dist_reason == 1u) {
        if (run_aborted()) s_dist_reason = 4u;
        else if (!imu_ok() || !isfinite(yaw) || !imu_zero_leg_heading()) s_dist_reason = 2u;
    }
    snprintf(s_dist_report, sizeof s_dist_report,
             "REC type=%s leg=%u test=%lu mode=%u status=%s cmd_mm=%.0f v_mms=%.0f ykp=%.3f precise=%u ff_ratio=%.5f brake_mm=%.1f enc_mm=%.1f run_ms=%lu settle_ms=%lu brake_yaw=%.2f yaw_deg=%.2f c0=%ld c1=%ld c2=%ld c3=%ld pre_yaw=%.2f yfix=%u align_ms=%lu",
             s_route_leg ? "ROUTE" : "DIST", (unsigned)s_route_leg,
             (unsigned long)s_active_test, (unsigned)s_msel,
             s_dist_reason == 1u ? "DONE" : (s_dist_reason == 2u ? "IMUERR" :
                 (s_dist_reason == 3u ? "YAW_TIMEOUT" : (s_dist_reason == 4u ? "ABORT" : "STOP"))),
             s_dist_target, s_v, s_dist_heading_kp, (unsigned)s_dist_precise,
             s_dist_ff_ratio, s_dist_brake_mm, s_dist_end_mm,
             (unsigned long)s_dist_run_ms, (unsigned long)settle_ms,
             s_dist_brake_yaw, yaw,
             (long)s_dist_end_counts[0], (long)s_dist_end_counts[1],
             (long)s_dist_end_counts[2], (long)s_dist_end_counts[3],
             s_dist_pre_align_yaw, (unsigned)s_dist_align_started, (unsigned long)align_ms);
    ctrl_enc_reset_all();
    s_dist_align_hold = s_dist_align_enabled = 0u;
    s_round = R_READY;
    send(s_dist_report);
    if (s_seq_state == SQ_RUN) {
        if (s_dist_reason == 1u && imu_ok()) route_seq_next();
        else route_seq_end(s_dist_reason == 2u || !imu_ok() ? "IMUERR" :
            (s_dist_reason == 3u ? "YAW_TIMEOUT" : (s_dist_reason == 4u ? "ABORT" : "STOP")));
    }
}

/* Post-DONE correction uses the ORIGINAL target, not a newly zeroed yaw.
 * It is separate from the validated 90/180 profile and never translates. */
static void dist_align_poll(void)
{
    uint32_t now = HAL_GetTick();
    float yaw = imu_leg_heading_deg(), e, w;
    int changed = 0;
    if (run_aborted() || !bench_ok()) {
        motion_brake(); s_dist_reason = 4u; dist_finish_report(); return;
    }
    if (!imu_ok() || !isfinite(yaw) || !isfinite(s_dist_heading0)) {
        motion_brake(); s_dist_reason = 2u; dist_finish_report(); return;
    }
    if ((uint32_t)(now - s_dist_align_t0) >= T_DIST_ALIGN_MAX_MS) {
        motion_brake(); s_dist_reason = 3u; dist_finish_report(); return;
    }
    e = dist_yaw_error(yaw);
    if (!isfinite(e)) {
        motion_brake(); s_dist_reason = 2u; dist_finish_report(); return;
    }
    for (int i = 0; i < 4; ++i) {
        int32_t count = ctrl_enc_total(i);
        if (count != s_dist_last[i]) changed = 1;
        s_dist_last[i] = count;
    }
    if (fabsf(e) > T_DIST_ALIGN_TOL_DEG) {
        s_dist_align_hold = 0u;
        w = T_DIST_ALIGN_KP * e;
        if (w > T_DIST_ALIGN_MAX_W) w = T_DIST_ALIGN_MAX_W;
        if (w < -T_DIST_ALIGN_MAX_W) w = -T_DIST_ALIGN_MAX_W;
        if (fabsf(w) < T_DIST_ALIGN_MIN_W) w = e > 0.0f ? T_DIST_ALIGN_MIN_W : -T_DIST_ALIGN_MIN_W;
        motion_vel_set_precise(0.0f, 0.0f, w);
    } else {
        if (!s_dist_align_hold) {
            motion_brake();
            s_dist_align_hold = 1u;
            s_dist_align_stable_t0 = now;
            s_dist_align_stable_yaw = yaw;
        } else if (changed || fabsf(yaw - s_dist_align_stable_yaw) > T_DIST_ALIGN_STILL_DEG) {
            s_dist_align_stable_t0 = now;
            s_dist_align_stable_yaw = yaw;
        }
        if ((uint32_t)(now - s_dist_align_stable_t0) >= T_DIST_ALIGN_STABLE_MS) {
            dist_finish_report(); return;
        }
    }
    if ((uint32_t)(now - s_dist_trace_t0) >= T_DIST_TRACE_MS) {
        static char b[160];
        s_dist_trace_t0 = now;
        snprintf(b, sizeof b, "YFIX test=%lu phase=%s target_deg=%.2f yaw_deg=%.2f error_deg=%.2f tol=0.30 zeroed=0",
                 (unsigned long)s_active_test, s_dist_align_hold ? "SETTLE" : "CORRECT",
                 s_dist_heading0, yaw, e);
        send(b);
    }
}

/* IMU 测量统计只在 RAM 中存在；每轮 g1 清、g2 报完再清，不写 Flash。 */
static void imu_stat_clear(void)
{
    s_imu_n = 0u; s_imu_bad = 0u;
    s_imu_y_min = s_imu_y_max = 0.0f;
    s_imu_p_min = s_imu_p_max = 0.0f;
    s_imu_r_min = s_imu_r_max = 0.0f;
}

static void imu_stat_take(void)
{
    float y, p, r;
    if (!imu_ok()) { s_imu_bad++; return; }
    y = imu_leg_heading_deg();
    p = imu_pitch_deg();
    r = imu_roll_deg();
    if (s_imu_n == 0u) {
        s_imu_y_min = s_imu_y_max = y;
        s_imu_p_min = s_imu_p_max = p;
        s_imu_r_min = s_imu_r_max = r;
    } else {
        if (y < s_imu_y_min) s_imu_y_min = y;
        if (y > s_imu_y_max) s_imu_y_max = y;
        if (p < s_imu_p_min) s_imu_p_min = p;
        if (p > s_imu_p_max) s_imu_p_max = p;
        if (r < s_imu_r_min) s_imu_r_min = r;
        if (r > s_imu_r_max) s_imu_r_max = r;
    }
    s_imu_n++;
}

/* 调试口发一行(内容 + CRLF;所有文本应答/状态都走它) */
static void send(const char *s)
{
    bp_debug_send(s);
    bp_debug_send("\r\n");
}

static float turn_abs(float v) { return v < 0.0f ? -v : v; }
static int turn_closed_loop_mode(void) { return s_msel == 20 || s_msel == 22 || s_msel == 30; }
static float turn_target_deg(void)
{
    if (s_msel == 22) return 180.0f;
    /* Cover PREP/RUN/brake/report while the route owner stays31; selecting
     * standalone20/30 sets SQ_OFF, so retained owner31 cannot leak here. */
    if (s_seq_mode == ROUTE_TEST_MODE && route_seq_active())
        return s_msel == 30 ? ROUTE31_LEFT_TARGET_DEG : ROUTE31_RIGHT_TARGET_DEG;
    return s_msel == 30 ? T_TURN_LEFT_TARGET_DEG : T_TURN_RIGHT_TARGET_DEG;
}
static const char *turn_record_type(void)
{
    if (s_msel == 22) return "TURN180";
    return s_msel == 30 ? "TURN90_LEFT" : "TURN90";
}
/* Direction-normalized range: right keeps its old -15..target+15 limits;
 * left uses the mirror target-15..+15, including brake/settle checks. */
static int turn_angle_outside(float yaw)
{
    float target = turn_target_deg();
    float progress = target < 0.0f ? -yaw : yaw;
    return progress > turn_abs(target) + T_TURN_LIMIT_DEG || progress < -T_TURN_LIMIT_DEG;
}
static uint32_t turn_max_ms(void) { return s_msel == 22 ? T_TURN180_MAX_MS : T_TURN_MAX_MS; }

static void turn_begin_settle(const char *reason)
{
    motion_brake();
    s_turn_result = reason;
    s_turn_settle_t0 = HAL_GetTick();
    s_turn_settle_yaw = imu_ok() ? imu_leg_heading_deg() : 0.0f;
    s_round = R_BRAKE;
}

static void turn_report(void)
{
    static char b[180];
    char yaw[20], err[20];
    int valid = imu_ok();
    float angle = valid ? imu_leg_heading_deg() : 0.0f;
    format_deg2(yaw, angle);
    format_deg2(err, turn_closed_loop_mode() ? turn_target_deg() - angle : -angle);
    if (s_msel == 19) {
        snprintf(b, sizeof b,
                 "REC type=TURN_SIGN test=%lu status=%s yaw_deg=%s ms=%lu c=%ld,%ld,%ld,%ld",
                 (unsigned long)s_active_test, valid ? s_turn_result : "IMUERR", yaw,
                 (unsigned long)(HAL_GetTick() - s_meas_t0),
                 (long)ctrl_enc_total(0), (long)ctrl_enc_total(1),
                 (long)ctrl_enc_total(2), (long)ctrl_enc_total(3));
    } else {
        snprintf(b, sizeof b,
                 "REC type=%s test=%lu status=%s cmd_deg=%d yaw_deg=%s err_deg=%s ms=%lu",
                 turn_record_type(), (unsigned long)s_active_test,
                 valid ? s_turn_result : "IMUERR", (int)turn_target_deg(),
                 yaw, err, (unsigned long)(HAL_GetTick() - s_meas_t0));
    }
    s_round = R_DONE;
    send(b);
    if (s_xy_state == XT_TURN_RUN) {
        xy_trial_turn_done(valid && strcmp(s_turn_result, "DONE") == 0);
        return;
    }
    if (s_seq_state == SQ_RUN) {
        if (valid && strcmp(s_turn_result, "DONE") == 0) route_seq_next();
        else route_seq_end(valid ? s_turn_result : "IMUERR");
    }
}

static void speed_probe_report(const char *status)
{
    static char b[150];
    motion_brake();
    snprintf(b, sizeof b,
             "REC type=YAWSPD test=%lu status=%s ms=%lu c=%ld,%ld,%ld,%ld",
             (unsigned long)s_active_test, status,
             (unsigned long)(HAL_GetTick() - s_meas_t0),
             (long)ctrl_enc_total(0), (long)ctrl_enc_total(1),
             (long)ctrl_enc_total(2), (long)ctrl_enc_total(3));
    s_round = R_DONE;
    send(b);
}

static void fwd_probe_report(const char *status)
{
    static char b[192];
    motion_brake();
    snprintf(b, sizeof b,
             "REC type=FWDPULSE test=%lu status=%s phase=%u open_c=%ld,%ld,%ld,%ld closed_c=%ld,%ld,%ld,%ld",
             (unsigned long)s_active_test, status, (unsigned)s_fwd_phase,
             (long)s_fwd_open_count[0], (long)s_fwd_open_count[1],
             (long)s_fwd_open_count[2], (long)s_fwd_open_count[3],
             (long)ctrl_enc_total(0), (long)ctrl_enc_total(1),
             (long)ctrl_enc_total(2), (long)ctrl_enc_total(3));
    s_round = R_DONE;
    send(b);
}

/* BOOT 且没开跑 = 调试/参数可服务。开跑后(含跑到 DONE/ABORT 驻留)锁死,
 * 要再测只能断电重启(mission_main 驻留,无回调试通道)。 */
static int bench_ok(void)
{
    return !s_go && mission_state() == MS_BOOT;
}

/* Mode32 keeps its prior independent tune; new distance candidates do not
 * silently change an untested mission flow. Select32 then fff to tune it. */
float test_forward_ff_ratio(void) { return s_mission_forward_ff_ratio; }

/* 调试执行器全态清零:行缓冲、槽置 -1、回空闲就绪(robot_init 调一次) */
void test_init(void)
{
    s_len = 0; s_over = 0;
    s_v = -1.0f; s_d = -1.0f; s_p = 0.0f; s_p_valid = 0u;
    s_left_ff_ratio = T_LEFT_FF_SEED;
    s_right_ff_ratio = T_RIGHT_FF_SEED;
    s_forward_ff_ratio = T_FORWARD_FF_SEED;
    s_backward_ff_ratio = T_BACKWARD_FF_SEED;
    s_mission_forward_ff_ratio = T_MISSION_FORWARD_FF_SEED;
    s_route_forward_ff_ratio = ROUTE_TEST_FORWARD_FF_SEED;
    s_route_backward_ff_ratio = T_BACKWARD_FF_SEED;
    s_route_left_ff_ratio = ROUTE_TEST_LEFT_FF_SEED;
    s_route_right_ff_ratio = ROUTE_TEST_RIGHT_FF_SEED;
    s_dist_align_on = 1u;
    s_route_heading_kp = ROUTE_TEST_HEADING_KP_SEED;
    memset(s_heading_profiles, 0, sizeof s_heading_profiles);
    s_jog_request = 0; s_jog_done = 0u; s_jog_hold_t0 = 0u;
    s_servo_target_us = 0u; s_servo_origin_us = 0u; s_servo_hold_t0 = 0u;
    s_go = 0;
    s_msel = R_FREE; s_round = R_READY;
    s_route_leg = 0u;
    s_seq_state = SQ_OFF; s_seq_stage = s_seq_prepared = 0u;
    s_seq_mode = ROUTE_TEST_MODE;
    s_seq_run = s_seq_still_t0 = s_seq_wait_t0 = 0u;
    s_seq_qr_report_t0 = 0u;
    memset(s_seq_qr, 0, sizeof s_seq_qr);
    for (int i = 0; i < 4; i++) s_seq_last[i] = 0;
    s_steps = 0; s_back = 0; s_leg_ms = 0; s_leg_run = 0;
    s_imu_mon = 0u; s_mon_last = 0u;
    s_meas_t0 = 0u;
    s_enc_report_t0 = 0u; s_enc_report_seq = 0u;
    s_test_seq = 0u; s_active_test = 0u;
    s_vdiag_phase = VD_OFF; s_vdiag_qr_pending = 0u; s_vdiag_report_t0 = 0u;
    memset(s_vdiag_sample, 0, sizeof s_vdiag_sample);
    memset(s_vdiag_qr, 0, sizeof s_vdiag_qr);
    memset(&s_vdiag_base, 0, sizeof s_vdiag_base);
    s_target35_phase = TA_OFF;
    s_xy_state = XT_OFF; s_xy_owner = 0u;
    memset(s_xy_qr, 0, sizeof s_xy_qr);
    memset(s_xy_last_counts, 0, sizeof s_xy_last_counts);
    memset(&s_xy_snapshot, 0, sizeof s_xy_snapshot);
    s_xy_phase_t0 = s_xy_report_t0 = s_xy_test = 0u;
    vision_align_test_init();
    s_bucket36_phase = BA_OFF;
    memset(&s_bucket36_sample, 0, sizeof s_bucket36_sample);
    s_bucket36_t0 = s_bucket36_report_t0 = s_bucket36_packet = 0u;
    s_bucket36_sequence = s_bucket36_back_mm = 0u;
    s_bucket36_good = s_bucket36_seen = 0u; s_bucket36_cmd = 0.0f;
    memset(s_target35_qr, 0, sizeof s_target35_qr);
    memset(s_target35_last_counts, 0, sizeof s_target35_last_counts);
    memset(&s_target35_sample, 0, sizeof s_target35_sample);
    s_target35_phase_t0 = s_target35_report_t0 = s_target35_packet = 0u;
    s_target35_sequence = 0u; s_target35_good = s_target35_seen = 0u;
    s_target35_heading = s_target35_cmd = 0.0f;
    s_target35_kp = step_heading_kp_deg();
    s_dist_odo0 = 0.0f; s_dist_heading0 = 0.0f; s_dist_orth0 = 0.0f; s_dist_target = 0.0f;
    s_dist_ff_ratio = 0.0f;
    s_dist_heading_kp = step_heading_kp_deg();
    s_dist_heading_profile = s_dist_precise = 0u;
    s_dist_run_ms = 0u; s_dist_brake_t0 = 0u; s_dist_still_t0 = 0u;
    s_dist_trace_t0 = 0u;
    s_dist_brake_mm = 0.0f; s_dist_brake_yaw = 0.0f;
    s_dist_reason = 0u;
    s_dist_align_enabled = s_dist_align_started = s_dist_align_hold = s_dist_end_saved = 0u;
    s_dist_align_t0 = s_dist_align_stable_t0 = 0u;
    s_dist_align_stable_yaw = s_dist_pre_align_yaw = s_dist_end_mm = 0.0f;
    memset(s_dist_end_counts, 0, sizeof s_dist_end_counts);
    s_turn_settle_t0 = 0u; s_turn_settle_yaw = 0.0f; s_turn_result = "STOP";
    s_turn_trace_t0 = 0u;
    s_speed_probe_trace_t0 = 0u;
    for (int i = 0; i < 4; i++) s_dist_last[i] = 0;
    imu_stat_clear();
#if BENCH_AUTO
    for (int i = 0; i < 4; i++) { g_bench_cnt[i] = 0; g_bench_raw_cnt[i] = 0; }
    g_bench_wheel = 0; s_bench_phase = 0; s_bench_t0 = 0;
    s_bench_boot = 0; s_bench_boot_ok = 0u;
#endif
}

static int target35_active(void)
{
    return s_target35_phase >= TA_QR_WAIT && s_target35_phase <= TA_FIRE;
}

static void target35_counts_start(void)
{
    for (int i = 0; i < 4; i++) s_target35_last_counts[i] = ctrl_enc_total(i);
    s_target35_phase_t0 = HAL_GetTick();
}

static int target35_counts_changed(void)
{
    int changed = 0;
    for (int i = 0; i < 4; i++) {
        int32_t count = ctrl_enc_total(i);
        if (count != s_target35_last_counts[i]) changed = 1;
        s_target35_last_counts[i] = count;
    }
    return changed;
}

static void target35_clear_sample(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    memset(&s_target35_sample, 0, sizeof s_target35_sample);
    __set_PRIMASK(pm);
    s_target35_good = s_target35_seen = 0u;
    s_target35_packet = 0u;
    s_target35_sequence = 0u;
}

static void target35_report(void)
{
    static char b[224];
    VisionDiagSample sample;
    ProtoStats stats;
    uint32_t pm = __get_PRIMASK();
    __disable_irq(); sample = s_target35_sample; proto_stats_get(&stats); __set_PRIMASK(pm);
    snprintf(b, sizeof b,
             "TGT35 test=%lu phase=%u QR=%ld,%ld,%ld target_digit=%ld cx=%d seq=%u latest=%u age_ms=%lu good=%u/5 vx=%.0f laser=%u point=255 band=250..260 v=50",
             (unsigned long)s_active_test, (unsigned)s_target35_phase,
             (long)s_target35_qr[0], (long)s_target35_qr[1], (long)s_target35_qr[2],
             (long)s_target35_qr[1], sample.seen ? sample.frame.cx : -1,
             (unsigned)sample.frame.sequence,
             (unsigned)(sample.seen && sample.packet_index == stats.obj &&
                 (uint32_t)(HAL_GetTick() - sample.tick) <= T_TARGET35_FRESH_MS),
             (unsigned long)(sample.seen ? HAL_GetTick() - sample.tick : 0u),
             (unsigned)s_target35_good, s_target35_cmd,
             (unsigned)(s_target35_phase == TA_FIRE));
    send(b);
}

static void target35_stop(const char *status)
{
    static char b[112];
    s_target35_phase = strcmp(status, "DONE") == 0 ? TA_DONE : TA_STOPPED;
    motion_brake(); bp_laser_set(0);
    s_target35_cmd = 0.0f;
    step_vision_receive_end(); /* camera can keep recognizing; no IDLE/STOP TX */
    s_round = R_DONE;
    snprintf(b, sizeof b, "REC type=TARGET35 test=%lu status=%s brake=1 laser=0 stay_here=1 no_auto_return=1",
             (unsigned long)s_active_test, status);
    send(b);
}

static void target35_g(void)
{
    if (target35_active()) { target35_stop("STOP"); return; }
    motion_brake(); bp_laser_set(0);
    /* READY may already contain this selection's manually presented QR.
     * A completed/stopped run starts a NEW scan, never reuses its target. */
    if (s_target35_phase != TA_READY) { step_vision_receive_end(); proto_qr_begin(); }
    target35_clear_sample();
    memset(s_target35_qr, 0, sizeof s_target35_qr);
    begin_recorded_test();
    s_target35_kp = step_heading_kp_deg(); s_target35_cmd = 0.0f;
    s_target35_phase = TA_QR_WAIT; s_round = R_RUN;
    s_target35_report_t0 = HAL_GetTick();
    robot_diag_report(); cmd_param_report();
    send("OK TARGET35_START QR_WAIT stopped=1; present_QR_manually; task2 uses_QR_second_digit; g/a/0 stops");
}

static void target35_poll(void)
{
    VisionDiagSample sample;
    ProtoStats stats;
    uint32_t now = HAL_GetTick(), pm;
    int fresh;
    if (!target35_active()) return;
    if (!bench_ok() || run_aborted()) { target35_stop("ABORT"); return; }
    if ((uint32_t)(now - s_target35_report_t0) >= T_TARGET35_REPORT_MS) {
        s_target35_report_t0 = now; target35_report();
    }
    if (s_target35_phase == TA_QR_WAIT) {
        motion_brake();
        if (proto_scene_status() < 0) { target35_stop("QR_LINK_ERROR"); return; }
        if (!proto_qr_get(s_target35_qr)) return;
        if (s_target35_qr[0] < 1 || s_target35_qr[0] > 3 ||
            s_target35_qr[1] < 1 || s_target35_qr[1] > 3 ||
            s_target35_qr[2] < 1 || s_target35_qr[2] > 3) return;
        step_vision_receive_end();
        target35_clear_sample();
        s_target35_phase = TA_TASK_WAIT;
        if (!proto_send_target(PROTO_TASK_TARGET, (uint8_t)s_target35_qr[1])) {
            target35_stop("TARGET_REQUEST_ERROR"); return;
        }
        send("OK TARGET35_QR_VALID stopped=1; requesting_selected_target task=2; prepare_then_forward_without_coordinates");
        target35_report();
        return;
    }
    if (s_target35_phase == TA_TASK_WAIT) {
        int status = proto_scene_status();
        motion_brake();
        if (status < 0) { target35_stop("TARGET_LINK_ERROR"); return; }
        /* A legal QR and queued task request permit forward search without
         * waiting for target ACK/coordinates. The parser still requires the
         * matching task ACK before accepting any coordinate for alignment. */
        s_target35_phase = TA_STILL;
        target35_counts_start();
        return;
    }
    if (!imu_ok()) { target35_stop("IMUERR"); return; }
    if (s_target35_phase == TA_STILL) {
        motion_brake();
        if (target35_counts_changed()) s_target35_phase_t0 = now;
        if ((uint32_t)(now - s_target35_phase_t0) < T_DIST_STILL_MS) return;
        if (!imu_zero_leg_heading()) { target35_stop("IMUERR"); return; }
        s_target35_heading = 0.0f;
        s_target35_phase = TA_PREP_WAIT; s_target35_phase_t0 = now;
        return;
    }
    if (s_target35_phase == TA_PREP_WAIT) {
        motion_brake();
        if (target35_counts_changed()) {
            s_target35_phase = TA_STILL; s_target35_phase_t0 = now; return;
        }
        if ((uint32_t)(now - s_target35_phase_t0) < NAV_SETTLE_MS) return;
        target35_clear_sample(); /* only post-preparation images count toward aim */
        s_target35_phase = TA_SEEK;
        send("OK TARGET35_SEARCH no_coordinates=forward50mm/s; x<250 backward; x>260 forward; 5_new_frames_in_band brakes_then_1s_laser_hold");
        /* Fall through: start forward search in this same nonblocking tick. */
    }
    if (s_target35_phase == TA_FIRE) {
        motion_brake();
        /* Mode35 holds the laser ON until manual g/a/0 or an abort/error.
         * Formal mission laser timing is intentionally unchanged. */
        return;
    }
    if (proto_scene_status() < 0) { target35_stop("TARGET_LINK_ERROR"); return; }
    pm = __get_PRIMASK();
    __disable_irq();
    sample = s_target35_sample; proto_stats_get(&stats); now = HAL_GetTick();
    __set_PRIMASK(pm); /* sample.tick cannot be newer than this snapshot's now */
    fresh = sample.seen && sample.packet_index == stats.obj &&
            (uint32_t)(now - sample.tick) <= T_TARGET35_FRESH_MS;
    if (!fresh) {
        s_target35_good = 0u;
        /* No selected target coordinate: resume forward search, even after
         * prior alignment/reverse correction. Never fire or follow an old x. */
        s_target35_phase = TA_SEEK;
        s_target35_cmd = T_TARGET35_V_MMS;
        motion_vel_set_precise(s_target35_cmd, 0.0f, step_heading_hold_w_kp(s_target35_heading, s_target35_kp));
        return;
    }
    if (sample.frame.img_w <= T_TARGET35_HIGH_CX) { target35_stop("IMAGE_WIDTH"); return; }
    if (sample.packet_index != s_target35_packet) {
        if (!s_target35_seen || sample.packet_index - s_target35_packet != 1u ||
            (uint16_t)(sample.frame.sequence - s_target35_sequence) != 1u) {
            s_target35_good = 0u;
            if (s_target35_phase == TA_SETTLE) s_target35_phase = TA_SEEK;
        }
        s_target35_seen = 1u;
        s_target35_packet = sample.packet_index; s_target35_sequence = sample.frame.sequence;
        if (sample.frame.cx >= T_TARGET35_LOW_CX && sample.frame.cx <= T_TARGET35_HIGH_CX) {
            if (s_target35_good < T_TARGET35_GOOD_FRAMES) s_target35_good++;
        } else s_target35_good = 0u;
    }
    if (sample.frame.cx < T_TARGET35_LOW_CX || sample.frame.cx > T_TARGET35_HIGH_CX) {
        s_target35_phase = TA_SEEK;
        s_target35_cmd = sample.frame.cx > T_TARGET35_HIGH_CX ? T_TARGET35_V_MMS : -T_TARGET35_V_MMS;
        motion_vel_set_precise(s_target35_cmd, 0.0f, step_heading_hold_w_kp(s_target35_heading, s_target35_kp));
        return;
    }
    motion_brake(); s_target35_cmd = 0.0f; /* SHORT BRAKE, never coasting/in-band creep */
    if (s_target35_good < T_TARGET35_GOOD_FRAMES) return;
    if (s_target35_phase != TA_SETTLE) {
        s_target35_phase = TA_SETTLE; target35_counts_start();
        send("OK TARGET35_AIMED brake=1; wait_1s_still_with_fresh_target; laser=0");
        return;
    }
    if (target35_counts_changed()) s_target35_phase_t0 = now;
    if ((uint32_t)(now - s_target35_phase_t0) < TARGET_AIM_SETTLE_MS) return;
    /* RX may have changed the latest packet since the earlier snapshot. Make
     * the final fresh/in-band check and local-close/laser transition atomic;
     * an intervening empty/out-of-band frame must never fire the old aim. */
    pm = __get_PRIMASK(); __disable_irq();
    sample = s_target35_sample; proto_stats_get(&stats); now = HAL_GetTick();
    if (!sample.seen || sample.packet_index != stats.obj ||
        sample.packet_index != s_target35_packet || sample.frame.sequence != s_target35_sequence ||
        (uint32_t)(now - sample.tick) > T_TARGET35_FRESH_MS ||
        sample.frame.cx < T_TARGET35_LOW_CX || sample.frame.cx > T_TARGET35_HIGH_CX ||
        proto_scene_status() < 0 || run_aborted() || !imu_ok()) {
        __set_PRIMASK(pm); return;
    }
    s_target35_phase = TA_FIRE; s_target35_phase_t0 = now;
    step_vision_receive_end(); bp_laser_set(1);
    __set_PRIMASK(pm);
    send("OK TARGET35_FIRE laser=1 hold_until_stop=1 brake=1; g/a/0 turns_off_immediately");
}

static int xy_trial_selected(void)
{
    return s_xy_owner >= 38u && s_xy_owner <= 41u && s_xy_state != XT_OFF;
}

static int xy_trial_active(void)
{
    return xy_trial_selected() && s_xy_state >= XT_QR_WAIT && s_xy_state <= XT_TURN_RUN;
}

static void xy_trial_report(void)
{
    static char b[512];
    vision_align_test_status(&s_xy_snapshot);
    snprintf(b, sizeof b,
             "XY test=%lu mode=%u phase=%u align=%u task=%u digit=%u req=%u QR=%ld,%ld,%ld x=%d y=%d seq=%u img=%u,%u latest=%u age=%lu good=%u/5 axis=%u vx=%.0f vy=%.0f w=%.3f reason=%s rx=%d,%d rx_seq=%u rx_img=%u,%u rx_fresh=%u yaw_target=%.2f yaw_err=%.2f yfix=%u y_moved=%u step=%lu step_mm=%.2f step_ms=%lu cap=%u",
             (unsigned long)s_xy_test, (unsigned)s_xy_owner, (unsigned)s_xy_state,
             (unsigned)s_xy_snapshot.state, (unsigned)s_xy_snapshot.task,
             (unsigned)s_xy_snapshot.digit, (unsigned)s_xy_snapshot.request,
             (long)s_xy_qr[0], (long)s_xy_qr[1], (long)s_xy_qr[2],
             s_xy_snapshot.cx, s_xy_snapshot.cy, (unsigned)s_xy_snapshot.sequence,
             (unsigned)s_xy_snapshot.img_w, (unsigned)s_xy_snapshot.img_h,
             (unsigned)s_xy_snapshot.latest, (unsigned long)s_xy_snapshot.age_ms,
             (unsigned)s_xy_snapshot.good, (unsigned)s_xy_snapshot.axis,
             s_xy_snapshot.vx, s_xy_snapshot.vy, s_xy_snapshot.w,
             s_xy_state == XT_QR_WAIT ? "QR_WAIT" : s_xy_snapshot.reason,
             s_xy_snapshot.rx_cx, s_xy_snapshot.rx_cy, (unsigned)s_xy_snapshot.rx_sequence,
             (unsigned)s_xy_snapshot.rx_img_w, (unsigned)s_xy_snapshot.rx_img_h,
             (unsigned)s_xy_snapshot.rx_fresh, s_xy_snapshot.yaw_target,
             s_xy_snapshot.yaw_error, (unsigned)s_xy_snapshot.yaw_dirty,
             (unsigned)s_xy_snapshot.yaw_ever, (unsigned long)s_xy_snapshot.step,
             s_xy_snapshot.step_mm, (unsigned long)s_xy_snapshot.step_ms,
             (unsigned)s_xy_snapshot.step_capped);
    send(b);
}

static void xy_trial_stop(const char *reason)
{
    static char b[128];
    s_xy_state = XT_STOPPED; /* RX callback can no longer feed this trial. */
    vision_align_test_cancel(); proto_qr_cancel();
    motion_brake(); bp_laser_set(0);
    s_seq_prepared = 0u; s_msel = s_xy_owner; s_round = R_DONE;
    snprintf(b, sizeof b, "REC type=XY test=%lu mode=%u status=%s brake=1 laser=0 no_grab=1 no_auto_return=1",
             (unsigned long)s_xy_test, (unsigned)s_xy_owner, reason);
    send(b);
}

static void xy_trial_g(void)
{
    if (xy_trial_active()) { xy_trial_stop("STOP"); return; }
    motion_brake(); bp_laser_set(0);
    /* 39 is direct bucket: cancel pending boot/old QR, never request new QR.
     * Other modes' first g preserves only this selection's NEW QR session;
     * finished/stopped runs always require another fresh request. */
    if (s_xy_owner == 39u) {
        vision_align_test_cancel(); proto_qr_cancel();
    } else if (s_xy_state != XT_READY) {
        vision_align_test_cancel(); proto_send_scene(SCENE_QR);
    }
    memset(s_xy_qr, 0, sizeof s_xy_qr);
    begin_recorded_test(); s_xy_test = s_active_test;
    s_xy_state = s_xy_owner == 39u ? XT_ACTIVE : XT_QR_WAIT;
    s_round = R_RUN; s_msel = s_xy_owner;
    s_xy_report_t0 = HAL_GetTick();
    if (s_xy_owner == 39u && !vision_align_test_start(39u, NULL)) {
        xy_trial_stop("ALIGN_START_ERROR"); return;
    }
    robot_diag_report(); cmd_param_report();
    if (s_xy_owner == 39u)
        send("OK XY_START BUCKET_DIRECT no_QR=1 task4/digit0_requested; new_ACK_and_coordinates_required; g/a/0 cancels; no_arm_or_laser");
    else
        send("OK XY_START QR_WAIT stopped=1; complete_fresh_QR_before_motion; g/a/0 cancels; no_arm_or_laser");
    xy_trial_report();
}

static void xy_trial_counts_start(void)
{
    for (int i = 0; i < 4; ++i) s_xy_last_counts[i] = ctrl_enc_total(i);
    s_xy_phase_t0 = HAL_GetTick();
}

static int xy_trial_counts_changed(void)
{
    int changed = 0;
    for (int i = 0; i < 4; ++i) {
        int32_t value = ctrl_enc_total(i);
        if (value != s_xy_last_counts[i]) changed = 1;
        s_xy_last_counts[i] = value;
    }
    return changed;
}

static void xy_trial_turn_done(int success)
{
    s_msel = s_xy_owner; s_round = R_RUN; s_active_test = s_xy_test;
    s_xy_state = XT_ACTIVE;
    vision_align_test_notify_turn_result(success);
    if (!success) { xy_trial_stop("TURN_FAILED"); return; }
    vision_align_test_status(&s_xy_snapshot);
    if (s_xy_snapshot.state == VAT_STOPPED) { xy_trial_stop(s_xy_snapshot.reason); return; }
    send("OK XY_TURN180_DONE bucket_new_request=1 new_heading_reference=1; reject_pre_turn_coordinates; fresh_XY_required");
}

static void xy_trial_poll(void)
{
    uint32_t now = HAL_GetTick();
    if (!xy_trial_active()) return;
    if (!bench_ok() || run_aborted()) { xy_trial_stop("ABORT"); return; }
    if ((uint32_t)(now - s_xy_report_t0) >= T_TARGET35_REPORT_MS) {
        s_xy_report_t0 = now; xy_trial_report();
    }
    if (s_xy_state == XT_QR_WAIT) {
        motion_brake();
        if (proto_scene_status() < 0) { xy_trial_stop("QR_LINK_ERROR"); return; }
        if (!proto_qr_get(s_xy_qr)) return;
        if (!vision_align_test_start(s_xy_owner, s_xy_qr)) {
            xy_trial_stop("ALIGN_START_ERROR"); return;
        }
        s_xy_state = XT_ACTIVE;
        send("OK XY_QR_VALID selected_task_requested; x190 both_axes_tol10; y_small_left/y_large_right; X20 Y30 step3mm cap250ms; post_Y_yawfix_then_new_XY");
        return;
    }
    if (s_xy_state >= XT_TURN_STILL && s_xy_state <= XT_TURN_RUN) {
        if (!imu_ok() || !isfinite(imu_heading_deg()) || !isfinite(imu_leg_heading_deg())) {
            xy_trial_stop("IMUERR"); return;
        }
        if (proto_scene_status() < 0) { xy_trial_stop("BUCKET_LINK_ERROR"); return; }
        if (s_xy_state == XT_TURN_RUN) { tick(); return; }
        motion_brake();
        if (xy_trial_counts_changed()) {
            s_xy_state = XT_TURN_STILL; s_xy_phase_t0 = now; return;
        }
        if (s_xy_state == XT_TURN_STILL) {
            if ((uint32_t)(now - s_xy_phase_t0) < T_DIST_STILL_MS) return;
            if (!imu_zero_leg_heading()) { xy_trial_stop("IMUERR"); return; }
            s_xy_state = XT_TURN_WAIT; s_xy_phase_t0 = now;
            return;
        }
        if ((uint32_t)(now - s_xy_phase_t0) < NAV_SETTLE_MS) return;
        s_msel = 22; s_seq_prepared = 1u; s_xy_state = XT_TURN_RUN;
        mode_start(); /* Existing +180 profile, skip its blocking preparation. */
        s_seq_prepared = 0u; s_active_test = s_xy_test;
        if (s_round != R_RUN) xy_trial_stop("TURN_START_ERROR");
        return;
    }
    vision_align_test_poll();
    vision_align_test_status(&s_xy_snapshot);
    if (s_xy_snapshot.state == VAT_STOPPED) { xy_trial_stop(s_xy_snapshot.reason); return; }
    if (s_xy_snapshot.state == VAT_DONE) {
        static char b[128];
        s_xy_state = XT_DONE; s_msel = s_xy_owner; s_round = R_DONE;
        motion_brake(); bp_laser_set(0);
        snprintf(b, sizeof b, "REC type=XY test=%lu mode=%u status=DONE brake=1 laser=0 no_grab=1 stay_here=1",
                 (unsigned long)s_xy_test, (unsigned)s_xy_owner);
        send(b); return;
    }
    if (vision_align_test_take_turn_request()) {
        s_xy_state = XT_TURN_STILL; xy_trial_counts_start();
        send("OK XY_BALL_HOLD5_DONE bucket_requested_before_turn=1; preparing_shared_180; turn_coordinates_ignored");
    }
}

/* BT 字节喂入(robot_bt_service 每轮从环里取出调):\r/\n 触发成行、超长丢、其余攒行记时间戳 */
void test_feed(uint8_t c)
{
    if (c == '\r' || c == '\n') { flush_line(); return; }
    if (s_len >= T_LINE_MAX) { s_over = 1; s_last = HAL_GetTick(); return; }
    s_line[s_len++] = (char)c;
    s_last = HAL_GetTick();
}

/* IMU 连续读数刷帧(imu_mon 开时每 ~250ms 一帧):只在 BOOT 空闲、没在打字时发,
 * 免得插话打断半行命令/测试应答;没解析出数据就打 -- 当查线用 */
static void imu_mon(void)
{
    if (!s_imu_mon) return;
    if (!bench_ok()) return;                          /* 开跑/非 BOOT 不刷 */
    if (s_msel != R_FREE || s_round != R_READY || s_len != 0u) return;
    if ((uint32_t)(HAL_GetTick() - s_mon_last) < 250u) return;
    s_mon_last = HAL_GetTick();
    char b[64];
    snprintf(b, sizeof b, "IMU %s Y=%.1f P=%.1f R=%.1f",
             imu_ok() ? "ok" : "--", imu_yaw_deg(), imu_pitch_deg(), imu_roll_deg());
    send(b);
}

/* ===== 旧版上电自动自检代码保留，但 BENCH_AUTO=0 时不编入动作 =====
 * BENCH_AUTO=1:上电不用任何命令,自动轮0→轮1→轮2→轮3 依次直通转 BENCH_RUN_MS,
 * 转完停下记录该轮编码器计数,间隔 BENCH_GAP_MS 换下一轮；四轮各跑一次后永久停车。
 * 看什么:① 轮子转不转、哪一轮转(认轮号↔位置) ② 转向对不对
 * ③ 蓝牙 BOOT_WHEEL 的 raw(硬件原始计数)及 count(校正计数)是否随轮胎标记变化。
 * ⚠️ 只能架空上电：先等 BENCH_DELAY_MS(默认 5s) 才开始转。
 * 后续落地测试须把这个宏改回 0 重新编译。 */
/* 自检推进(每 20ms 由 test_poll 调;非阻塞,四轮依次各转一次) */
static void bench_auto_tick(void)
{
#if BENCH_AUTO == 1u
    uint32_t now = HAL_GetTick();
    if (!s_bench_boot_ok) { s_bench_boot = now; s_bench_boot_ok = 1u; }  /* 记上电时刻 */
    if ((uint32_t)(now - s_bench_boot) < BENCH_DELAY_MS) return;  /* 上电先等,别一上电就转 */
    if (s_bench_phase == 3) return;
    if (s_go || !bench_ok() || s_msel != R_FREE) {
        ctrl_stop_all();                /* 蓝牙进入调试/主程序时必须中止自动转轮 */
        s_bench_phase = 3;
        return;
    }

    if (s_bench_phase == 0) {            /* 起测当前轮:清零计数 → 直通给 duty */
        char b[80];
        ctrl_enc_reset_all();
        ctrl_set_duty_open(g_bench_wheel, BENCH_DUTY);
        s_bench_t0 = now;
        s_bench_phase = 1;
        snprintf(b, sizeof b, "AUTO wheel=%d duty=%d start", g_bench_wheel, BENCH_DUTY);
        send(b);
        return;
    }
    if (s_bench_phase == 1) {            /* 转中:到时刹停，等稳定后再读计数 */
        if ((uint32_t)(now - s_bench_t0) >= BENCH_RUN_MS) {
            ctrl_stop_all();
            s_bench_t0 = now;
            s_bench_phase = 2;
        }
        return;
    }
    if ((uint32_t)(now - s_bench_t0) >= BENCH_GAP_MS) {
        char b[160];
        g_bench_cnt[g_bench_wheel] = ctrl_enc_total(g_bench_wheel);
        g_bench_raw_cnt[g_bench_wheel] = bp_enc_raw_total(g_bench_wheel);
        snprintf(b, sizeof b,
                 "REC type=BOOT_WHEEL wheel=%d duty=%d ms=%lu raw=%ld count=%ld raw_all=%ld,%ld,%ld,%ld",
                 g_bench_wheel, BENCH_DUTY, (unsigned long)BENCH_RUN_MS,
                 (long)g_bench_raw_cnt[g_bench_wheel], (long)g_bench_cnt[g_bench_wheel],
                 (long)bp_enc_raw_total(0), (long)bp_enc_raw_total(1),
                 (long)bp_enc_raw_total(2), (long)bp_enc_raw_total(3));
        send(b);
        if (g_bench_wheel >= 3) {
            ctrl_stop_all();
            s_bench_phase = 3;
            send("AUTO done wheels=4 stopped=1");
        } else {
            g_bench_wheel++;
            s_bench_phase = 0;
        }
    }
#elif BENCH_AUTO == 3u
    /* 直走测试:上电等 BENCH_DELAY_MS → 四轮同向前 BENCH_FWD_MS → 停 BENCH_FWD_GAP → 循环。
     * 走开环直通,不经过 IK/速度环 —— 先看纯机械+接线的直走效果。
     * ⚠️ 轮子会真的往前跑!先架空看方向,放地上跑要留出 1~2m 空间,人站边上随时断电。 */
    uint32_t now = HAL_GetTick();
    if (!s_bench_boot_ok) { s_bench_boot = now; s_bench_boot_ok = 1u; }
    if ((uint32_t)(now - s_bench_boot) < BENCH_DELAY_MS) return;  /* 上电先等 */
    if (s_go || !bench_ok()) return;
    if (s_msel != R_FREE) return;

    if (s_bench_phase == 0) {                            /* 起跑:四轮同向给 duty */
        for (int m = 0; m < MOTOR_NUM; m++) ctrl_set_duty_open(m, BENCH_DUTY);
        s_bench_t0 = now;
        s_bench_phase = 1;
        return;
    }
    if (s_bench_phase == 1) {                            /* 跑中 */
        if ((uint32_t)(now - s_bench_t0) >= BENCH_FWD_MS) {
            ctrl_stop_all();
            s_bench_t0 = now;
            s_bench_phase = 2;
        }
        return;
    }
    if ((uint32_t)(now - s_bench_t0) >= BENCH_FWD_GAP) s_bench_phase = 0;   /* 停够 → 再跑 */
#endif
}

/* ===== 周期主体:关线成行 + 动作推进(每 ~20ms 一次) ===== */
void test_poll(void)
{
    if (s_len != 0u && (uint32_t)(HAL_GetTick() - s_last) >= T_IDLE_MS)
        flush_line();   /* 无 CR:空闲成行(选号/单键 g/0/a 用) */
    if (s_msel == VISION_DIAG_MODE) { vision_diag_poll(); return; }
    if (s_msel == TARGET_TRIAL_MODE) { target35_poll(); return; }
    if (xy_trial_selected()) { xy_trial_poll(); return; }
    if (route_seq_active()) route_seq_poll();
    if (s_seq_state != SQ_STILL && s_seq_state != SQ_WAIT && s_seq_state != SQ_QR_WAIT &&
        s_seq_state != SQ_BUCKET_ALIGN && s_seq_state != SQ_MANUAL_D_WAIT)
        tick();         /* 段走/回程/步进的非阻塞推进(g 随时可插) */
    bench_auto_tick();  /* 仅在显式开启 BENCH_AUTO 时执行 */
    imu_mon();          /* IMU 读数(若开着) */
}

/* 把攒好的一行交给 run_cmd 执行并清空;超长行(s_over)直接丢不执行 */
static void flush_line(void)
{
    if (s_len == 0u) { s_over = 0; return; }
    s_line[s_len] = '\0';
    if (!s_over) run_cmd(s_line);
    s_len = 0; s_over = 0;
}

/* ---- 每 20ms 的动作推进 ---- */
static void tick(void)
{
    if (s_round == R_RUN) {
        if (s_msel == 5 || s_msel == 6) {               /* 段走到点自动停 */
            if ((uint32_t)(HAL_GetTick() - s_leg_start) >= s_leg_ms) {
                s_leg_run = s_leg_ms;
                motion_brake();
                s_round = R_DONE;
                char b[40];
                snprintf(b, sizeof b, "OK LEG_DONE cmd=%dmm (g3 return)", (int)s_d);
                send(b);
            }
        } else if (s_msel == 11 || s_msel == 12) {      /* 慢伸,每 tick 一步 */
            arm_stepper_step(s_axis);
            s_steps++;
        } else if (jog_mode()) {                         /* 一拍一步；只在完整到点后才可能自动回 */
            static char b[112];
            arm_stepper_step(s_axis);
            s_jog_done++;
            if (s_jog_done >= (uint32_t)(s_jog_request < 0 ? -s_jog_request : s_jog_request)) {
                s_round = R_DONE;
                s_jog_hold_t0 = HAL_GetTick();
                snprintf(b, sizeof b, "REC type=JOG test=%lu mode=%d axis=%d dir=%d out=%lu status=%s",
                         (unsigned long)s_active_test, s_msel, s_axis, s_jog_request < 0 ? 0 : 1,
                         (unsigned long)s_jog_done,
                         jog_return_mode() ? "WAIT_2S_THEN_RETURN" : "DONE_NO_RETURN");
                send(b);
            }
        } else if (s_msel == 13) {                      /* 手转一圈后停稳，以最后几帧为准 */
            uint32_t now = HAL_GetTick();
            if ((uint32_t)(now - s_enc_report_t0) >= T_ENC_REPORT_MS) {
                static char b[80];
                s_enc_report_t0 = now;
                s_enc_report_seq++;
                snprintf(b, sizeof b, "ENC n=%lu c=%ld,%ld,%ld,%ld",
                         (unsigned long)s_enc_report_seq,
                         (long)ctrl_enc_total(0), (long)ctrl_enc_total(1),
                         (long)ctrl_enc_total(2), (long)ctrl_enc_total(3));
                send(b);
            }
        } else if (s_msel == 14) {                       /* IMU 本轮统计 */
            imu_stat_take();
        } else if (s_msel == 19) {                       /* 架空短促自转符号检查 */
            if (!imu_ok()) turn_begin_settle("IMUERR");
            else if ((uint32_t)(HAL_GetTick() - s_meas_t0) >= T_TURN_SIGN_MS)
                turn_begin_settle("DONE");
        } else if (turn_closed_loop_mode()) {             /* +90/+180/-90 闭环，自转后检查惯性 */
            float e, w, yaw;
            float target = turn_target_deg();
            uint32_t now;
            if (!imu_ok()) { turn_begin_settle("IMUERR"); return; }
            yaw = imu_leg_heading_deg();
            if (turn_angle_outside(yaw)) {
                turn_begin_settle("ANGLE_LIMIT"); return;
            }
            if ((uint32_t)(HAL_GetTick() - s_meas_t0) >= turn_max_ms()) {
                turn_begin_settle("TIMEOUT"); return;
            }
            e = target - yaw;
            if (turn_abs(e) <= T_TURN_TOL_DEG) {
                turn_begin_settle("DONE"); return;
            }
            w = T_TURN_KP * e;
            if (w > T_TURN_MAX_W) w = T_TURN_MAX_W;
            if (w < -T_TURN_MAX_W) w = -T_TURN_MAX_W;
            if (w > 0.0f && w < T_TURN_MIN_W) w = T_TURN_MIN_W;
            if (w < 0.0f && w > -T_TURN_MIN_W) w = -T_TURN_MIN_W;
            motion_vel_set(0.0f, 0.0f, w);
            now = HAL_GetTick();
            if ((uint32_t)(now - s_turn_trace_t0) >= T_TURN_TRACE_MS) {
                static char b[150];
                char yaw_s[20];
                int16_t target[4];
                float actual[4];
                s_turn_trace_t0 = now;
                format_deg2(yaw_s, yaw);
                motion_ik(0.0f, 0.0f, w, target);
                ctrl_get_rpm_fast_all(actual);
                snprintf(b, sizeof b,
                         "TRC type=%s test=%lu ms=%lu yaw=%s tgt=%d,%d,%d,%d rpm=%d,%d,%d,%d",
                         turn_record_type(),
                         (unsigned long)s_active_test,
                         (unsigned long)(now - s_meas_t0), yaw_s,
                         (int)target[0], (int)target[1], (int)target[2], (int)target[3],
                         (int)actual[0], (int)actual[1], (int)actual[2], (int)actual[3]);
                send(b);
            }
        } else if (s_msel == 21) {                       /* 架空短测速度环正反换向 */
            uint32_t now = HAL_GetTick();
            uint32_t elapsed = (uint32_t)(now - s_meas_t0);
            float rpm[4];
            char b[120];
            if (elapsed >= 2u * T_SPEED_PROBE_PHASE_MS) {
                motion_brake();
                s_turn_settle_t0 = now;
                s_round = R_BRAKE;
                return;
            }
            motion_vel_set(0.0f, 0.0f,
                           elapsed < T_SPEED_PROBE_PHASE_MS ? T_SPEED_PROBE_W : -T_SPEED_PROBE_W);
            if ((uint32_t)(now - s_speed_probe_trace_t0) >= T_SPEED_PROBE_TRACE_MS) {
                s_speed_probe_trace_t0 = now;
                ctrl_get_rpm_fast_all(rpm);
                snprintf(b, sizeof b,
                         "TRC type=YAWSPD test=%lu phase=%c ms=%lu rpm=%d,%d,%d,%d",
                         (unsigned long)s_active_test,
                         elapsed < T_SPEED_PROBE_PHASE_MS ? 'P' : 'N',
                         (unsigned long)elapsed,
                         (int)rpm[0], (int)rpm[1], (int)rpm[2], (int)rpm[3]);
                send(b);
            }
        } else if (s_msel == 23) {                       /* 架空同向开环→闭环，定时自动刹停 */
            uint32_t now = HAL_GetTick();
            uint32_t elapsed = (uint32_t)(now - s_fwd_phase_t0);
            if (s_fwd_phase == 0u && elapsed >= T_FWD_OPEN_MS) {
                static char b[120];
                motion_brake();
                for (int i = 0; i < 4; i++) s_fwd_open_count[i] = ctrl_enc_total(i);
                snprintf(b, sizeof b,
                         "TRC type=FWDPULSE test=%lu phase=OPEN c=%ld,%ld,%ld,%ld",
                         (unsigned long)s_active_test,
                         (long)s_fwd_open_count[0], (long)s_fwd_open_count[1],
                         (long)s_fwd_open_count[2], (long)s_fwd_open_count[3]);
                send(b);
                s_fwd_phase = 1u;
                s_fwd_phase_t0 = now;
            } else if (s_fwd_phase == 1u && elapsed >= T_FWD_PAUSE_MS) {
                ctrl_enc_reset_all();
                motion_vel_set(T_FWD_V_MMS, 0.0f, 0.0f);
                s_fwd_phase = 2u;
                s_fwd_phase_t0 = now;
                s_fwd_trace_t0 = now;
                send("OK FWDPULSE phase=CLOSED target_v=80mm/s; auto_brake");
            } else if (s_fwd_phase == 2u) {
                if (elapsed >= T_FWD_CLOSED_MS) {
                    fwd_probe_report("DONE");
                } else if ((uint32_t)(now - s_fwd_trace_t0) >= T_FWD_TRACE_MS) {
                    static char b[152];
                    float actual[4];
                    int16_t target[4];
                    s_fwd_trace_t0 = now;
                    motion_ik(T_FWD_V_MMS, 0.0f, 0.0f, target);
                    ctrl_get_rpm_fast_all(actual);
                    snprintf(b, sizeof b,
                             "TRC type=FWDPULSE test=%lu phase=CLOSED ms=%lu tgt=%d,%d,%d,%d rpm=%d,%d,%d,%d",
                             (unsigned long)s_active_test, (unsigned long)elapsed,
                             (int)target[0], (int)target[1], (int)target[2], (int)target[3],
                             (int)actual[0], (int)actual[1], (int)actual[2], (int)actual[3]);
                    send(b);
                }
            }
        } else if (dist_mode()) {                        /* 编码器定距 + yaw 保持 */
            float d, remain, cmd;
            if (!imu_ok() || !isfinite(imu_leg_heading_deg()) || !isfinite(s_dist_heading0) ||
                !isfinite(s_dist_heading0 - imu_leg_heading_deg())) {
                dist_begin_finish(2u);
                return;
            }
            d = dist_odo() - s_dist_odo0;
            if (s_dist_target > 0.0f ? (d >= s_dist_target) : (d <= s_dist_target)) {
                dist_begin_finish(1u);
                return;
            }
            remain = s_dist_target - d;
            if (s_seq_state == SQ_RUN)
                cmd = motion_linear_ramp_step(&s_dist_ramp,
                                             (s_dist_target > 0.0f) ? s_v : -s_v,
                                             remain, 0.020f);
            else
                cmd = motion_linear_profile_step(&s_dist_ramp,
                                                 (s_dist_target > 0.0f) ? s_v : -s_v,
                                                 remain, 0.020f);
            {
                float orth_cmd = dist_ordinary_leg() ? step_orth_hold_cmd(dist_lateral(), s_dist_orth0) : 0.0f;
                /* 正 fff/bff 都给负 vy(车身左)，后退系数独立且不再恒0。
                 * 左移 cmd<0 给负 vx(车尾)，右移 cmd>0 给正 vx(车头)。 */
                if (!dist_lateral()) orth_cmd -= s_dist_ff_ratio * fabsf(cmd);
                if (dist_lateral()) orth_cmd += s_dist_ff_ratio * cmd;
                float w = s_dist_precise ? step_heading_hold_w_kp(s_dist_heading0, s_dist_heading_kp)
                                        : step_heading_hold_w(s_dist_heading0);
                if (s_dist_precise)
                    motion_vel_set_precise(dist_lateral() ? orth_cmd : cmd,
                                          dist_lateral() ? cmd : orth_cmd, w);
                else
                    motion_vel_set(dist_lateral() ? orth_cmd : cmd,
                                   dist_lateral() ? cmd : orth_cmd, w);
                if ((uint32_t)(HAL_GetTick() - s_dist_trace_t0) >= T_DIST_TRACE_MS) {
                    static char trace[192];
                    s_dist_trace_t0 = HAL_GetTick();
                    snprintf(trace, sizeof trace,
                             "TRC type=DIST test=%lu mode=%u ms=%lu axis_mm=%.1f orth_mm=%.1f yaw_deg=%.2f ykp=%.3f cmd_mms=%.1f w_rads=%.4f",
                             (unsigned long)s_active_test, (unsigned)s_msel,
                             (unsigned long)(s_dist_trace_t0 - s_meas_t0), d,
                             (dist_lateral() ? motion_odo_mm() : motion_lateral_odo_mm()) - s_dist_orth0,
                             imu_leg_heading_deg(), s_dist_heading_kp, cmd, w);
                    send(trace);
                }
            }
        }
    } else if (s_round == R_DONE && jog_return_mode()) {
        if ((uint32_t)(HAL_GetTick() - s_jog_hold_t0) >= T_JOG_RETURN_WAIT_MS) {
            arm_stepper_dir(s_axis, s_jog_request < 0 ? 1 : 0);
            s_back = 0u;
            s_round = R_RET;
            send("OK JOG_RETURN_START g/a/0 stops_no_further_steps");
        }
    } else if (s_round == R_DONE && s_msel == 28) {
        if ((uint32_t)(HAL_GetTick() - s_servo_hold_t0) >= T_SERVO_RETURN_WAIT_MS) {
            static char b[112];
            arm_claw_set_us(s_servo_origin_us);
            s_round = R_READY;
            snprintf(b, sizeof b,
                     "REC type=SERVO test=%lu mode=28 status=RETURN_COMMAND_SENT us=%u no_feedback=1",
                     (unsigned long)s_active_test, (unsigned)s_servo_origin_us);
            send(b);
        }
    } else if (s_round == R_BRAKE && s_msel == 21) {
        if ((uint32_t)(HAL_GetTick() - s_turn_settle_t0) >= T_SPEED_PROBE_SETTLE_MS)
            speed_probe_report("DONE");
    } else if (s_round == R_BRAKE && (s_msel == 19 || turn_closed_loop_mode())) {
        uint32_t now = HAL_GetTick();
        float yaw;
        if (!imu_ok()) { s_turn_result = "IMUERR"; turn_report(); return; }
        yaw = imu_leg_heading_deg();
        if (turn_closed_loop_mode() && turn_angle_outside(yaw)) {
            s_turn_result = "ANGLE_LIMIT";
            turn_report();
            return;
        }
        if (turn_closed_loop_mode() && strcmp(s_turn_result, "DONE") == 0) {
            if (turn_abs(turn_target_deg() - yaw) > T_TURN_TOL_DEG) {
                if ((uint32_t)(now - s_meas_t0) >= turn_max_ms()) {
                    s_turn_result = "TIMEOUT";
                } else {
                    s_round = R_RUN;    /* 惯性越界：反向微调，不能假报到位 */
                    return;
                }
            } else if (turn_abs(yaw - s_turn_settle_yaw) > TURN90_STILL_DEG) {
                s_turn_settle_t0 = now; /* 在容差内但仍在移动，重新等稳定 */
                s_turn_settle_yaw = yaw;
            }
        }
        if ((uint32_t)(now - s_turn_settle_t0) >= T_TURN_SETTLE_MS)
            turn_report();
    } else if (s_round == R_ALIGN && dist_mode()) {
        dist_align_poll();
    } else if (s_round == R_BRAKE && dist_mode()) {
        int changed = 0;
        for (int i = 0; i < 4; i++) {
            int32_t now = ctrl_enc_total(i);
            if (now != s_dist_last[i]) changed = 1;
            s_dist_last[i] = now;
        }
        if (changed) s_dist_still_t0 = HAL_GetTick();
        if ((uint32_t)(HAL_GetTick() - s_dist_still_t0) >= T_DIST_STILL_MS) {
            dist_save_translation_end();
            if (s_dist_reason == 1u && s_dist_align_enabled && !run_aborted() && imu_ok() &&
                isfinite(imu_leg_heading_deg()) && isfinite(s_dist_heading0)) {
                s_dist_align_t0 = HAL_GetTick(); s_dist_align_started = 1u;
                s_dist_align_hold = 0u; s_round = R_ALIGN;
                send("OK DIST_YFIX original_heading_preserved; g/a/0 cancels");
            } else dist_finish_report();
        }
    } else if (s_round == R_RET) {
        if (s_msel == 5 || s_msel == 6) {               /* 回程:同速反向等时长 */
            if ((uint32_t)(HAL_GetTick() - s_leg_start) >= s_leg_ms) {
                motion_brake();
                s_round = R_READY;
                s_leg_run = 0u;
                send("OK RETURN_DONE ready_for_g1");
            }
        } else if (s_msel == 11 || s_msel == 12) {      /* 收臂:反向收已伸步数 */
            arm_stepper_step(s_axis);
            s_back++;
            if (s_back >= s_steps) {
                s_round = R_READY;
                s_steps = 0u; s_back = 0u;
                send("OK STEPPER_RETURN_DONE ready_for_g1");
            }
        } else if (jog_return_mode()) {
            static char b[128];
            arm_stepper_step(s_axis);
            s_back++;
            if (s_back >= s_jog_done) {
                s_round = R_READY;
                snprintf(b, sizeof b, "REC type=JOG test=%lu mode=%d axis=%d out=%lu back=%lu status=RETURN_DONE estimate_only=1",
                         (unsigned long)s_active_test, s_msel, s_axis,
                         (unsigned long)s_jog_done, (unsigned long)s_back);
                s_jog_request = 0; s_jog_done = 0u; s_back = 0u;
                send(b);
            }
        }
    }
}

/* ---- g1:启动动作 ---- */
static void mode_start(void)
{
    if (route_seq_selected()) {
        send(s_msel == CROSS_ONLY_MODE ? "ERR ROUTE37_USE_G" :
             s_msel == BUCKET_ROUTE_MODE ? "ERR ROUTE36_USE_G" :
             s_msel == ROUTE_NO_QR_MODE ? "ERR ROUTE34_USE_G" : "ERR ROUTE31_USE_G");
        return; /* never fall into the legacy stepper fallback */
    }
    if (s_v < 1.0f && ((s_msel >= 1 && s_msel <= 6) || dist_mode())) {
        send("ERR SET_V first, example v120 mm/s");
        return;
    }
    if ((s_msel == 5 || s_msel == 6 || dist_mode()) && s_d < 1.0f) {
        send(s_route_leg ? "ERR ROUTE_SET_D first; measure car-center leg and send d<mm>"
                         : "ERR SET_D first, example d400 mm");
        return;
    }
    motion_brake();   /* 清残留轮速目标,再按本模式设目标 */
    char b[48];
    if (s_msel >= 1 && s_msel <= 6) {
        if      (s_msel == 1 || s_msel == 5) { s_rvx =  1.0f; s_rvy =  0.0f; }
        else if (s_msel == 2)                 { s_rvx = -1.0f; s_rvy =  0.0f; }
        else if (s_msel == 3 || s_msel == 6)  { s_rvx =  0.0f; s_rvy = -1.0f; } /* 左=负横移(mission 口径) */
        else                                  { s_rvx =  0.0f; s_rvy =  1.0f; }
        motion_vel_set(s_rvx * s_v, s_rvy * s_v, 0.0f);
        if (s_msel == 5 || s_msel == 6) {
            s_leg_start = HAL_GetTick();
            s_leg_ms = (uint32_t)(s_d * 1000.0f / s_v);
            if (s_leg_ms == 0u) s_leg_ms = 1u;
            s_leg_run = 0u;
            snprintf(b, sizeof b, "OK %s leg=%dmm (g2 stop/auto stop)", s_mname[s_msel], (int)s_d);
        } else {
            snprintf(b, sizeof b, "OK %s v=%dmm/s (g2 stop)", s_mname[s_msel], (int)s_v);
        }
        s_round = R_RUN;
        send(b);
        return;
    }
    if (s_msel >= 7 && s_msel <= 10) {          /* 单轮驱动冒烟 + 计数 readout */
        if (!s_p_valid || s_p == 0.0f) { send("ERR SET_P duty=-199..-1 or 1..199, example p120/p-120"); return; }
        begin_recorded_test();
        s_wheel = s_msel - 7;
        ctrl_enc_reset_all();                    /* 本轮从 0 起计 */
        s_meas_t0 = HAL_GetTick();
        ctrl_set_duty_open(s_wheel, (int16_t)s_p);   /* 直通 duty,验驱动/方向(不开环别走速度环) */
        snprintf(b, sizeof b, "OK WHEEL wheel=%d duty=%d (g2 stop)", s_wheel, (int)s_p);
        s_round = R_RUN;
        send(b);
        return;
    }
    if (s_msel == 13) {                         /* 四轮编码器手转/单圈 CPR */
        begin_recorded_test();
        ctrl_coast_all();                       /* 手转必须自由滑行，不能让 TB6612 短接刹车 */
        ctrl_enc_reset_all();
        s_meas_t0 = HAL_GetTick();
        s_enc_report_t0 = s_meas_t0;
        s_enc_report_seq = 0u;
        s_round = R_RUN;
        send("ENC START");
        return;
    }
    if (s_msel == 14) {                         /* IMU 静止漂移/转向符号统计 */
        motion_brake();
        if (!imu_zero_leg_heading()) { send("ERR IMU_NO_VALID_FRAME"); return; }
        begin_recorded_test();
        imu_stat_clear();
        s_meas_t0 = HAL_GetTick();
        imu_stat_take();
        s_round = R_RUN;
        send("OK IMU_START keep_still_for_drift; g_to_stop");
        return;
    }
    if (s_msel == 19 || turn_closed_loop_mode()) {
        if (!s_seq_prepared && !step_prepare_leg()) { send("ERR TURN_PREP check_IMU_and_stop_state"); return; }
        if (!xy_trial_active()) begin_recorded_test(); /*41 preserves one test id through its +180. */
        s_meas_t0 = HAL_GetTick();
        s_turn_trace_t0 = s_meas_t0;
        s_round = R_RUN;
        if (s_msel == 19) {
            /* 只验证正 w 方向：右侧倒转、左侧前转；先架空观察，绝不跑整场。 */
            int16_t target[4];
            ctrl_enc_reset_all();
            motion_ik(0.0f, 0.0f, 1.0f, target);
            for (int i = 0; i < 4; i++)
                ctrl_set_duty_open(i, target[i] >= 0 ? T_TURN_SIGN_DUTY : -T_TURN_SIGN_DUTY);
            send("OK TURN_SIGN 250ms pulse; auto_brake; g/a emergency_stop");
        } else if (s_msel == 20 || s_msel == 30) {
            static char turn_msg[112]; /* keep the longer angle report off the task stack */
            snprintf(turn_msg, sizeof turn_msg, "OK %s target=%+ddeg hold_tol=0.3deg; comp=%ddeg; g/a stop",
                     turn_record_type(), (int)turn_target_deg(),
                     (int)(turn_abs(turn_target_deg()) - TURN90_TARGET_DEG));
            send(turn_msg);
        } else {
            send("OK TURN180 target=+180deg hold_tol=0.3deg; g/a stop");
        }
        return;
    }
    if (s_msel == 21) {
        int16_t target[4];
        static char pb[100];
        begin_recorded_test();
        ctrl_enc_reset_all();
        motion_ik(0.0f, 0.0f, T_SPEED_PROBE_W, target);
        s_meas_t0 = HAL_GetTick();
        s_speed_probe_trace_t0 = s_meas_t0;
        s_round = R_RUN;
        motion_vel_set(0.0f, 0.0f, T_SPEED_PROBE_W);
        snprintf(pb, sizeof pb,
                 "OK YAWSPD 800ms P then 800ms N target_rpm=%d,%d,%d,%d; g/a stop",
                 (int)target[0], (int)target[1], (int)target[2], (int)target[3]);
        send(pb);
        return;
    }
    if (s_msel == 23) {
        begin_recorded_test();
        ctrl_enc_reset_all();
        for (int i = 0; i < 4; i++) s_fwd_open_count[i] = 0;
        s_fwd_phase = 0u;
        s_fwd_phase_t0 = HAL_GetTick();
        s_round = R_RUN;
        for (int i = 0; i < 4; i++) ctrl_set_duty_open(i, T_FWD_PULSE_DUTY);
        send("OK FWDPULSE SUSPENDED_ONLY open_duty=+45/400ms gap=800ms closed_v=80/650ms; auto_brake; g/a stop");
        return;
    }
    if (dist_mode()) {                          /* 落地四方向编码器定距 */
        if (!s_seq_prepared && !step_prepare_leg()) { send("ERR DIST_PREP check_IMU_and_stop_state"); return; }
        int profile;
        begin_recorded_test();
        s_dist_heading_kp = dist_heading_kp_get(&profile);
        s_dist_heading_profile = (uint8_t)profile;
        s_dist_precise = 1u; /* manual 15..18 and mode31 soft-stop distance legs */
        if      (s_msel == 15) { s_rvx =  1.0f; s_rvy =  0.0f; }
        else if (s_msel == 16) { s_rvx = -1.0f; s_rvy =  0.0f; }
        else if (s_msel == 17) { s_rvx =  0.0f; s_rvy = -1.0f; }
        else                    { s_rvx =  0.0f; s_rvy =  1.0f; }
        ctrl_enc_reset_all();
        s_dist_odo0 = dist_odo();
        s_dist_heading0 = imu_leg_heading_deg();
        s_dist_orth0 = dist_lateral() ? motion_odo_mm() : motion_lateral_odo_mm();
        s_dist_target = (s_msel == 16 || s_msel == 17) ? -s_d : s_d;
        /* Normal straight trials: independent fff/bff at EVERY supported v.
         * Mechanical squaring/crossing deliberately bypass feed-forward. */
        s_dist_ff_ratio = !dist_ordinary_leg() ? 0.0f : (s_msel == 15) ?
            (s_seq_state == SQ_RUN ? s_route_forward_ff_ratio : s_forward_ff_ratio)
            : (s_msel == 16) ? (s_seq_state == SQ_RUN ? s_route_backward_ff_ratio : s_backward_ff_ratio)
            : (s_msel == 17) ? (s_seq_state == SQ_RUN ? s_route_left_ff_ratio :
                (s_v == 300.0f ? s_left_ff_ratio : 0.0f))
            : (s_msel == 18) ? (s_seq_state == SQ_RUN ? s_route_right_ff_ratio :
                (s_v == 300.0f ? s_right_ff_ratio : 0.0f)) : 0.0f;
        s_dist_align_enabled = (uint8_t)(s_dist_align_on && dist_ordinary_leg());
        s_dist_align_started = s_dist_align_hold = s_dist_end_saved = 0u;
        s_dist_align_t0 = s_dist_align_stable_t0 = 0u;
        if (s_seq_state == SQ_RUN)
            motion_ramp_init(&s_dist_ramp, ROUTE_TEST_ACC_MMS2, ROUTE_TEST_DEC_MMS2);
        else
            motion_linear_ramp_init(&s_dist_ramp);
        s_meas_t0 = HAL_GetTick();
        s_dist_trace_t0 = s_meas_t0;
        s_round = R_RUN;
        snprintf(s_dist_report, sizeof s_dist_report,
                 "OK %s leg=%u %s start cmd=%.0fmm v=%.0fmm/s ykp=%.3f yaw_hold=%u precise=%u ff_ratio=%.5f ff_vx=%.1f ff_vy=%.1fmm/s yfix=%u; auto_stop_or_g",
                 s_route_leg ? "ROUTE" : "DIST", (unsigned)s_route_leg,
                 s_mname[s_msel], s_dist_target, s_v, s_dist_heading_kp,
                 (unsigned)(s_dist_heading_kp > 0.0f), (unsigned)s_dist_precise, s_dist_ff_ratio,
                 dist_lateral() ? s_dist_ff_ratio * (s_msel == 17 ? -s_v : s_v) : 0.0f,
                 !dist_lateral() ? -s_dist_ff_ratio * s_v : 0.0f, (unsigned)s_dist_align_enabled);
        send(s_dist_report);
        cmd_param_report(); /* once before the first motor tick; no manual param needed */
        return;
    }
    if (jog_mode()) {
        static char jb[112];
        if (s_jog_request == 0) { send("ERR SET_N first: nl1..nl50 or nr1..nr50; start away from stop"); return; }
        begin_recorded_test();
        s_axis = jog_axis();
        s_jog_done = 0u; s_back = 0u; s_jog_hold_t0 = 0u;
        arm_stepper_dir(s_axis, s_jog_request < 0 ? 0 : 1);
        s_round = R_RUN;
        snprintf(jb, sizeof jb, "OK JOG test=%lu mode=%d axis=%d dir=%d n=%lu %s; g_stops_no_return",
                 (unsigned long)s_active_test, s_msel, s_axis, s_jog_request < 0 ? 0 : 1,
                 (unsigned long)(s_jog_request < 0 ? -s_jog_request : s_jog_request),
                 jog_return_mode() ? "wait2s_then_reverse" : "no_return");
        send(jb);
        return;
    }
    if (servo_mode()) {
        static char sb[112];
        if (s_servo_target_us == 0u) { send("ERR SET_U first: u1000..u1800; test without horn first"); return; }
        begin_recorded_test();
        s_servo_origin_us = arm_claw_command_us();
        arm_claw_set_us(s_servo_target_us);
        s_servo_hold_t0 = HAL_GetTick();
        s_round = R_DONE;
        snprintf(sb, sizeof sb,
                 "REC type=SERVO test=%lu mode=%d from_us=%u to_us=%u status=%s no_feedback=1",
                 (unsigned long)s_active_test, s_msel, (unsigned)s_servo_origin_us,
                 (unsigned)s_servo_target_us,
                 s_msel == 28 ? "WAIT_2S_THEN_RETURN" : "HOLD_NO_RETURN");
        send(sb);
        return;
    }
    /* 11/12 旧步进:慢伸,接触时 g2 会合爪；未标定机构禁用 */
    begin_recorded_test();
    s_axis = s_msel - 11;
    s_axis_dir = s_ext_dir[s_axis];
    s_steps = 0;
    arm_stepper_dir(s_axis, s_axis_dir);
    snprintf(b, sizeof b, "OK STEPPER axis=%d extending (g to stop)", s_axis);
    s_round = R_RUN;
    send(b);
}

static int route_seq_active(void)
{
    return s_seq_state == SQ_STILL || s_seq_state == SQ_WAIT ||
           s_seq_state == SQ_RUN || s_seq_state == SQ_QR_WAIT ||
           s_seq_state == SQ_BUCKET_ALIGN || s_seq_state == SQ_MANUAL_D_WAIT;
}

static int route_seq_selected(void)
{
    return s_msel == ROUTE_TEST_MODE || s_msel == ROUTE_NO_QR_MODE ||
           s_msel == BUCKET_ROUTE_MODE || s_msel == CROSS_ONLY_MODE;
}

static unsigned route_seq_stage_count(void)
{
    return s_seq_mode == ROUTE_TEST_MODE ? ROUTE31_STAGES :
           s_seq_mode == CROSS_ONLY_MODE ? CROSS37_STAGES :
           s_seq_mode == BUCKET_ROUTE_MODE ? BUCKET_ROUTE_STAGES : ROUTE_TEST_STAGES;
}

static const RouteTestLeg *route_seq_leg(void)
{
    return s_seq_mode == ROUTE_TEST_MODE ? &s_route31_plan[s_seq_stage]
         : s_seq_mode == CROSS_ONLY_MODE ? &s_cross37_plan[s_seq_stage]
         : s_seq_mode == BUCKET_ROUTE_MODE ? &s_bucket36_plan[s_seq_stage]
                                         : &s_route_test_plan[s_seq_stage];
}

static int route_seq_bucket_enabled(void)
{
    /* Private31 and independent36/37 do not request or align the bucket. */
    return s_seq_mode == ROUTE_NO_QR_MODE;
}

static unsigned route_seq_bucket_align_stage(void)
{
    return s_seq_mode == BUCKET_ROUTE_MODE ? BUCKET36_ALIGN_STAGE : ROUTE_TEST_ALIGN_STAGE;
}

static unsigned route_seq_bucket_back_stage(void)
{
    return s_seq_mode == BUCKET_ROUTE_MODE ? BUCKET36_BACK_STAGE : ROUTE_TEST_BACK_STAGE;
}

static void route_seq_end(const char *status)
{
    static char b[224];
    int mode = s_msel;
    float yaw = imu_ok() ? imu_leg_heading_deg() : 9999.0f;
    motion_brake();                     /* cancel chain before any later correction */
    s_dist_align_enabled = s_dist_align_hold = 0u;
    if (s_seq_mode == ROUTE_TEST_MODE || route_seq_bucket_enabled() || s_seq_mode == BUCKET_ROUTE_MODE) {
        s_bucket36_phase = BA_OFF; s_bucket36_cmd = 0.0f;
        s_bucket36_back_mm = 0u;
        step_vision_receive_end(); bp_laser_set(0);
    }
    proto_qr_cancel(); /* R1 preparation/running and stationary wait all cancel. */
    s_seq_state = strcmp(status, "DONE") == 0 ? SQ_DONE : SQ_STOPPED;
    s_seq_prepared = 0u;
    snprintf(b, sizeof b,
             "REC type=ROUTE_SEQ run=%lu status=%s step=%u/%u mode=%d test=%lu fore_mm=%.1f lat_mm=%.1f yaw_deg=%.2f c=%ld,%ld,%ld,%ld no_auto_return=1",
             (unsigned long)s_seq_run, status, (unsigned)(s_seq_stage + 1u), route_seq_stage_count(), mode,
             (unsigned long)s_active_test, motion_odo_mm(), motion_lateral_odo_mm(), yaw,
             (long)ctrl_enc_total(0), (long)ctrl_enc_total(1),
             (long)ctrl_enc_total(2), (long)ctrl_enc_total(3));
    s_msel = s_seq_mode; s_round = R_DONE;
    send(b);
}

static void route_seq_prepare(void)
{
    static char b[176];
    const RouteTestLeg *leg = route_seq_leg();
    motion_brake();
    if (route_seq_bucket_enabled() && s_seq_stage == route_seq_bucket_align_stage()) {
        bucket36_begin(); return;
    }
    if (route_seq_bucket_enabled() && s_seq_stage == route_seq_bucket_back_stage() &&
        !s_bucket36_back_mm) {
        s_msel = s_seq_mode; s_route_leg = (uint8_t)(s_seq_stage + 1u);
        s_d = -1.0f; s_round = R_DONE; s_seq_state = SQ_MANUAL_D_WAIT;
        snprintf(b, sizeof b, "OK BUCKET%u_WAIT_D brake=1; send_d<mm>_to_start_backward_v100; g/a/0 cancels",
                 (unsigned)s_seq_mode);
        send(b);
        return;
    }
    s_msel = leg->mode; s_route_leg = (uint8_t)(s_seq_stage + 1u);
    s_v = leg->speed_mms;
    s_d = leg->distance_mm ? (float)leg->distance_mm : -1.0f;
    if (route_seq_bucket_enabled() && s_seq_stage == route_seq_bucket_back_stage())
        s_d = (float)s_bucket36_back_mm;
    s_round = R_READY; s_seq_state = SQ_STILL; s_seq_prepared = 0u;
    s_seq_still_t0 = HAL_GetTick();
    for (int i = 0; i < 4; i++) s_seq_last[i] = ctrl_enc_total(i);
    snprintf(b, sizeof b,
             "SEQ run=%lu step=%u/%u phase=PREP name=%s mode=%u d=%u v=%.0f turn=%d QR_gate_R1=%u tasks=0; g/a/0 stop",
             (unsigned long)s_seq_run, (unsigned)(s_seq_stage + 1u), route_seq_stage_count(), leg->name,
             (unsigned)leg->mode, (unsigned)(s_d > 0.0f ? s_d : 0.0f), leg->speed_mms,
             turn_closed_loop_mode() ? (int)turn_target_deg() : 0,
             (unsigned)(s_seq_mode == ROUTE_TEST_MODE));
    send(b);
}

static void route_seq_next(void)
{
    static char b[96];
    if (s_seq_state != SQ_RUN) return;
    snprintf(b, sizeof b, "SEQ run=%lu step=%u/%u phase=DONE test=%lu",
             (unsigned long)s_seq_run, (unsigned)(s_seq_stage + 1u), route_seq_stage_count(),
             (unsigned long)s_active_test);
    send(b);
    if (s_seq_stage == 0u && s_seq_mode == ROUTE_TEST_MODE) {
        motion_brake();
        proto_qr_begin(); /* Preserve a valid power-on/READY/R1 task tuple. */
        s_seq_state = SQ_QR_WAIT;
        s_seq_qr_report_t0 = HAL_GetTick();
        send("SEQ phase=QR_WAIT after_R1 stopped=1 valid_tuple_required=1 no_timeout_no_scan; g/a/0 stop");
        return;
    }
    if (s_seq_stage + 1u >= route_seq_stage_count()) { route_seq_end("DONE"); return; }
    s_seq_stage++;
    route_seq_prepare();              /* no synchronous wait; next poll serves g first */
}

static void route_seq_poll(void)
{
    uint32_t now = HAL_GetTick();
    if (!route_seq_active()) return;
    if (!bench_ok() || run_aborted()) { route_seq_end("ABORT"); return; }
    if (!imu_ok()) { route_seq_end("IMUERR"); return; }
    if (s_seq_state == SQ_BUCKET_ALIGN) { bucket36_poll(); return; }
    if (s_seq_state == SQ_MANUAL_D_WAIT) { motion_brake(); return; }
    if (s_seq_state == SQ_QR_WAIT) {
        int status = proto_scene_status();
        static char b[128];
        if (proto_qr_get(s_seq_qr)) {
            uint32_t pm = __get_PRIMASK();
            __disable_irq();
            s_seq_state = SQ_STILL;
            step_vision_receive_end(); /* Route31 ends only its local QR receive stage. */
            __set_PRIMASK(pm);
            snprintf(b, sizeof b, "SEQ run=%lu phase=QR_VALID QR=%ld,%ld,%ld; preparing_R2",
                     (unsigned long)s_seq_run,
                     (long)s_seq_qr[0], (long)s_seq_qr[1], (long)s_seq_qr[2]);
            send(b);
            s_seq_stage = 1u;
            route_seq_prepare();
        } else if ((uint32_t)(now - s_seq_qr_report_t0) >= 1000u) {
            s_seq_qr_report_t0 = now;
            snprintf(b, sizeof b, "SEQ run=%lu phase=QR_WAIT link=%d valid=%u stopped=1; g/a/0 stop",
                     (unsigned long)s_seq_run, status, (unsigned)proto_qr_get(0));
            send(b);
            vision_wire_report();
        }
        return; /* No camera result, timeout or failed request may start R2. */
    }
    if (s_seq_state == SQ_STILL) {
        int changed = 0;
        for (int i = 0; i < 4; i++) {
            int32_t count = ctrl_enc_total(i);
            if (count != s_seq_last[i]) changed = 1;
            s_seq_last[i] = count;
        }
        if (changed) s_seq_still_t0 = now;
        if ((uint32_t)(now - s_seq_still_t0) < T_DIST_STILL_MS) return;
        if (!imu_zero_leg_heading()) { route_seq_end("IMUERR"); return; }
        s_seq_state = SQ_WAIT; s_seq_wait_t0 = HAL_GetTick();
        return;
    }
    if (s_seq_state == SQ_WAIT && (uint32_t)(now - s_seq_wait_t0) >= NAV_SETTLE_MS) {
        s_seq_state = SQ_RUN;
        s_seq_prepared = 1u;          /* reuse actuator setup, not blocking preparation */
        mode_start();
        s_seq_prepared = 0u;
        if (s_seq_state == SQ_RUN && s_round != R_RUN) route_seq_end("START_FAILED");
    }
}

static void route_seq_g(void)
{
    if (route_seq_active()) { route_seq_end("STOP"); return; }
    if (s_seq_state != SQ_READY) {
        send(s_seq_mode == CROSS_ONLY_MODE
             ? "OK ROUTE37 stopped; manually_return_to_CROSS_START; select37_then_g_to_rerun"
             : s_seq_mode == BUCKET_ROUTE_MODE
             ? "OK ROUTE36 stopped; manually_return_to_START; select36_then_g_to_rerun"
             : s_seq_mode == ROUTE_NO_QR_MODE
             ? "OK ROUTE_SEQ stopped; manually_return_to_START; select34_then_g_to_rerun"
             : "OK ROUTE_SEQ stopped; manually_return_to_START; select31_then_g_to_rerun");
        return;
    }
    s_seq_run++;
    if (s_seq_run == 0u) s_seq_run = 1u;
    s_seq_stage = 0u;
    s_bucket36_back_mm = 0u; s_bucket36_phase = BA_OFF;
    memset(s_seq_qr, 0, sizeof s_seq_qr);
    if (s_seq_mode == ROUTE_TEST_MODE)
        proto_qr_begin(); /* Start moving only on g; preserve the boot QR request. */
    else {
        step_vision_receive_end(); /* Close background RX locally; never command the camera. */
        proto_qr_cancel();
    }
    robot_diag_report();              /* automatic provenance and parameter snapshot */
    cmd_param_report();
    route_seq_prepare();
}

static void bucket36_clear_sample(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq(); memset(&s_bucket36_sample, 0, sizeof s_bucket36_sample); __set_PRIMASK(pm);
    s_bucket36_packet = 0u; s_bucket36_sequence = 0u;
    s_bucket36_good = s_bucket36_seen = 0u;
}

static void bucket36_counts_start(void)
{
    for (int i = 0; i < 4; i++) s_seq_last[i] = ctrl_enc_total(i);
    s_bucket36_t0 = HAL_GetTick();
}

static int bucket36_counts_changed(void)
{
    int changed = 0;
    for (int i = 0; i < 4; i++) {
        int32_t count = ctrl_enc_total(i);
        if (count != s_seq_last[i]) changed = 1;
        s_seq_last[i] = count;
    }
    return changed;
}

static void bucket36_report(void)
{
    static char b[208];
    VisionDiagSample sample;
    ProtoStats stats;
    uint32_t now, pm = __get_PRIMASK();
    __disable_irq(); sample = s_bucket36_sample; proto_stats_get(&stats); now = HAL_GetTick(); __set_PRIMASK(pm);
    snprintf(b, sizeof b,
             "BKT%u run=%lu phase=%u task=4 digit=0 cx=%d point=500 band=495..505 img_w=%u seq=%u latest=%u age_ms=%lu good=%u/5 vx=%.0f laser=0",
             (unsigned)s_seq_mode, (unsigned long)s_seq_run, (unsigned)s_bucket36_phase,
             sample.seen ? sample.frame.cx : -1, (unsigned)sample.frame.img_w,
             (unsigned)sample.frame.sequence,
             (unsigned)(sample.seen && sample.packet_index == stats.obj &&
                        (uint32_t)(now - sample.tick) <= BUCKET36_FRESH_MS),
             (unsigned long)(sample.seen ? now - sample.tick : 0u),
             (unsigned)s_bucket36_good, s_bucket36_cmd);
    send(b);
}

static void bucket36_begin(void)
{
    static char b[192];
    motion_brake(); bp_laser_set(0); step_vision_receive_end();
    s_msel = s_seq_mode; s_route_leg = (uint8_t)(s_seq_stage + 1u);
    s_round = R_RUN; s_seq_state = SQ_BUCKET_ALIGN;
    bucket36_clear_sample(); s_bucket36_back_mm = 0u; s_bucket36_cmd = 0.0f;
    begin_recorded_test();
    s_bucket36_phase = BA_STILL; bucket36_counts_start();
    s_bucket36_report_t0 = HAL_GetTick();
    if (!proto_send_target(PROTO_TASK_BUCKET, 0u)) {
        route_seq_end("BUCKET_REQUEST_ERROR"); return;
    }
    snprintf(b, sizeof b, "OK BUCKET%u_REQUEST task=4 digit=0 model=9; x<495 backward50 x>505 forward50; no_coordinate=brake_wait; laser=0",
             (unsigned)s_seq_mode);
    send(b);
}

static void bucket36_poll(void)
{
    static char b[144];
    VisionDiagSample sample;
    ProtoStats stats;
    uint32_t now = HAL_GetTick(), pm;
    int fresh;
    if (proto_scene_status() < 0) { route_seq_end("BUCKET_LINK_ERROR"); return; }
    if ((uint32_t)(now - s_bucket36_report_t0) >= BUCKET36_REPORT_MS) {
        s_bucket36_report_t0 = now; bucket36_report();
    }
    if (s_bucket36_phase == BA_STILL) {
        motion_brake();
        if (bucket36_counts_changed()) s_bucket36_t0 = now;
        if ((uint32_t)(now - s_bucket36_t0) < T_DIST_STILL_MS) return;
        if (!imu_zero_leg_heading()) { route_seq_end("IMUERR"); return; }
        s_bucket36_phase = BA_PREP_WAIT; s_bucket36_t0 = now;
        return;
    }
    if (s_bucket36_phase == BA_PREP_WAIT) {
        motion_brake();
        if (bucket36_counts_changed()) {
            s_bucket36_phase = BA_STILL; s_bucket36_t0 = now; return;
        }
        if ((uint32_t)(now - s_bucket36_t0) < NAV_SETTLE_MS) return;
        bucket36_clear_sample(); s_bucket36_phase = BA_SEEK;
        snprintf(b, sizeof b, "OK BUCKET%u_ALIGN point=500 band=495..505; wait_fresh_bucket; g/a/0 stops", (unsigned)s_seq_mode);
        send(b);
    }
    pm = __get_PRIMASK(); __disable_irq();
    sample = s_bucket36_sample; proto_stats_get(&stats); now = HAL_GetTick();
    __set_PRIMASK(pm);
    fresh = sample.seen && sample.packet_index == stats.obj &&
            (uint32_t)(now - sample.tick) <= BUCKET36_FRESH_MS;
    if (!fresh || proto_scene_status() != 1) {
        s_bucket36_good = 0u; s_bucket36_phase = BA_SEEK;
        motion_brake(); s_bucket36_cmd = 0.0f; return;
    }
    if (sample.frame.img_w <= BUCKET36_CX + BUCKET36_TOL_PX) {
        route_seq_end("BUCKET_IMAGE_WIDTH"); return;
    }
    if (sample.packet_index != s_bucket36_packet) {
        if (!s_bucket36_seen || sample.packet_index - s_bucket36_packet != 1u ||
            (uint16_t)(sample.frame.sequence - s_bucket36_sequence) != 1u) {
            s_bucket36_good = 0u; s_bucket36_phase = BA_SEEK;
        }
        s_bucket36_seen = 1u; s_bucket36_packet = sample.packet_index;
        s_bucket36_sequence = sample.frame.sequence;
        if (sample.frame.cx >= BUCKET36_CX - BUCKET36_TOL_PX &&
            sample.frame.cx <= BUCKET36_CX + BUCKET36_TOL_PX) {
            if (s_bucket36_good < BUCKET36_GOOD_FRAMES) s_bucket36_good++;
        } else s_bucket36_good = 0u;
    }
    if (sample.frame.cx < BUCKET36_CX - BUCKET36_TOL_PX ||
        sample.frame.cx > BUCKET36_CX + BUCKET36_TOL_PX) {
        s_bucket36_phase = BA_SEEK;
        s_bucket36_cmd = sample.frame.cx > BUCKET36_CX + BUCKET36_TOL_PX
                         ? BUCKET36_ALIGN_V_MMS : -BUCKET36_ALIGN_V_MMS;
        motion_vel_set_precise(s_bucket36_cmd, 0.0f,
                              step_heading_hold_w_kp(0.0f, s_route_heading_kp));
        return;
    }
    motion_brake(); s_bucket36_cmd = 0.0f;
    if (s_bucket36_good < BUCKET36_GOOD_FRAMES) return;
    if (s_bucket36_phase != BA_SETTLE) {
        s_bucket36_phase = BA_SETTLE; bucket36_counts_start();
        snprintf(b, sizeof b, "OK BUCKET%u_AIMED brake=1; wait_1s_still_with_fresh_bucket; laser=0", (unsigned)s_seq_mode);
        send(b);
        return;
    }
    if (bucket36_counts_changed()) s_bucket36_t0 = now;
    if ((uint32_t)(now - s_bucket36_t0) < TARGET_AIM_SETTLE_MS) return;
    /* Freshness and the transition/local-close are one atomic decision. */
    pm = __get_PRIMASK(); __disable_irq();
    sample = s_bucket36_sample; proto_stats_get(&stats); now = HAL_GetTick();
    if (!sample.seen || sample.packet_index != stats.obj ||
        sample.packet_index != s_bucket36_packet || sample.frame.sequence != s_bucket36_sequence ||
        (uint32_t)(now - sample.tick) > BUCKET36_FRESH_MS ||
        sample.frame.cx < BUCKET36_CX - BUCKET36_TOL_PX ||
        sample.frame.cx > BUCKET36_CX + BUCKET36_TOL_PX ||
        proto_scene_status() != 1 || run_aborted() || !imu_ok()) {
        __set_PRIMASK(pm); return;
    }
    s_bucket36_phase = BA_OFF; step_vision_receive_end(); s_seq_state = SQ_RUN;
    __set_PRIMASK(pm);
    route_seq_next(); /* The manual-back node waits for a NEW valid d command. */
}

/* ---- g2:暂停 + 补终点动作 + 报。
 * 1-4/7-10 刹停就是终点动作本身 → 直接回就绪(g 再按 = 继续);
 * 5/6、11/12 停在 DONE(车停住/爪合住)让你看准不准,看准了 g3 才回原点。 ---- */
static void format_deg2(char out[20], float deg)
{
    float abs_deg = (deg < 0.0f) ? -deg : deg;
    uint32_t centideg = (uint32_t)(abs_deg * 100.0f + 0.5f);
    snprintf(out, 20, "%s%lu.%02lu", (deg < 0.0f) ? "-" : "",
             (unsigned long)(centideg / 100u), (unsigned long)(centideg % 100u));
}

static void mode_pause(void)
{
    static char b[160]; /* Keep large report buffers off the 1 KB DefaultTask stack. */
    if (s_msel >= 1 && s_msel <= 6) {
        motion_brake();
        if (s_msel == 5 || s_msel == 6) {
            s_leg_run = (uint32_t)(HAL_GetTick() - s_leg_start);
            if (s_leg_run > s_leg_ms) s_leg_run = s_leg_ms;
            int mm = (int)((float)s_leg_run * s_v / 1000.0f + 0.5f);
            snprintf(b, sizeof b, "OK STOP est=%dmm (g3 return)", mm);
            s_round = R_DONE;
        } else {
            snprintf(b, sizeof b, "OK %s STOP (g to resume)", s_mname[s_msel]);
            s_round = R_READY;
        }
        send(b);
        return;
    }
    if (s_msel >= 7 && s_msel <= 10) {
        int32_t cnt;
        int32_t raw[4];
        uint32_t ms;
        motion_brake();
        cnt = ctrl_enc_total(s_wheel);
        for (int i = 0; i < 4; i++) raw[i] = bp_enc_raw_total(i);
        ms = (uint32_t)(HAL_GetTick() - s_meas_t0);
        ctrl_enc_reset_all();
        s_round = R_READY;
        snprintf(b, sizeof b, "REC type=WHEEL test=%lu wheel=%d duty=%d ms=%lu raw=%ld count=%ld raw_all=%ld,%ld,%ld,%ld",
                 (unsigned long)s_active_test, s_wheel, (int)s_p,
                 (unsigned long)ms, (long)raw[s_wheel], (long)cnt,
                 (long)raw[0], (long)raw[1], (long)raw[2], (long)raw[3]);
        send(b);
        return;
    }
    if (s_msel == 13) {
        int32_t c0 = ctrl_enc_total(0), c1 = ctrl_enc_total(1);
        int32_t c2 = ctrl_enc_total(2), c3 = ctrl_enc_total(3);
        uint32_t ms = (uint32_t)(HAL_GetTick() - s_meas_t0);
        static char eb[128];
        ctrl_enc_reset_all();
        s_round = R_READY;
        snprintf(eb, sizeof eb, "REC type=ENC test=%lu ms=%lu c0=%ld c1=%ld c2=%ld c3=%ld",
                 (unsigned long)s_active_test, (unsigned long)ms,
                 (long)c0, (long)c1, (long)c2, (long)c3);
        send(eb);
        return;
    }
    if (s_msel == 14) {
        uint32_t ms = (uint32_t)(HAL_GetTick() - s_meas_t0);
        static char ib[176];
        imu_stat_take();
        if (s_imu_n == 0u) {
            snprintf(ib, sizeof ib, "REC type=IMU test=%lu status=NO_VALID ms=%lu n=0 bad=%lu",
                     (unsigned long)s_active_test, (unsigned long)ms,
                     (unsigned long)s_imu_bad);
        } else {
            char yend[20], ypp[20], ppp[20], rpp[20];
            format_deg2(yend, imu_leg_heading_deg());
            format_deg2(ypp, s_imu_y_max - s_imu_y_min);
            format_deg2(ppp, s_imu_p_max - s_imu_p_min);
            format_deg2(rpp, s_imu_r_max - s_imu_r_min);
            snprintf(ib, sizeof ib, "REC type=IMU test=%lu status=OK ms=%lu n=%lu bad=%lu yend_deg=%s ypp_deg=%s ppp_deg=%s rpp_deg=%s",
                     (unsigned long)s_active_test, (unsigned long)ms,
                     (unsigned long)s_imu_n, (unsigned long)s_imu_bad,
                     yend, ypp, ppp, rpp);
        }
        (void)imu_zero_leg_heading();
        imu_stat_clear();
        s_round = R_READY;
        send(ib);
        return;
    }
    if (s_msel == 19 || turn_closed_loop_mode()) {
        turn_begin_settle("STOP");
        send("OK TURN_BRAKE waiting_for_settle");
        return;
    }
    if (s_msel == 21) {
        speed_probe_report("STOP");
        return;
    }
    if (s_msel == 23) {
        fwd_probe_report("STOP");
        return;
    }
    if (dist_mode()) {
        dist_begin_finish(0u);
        send("OK DIST_BRAKE waiting_for_encoder_settle");
        return;
    }
    if (jog_return_mode()) { jog_stop("OUT"); return; }
    if (s_msel == 24 || s_msel == 25) {
        s_round = R_DONE;
        snprintf(b, sizeof b, "REC type=JOG test=%lu axis=%d dir=%d requested=%lu done=%lu status=STOP g_clear_no_return",
                 (unsigned long)s_active_test, s_msel - 24, s_jog_request < 0 ? 0 : 1,
                 (unsigned long)(s_jog_request < 0 ? -s_jog_request : s_jog_request),
                 (unsigned long)s_jog_done);
        send(b);
        return;
    }
    /* 11/12:接触瞬间 → 爪合 + 报当前步,停在 DONE 让你看抓没抓准 */
    arm_claw_close();
    s_round = R_DONE;
    snprintf(b, sizeof b, "REC type=STEP test=%lu axis=%d steps=%lu status=HOLD",
             (unsigned long)s_active_test, s_axis, (unsigned long)s_steps);
    send(b);
}

/* 0 / a / g3:回原点/清态,回就绪。已在就绪且没位移 → 只回一句。 */
static void ret_drive(uint32_t ms)   /* 5/6 回程:同速反向等时长 */
{
    motion_vel_set(-s_rvx * s_v, -s_rvy * s_v, 0.0f);
    s_leg_start = HAL_GetTick();
    s_leg_ms = ms;
    s_round = R_RET;
}

/* g3/0 回原点清态:1-4/7-10 直接刹停回就绪;5/6 反走已走等长时长;11/12 开爪反向收步 */
static void cmd_reset(void)
{
    if (s_go || mission_state() != MS_BOOT) { cmd_abort(); return; }
    if (xy_trial_selected()) { xy_trial_stop("STOP"); return; }
    if (s_msel == TARGET_TRIAL_MODE) { target35_stop("STOP"); return; }
    proto_qr_cancel();
    if (s_msel == R_FREE) { send("OK IDLE select_mode_first"); return; }
    if (s_msel == VISION_DIAG_MODE) { vision_diag_stop(); return; }
    if (s_msel == MISSION_TRIAL_MODE) {
        motion_brake();
        send("OK TRIAL32_IDLE no_motion_no_auto_return; g starts after calibration");
        return;
    }
    if (route_seq_selected() || route_seq_active()) { route_seq_end("STOP"); return; }
    if (jog_return_mode()) {
        jog_stop(s_round == R_RET ? "RETURN" : (s_round == R_DONE ? "WAIT" : "OUT"));
        return;
    }
    if (servo_mode()) {
        if (s_round == R_DONE) servo_stop();
        else send("OK SERVO_READY no_action");
        return;
    }
    if (s_round == R_RET) { send("OK RETURN_IN_PROGRESS"); return; }
    if (s_msel >= 1 && s_msel <= 4) {
        motion_brake();
        s_round = R_READY;
        send("OK STOP ready");
        return;
    }
    if (s_msel == 5 || s_msel == 6) {
        uint32_t run = (s_round == R_RUN)
            ? (uint32_t)(HAL_GetTick() - s_leg_start)
            : s_leg_run;                       /* R_DONE/R_READY 用已记的走行 */
        motion_brake();
        if (run > s_leg_ms) run = s_leg_ms;
        if (run > 0u && s_v >= 1.0f) {
            ret_drive(run);
            send("OK RETURN_IN_PROGRESS (a to abort)");
        } else {
            s_round = R_READY;
            s_leg_run = 0u;
            send("OK AT_ORIGIN ready");
        }
        return;
    }
    if (s_msel >= 7 && s_msel <= 10) {
        motion_brake();
        ctrl_enc_reset_all();                    /* 测量卫生:本轮量清零 */
        s_round = R_READY;
        send("OK STOP encoder_reset ready");
        return;
    }
    if (s_msel == 13) {
        int32_t c0, c1, c2, c3;
        char b[80];
        motion_brake();
        c0 = ctrl_enc_total(0); c1 = ctrl_enc_total(1);
        c2 = ctrl_enc_total(2); c3 = ctrl_enc_total(3);
        ctrl_enc_reset_all();
        s_round = R_READY;
        snprintf(b, sizeof b, "ENC FINAL c=%ld,%ld,%ld,%ld",
                 (long)c0, (long)c1, (long)c2, (long)c3);
        send(b);
        return;
    }
    if (s_msel == 14) {
        (void)imu_zero_leg_heading();
        imu_stat_clear();
        s_round = R_READY;
        send("OK IMU_STOP reset ready");
        return;
    }
    if (s_msel == 19 || turn_closed_loop_mode()) {
        motion_brake();
        s_round = R_READY;
        send("OK TURN_RESET ready; position_not_reversed");
        return;
    }
    if (s_msel == 21) {
        motion_brake();
        s_round = R_READY;
        send("OK YAWSPD_RESET ready");
        return;
    }
    if (s_msel == 23) {
        motion_brake();
        s_round = R_READY;
        send("OK FWDPULSE_RESET ready");
        return;
    }
    if (dist_mode()) {
        motion_brake();
        ctrl_enc_reset_all();
        s_dist_align_enabled = s_dist_align_hold = 0u;
        s_round = R_READY;
        send("OK DIST_ABORT encoder_reset heading_preserved no_yfix_no_return");
        return;
    }
    if (s_msel == 24 || s_msel == 25) {
        s_round = R_READY;
        send("OK JOG_STOP no_auto_return; physical_position_uncertain");
        return;
    }
    /* 11/12 */
    if (s_steps > 0u) {
        arm_claw_open();
        arm_stepper_dir(s_axis, s_axis_dir ^ 1);
        s_back = 0;
        s_round = R_RET;
        send("OK STEPPER_RETURN claw_open (a to abort)");
    } else {
        s_round = R_READY;
        send("OK AT_ORIGIN ready");
    }
}

/* 'g' 一键推一轮:READY→启动(g1)、RUN→暂停补终点动作(g2)、DONE→回原点(g3) */
static void mode_g(void)
{
    if (xy_trial_selected()) { xy_trial_g(); return; }
    if (s_msel == TARGET_TRIAL_MODE) { target35_g(); return; }
    if (s_msel == VISION_DIAG_MODE) {
        if (s_vdiag_phase == VD_OFF) vision_diag_start();
        else vision_diag_stop();
        return;
    }
    if (route_seq_selected() || route_seq_active()) { route_seq_g(); return; }
    if (jog_return_mode() && s_round != R_READY) {
        jog_stop(s_round == R_RET ? "RETURN" : (s_round == R_DONE ? "WAIT" : "OUT"));
        return;
    }
    if (servo_mode() && s_round == R_DONE) { servo_stop(); return; }
    if (s_round == R_RET) { cmd_abort(); return; }      /* 回程运行中g同样只停，不再继续回程 */
    if (dist_mode() && s_round == R_ALIGN) { cmd_reset(); return; }
    if (dist_mode() && s_round == R_BRAKE) {
        s_dist_reason = 0u; s_dist_align_enabled = 0u;
        motion_brake();
        send("OK DIST_STOP pending_yfix_cancelled waiting_for_encoder_settle");
        return;
    }
    if (s_round == R_READY) { mode_start(); return; }   /* g1 */
    if (s_round == R_RUN)   { mode_pause(); return; }   /* g2 */
    if (s_round == R_DONE)  { cmd_reset(); return; }    /* g3:回原点(5/6、11/12 停在 DONE 等你过目) */
    if (s_round == R_BRAKE && s_msel == 30) {
        turn_begin_settle("STOP"); /* left90: g during hold cancels any later inertial correction */
        send("OK TURN_BRAKE waiting_for_settle");
        return;
    }
    if (s_round == R_BRAKE) { send("OK BRAKING waiting_for_encoder_settle"); return; }
    send("OK RETURN_IN_PROGRESS");
}

/* ---- 全局键 ---- */
/* 'a' 急停:跑整场→run_abort;回程中→刹停作废回程;调试中→同 0 收回;空闲仅回一句 */
static void cmd_abort(void)
{
    if (s_go || mission_state() != MS_BOOT) { /* g/a共用：先锁中止，再刹车；不假报物理停稳 */
        run_abort();
        motion_brake();
        bp_laser_set(0);
        proto_qr_cancel();
        send("OK ABORT_REQUEST brake_commanded; physical_stop_unverified");
        return;
    }
    if (s_msel == TARGET_TRIAL_MODE) { target35_stop("STOP"); return; }
    if (xy_trial_selected()) { xy_trial_stop("STOP"); return; }
    proto_qr_cancel();
    if (s_msel != R_FREE) {
        if (jog_return_mode()) {
            jog_stop(s_round == R_RET ? "RETURN" : (s_round == R_DONE ? "WAIT" : "OUT"));
            return;
        }
        if (servo_mode()) {
            if (s_round == R_DONE) servo_stop();
            else send("OK SERVO_READY no_action");
            return;
        }
        if (s_msel == 11 || s_msel == 12) {
            s_round = R_READY;
            s_steps = 0u; s_back = 0u;
            send("OK STEPPER_EMERGENCY_STOP no_return; physical_position_uncertain");
            return;
        }
        if (s_round == R_RET) {        /* 回程中按 a:刹停作废回程,停半路(位移记忆清掉) */
            motion_brake();
            s_leg_run = 0u; s_steps = 0u; s_back = 0u;
            s_round = R_READY;
            send("OK EMERGENCY_STOP return_aborted");
            return;
        }
        cmd_reset();   /* 与 0 同:中途作废,收爪/反向回原点(不算一轮、不补动作) */
        return;
    }
    send("OK IDLE no_action");
}

/* 数字选号进调试模式(1..41;仅 BOOT 空闲可,先刹掉当前动作再切,顺带清回程量) */
static void cmd_select(int32_t m, int quiet)
{
    if (!bench_ok()) { send("ERR BENCH_LOCKED power_cycle_to_retest"); return; }
    if (m < 1 || m > T_MODE_MAX) { send("ERR MODE_RANGE 1..41"); return; }
    if (xy_trial_active()) { send("ERR XY_ACTIVE stop_with_g_first"); return; }
    if (route_seq_active()) { send("ERR ROUTE_SEQ_ACTIVE stop_with_g_first"); return; }
    if (s_round == R_ALIGN) { send("ERR STOP_WITH_G before_mode_change"); return; }
    if (m == 11 || m == 12) { send("ERR STEPPER_UNBOUNDED_DISABLED use_mode24_or25"); return; }
    if (jog_return_mode() && s_round != R_READY) {
        send("ERR JOG_ACTIVE stop_with_g_or_a_before_mode_change"); return;
    }
    if (servo_mode() && s_round != R_READY) {
        send("ERR SERVO_ACTIVE stop_with_g_or_a_before_mode_change"); return;
    }
    if (s_msel == TARGET_TRIAL_MODE) {
        target35_stop("MODE_CHANGE"); s_target35_phase = TA_OFF;
    }
    if (xy_trial_selected()) {
        vision_align_test_cancel();
        s_xy_state = XT_OFF; s_xy_owner = 0u;
    }
    if (m != ROUTE_TEST_MODE && m != MISSION_TRIAL_MODE) proto_qr_cancel();
    if (s_msel != R_FREE && s_round != R_READY) {        /* 切号先刹当前动作 */
        motion_brake();
        if ((s_msel >= 7 && s_msel <= 10) || s_msel == 13) ctrl_enc_reset_all();
        if (dist_mode()) ctrl_enc_reset_all();
        if (s_msel == 14) { (void)imu_zero_leg_heading(); imu_stat_clear(); }
        s_round = R_READY;
    }
    s_msel = (int)m;
    s_seq_state = SQ_OFF; s_seq_prepared = 0u;
    s_route_leg = 0u;
    s_steps = 0; s_back = 0; s_leg_ms = 0; s_leg_run = 0;
    s_jog_request = 0; s_jog_done = 0u; s_jog_hold_t0 = 0u;
    s_servo_target_us = 0u; s_servo_origin_us = 0u; s_servo_hold_t0 = 0u;
    if (s_msel >= 38 && s_msel <= 41) {
        static char xy_msg[224];
        motion_brake(); bp_laser_set(0); step_vision_receive_end();
        vision_align_test_init();
        memset(s_xy_qr, 0, sizeof s_xy_qr);
        s_xy_owner = (uint8_t)s_msel; s_xy_state = XT_READY; s_round = R_READY;
        s_v = VAT_SPEED_MMS; s_d = -1.0f;
        if (s_msel != 39)
            proto_send_scene(SCENE_QR); /* Always NEW, not cached boot/old-run QR. */
        snprintf(xy_msg, sizeof xy_msg,
                 "OK MODE=%d %s %s; x190/y%d +/-10 frames5 X20 Y30 step3mm/cap250ms; post_Y_yawfix_new_XY; no_arm/laser; g/a/0 stops",
                 s_msel, s_mname[s_msel], s_msel == 39 ? "no_QR; g_requests_bucket4/digit0" :
                     "fresh_QR_scan_active; g_starts_after_QR", s_msel == 39 ? VAT_BUCKET_Y_PX :
                     s_msel == 40 ? VAT_HOSTAGE_Y_PX : VAT_BALL_Y_PX);
        send(xy_msg); return;
    }
    if (s_msel == TARGET_TRIAL_MODE) {
        motion_brake(); bp_laser_set(0); step_vision_receive_end();
        memset(s_target35_qr, 0, sizeof s_target35_qr);
        target35_clear_sample();
        s_target35_phase = TA_READY; s_round = R_READY;
        s_v = T_TARGET35_V_MMS; s_d = -1.0f;
        proto_qr_begin();
        send("OK MODE=35 TARGET_AIM_FIRE; QR_scan_active no_motion_until_g_and_valid_QR; v50 x255 band250..260 frames5; g/a/0 stops; no_auto_return");
        return;
    }
    if (s_msel == VISION_DIAG_MODE) {
        motion_brake(); bp_laser_set(0);
        send("OK MODE=33 VISION_RX_ONLY; g requests_QR_then_OBJECT; next_g/a/0 stops; no_motion_no_laser");
        return;
    }
    if (route_seq_selected()) {
        s_seq_mode = (uint8_t)s_msel;
        s_seq_state = SQ_READY; s_seq_stage = 0u; s_round = R_READY;
        s_bucket36_phase = BA_OFF; s_bucket36_back_mm = 0u; s_d = -1.0f;
        if (s_seq_mode == ROUTE_TEST_MODE) {
            proto_qr_begin();
            send("OK MODE=31 12_steps R1=LEFT520/v100; QR_receiving_before_g; g_starts_motion; R1_still_yawfix_then_legal_QR_gates_R2; FWD70/v20 RIGHT730 BACK780 all_turns90; no_bucket_wait_d/grab/fire; g/a/0 stops");
        } else if (s_seq_mode == BUCKET_ROUTE_MODE) {
            motion_brake(); bp_laser_set(0); step_vision_receive_end();
            s_bucket36_phase = BA_OFF; s_bucket36_back_mm = 0u; s_d = -1.0f;
            send("OK MODE=36 no_QR_bucket RIGHT90 BACK650/v300 FWD80/v20 BACK190/v100 RIGHT730/v100 BACK780/v100 LEFT92 FWD200/v100 STOP; g starts g/a/0 cancels");
        } else if (s_seq_mode == CROSS_ONLY_MODE) {
            motion_brake(); bp_laser_set(0); step_vision_receive_end();
            s_d = -1.0f;
            send("OK MODE=37 CROSS_ONLY 3_steps BACK650/v300 FWD80/v20 BACK190/v100 STOP; no_QR/turn/vision/arm/laser; g starts g/a/0 cancels no_resume_no_return");
        } else {
            motion_brake(); bp_laser_set(0);
            step_vision_receive_end();
            send("OK MODE=34 ROUTE_SEQ 15_steps RIGHT90 BACK650/v300 FWD80/v20 BACK190/v100 LEFT92 bucket_x500 wait_d_back100; QR=0 bucket_vision=1 no_grab_fire; g/a/0 cancels; no_resume_no_return");
        }
        return;
    }
    if (s_msel == MISSION_TRIAL_MODE) {
        proto_qr_begin();
        send("OK MODE=32 NO_ARM_SINGLEPASS; trial reports RAM alignment; g starts, next g/a/0 aborts; no_resume_no_return");
        mission_trial_report();
        return;
    }
    if (s_msel == 13) { mode_start(); return; } /* 只发 13 即进入手转并每秒主动回传，无电机动作 */
    static char b[144];
    if (s_msel == 17 || s_msel == 18) {
        /* 切换左右横移时覆盖之前模式的旧槽值；之后仍可用 v/d 手动调整。 */
        s_v = T_LATERAL_DEFAULT_V_MMS;
        s_d = T_LATERAL_DEFAULT_D_MM;
        snprintf(b, sizeof b, "OK MODE=%d %s v=300 d=1500 (g start/stop)",
                 s_msel, s_mname[s_msel]);
        if (!quiet) send(b);
        return;
    }
    if (s_msel == 24 || s_msel == 25) {
        snprintf(b, sizeof b, "OK MODE=%d %s: nl1..nl50/nr1..nr50 then g; next_g_clear_no_return", s_msel, s_mname[s_msel]);
        if (!quiet) send(b);
        return;
    }
    if (jog_return_mode()) {
        snprintf(b, sizeof b, "OK MODE=%d %s: nl1..nl50/nr1..nr50 then g; 2s auto_reverse; g stops_no_return", s_msel, s_mname[s_msel]);
        if (!quiet) send(b);
        return;
    }
    if (servo_mode()) {
        snprintf(b, sizeof b, "OK MODE=%d %s: u1000..u1800 then g; %s; g stops_and_holds",
                 s_msel, s_mname[s_msel], s_msel == 28 ? "2s_auto_return" : "no_return");
        if (!quiet) send(b);
        return;
    }
    snprintf(b, sizeof b, "OK MODE=%d %s (g start/stop;5/6/11/12 g return)",
             s_msel, s_mname[s_msel]);
    if (!quiet) send(b);
}

/* 无障碍段按赛题图顺序单段试跑：出发前进、左移到第三段、沿第三段前进并
 * 在减速带前停车。地图不给车中心起停点，故每次必须用 d 显式填距离。
 * 复用模式15/17的已落地定距控制、测试序号、g 停车和 BLE 回传。 */
static void cmd_route_leg(int32_t leg)
{
    if (!bench_ok()) { send("ERR ROUTE_LOCKED not_in_boot"); return; }
    if (s_round == R_RUN || s_round == R_BRAKE || s_round == R_RET || s_round == R_ALIGN) {
        send("ERR ROUTE_STOP_FIRST g_or_a"); return;
    }
    cmd_select(leg == 1 ? 17 : (leg == 2 ? 16 : 15), 1);
    s_route_leg = (uint8_t)leg;
    s_v = leg == 1 ? 300.0f : 80.0f; /* 仅台测起始值；不是正式赛道速度 */
    s_d = -1.0f;                     /* 禁止沿用上一段/模式17默认的 1500 mm */
    if (leg == 1) send("OK ROUTE leg=1 START_LEFT v300; set d<mm>, then g");
    else if (leg == 2) send("OK ROUTE leg=2 BACK_TO_3RD v80; set d<mm>, then g");
    else send("OK ROUTE leg=3 AFTER_LEFT90_FWD v80; turn separately FIRST; set d, STOP BEFORE bump");
}

/* 设参数槽(key=d/v/p 之一;仅 BOOT 空闲可;越界/非正数拒掉并报原因) */
static void cmd_set(char key, int32_t val)
{
    if (!bench_ok()) { send("ERR PARAM_LOCKED mission_running"); return; }
    if (xy_trial_selected()) {
        if (key == 'v' && val == (int32_t)VAT_SPEED_MMS && !xy_trial_active()) {
            s_v = VAT_SPEED_MMS; send("OK XY_V=20 X20_Y30_fixed");
        } else send("ERR XY_FIXED_X20_Y30 no_duty_or_distance_slot; points_in_vision_align_test.h");
        return;
    }
    if (s_round == R_RUN || s_round == R_BRAKE || s_round == R_RET || s_round == R_ALIGN) {
        send("ERR STOP_WITH_G before_parameter_change"); return;
    }
    float *dst;
    int lo, hi;
    if (key == 'v')      { dst = &s_v; lo = 1; hi = T_V_MAX; }
    else if (key == 'd') { dst = &s_d; lo = 1; hi = T_D_MAX; }
    else                 { dst = &s_p; lo = -(int)MOTOR_PWM_PERIOD; hi = (int)MOTOR_PWM_PERIOD; }
    if (val < lo || val > hi || (key == 'p' && val == 0)) {
        char b[48];
        if (key == 'p') snprintf(b, sizeof b, "ERR p range=-%d..-1 or 1..%d", hi, hi);
        else snprintf(b, sizeof b, "ERR %c range=%d..%d", key, lo, hi);
        send(b);
        return;
    }
    *dst = (float)val;
    if (key == 'p') s_p_valid = 1u;
    char b[32];
    snprintf(b, sizeof b, "OK %c=%ld", key, (long)val);
    send(b);
    if (key == 'v' && dist_mode()) cmd_param_report();
}

/* '?' 打印命令语法、g 键一圈说明与当前槽/模式值 */
static void cmd_help(void)
{
    send("? Default=mission; select 1..41 for bench mode.");
    send("  Mission: first g starts if calibrated; next g aborts. a also stops; restart board to rerun.");
    send("  1..4 continuous move; 5/6 timed leg; 7..10 wheels; 11/12 unsafe disabled; 13 enc; 14 IMU.");
    send("  15..18 distance: forward/back/left/right; 17/18 select loads v300 d1500.");
    send("  route: r1 left, r2 backward; separate LEFT90; r3 forward to BEFORE bump.");
    send("  r1/r2/r3 select leg only; set measured d<mm>, then g; no auto next leg.");
    send("  v<mm/s> and d<mm> override slots after selecting; select 17/18 resets defaults.");
    send("  19 turn sign 250ms (suspended); 20 turn +90 hold (right compensation=0deg).");
    send("  21 suspended speed probe: +w 800ms, -w 800ms, auto brake, 200ms RPM trace.");
    send("  22 turn +180 hold within 0.3deg (ground, experimental).");
    send("  38 QR->ball XY(190,420);39 noQR g->bucket XY(190,400);40 QR->hostage XY(190,220), then STOP.");
    send("  41 QR->ball XY,hold5s->request_bucket->+180->new_bucket_request->bucket XY,hold5s->STOP.");
    send("  38..41: X20/Y30,w0,3mm encoder step/cap250ms->brake250ms->NEW_XY; afterY original_heading_fix first. +/-10px,5 new frames; X+ forward/X- backward,Y- left/Y+ right.");
    send("  Select38/40/41 starts freshQR, g arms motion;39 waits for g then requests bucket directly. nextg/a/0 cancels; no arm/laser/auto-return. XY logs each500ms.");
    send("  30 turn LEFT -92 hold within 0.3deg (90 + 2deg trial compensation).");
    send("  31 fixed12: LEFT520/BACK650/RIGHT90/BACK650-v300/FWD70-v20/BACK190/RIGHT730/BACK780/LEFT90/FWD2450/RIGHT90/FWD2125.");
    send("  Cross/board yaw_hold=0; fresh stopped heading before BACK190;31 has no bucket/manual-d wait, all right-angle turns90.");
    send("  31 scans QR from boot/R1; R1 must finish and QR must be legal before R2; no timed release; g/a/0 cancels.");
    send("  31 local acc700/dec350 soft start/stop, r1=520/r2=650; select31 startsQR beforeg; final brake and manual g remain immediate.");
    send("  32 no-arm single-pass mission: QR/vision/two+180/laser; ball/bucket/hostage hold10s; next g/a/0 aborts.");
    send("  33 receive-only: g requests QR then OBJECT; ASCII VD33 snapshots each second; g/a/0 closes local RX, no camera STOP.");
    send("  34 retains old15 without QR: LEFT530/BACK650/RIGHT90/BACK650-v300/FWD80-v20/BACK190/LEFT92/bucket4/0 x500/WAIT_D.");
    send("  34 only: d<mm> backs100 then RIGHT90/fwd780/RIGHT90/fwd2450/RIGHT90/fwd2125;31 does not request bucket.");
    send("  36 no_QR/bucket: LEFT530/BACK650/RIGHT90/BACK650-v300/FWD80-v20/BACK190/RIGHT730/BACK780/LEFT92/FWD200 STOP.");
    send("  36 no new d required, no arm/laser; g/a/0 cancels whole chain, reselect36 then g after manual placement.");
    send("  37 crossing-only: manually_start_after_RIGHT90; g runs BACK650/v300 FWD80/v20 BACK190/v100 then stops.");
    send("  37 no approach/QR/turn/vision/arm/laser; node still-zero/750ms; g/a/0 cancels, select37+g reruns after manual placement.");
    send("  vision reports mode33 now; latest=1 means in last accepted packet, age/lastcx are receive-time snapshots.");
    send("  trial32 X: vsg1 forward/vsg2 back; bcx/tcx/hcx/kcx<pixel>. BALL/HOSTAGE require measured bcy/hcy too.");
    send("  trial32 Y: ysg1 positive cy error -> right; ysg2 -> left. Alternating XY, both +/-8px, 5 fresh frames +250ms still.");
    send("  b1d<mm>: measured entry-corner to bucket first leg; 1..2449, RAM-only; mode32 second=2450-first.");
    send("  23 forward sign probe: suspended only, open+closed pulse, auto brake.");
    send("  24/25 stepper jog: nl1..nl50=dir0, nr1..nr50=dir1, then g; no auto return.");
    send("  26/27 stepper jog: same nl/nr; g start, wait 2s, reverse same steps.");
    send("  Old n-5=nl5 and n5=nr5 still work. g/a/0 cancels remaining steps.");
    send("  28 servo auto return: u1000..u1800 then g; wait 2s, command prior pulse; no angle feedback.");
    send("  29 servo hold: u1000..u1800 then g; no auto return. g/a/0 cancels, holds PWM.");
    send("  su1000..1800: immediate servo pulse in us (unmounted horn only at first); no save.");
    send("  g=start/stop; 5/6/11/12 need third g to return; 13 starts on selection and 0 stops.");
    send("  imu toggles 4Hz readings; cc=claw close; co=claw open; a=abort.");
    send("  p<-199..199> wheel duty; param/diag; kp/ki/lp/dead/ykp/okp/acc/dec/lff/rff/fff/bff/yfix RAM tune.");
    send("  15..18: select mode, set v, then ykp0..5 stores that direction/speed RAM profile; g keeps it.");
    send("  Untuned speed uses global ykp; no interpolation. 31/34 share local ykp(default0.3), ramp and fractional RPM.");
    send("  lff-0.05..0.05: mode17 v300 only; +backward/-forward, 0 disables.");
    send("  rff-0.05..0.05: mode18 v300 only; +forward/-backward, 0 disables.");
    send("  Select31/34/36/37 then lff/rff: independent route slots at ALL route speeds; default0 pending measurement.");
    send("  fff/bff-0.05..0.05: ordinary forward/backward ALL speeds; +body-left/-body-right, 0 off.");
    send("  Bench trial seeds fff=-0.00625(right), bff=+0.00625(left), not final calibration. Routes use independent slots.");
    send("  yfix1(default)/yfix0: normal distance DONE brake->original yaw->still700ms->zero; g/a/0 cancels. Crossing/contact excluded.");
    send("  35: g waits_valid_QR; task2 request; no_coordinates forward50; x<250 backward, x>260 forward; band5_frames brake1s laser_hold stay_here; g/a/0 stop.");
    char nv[16], nd[16], np[16];
    if (s_v >= 1.0f) snprintf(nv, sizeof nv, "%d", (int)s_v); else strcpy(nv, "unset");
    if (s_d >= 1.0f) snprintf(nd, sizeof nd, "%d", (int)s_d); else strcpy(nd, "unset");
    if (s_p_valid) snprintf(np, sizeof np, "%d", (int)s_p); else strcpy(np, "unset");
    char b[80];
    snprintf(b, sizeof b, "  slots:v=%s d=%s p=%s mode=%s",
             nv, nd, np, s_msel ? s_mname[s_msel] : "idle");
    send(b);
}

/* 把纯数字字符串解析成 int32;必须消费到串尾,尾随垃圾=拒(回 0) */
static int parse_num(const char *p, int32_t *v)
{
    int neg = 0;
    if (*p == '-') { neg = 1; p++; }
    if (*p < '0' || *p > '9') return 0;
    int32_t x = 0;
    while (*p >= '0' && *p <= '9') {
        if (x < 1000000) x = x * 10 + (*p - '0');
        p++;
    }
    if (*p != '\0') return 0;   /* 尾随垃圾 → 拒绝整行 */
    *v = neg ? -x : x;
    return 1;
}

/* 小型十进制定点解析：支持可选负号和一个小数点，不接受指数/nan/尾随字符。 */
static int parse_float(const char *p, float *v)
{
    double x = 0.0, scale = 0.1; /* round once: .050 must not exceed float .05 */
    int neg = 0, have = 0, dot = 0;
    if (*p == '-') { neg = 1; p++; }
    while (*p) {
        if (*p >= '0' && *p <= '9') {
            have = 1;
            if (!dot) x = x * 10.0 + (double)(*p - '0');
            else { x += (double)(*p - '0') * scale; scale *= 0.1; }
        } else if (*p == '.' && !dot) dot = 1;
        else return 0;
        p++;
    }
    if (!have) return 0;
    *v = (float)(neg ? -x : x);
    return 1;
}

static void cmd_param_report(void)
{
    CtrlTune t;
    MotionProfileTune p;
    static char b[240]; /* float snprintf 不占用 DefaultTask 的大块局部数组 */
    int profile;
    float kp = dist_heading_kp_get(&profile);
    if (dist_mode() && (s_round == R_RUN || s_round == R_BRAKE || s_round == R_ALIGN)) {
        kp = s_dist_heading_kp;
        profile = s_dist_heading_profile;
    }
    ctrl_tune_get(&t);
    motion_profile_get(&p);
    if (s_seq_state != SQ_OFF) {
        p.acc_mms2 = ROUTE_TEST_ACC_MMS2;
        p.dec_mms2 = ROUTE_TEST_DEC_MMS2;
    }
    snprintf(b, sizeof b,
             "PARAM kp=%.4f ki=%.5f lp=%.3f dead=%u ykp=%.3f okp=%.3f acc=%.1f dec=%.1f lff=%.4f rff=%.4f fff=%.5f bff=%.5f yfix=%u RAM-only",
             t.kp, t.ki, t.lp_alpha, (unsigned)t.dead_min, kp,
             step_orth_kp(), p.acc_mms2, p.dec_mms2,
             s_seq_state != SQ_OFF ? s_route_left_ff_ratio : s_left_ff_ratio,
             s_seq_state != SQ_OFF ? s_route_right_ff_ratio : s_right_ff_ratio,
             s_seq_state != SQ_OFF ? s_route_forward_ff_ratio :
                 (s_msel == MISSION_TRIAL_MODE || s_msel == R_FREE ? s_mission_forward_ff_ratio : s_forward_ff_ratio),
             s_seq_state != SQ_OFF ? s_route_backward_ff_ratio : s_backward_ff_ratio,
             (unsigned)s_dist_align_on);
    send(b);
    if (s_seq_state != SQ_OFF) {
        snprintf(b, sizeof b,
                 "ROUTE_PROFILE mode=%u source=LOCAL acc=%.0f dec=%.0f precise=1 final_brake=1 g_immediate=1 fff=%.5f bff=%.5f ykp=%.3f lff=%.5f rff=%.5f yfix=%u",
                 (unsigned)s_seq_mode, p.acc_mms2, p.dec_mms2, s_route_forward_ff_ratio,
                 s_route_backward_ff_ratio, s_route_heading_kp,
                 s_route_left_ff_ratio, s_route_right_ff_ratio, (unsigned)s_dist_align_on);
        send(b);
    }
    if (xy_trial_selected()) {
        snprintf(b, sizeof b,
                 "YAW mode=%u source=POST_Y original_heading=1 zeroed=0 tol_deg=0.30; X20 Y30 step3mm cap250ms translation_w0; explicit_mode41_turn_uses22",
                 (unsigned)s_xy_owner);
    } else {
        snprintf(b, sizeof b,
                 "YAW mode=%d v_mms=%.0f ykp=%.3f source=%s global_ykp=%.3f RAM-only",
                 s_msel, s_v, kp, s_seq_state != SQ_OFF
                     ? (s_seq_mode == CROSS_ONLY_MODE ? "ROUTE37" :
                        (s_seq_mode == BUCKET_ROUTE_MODE ? "ROUTE36" :
                         (s_seq_mode == ROUTE_NO_QR_MODE ? "ROUTE34" : "ROUTE31"))) : (profile ? "PROFILE" : "GLOBAL"),
                 step_heading_kp_deg());
    }
    send(b);
    if (s_msel == TARGET_TRIAL_MODE) target35_report();
    if (route_seq_bucket_enabled() && s_seq_state != SQ_OFF) bucket36_report();
}

static void cmd_tune(const char *key, float val)
{
    CtrlTune t;
    MotionProfileTune p;
    int ok = 0;
    if (!bench_ok()) { send("ERR TUNE_LOCKED mission_running"); return; }
    if (s_round == R_RUN || s_round == R_BRAKE || s_round == R_RET || s_round == R_ALIGN) {
        send("ERR STOP_WITH_G before_tuning"); return;
    }
    ctrl_tune_get(&t);
    motion_profile_get(&p);
    if      (strcmp(key, "kp") == 0)   { t.kp = val; ok = ctrl_tune_set(&t); }
    else if (strcmp(key, "ki") == 0)   { t.ki = val; ok = ctrl_tune_set(&t); }
    else if (strcmp(key, "lp") == 0)   { t.lp_alpha = val; ok = ctrl_tune_set(&t); }
    else if (strcmp(key, "dead") == 0) {
        if (val >= 0.0f && val <= (float)MOTOR_PWM_PERIOD && val == (float)(uint16_t)val) {
            t.dead_min = (uint16_t)val; ok = ctrl_tune_set(&t);
        }
    } else if (strcmp(key, "ykp") == 0) {
        if (s_seq_state != SQ_OFF) {
            if (val >= 0.0f && val <= 5.0f) { s_route_heading_kp = val; ok = 1; }
        } else if (dist_mode()) {
            if (s_v < 1.0f || s_v > T_V_MAX) { send("ERR SET_V first; ykp belongs to selected direction/speed"); return; }
            ok = dist_heading_kp_set(val);
            if (ok < 0) { send("ERR YKP_TABLE_FULL 32 profiles; existing keys can still be updated"); return; }
        } else ok = step_heading_kp_set(val);
    }
    else if (strcmp(key, "okp") == 0) ok = step_orth_kp_set(val);
    else if (strcmp(key, "lff") == 0) {
        if (val >= -0.05f && val <= 0.05f) {
            if (s_seq_state != SQ_OFF) s_route_left_ff_ratio = val;
            else s_left_ff_ratio = val;
            ok = 1;
        }
    }
    else if (strcmp(key, "rff") == 0) {
        if (val >= -0.05f && val <= 0.05f) {
            if (s_seq_state != SQ_OFF) s_route_right_ff_ratio = val;
            else s_right_ff_ratio = val;
            ok = 1;
        }
    }
    else if (strcmp(key, "fff") == 0) {
        if (val >= -0.05f && val <= 0.05f) {
            if (s_seq_state != SQ_OFF) s_route_forward_ff_ratio = val;
            else if (s_msel == MISSION_TRIAL_MODE || s_msel == R_FREE) s_mission_forward_ff_ratio = val;
            else s_forward_ff_ratio = val;
            ok = 1;
        }
    }
    else if (strcmp(key, "bff") == 0) {
        if (val >= -0.05f && val <= 0.05f) {
            if (s_seq_state != SQ_OFF) s_route_backward_ff_ratio = val;
            else s_backward_ff_ratio = val;
            ok = 1;
        }
    }
    else if (strcmp(key, "yfix") == 0) {
        if (val == 0.0f || val == 1.0f) { s_dist_align_on = (uint8_t)val; ok = 1; }
    }
    else if (strcmp(key, "acc") == 0) { p.acc_mms2 = val; ok = motion_profile_set(&p); }
    else if (strcmp(key, "dec") == 0) { p.dec_mms2 = val; ok = motion_profile_set(&p); }
    if (!ok) { send("ERR TUNE_RANGE kp0..2 ki0..0.2 lp0.01..1 dead0..199 ykp/okp0..5 lff/rff/fff/bff-0.05..0.05 yfix0/1 acc/dec0..10000"); return; }
    cmd_param_report();
}

/* 大写转小写(命令大小写不敏感,整行统一转小后比对) */
static char lc(char c)
{
    return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
}

/* co/cc:开/合爪(仅 BOOT 空闲;给台上单独调爪用,不参与任务时序) */
static void cmd_claw(int open)
{
    if (!bench_ok()) { send("ERR CLAW_LOCKED mission_running"); return; }
    if (s_round == R_RUN || s_round == R_RET || s_round == R_BRAKE || s_round == R_ALIGN ||
        (servo_mode() && s_round == R_DONE)) {
        send("ERR CLAW_STOP_OTHER_TEST_FIRST"); return;
    }
    if (open) arm_claw_open(); else arm_claw_close();
    send(open ? "OK CLAW_OPEN" : "OK CLAW_CLOSE");
}

/* imu:开/关 BOOT 空闲连续读数(看模块/查接线;开了后停在空闲态就 ~4Hz 刷) */
static void cmd_imu(void)
{
    if (!bench_ok()) { send("ERR IMU_MON_LOCKED mission_running"); return; }
    s_imu_mon = (uint8_t)!s_imu_mon;
    s_mon_last = HAL_GetTick();
    send(s_imu_mon ? "OK IMU_MON_ON yaw/pitch/roll ~4Hz" : "OK IMU_MON_OFF");
}

/* BT 一行命令总路由:小写化后分发——?帮助 / g 开跑·走停 / a 作废 / d·v·p 设槽 / cc·co 爪 / 数字选号或0复位 */
static void run_cmd(const char *ln)
{
    char buf[T_LINE_MAX + 1u];
    size_t i;
    for (i = 0; ln[i] && i < T_LINE_MAX; i++) buf[i] = lc(ln[i]);
    buf[i] = '\0';
    if (i == 0u) return;

    /* Receive-only session owns the interface: no arm, mode or tuning writes. */
    if (s_vdiag_phase != VD_OFF) {
        if (strcmp(buf, "g") == 0 || strcmp(buf, "a") == 0 || strcmp(buf, "0") == 0) {
            vision_diag_stop(); return;
        }
        if (strcmp(buf, "?") != 0 && strcmp(buf, "diag") != 0 &&
            strcmp(buf, "param") != 0 && strcmp(buf, "vision") != 0) {
            send("ERR VISION_DIAG_ACTIVE stop_with_g_first"); return;
        }
    }

    /* Mode35 owns motion/laser even during QR/ACK/preparation/settling waits.
     * Stop stays immediate; no other command may move the arm or alter aim. */
    if (target35_active()) {
        if (strcmp(buf, "g") == 0 || strcmp(buf, "a") == 0 || strcmp(buf, "0") == 0) {
            target35_stop("STOP"); return;
        }
        if (strcmp(buf, "?") != 0 && strcmp(buf, "diag") != 0 &&
            strcmp(buf, "param") != 0 && strcmp(buf, "vision") != 0) {
            send("ERR TARGET35_ACTIVE stop_with_g_first"); return;
        }
    }

    /*38..41 retain ownership during QR, correction, placeholders and mode22.
     * Only read-only diagnostics and immediate cancellation may interrupt. */
    if (xy_trial_active()) {
        if (strcmp(buf, "g") == 0 || strcmp(buf, "a") == 0 || strcmp(buf, "0") == 0) {
            xy_trial_stop("STOP"); return;
        }
        if (strcmp(buf, "?") != 0 && strcmp(buf, "diag") != 0 &&
            strcmp(buf, "param") != 0 && strcmp(buf, "vision") != 0) {
            send("ERR XY_ACTIVE stop_with_g_first"); return;
        }
    }

    /* Fixed route: no mode/parameter/arm writes while any phase owns the
     * wheels, including preparation and braking. Read-only reports are allowed. */
    if (route_seq_active()) {
        if (strcmp(buf, "g") == 0 || strcmp(buf, "a") == 0 || strcmp(buf, "0") == 0) {
            route_seq_end("STOP"); return;
        }
        if (route_seq_bucket_enabled() && s_seq_state == SQ_MANUAL_D_WAIT &&
            buf[0] == 'd' && strcmp(buf, "diag") != 0) {
            int32_t mm;
            if (!parse_num(buf + 1, &mm) || mm < 1 || mm > T_D_MAX) {
                send("ERR BUCKET_D_RANGE use_d1..d20000; stays_stopped"); return;
            }
            if (!bench_ok() || run_aborted() || !imu_ok()) {
                route_seq_end("ABORT"); return;
            }
            s_bucket36_back_mm = (uint16_t)mm;
            route_seq_prepare(); /* v100 backward; no additional g required. */
            return;
        }
        if (strcmp(buf, "?") != 0 && strcmp(buf, "diag") != 0 &&
            strcmp(buf, "param") != 0 && strcmp(buf, "route") != 0 &&
            !(route_seq_bucket_enabled() && strcmp(buf, "vision") == 0)) {
            send("ERR ROUTE_SEQ_ACTIVE stop_with_g_first"); return;
        }
    }
    if (route_seq_bucket_enabled() && s_seq_state != SQ_OFF &&
        buf[0] == 'd' && strcmp(buf, "diag") != 0) {
        send("ERR BUCKET_D_ONLY_AFTER_ALIGNMENT_WAIT_D"); return;
    }

    if (strcmp(buf, "?") == 0) { cmd_help(); return; }
    if (strcmp(buf, "g") == 0) {
        const char *missing;
        /* 必须在选号和BOOT检查之前：首个g已接受但MissionTask尚未苏醒时也能停。
         * 整场中止后不清s_go、不清run_abort，不把第三次g变成未经确认的重启。 */
        if (s_go || mission_state() != MS_BOOT) { cmd_abort(); return; }
        if (s_msel == MISSION_TRIAL_MODE) {
            motion_brake();
            robot_diag_report();
            cmd_param_report();
            mission_trial_report();
            missing = mission_trial_config_missing();
            if (missing || !mission_start_trial()) {
                static char tb[96];
                snprintf(tb, sizeof tb, "ERR TRIAL32_UNCALIBRATED:%s", missing ? missing : "START_LOCKED");
                send(tb);
                return;
            }
            s_go = 1;
            send("OK TRIAL32_START queued_for_MissionTask; g/a/0 aborts_no_resume_no_return");
            return;
        }
        if (s_msel != R_FREE) { mode_g(); return; }
        if (!bench_ok()) { send("ERR MISSION_LOCKED not_in_boot"); return; }
        motion_brake();
        missing = mission_config_missing();
        if (missing || !mission_start()) {
            char b[72];
            snprintf(b, sizeof b, "ERR MISSION_UNCALIBRATED:%s", missing ? missing : "UNKNOWN");
            send(b);
            return;
        }
        s_go = 1;
        send("OK MISSION_START (g)");
        return;
    }
    if (strcmp(buf, "a") == 0) { cmd_abort(); return; }
    if (strcmp(buf, "0") == 0 && (s_go || mission_state() != MS_BOOT)) { cmd_abort(); return; }
    if (strcmp(buf, "trial") == 0) { mission_trial_report(); return; }
    if (strcmp(buf, "vision") == 0) {
        if (route_seq_bucket_enabled() && s_seq_state != SQ_OFF) bucket36_report();
        else if (s_msel == TARGET_TRIAL_MODE) target35_report(); else vision_diag_report();
        return;
    }
    if (strncmp(buf, "b1d", 3u) == 0) {
        int32_t value;
        if (!bench_ok() || s_round == R_RUN || s_round == R_RET || s_round == R_BRAKE || s_round == R_ALIGN ||
            (servo_mode() && s_round == R_DONE)) {
            send("ERR TRIAL_DISTANCE_LOCKED stop_test_first; mission_requires_restart");
            return;
        }
        if (!parse_num(buf + 3, &value) || value <= 0 ||
            value >= (int32_t)MISSION_TRIAL_TASK_CORRIDOR_MM) {
            send("ERR B1D_RANGE measured_first_leg_mm_1..2449");
            return;
        }
        if (!mission_trial_set_first_leg((uint16_t)value)) {
            send("ERR TRIAL_DISTANCE_LOCKED");
            return;
        }
        mission_trial_report();
        return;
    }
    if (strncmp(buf, "ysg", 3u) == 0 || strncmp(buf, "bcy", 3u) == 0 ||
        strncmp(buf, "hcy", 3u) == 0) {
        int32_t value;
        int cls = -1, cy = -1, sign = 0;
        if (!bench_ok() || s_round == R_RUN || s_round == R_RET || s_round == R_BRAKE || s_round == R_ALIGN ||
            (servo_mode() && s_round == R_DONE)) {
            send("ERR TRIAL_ALIGNMENT_LOCKED stop_test_first; mission_requires_restart"); return;
        }
        if (!parse_num(buf + 3, &value) || value < 0 || value >= 65535) {
            send("ERR GRAB_Y_FORMAT bcy/hcy<pixel> or ysg1/ysg2"); return;
        }
        if (buf[0] == 'y') {
            if (value != 1 && value != 2) { send("ERR YSG_RANGE use_ysg1_or_ysg2"); return; }
            sign = value == 1 ? 1 : -1;
        } else { cls = buf[0] == 'b' ? CLS_BALL : CLS_HOSTAGE; cy = (int)value; }
        if (!mission_trial_set_grab_y(cls, cy, sign)) {
            send("ERR GRAB_Y_RANGE_OR_RUNNING"); return;
        }
        mission_trial_report(); return;
    }
    if (strncmp(buf, "vsg", 3u) == 0 || strncmp(buf, "bcx", 3u) == 0 ||
        strncmp(buf, "tcx", 3u) == 0 || strncmp(buf, "hcx", 3u) == 0 ||
        strncmp(buf, "kcx", 3u) == 0) {
        int32_t value;
        int cls = -1, sign = 0, cx = -1;
        if (!bench_ok() || s_round == R_RUN || s_round == R_RET || s_round == R_BRAKE || s_round == R_ALIGN ||
            (servo_mode() && s_round == R_DONE)) {
            send("ERR TRIAL_ALIGNMENT_LOCKED stop_test_first; mission_requires_restart");
            return;
        }
        if (!parse_num(buf + 3, &value) || value < 0) {
            send("ERR TRIAL_ALIGNMENT_FORMAT vsg1/vsg2 or bcx/tcx/hcx/kcx<pixel>");
            return;
        }
        if (buf[0] == 'v') {
            if (value != 1 && value != 2) { send("ERR VSG_RANGE use_vsg1_or_vsg2"); return; }
            sign = value == 1 ? 1 : -1;
        } else {
            cls = buf[0] == 'b' ? CLS_BALL : (buf[0] == 't' ? CLS_TARGET :
                  (buf[0] == 'h' ? CLS_HOSTAGE : CLS_BUCKET));
            cx = (int)value;
        }
        if (!mission_trial_set_alignment(cls, cx, sign)) {
            send("ERR TRIAL_ALIGNMENT_RANGE pixel_must_match_actual_image");
            return;
        }
        mission_trial_report();
        return;
    }
    if (strcmp(buf, "cc") == 0) { cmd_claw(0); return; }
    if (strcmp(buf, "co") == 0) { cmd_claw(1); return; }
    if (strncmp(buf, "su", 2u) == 0) {
        int32_t us;
        static char sb[56];
        if (!bench_ok() || s_round == R_RUN || s_round == R_RET || s_round == R_BRAKE || s_round == R_ALIGN ||
            (servo_mode() && s_round == R_DONE)) {
            send("ERR SERVO_STOP_OTHER_TEST_FIRST"); return;
        }
        if (!parse_num(buf + 2, &us) || us < 1000 || us > 1800) {
            send("ERR SERVO_RANGE su1000..su1800; no_horn_first"); return;
        }
        arm_claw_set_us((uint16_t)us);
        snprintf(sb, sizeof sb, "OK SERVO us=%ld RAM-only", (long)us);
        send(sb);
        return;
    }
    if (buf[0] == 'u') {
        int32_t us;
        static char sb[56];
        if (!bench_ok() || !servo_mode()) { send("ERR U_SELECT_MODE28_OR29"); return; }
        if (s_round != R_READY) { send("ERR U_STOP_WITH_G_FIRST"); return; }
        if (!parse_num(buf + 1, &us) || us < 1000 || us > 1800) {
            send("ERR U_RANGE u1000..u1800; no_horn_first"); return;
        }
        s_servo_target_us = (uint16_t)us;
        snprintf(sb, sizeof sb, "OK SERVO_TARGET us=%ld; g to start", (long)us);
        send(sb);
        return;
    }
    if (buf[0] == 'n') {
        int32_t n;
        const char *arg = buf + 1;
        int alias_dir = -1;
        static char nb[64];
        if (!jog_mode()) { send("ERR N_SELECT_MODE24_TO27"); return; }
        if (s_round != R_READY) { send("ERR N_STOP_WITH_G_FIRST"); return; }
        if (buf[1] == 'l' || buf[1] == 'r') {
            alias_dir = (buf[1] == 'l') ? 0 : 1;
            arg = buf + 2;
        }
        if (!parse_num(arg, &n) || n == 0 ||
            n < -(int32_t)T_JOG_MAX_STEPS || n > (int32_t)T_JOG_MAX_STEPS ||
            (alias_dir >= 0 && n < 0)) {
            send("ERR N_RANGE nl1..nl50 or nr1..nr50 (old n-5/n5 also valid)"); return;
        }
        if (alias_dir == 0) n = -n;
        s_jog_request = n;
        snprintf(nb, sizeof nb, "OK JOG_N dir=%d steps=%ld; g starts bounded",
                 n < 0 ? 0 : 1, (long)(n < 0 ? -n : n));
        send(nb);
        return;
    }
    if (strcmp(buf, "imu") == 0) { cmd_imu(); return; }
    if (strcmp(buf, "diag") == 0) { robot_diag_report(); return; }
    if (strcmp(buf, "param") == 0) { cmd_param_report(); return; }
    if (strcmp(buf, "route") == 0) {
        if (s_seq_state != SQ_OFF) {
            static char rb[176];
            const RouteTestLeg *leg = route_seq_leg();
            snprintf(rb, sizeof rb,
                     "SEQ run=%lu step=%u/%u name=%s state=%u v=%.0f d=%u; mode%u QR_gate_R1=%u; tasks=0; g/a/0 stops; no_resume_no_return",
                     (unsigned long)s_seq_run, (unsigned)(s_seq_stage + 1u), route_seq_stage_count(),
                     leg->name, (unsigned)s_seq_state, leg->speed_mms,
                     (unsigned)(route_seq_bucket_enabled() && s_seq_stage == route_seq_bucket_back_stage()
                                ? s_bucket36_back_mm : leg->distance_mm),
                     (unsigned)s_seq_mode, (unsigned)(s_seq_mode == ROUTE_TEST_MODE));
            send(rb);
            return;
        }
        send("ROUTE no-obstacle: r1 left; r2 backward; separate LEFT90; r3 forward STOP before bump.");
        send("Each leg: select r1/r2/r3 FIRST, then set measured d<mm>, then g.");
        send("No automatic next leg; QR/tasks/bump crossing are not run.");
        send("Mode31 fixed12: left520/back650/RIGHT90/BACK650-v300/FWD70-v20/BACK190/RIGHT730/BACK780/LEFT90/fwd2450/RIGHT90/fwd2125.");
        send("Mode31 gates R2 with legal QR; all right-angle turns90; no bucket/NEW_d/grab/fire; g/a/0 cancels.");
        send("Mode34 old15 remainsleft530/FWD80/LEFT92/bucket500/WAIT_D with its original tail; currentleg reported bySEQ PREP.");
        send("Mode36 noQR/noBucket: LEFT530/BACK650/RIGHT90/BACK650-v300/FWD80-v20/BACK190/RIGHT730/BACK780/LEFT92/FWD200 STOP.");
        send("Mode37 runs only reverse-cross650/v300,forward-square80/v20,back-clear190/v100; no turns/tasks.");
        return;
    }
    if (strlen(buf) == 2u && buf[0] == 'r' && buf[1] >= '1' && buf[1] <= '3') {
        cmd_route_leg((int32_t)(buf[1] - '0'));
        return;
    }

    /* RAM-only 控制参数；最长前缀先匹配。 */
    {
        static const char *const key[] = { "dead", "ykp", "okp", "lff", "rff", "fff", "bff", "yfix", "acc", "dec", "kp", "ki", "lp" };
        for (size_t k = 0; k < sizeof key / sizeof key[0]; ++k) {
            size_t n = strlen(key[k]);
            if (strncmp(buf, key[k], n) == 0) {
                float val;
                if (parse_float(buf + n, &val)) { cmd_tune(key[k], val); return; }
                send("ERR TUNE_FORMAT example ykp2 / lff0.0233 / rff0.0233 / fff0.0133 / acc300");
                return;
            }
        }
    }

    /* 参数槽:d/v/p + 整数；仅 p 允许负号。 */
    if ((buf[0] == 'd' || buf[0] == 'v' || buf[0] == 'p') &&
        ((buf[1] >= '0' && buf[1] <= '9') || (buf[0] == 'p' && buf[1] == '-'))) {
        int32_t val;
        if (parse_num(buf + 1, &val)) { cmd_set(buf[0], val); return; }
    }

    /* 纯数字:选号 / 0 作废回原点 */
    if (buf[0] >= '0' && buf[0] <= '9') {
        int32_t val;
        if (!parse_num(buf, &val)) { send("ERR UNKNOWN_CMD send ? for help"); return; }
        if (val == 0) { cmd_reset(); return; }
        cmd_select(val, 0);
        return;
    }

    send("ERR UNKNOWN_CMD send ? for help");
}
