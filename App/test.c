#include "test.h"
#include "board_pins.h"    /* bp_debug_send, MOTOR_PWM_PERIOD */
#include "control.h"       /* ctrl_set_speed */
#include "motion.h"        /* motion_vel_set / motion_brake */
#include "arm.h"           /* arm_claw_open/close, arm_stepper_dir/step */
#include "imu.h"           /* imu_yaw/pitch/roll_deg, imu_ok */
#include "steps.h"         /* run_abort */
#include "test_config.h"   /* bench-only defaults; turn-90 profile is shared */
#include "mission.h"       /* mission_state / mission_start / MS_BOOT */
#include "robot.h"         /* robot_diag_report */
#include "main.h"          /* HAL_GetTick */
#include <stdio.h>
#include <string.h>

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
enum { R_READY = 0, R_RUN, R_DONE, R_RET, R_BRAKE };
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
    "jog_return_axis0", "jog_return_axis1", "servo_return", "servo_hold"
};

static char     s_line[T_LINE_MAX + 1u];
static uint8_t  s_len;
static uint8_t  s_over;
static uint32_t s_last;

static float    s_v = -1.0f;   /* 槽:巡航 mm/s(模式1-6);-1=未设 */
static float    s_left_ff_ratio = T_LEFT_FF_SEED; /* RAM-only: 正=向车尾补偿，负=向车头 */
static float    s_right_ff_ratio = T_RIGHT_FF_SEED; /* RAM-only: 正=右移时向车头补偿 */
static float    s_forward_ff_ratio = 0.0f; /* RAM-only: 正=直走时向车头左侧补偿；默认关 */
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
static float    s_dist_ff_ratio; /* 本轮模式17/18独立前馈快照；其它模式恒0 */
static uint32_t s_dist_run_ms;   /* 发出刹车命令前的运行时间 */
static uint32_t s_dist_trace_t0;
static float s_dist_brake_mm;    /* 刹车瞬间编码器段里程，用于分离制动过冲与尺度误差 */
static float s_dist_brake_yaw;   /* 刹车瞬间本段航向 */
static uint32_t s_dist_brake_t0;
static uint32_t s_dist_still_t0;
static int32_t  s_dist_last[4];
static uint8_t  s_dist_reason;   /* 0=人工 g 暂停，1=自动到目标，2=IMU失效 */
static char     s_dist_report[288];
static MotionRamp s_dist_ramp;
static uint32_t s_turn_settle_t0;
static float s_turn_settle_yaw;
static const char *s_turn_result;
static uint32_t s_turn_trace_t0;
static uint32_t s_speed_probe_trace_t0;
static uint32_t s_fwd_phase_t0, s_fwd_trace_t0;
static uint8_t s_fwd_phase;       /* 0=open-loop, 1=braked gap, 2=closed-loop */
static int32_t s_fwd_open_count[4];

static void run_cmd(const char *ln);
static void flush_line(void);
static void tick(void);
static void send(const char *s);
static void format_deg2(char out[20], float deg);
static void jog_stop(const char *phase);
static void servo_stop(void);
static void cmd_abort(void);

static void begin_recorded_test(void)
{
    s_test_seq++;
    if (s_test_seq == 0u) s_test_seq = 1u; /* 理论回绕时也不使用 0 */
    s_active_test = s_test_seq;
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
    int32_t c0 = ctrl_enc_total(0), c1 = ctrl_enc_total(1);
    int32_t c2 = ctrl_enc_total(2), c3 = ctrl_enc_total(3);
    float enc_mm = dist_odo() - s_dist_odo0;
    float yaw = imu_ok() ? imu_leg_heading_deg() : 9999.0f;
    uint32_t settle_ms = (uint32_t)(HAL_GetTick() - s_dist_brake_t0);
    snprintf(s_dist_report, sizeof s_dist_report,
             "REC type=%s leg=%u test=%lu mode=%u status=%s cmd_mm=%.0f v_mms=%.0f ff_ratio=%.4f brake_mm=%.1f enc_mm=%.1f run_ms=%lu settle_ms=%lu brake_yaw=%.2f yaw_deg=%.2f c0=%ld c1=%ld c2=%ld c3=%ld",
             s_route_leg ? "ROUTE" : "DIST", (unsigned)s_route_leg,
             (unsigned long)s_active_test, (unsigned)s_msel,
             s_dist_reason == 1u ? "DONE" : (s_dist_reason == 2u ? "IMUERR" : "STOP"),
             s_dist_target, s_v, s_dist_ff_ratio, s_dist_brake_mm, enc_mm,
             (unsigned long)s_dist_run_ms, (unsigned long)settle_ms,
             s_dist_brake_yaw, yaw,
             (long)c0, (long)c1, (long)c2, (long)c3);
    ctrl_enc_reset_all();
    (void)imu_zero_leg_heading();
    s_round = R_READY;
    send(s_dist_report);
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
static int turn_closed_loop_mode(void) { return s_msel == 20 || s_msel == 22; }
static float turn_target_deg(void) { return s_msel == 22 ? 180.0f : T_TURN_TARGET_DEG; }
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
                 s_msel == 22 ? "TURN180" : "TURN90", (unsigned long)s_active_test,
                 valid ? s_turn_result : "IMUERR", (int)turn_target_deg(),
                 yaw, err, (unsigned long)(HAL_GetTick() - s_meas_t0));
    }
    s_round = R_DONE;
    send(b);
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

/* 调试执行器全态清零:行缓冲、槽置 -1、回空闲就绪(robot_init 调一次) */
void test_init(void)
{
    s_len = 0; s_over = 0;
    s_v = -1.0f; s_d = -1.0f; s_p = 0.0f; s_p_valid = 0u;
    s_left_ff_ratio = T_LEFT_FF_SEED;
    s_right_ff_ratio = T_RIGHT_FF_SEED;
    s_forward_ff_ratio = 0.0f;
    s_jog_request = 0; s_jog_done = 0u; s_jog_hold_t0 = 0u;
    s_servo_target_us = 0u; s_servo_origin_us = 0u; s_servo_hold_t0 = 0u;
    s_go = 0;
    s_msel = R_FREE; s_round = R_READY;
    s_route_leg = 0u;
    s_steps = 0; s_back = 0; s_leg_ms = 0; s_leg_run = 0;
    s_imu_mon = 0u; s_mon_last = 0u;
    s_meas_t0 = 0u;
    s_enc_report_t0 = 0u; s_enc_report_seq = 0u;
    s_test_seq = 0u; s_active_test = 0u;
    s_dist_odo0 = 0.0f; s_dist_heading0 = 0.0f; s_dist_orth0 = 0.0f; s_dist_target = 0.0f;
    s_dist_ff_ratio = 0.0f;
    s_dist_run_ms = 0u; s_dist_brake_t0 = 0u; s_dist_still_t0 = 0u;
    s_dist_trace_t0 = 0u;
    s_dist_brake_mm = 0.0f; s_dist_brake_yaw = 0.0f;
    s_dist_reason = 0u;
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
    tick();             /* 段走/回程/步进的非阻塞推进(g 随时可插) */
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
        } else if (turn_closed_loop_mode()) {             /* +90/+180 闭环，自转后检查惯性 */
            float e, w, yaw;
            float target = turn_target_deg();
            uint32_t now;
            if (!imu_ok()) { turn_begin_settle("IMUERR"); return; }
            yaw = imu_leg_heading_deg();
            if (yaw > target + T_TURN_LIMIT_DEG || yaw < -T_TURN_LIMIT_DEG) {
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
                         s_msel == 22 ? "TURN180" : "TURN90",
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
            if (!imu_ok()) {
                dist_begin_finish(2u);
                return;
            }
            d = dist_odo() - s_dist_odo0;
            if (s_dist_target > 0.0f ? (d >= s_dist_target) : (d <= s_dist_target)) {
                dist_begin_finish(1u);
                return;
            }
            remain = s_dist_target - d;
            cmd = motion_linear_profile_step(&s_dist_ramp,
                                               (s_dist_target > 0.0f) ? s_v : -s_v,
                                               remain, 0.020f);
            {
                float orth_cmd = step_orth_hold_cmd(dist_lateral(), s_dist_orth0);
                /* 前进 cmd>0 时正 fff 给负 vy(左)；左右横移用各自独立系数。
                 * 左移 cmd<0 给负 vx(车尾)，右移 cmd>0 给正 vx(车头)。 */
                if (s_msel == 15) orth_cmd -= s_dist_ff_ratio * cmd;
                if (dist_lateral()) orth_cmd += s_dist_ff_ratio * cmd;
                motion_vel_set(dist_lateral() ? orth_cmd : cmd,
                           dist_lateral() ? cmd : orth_cmd,
                           step_heading_hold_w(s_dist_heading0));
                if ((uint32_t)(HAL_GetTick() - s_dist_trace_t0) >= T_DIST_TRACE_MS) {
                    static char trace[144];
                    s_dist_trace_t0 = HAL_GetTick();
                    snprintf(trace, sizeof trace,
                             "TRC type=DIST test=%lu mode=%u ms=%lu axis_mm=%.1f orth_mm=%.1f yaw_deg=%.2f cmd_mms=%.1f",
                             (unsigned long)s_active_test, (unsigned)s_msel,
                             (unsigned long)(s_dist_trace_t0 - s_meas_t0), d,
                             (dist_lateral() ? motion_odo_mm() : motion_lateral_odo_mm()) - s_dist_orth0,
                             imu_leg_heading_deg(), cmd);
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
        if (turn_closed_loop_mode() && (yaw > turn_target_deg() + T_TURN_LIMIT_DEG
                            || yaw < -T_TURN_LIMIT_DEG)) {
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
                s_turn_settle_t0 = now; /* 在 ±1° 内但仍在移动，重新等稳定 */
                s_turn_settle_yaw = yaw;
            }
        }
        if ((uint32_t)(now - s_turn_settle_t0) >= T_TURN_SETTLE_MS)
            turn_report();
    } else if (s_round == R_BRAKE && dist_mode()) {
        int changed = 0;
        for (int i = 0; i < 4; i++) {
            int32_t now = ctrl_enc_total(i);
            if (now != s_dist_last[i]) changed = 1;
            s_dist_last[i] = now;
        }
        if (changed) s_dist_still_t0 = HAL_GetTick();
        if ((uint32_t)(HAL_GetTick() - s_dist_still_t0) >= T_DIST_STILL_MS)
            dist_finish_report();
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
        if (!step_prepare_leg()) { send("ERR TURN_PREP check_IMU_and_stop_state"); return; }
        begin_recorded_test();
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
        } else if (s_msel == 20) {
            send("OK TURN90 target=+90deg hold_tol=0.3deg; g/a stop");
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
        if (!step_prepare_leg()) { send("ERR DIST_PREP check_IMU_and_stop_state"); return; }
        begin_recorded_test();
        if      (s_msel == 15) { s_rvx =  1.0f; s_rvy =  0.0f; }
        else if (s_msel == 16) { s_rvx = -1.0f; s_rvy =  0.0f; }
        else if (s_msel == 17) { s_rvx =  0.0f; s_rvy = -1.0f; }
        else                    { s_rvx =  0.0f; s_rvy =  1.0f; }
        ctrl_enc_reset_all();
        s_dist_odo0 = dist_odo();
        s_dist_heading0 = imu_leg_heading_deg();
        s_dist_orth0 = dist_lateral() ? motion_odo_mm() : motion_lateral_odo_mm();
        s_dist_target = (s_msel == 16 || s_msel == 17) ? -s_d : s_d;
        /* 三轴独立 RAM 系数，只在已有观察的方向/速度试验生效。 */
        s_dist_ff_ratio = (s_msel == 15 && s_v == 200.0f) ? s_forward_ff_ratio
            : (s_msel == 17 && s_v == 300.0f) ? s_left_ff_ratio
            : (s_msel == 18 && s_v == 300.0f) ? s_right_ff_ratio : 0.0f;
        motion_linear_ramp_init(&s_dist_ramp);
        s_meas_t0 = HAL_GetTick();
        s_dist_trace_t0 = s_meas_t0;
        s_round = R_RUN;
        snprintf(s_dist_report, sizeof s_dist_report,
                 "OK %s leg=%u %s start cmd=%.0fmm v=%.0fmm/s ff_ratio=%.4f ff_vx=%.1f ff_vy=%.1fmm/s; auto_stop_or_g",
                 s_route_leg ? "ROUTE" : "DIST", (unsigned)s_route_leg,
                 s_mname[s_msel], s_dist_target, s_v, s_dist_ff_ratio,
                 dist_lateral() ? s_dist_ff_ratio * (s_msel == 17 ? -s_v : s_v) : 0.0f,
                 s_msel == 15 ? -s_dist_ff_ratio * s_v : 0.0f);
        send(s_dist_report);
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
    if (s_msel == R_FREE) { send("OK IDLE select_mode_first"); return; }
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
    char b[48];
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
        (void)imu_zero_leg_heading();
        s_round = R_READY;
        send("OK DIST_ABORT encoder_heading_reset");
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
    if (jog_return_mode() && s_round != R_READY) {
        jog_stop(s_round == R_RET ? "RETURN" : (s_round == R_DONE ? "WAIT" : "OUT"));
        return;
    }
    if (servo_mode() && s_round == R_DONE) { servo_stop(); return; }
    if (s_round == R_RET) { cmd_abort(); return; }      /* 回程运行中g同样只停，不再继续回程 */
    if (s_round == R_READY) { mode_start(); return; }   /* g1 */
    if (s_round == R_RUN)   { mode_pause(); return; }   /* g2 */
    if (s_round == R_DONE)  { cmd_reset(); return; }    /* g3:回原点(5/6、11/12 停在 DONE 等你过目) */
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
        send("OK ABORT_REQUEST brake_commanded; physical_stop_unverified");
        return;
    }
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

/* 数字选号进调试模式(1..29;仅 BOOT 空闲可,先刹掉当前动作再切,顺带清回程量) */
static void cmd_select(int32_t m, int quiet)
{
    if (!bench_ok()) { send("ERR BENCH_LOCKED power_cycle_to_retest"); return; }
    if (m < 1 || m > T_MODE_MAX) { send("ERR MODE_RANGE 1..29"); return; }
    if (m == 11 || m == 12) { send("ERR STEPPER_UNBOUNDED_DISABLED use_mode24_or25"); return; }
    if (jog_return_mode() && s_round != R_READY) {
        send("ERR JOG_ACTIVE stop_with_g_or_a_before_mode_change"); return;
    }
    if (servo_mode() && s_round != R_READY) {
        send("ERR SERVO_ACTIVE stop_with_g_or_a_before_mode_change"); return;
    }
    if (s_msel != R_FREE && s_round != R_READY) {        /* 切号先刹当前动作 */
        motion_brake();
        if ((s_msel >= 7 && s_msel <= 10) || s_msel == 13) ctrl_enc_reset_all();
        if (dist_mode()) ctrl_enc_reset_all();
        if (s_msel == 14) { (void)imu_zero_leg_heading(); imu_stat_clear(); }
        s_round = R_READY;
    }
    s_msel = (int)m;
    s_route_leg = 0u;
    s_steps = 0; s_back = 0; s_leg_ms = 0; s_leg_run = 0;
    s_jog_request = 0; s_jog_done = 0u; s_jog_hold_t0 = 0u;
    s_servo_target_us = 0u; s_servo_origin_us = 0u; s_servo_hold_t0 = 0u;
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
    if (s_round == R_RUN || s_round == R_BRAKE || s_round == R_RET) {
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
    if (s_round == R_RUN || s_round == R_BRAKE || s_round == R_RET) {
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
}

/* '?' 打印命令语法、g 键一圈说明与当前槽/模式值 */
static void cmd_help(void)
{
    send("? Default=mission; select 1..29 for bench mode.");
    send("  Mission: first g starts if calibrated; next g aborts. a also stops; restart board to rerun.");
    send("  1..4 continuous move; 5/6 timed leg; 7..10 wheels; 11/12 unsafe disabled; 13 enc; 14 IMU.");
    send("  15..18 distance: forward/back/left/right; 17/18 select loads v300 d1500.");
    send("  route: r1 left, r2 backward; separate LEFT90; r3 forward to BEFORE bump.");
    send("  r1/r2/r3 select leg only; set measured d<mm>, then g; no auto next leg.");
    send("  v<mm/s> and d<mm> override slots after selecting; select 17/18 resets defaults.");
    send("  19 turn sign 250ms (suspended); 20 turn +90 hold within 0.3deg (ground).");
    send("  21 suspended speed probe: +w 800ms, -w 800ms, auto brake, 200ms RPM trace.");
    send("  22 turn +180 hold within 0.3deg (ground, experimental).");
    send("  23 forward sign probe: suspended only, open+closed pulse, auto brake.");
    send("  24/25 stepper jog: nl1..nl50=dir0, nr1..nr50=dir1, then g; no auto return.");
    send("  26/27 stepper jog: same nl/nr; g start, wait 2s, reverse same steps.");
    send("  Old n-5=nl5 and n5=nr5 still work. g/a/0 cancels remaining steps.");
    send("  28 servo auto return: u1000..u1800 then g; wait 2s, command prior pulse; no angle feedback.");
    send("  29 servo hold: u1000..u1800 then g; no auto return. g/a/0 cancels, holds PWM.");
    send("  su1000..1800: immediate servo pulse in us (unmounted horn only at first); no save.");
    send("  g=start/stop; 5/6/11/12 need third g to return; 13 starts on selection and 0 stops.");
    send("  imu toggles 4Hz readings; cc=claw close; co=claw open; a=abort.");
    send("  p<-199..199> wheel duty; param/diag; kp/ki/lp/dead/ykp/okp/acc/dec/lff/rff/fff RAM tune.");
    send("  lff-0.05..0.05: mode17 v300 only; +backward/-forward, 0 disables.");
    send("  rff-0.05..0.05: mode18 v300 only; +forward/-backward, 0 disables.");
    send("  fff-0.05..0.05: mode15 v200 only; +left/-right, 0 disables.");
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
    float x = 0.0f, scale = 0.1f;
    int neg = 0, have = 0, dot = 0;
    if (*p == '-') { neg = 1; p++; }
    while (*p) {
        if (*p >= '0' && *p <= '9') {
            have = 1;
            if (!dot) x = x * 10.0f + (float)(*p - '0');
            else { x += (float)(*p - '0') * scale; scale *= 0.1f; }
        } else if (*p == '.' && !dot) dot = 1;
        else return 0;
        p++;
    }
    if (!have) return 0;
    *v = neg ? -x : x;
    return 1;
}

static void cmd_param_report(void)
{
    CtrlTune t;
    MotionProfileTune p;
    static char b[192]; /* float snprintf 不占用 DefaultTask 的大块局部数组 */
    ctrl_tune_get(&t);
    motion_profile_get(&p);
    snprintf(b, sizeof b,
             "PARAM kp=%.4f ki=%.5f lp=%.3f dead=%u ykp=%.3f okp=%.3f acc=%.1f dec=%.1f lff=%.4f rff=%.4f fff=%.4f RAM-only",
             t.kp, t.ki, t.lp_alpha, (unsigned)t.dead_min, step_heading_kp_deg(),
             step_orth_kp(), p.acc_mms2, p.dec_mms2, s_left_ff_ratio, s_right_ff_ratio,
             s_forward_ff_ratio);
    send(b);
}

static void cmd_tune(const char *key, float val)
{
    CtrlTune t;
    MotionProfileTune p;
    int ok = 0;
    if (!bench_ok()) { send("ERR TUNE_LOCKED mission_running"); return; }
    if (s_round == R_RUN || s_round == R_BRAKE || s_round == R_RET) {
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
    } else if (strcmp(key, "ykp") == 0) ok = step_heading_kp_set(val);
    else if (strcmp(key, "okp") == 0) ok = step_orth_kp_set(val);
    else if (strcmp(key, "lff") == 0) {
        if (val >= -0.05f && val <= 0.05f) { s_left_ff_ratio = val; ok = 1; }
    }
    else if (strcmp(key, "rff") == 0) {
        if (val >= -0.05f && val <= 0.05f) { s_right_ff_ratio = val; ok = 1; }
    }
    else if (strcmp(key, "fff") == 0) {
        if (val >= -0.05f && val <= 0.05f) { s_forward_ff_ratio = val; ok = 1; }
    }
    else if (strcmp(key, "acc") == 0) { p.acc_mms2 = val; ok = motion_profile_set(&p); }
    else if (strcmp(key, "dec") == 0) { p.dec_mms2 = val; ok = motion_profile_set(&p); }
    if (!ok) { send("ERR TUNE_RANGE kp0..2 ki0..0.2 lp0.01..1 dead0..199 ykp/okp0..5 lff/rff/fff-0.05..0.05 acc/dec0..10000"); return; }
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
    if (s_round == R_RUN || s_round == R_RET || s_round == R_BRAKE ||
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

    if (strcmp(buf, "?") == 0) { cmd_help(); return; }
    if (strcmp(buf, "g") == 0) {
        const char *missing;
        /* 必须在选号和BOOT检查之前：首个g已接受但MissionTask尚未苏醒时也能停。
         * 整场中止后不清s_go、不清run_abort，不把第三次g变成未经确认的重启。 */
        if (s_go || mission_state() != MS_BOOT) { cmd_abort(); return; }
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
    if (strcmp(buf, "cc") == 0) { cmd_claw(0); return; }
    if (strcmp(buf, "co") == 0) { cmd_claw(1); return; }
    if (strncmp(buf, "su", 2u) == 0) {
        int32_t us;
        static char sb[56];
        if (!bench_ok() || s_round == R_RUN || s_round == R_RET || s_round == R_BRAKE ||
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
        send("ROUTE no-obstacle: r1 left; r2 backward; separate LEFT90; r3 forward STOP before bump.");
        send("Each leg: select r1/r2/r3 FIRST, then set measured d<mm>, then g.");
        send("No automatic next leg; QR/tasks/bump crossing are not run.");
        return;
    }
    if (strlen(buf) == 2u && buf[0] == 'r' && buf[1] >= '1' && buf[1] <= '3') {
        cmd_route_leg((int32_t)(buf[1] - '0'));
        return;
    }

    /* RAM-only 控制参数；最长前缀先匹配。 */
    {
        static const char *const key[] = { "dead", "ykp", "okp", "lff", "rff", "fff", "acc", "dec", "kp", "ki", "lp" };
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
