/*
 * 初学者导读：排爆：按 QR 指定球色抓球、转身对桶放球，然后收回并转回。
 * 这里按顺序调用 steps.c；任一步失败就返回 TASK_ABORT，让 mission 停止。
 * TASK_OK=0、TASK_ABORT=1 是任务返回约定；步骤函数多数用 1 成功、0 失败。
 * if (!step_xxx(...)) 的 ! 是逻辑非：步骤返回0时进入失败分支。
 */

#include "robot_tasks.h"
#include "steps.h"
#include "proto.h"
#include "motion.h"

/* EOD 放桶垂直量(2026-09-06 用户):球底座比桶低、球与球又近 → 咬球后先抬出球座/避邻球,
 * 原地转 180° 对桶后再降进桶口。steps = axis1(垂直=s1)实际要走的步数,台上实测填(见实测表):
 * 量法 = 轴1 慢伸(蓝牙模式12)从"咬着球在球座"抬到"够离球座且不碰邻球"数步数。 */
#define EOD_BALL_PRELOWER_STEPS 0u /* TODO 实测:从统一丝杆起点降至球抓取高度的 axis1 步数 */
#define EOD_BALL_EXTEND_STEPS  0u /* TODO 实测:从人工确认的齿条起点伸到球的 axis0 步数 */
#define EOD_LIFT_STEPS    0u   /* TODO 实测:咬球后抬离球座/避邻球、够转体不碰的轴1步数 */
#define EOD_LOWER_STEPS   0u   /* TODO 实测:对桶后降进桶口的轴1步数(≤LIFT,别压桶底) */

const char *task_eod_config_missing(void)
{
    if (EOD_BALL_PRELOWER_STEPS == 0u) return "EOD_BALL_PRELOWER_STEPS";
    if (EOD_BALL_EXTEND_STEPS == 0u) return "EOD_BALL_EXTEND_STEPS";
    if (EOD_LIFT_STEPS == 0u) return "EOD_LIFT_STEPS";
    if (EOD_LOWER_STEPS == 0u) return "EOD_LOWER_STEPS";
    return 0;
}

/* EOD 排爆(第一个抓取任务):带内扫到 QR 选的色球 → 视觉锁 → 抓 → 原地转 180°
 * → 扫到排爆桶(单桶)并锁 → 松爪放进 → 再原地转 180° 回(朝向不变,走向下个区)。
 * 找目标=step_sweep(没出现→前后扫到出现)、对准=step_align(出现后锁),成对复用;
 * 抓球/放桶两次都走这一对。
 * 任务区间导航(驶入带、区与区间)由 mission 顶层负责,路点待实测。 */

/**
 * @brief 执行本文件负责的任务步骤，失败立即交还顶层处理。
 * @param ball_color 球色标签LAB_R/LAB_G/LAB_B。
 * @retval TASK_OK（0）=步骤链完成，TASK_ABORT（1）=步骤失败或中止。
 * @note 软件步骤完成不等于实物抓取/命中效果已经验收。
 */
int task_eod_run(int ball_color)
{
    float bucket_scan_origin;
    if (run_aborted()) return TASK_ABORT;

    proto_send_scene(SCENE_EOD);

    /* 1 抓:左侧相机沿车身前后方向扫d1色球；真实入口/远端/像素符号待实测。
     * 前后走补扫,to=0 **一直扫到出现、不弃站**(垫底,正常别走到这)。
     *   找到后视觉锁(补相机↔爪偏移) */
    if (!step_sweep(PF_OBJ, CLS_BALL, ball_color, 0, 0)) return TASK_ABORT;   /* to=0 不限时 */
    if (!step_align(CLS_BALL, ball_color, 0)) return TASK_ABORT;   /* to=0 对到成为止 */
    if (!step_arm_prepare("EOD", "BALL_STILL")) return TASK_ABORT;
    if (!step_arm_run("EOD", "BALL_PRELOWER", EOD_BALL_PRELOWER_STEPS, step_arm_lower)) return TASK_ABORT;
    if (!step_arm_run("EOD", "BALL_GRASP", EOD_BALL_EXTEND_STEPS, step_grasp)) return TASK_ABORT;

    /* 2 先抬再转:球已咬在爪里,球底座比桶低、球又近 → axis1 抬 EOD_LIFT_STEPS 出球座/避邻球
     *   再原地转 180°(无 x/y 位移)背朝桶。转的判据 = IMU yaw 闭环；
     *   IMU 无有效帧会停车返回失败。抬升步数未实测时主程序启动闸门会拒绝开跑。 */
    if (!step_arm_run("EOD", "BALL_LIFT", EOD_LIFT_STEPS, step_arm_lift)) return TASK_ABORT;
    if (!step_rotate_deg(180, 0)) return TASK_ABORT;   /* to=0 不限时:转到位 / 外部 stop 才停 */
    bucket_scan_origin = motion_odo_mm();

    /* 3 放:排爆桶全场单只、在带内——扫到桶(不挑 label)→ 锁桶对正 → 先 axis1 降
     * EOD_LOWER_STEPS 落进桶口 → 开爪放进 → 抬回(出桶口)。不留限时兜底(2026-09-06 用户:
     * 别限自己任务时间,先完成再完美)——咬着球也一直扫到桶放进为止,不主动"超时丢球"弃分;
     * 停整场只靠外部 stop / 比赛时限。 */
    if (!step_sweep(PF_OBJ, CLS_BUCKET, -1, 0, 0)) return TASK_ABORT;   /* to=0 不限时 */
    if (!step_align(CLS_BUCKET, -1, 0)) return TASK_ABORT;   /* to=0 对到成为止 */
    if (!step_arm_prepare("EOD", "BUCKET_STILL")) return TASK_ABORT;
    if (!step_arm_run("EOD", "BUCKET_LOWER", EOD_LOWER_STEPS, step_arm_lower)) return TASK_ABORT;
    if (!step_arm_release("EOD", "BUCKET_RELEASE")) return TASK_ABORT;
    if (!step_arm_run("EOD", "BUCKET_LIFT_OUT", EOD_LOWER_STEPS, step_arm_lift)) return TASK_ABORT;  /* 先原路抬离桶口，不在桶内横向退爪 */

    /* 放球后两轴回抓球前的人工起点：先收齿条，再按升降净位移回丝杆。
     * 只按成功执行过的步数反向走，不撞机械挡块找零；丢步/中止/断电后须人工重定起点。
     * 最终装车须空桶验证收爪及升降的扫掠空间，再允许第二次180°与离区。 */
    if (!step_arm_run("EOD", "RACK_RETRACT", EOD_BALL_EXTEND_STEPS, step_rack_retract)) return TASK_ABORT;
    /* 起点为0；抓前下降 P、抓后抬 L；放桶下 D 后又抬 D，当前净位置=L-P。 */
    if (EOD_LIFT_STEPS > EOD_BALL_PRELOWER_STEPS) {
        if (!step_arm_run("EOD", "VERTICAL_HOME_DOWN", EOD_LIFT_STEPS - EOD_BALL_PRELOWER_STEPS, step_arm_lower)) return TASK_ABORT;
    } else if (EOD_BALL_PRELOWER_STEPS > EOD_LIFT_STEPS) {
        if (!step_arm_run("EOD", "VERTICAL_HOME_UP", EOD_BALL_PRELOWER_STEPS - EOD_LIFT_STEPS, step_arm_lift)) return TASK_ABORT;
    }

    /* 此时车头反向，桶区前后扫描轴与抓球时反向；先回第一次转身后的前后基准，
     * 再转回车头，避免把放桶期间的扫描位移带到离区。其余离区路线待逐项确认。 */
    if (!step_return_forward_odo(bucket_scan_origin, 0)) return TASK_ABORT;

    /* 4 再原地转 180° 回:相对当前航向再转 180 → 绝对朝向回原位,两次净转=0,出区朝向=进区朝向。
     *   然后 mission 导航去反恐区。 */
    if (!step_rotate_deg(180, 0)) return TASK_ABORT;

    return TASK_OK;
}
