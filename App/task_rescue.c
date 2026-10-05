#include "robot_tasks.h"
#include "steps.h"
#include "proto.h"

/* RESCUE 救援(第三个任务):左侧相机沿车身前后轴找QR指定形状；
 * mission在救援前右90°后直走进区，抓后保持夹持继续直走返回；本函数不走回程。
 * 人质**咬着带回、不放下**——评分看整车投影(除机械臂)进返回区,不是放人质到某点。
 * 三种人质纯白无颜色线索 → 只靠形状认(label=d3 圆柱/圆锥/腰鼓)。 */

/* 抓后小抬离台量(2026-09-09 用户:人质在 400×150×h100 平台上,不抬离平台开不走)。
 * steps = axis1(垂直=s1)实际步数,台上实测填(见实测表):量法同 EOD——轴1 慢伸(蓝牙模式12)
 * 从咬着人质到抬离平台面数步数。 */
#define RESCUE_PRELOWER_STEPS 0u /* TODO 实测:从统一丝杆起点降至人质抓取高度的 axis1 步数 */
#define RESCUE_EXTEND_STEPS   0u /* TODO 实测:从齿条起点伸到人质的 axis0 步数 */
#define RESCUE_LIFT_STEPS  0u   /* TODO 实测:咬住人质后抬离平台的 axis1 步数 */

const char *task_rescue_config_missing(void)
{
    if (RESCUE_PRELOWER_STEPS == 0u) return "RESCUE_PRELOWER_STEPS";
    if (RESCUE_EXTEND_STEPS == 0u) return "RESCUE_EXTEND_STEPS";
    if (RESCUE_LIFT_STEPS == 0u) return "RESCUE_LIFT_STEPS";
    return 0;
}

int task_rescue_run(int hostage_shape)
{
    if (run_aborted() || hostage_shape < LAB_CYL || hostage_shape > LAB_WAIST) return TASK_ABORT;

    if (!step_vision_target(PROTO_TASK_HOSTAGE, (uint8_t)(hostage_shape - 2))) return TASK_ABORT;
    /* 1 找目标=沿图上方的左右目标带慢慢经过时锁 d3 形人质(纯白只靠形状认);真没
     * 看到才沿车身前后补扫,to=0 **一直扫到出现、不弃站**(垫底,正常别走到这)。
     *   找到后视觉锁(补相机↔爪偏移) */
    if (!step_sweep(PF_OBJ, CLS_HOSTAGE, hostage_shape, 0, 0)) return TASK_ABORT;   /* to=0 不限时 */
    if (!step_align(CLS_HOSTAGE, hostage_shape, 0)) return TASK_ABORT;   /* to=0 对到成为止 */
    if (!step_arm_prepare("RESCUE", "HOSTAGE_STILL")) return TASK_ABORT;
    if (!step_arm_run("RESCUE", "HOSTAGE_PRELOWER", RESCUE_PRELOWER_STEPS, step_arm_lower)) return TASK_ABORT;
    if (!step_arm_run("RESCUE", "HOSTAGE_GRASP", RESCUE_EXTEND_STEPS, step_grasp)) return TASK_ABORT;

    /* 2 先抬离台再带:人质在平台上,axis1 抬 RESCUE_LIFT_STEPS 离台；未实测时启动闸门拒绝开跑。
     *   人质不放下；车身不再转，mission沿车头继续直走返回，不沿用旧横移假设。
     *   回返移动由 mission 顶层按救援入口里程基准带出；本函数到此 = 抓稳+已离台。 */
    if (!step_arm_run("RESCUE", "HOSTAGE_LIFT", RESCUE_LIFT_STEPS, step_arm_lift)) return TASK_ABORT;

    return TASK_OK;
}
