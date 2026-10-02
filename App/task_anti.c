#include "robot_tasks.h"
#include "steps.h"
#include "proto.h"

/* ANTI 反恐(第二个任务):带内视觉锁到 QR 选环色的靶(同心环)→ 射击位稳住 → 激光。
 * 相机朝车身左侧已确认；激光同侧/共轴与靶工作点仍须装车实测。
 * 不从旧共轴注释推定画面中心=射击点；距离及cx使用真实靶站位标定。 */

#define LASER_ON_MS       2000u   /* 触发保持(靶面自动评分) */

int task_anti_run(int target_color)
{
    if (run_aborted()) return TASK_ABORT;

    if (!step_vision_scene(SCENE_ANTI)) return TASK_ABORT;
    /* 1 左侧相机沿车身前后轴扫描d2色靶；激光是否同侧/共轴尚需实物确认。
     * 前后走补扫,to=0 **一直扫到出现、不弃站**(垫底,正常别走到这)。
     *   找到后锁到标定靶工作点；激光方向和实体命中需单独验收。 */
    if (!step_sweep(PF_OBJ, CLS_TARGET, target_color, 0, 0)) return TASK_ABORT;   /* to=0 不限时 */
    if (!step_align(CLS_TARGET, target_color, 0)) return TASK_ABORT;   /* to=0 对到成为止 */
    proto_send_scene(SCENE_IDLE);

    /* 2 对准后复用节点停稳：刹车、编码器连续静止、本段航向清零、等待。
     * 只能证明轮计数暂未变化；打靶角度/余晃仍须实车验收。 */
    if (!step_prepare_leg()) return TASK_ABORT;

    /* 3 开枪(亮 LASER_ON_MS 后自动灭) */
    if (!step_fire(LASER_ON_MS)) return TASK_ABORT;

    return TASK_OK;
}
