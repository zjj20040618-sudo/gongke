#include "robot_tasks.h"
#include "steps.h"
#include "proto.h"

/* ANTI 反恐(第二个任务):带内视觉锁到 QR 选环色的靶(同心环)→ 射击位稳住 → 激光。
 * 相机朝车身左侧已确认；激光同侧/共轴与靶工作点仍须装车实测。
 * 不从旧共轴注释推定画面中心=射击点；距离及cx使用真实靶站位标定。 */

int task_anti_run(int target_color)
{
    if (run_aborted() || target_color < LAB_R || target_color > LAB_B) return TASK_ABORT;

    if (!step_vision_target(PROTO_TASK_TARGET, (uint8_t)(target_color + 1))) return TASK_ABORT;
    /* 1 左侧相机沿车身前后轴扫描d2色靶；激光是否同侧/共轴尚需实物确认。
     * 前后走补扫,to=0 **一直扫到出现、不弃站**(垫底,正常别走到这)。
     *   找到后锁到标定靶工作点；激光方向和实体命中需单独验收。 */
    if (!step_sweep(PF_OBJ, CLS_TARGET, target_color, 0, 0)) return TASK_ABORT;   /* to=0 不限时 */
    if (!step_align(CLS_TARGET, target_color, 0)) return TASK_ABORT;   /* to=0 对到成为止 */

    /* 2 已对准：保持刹车、激光关，静置1秒；不重置路程或像素工作点。 */
    if (!step_target_settle()) return TASK_ABORT;

    /* 3 激光亮2秒后关闭，调用方再继续剩余路线。 */
    if (!step_fire(TARGET_LASER_ON_MS)) return TASK_ABORT;

    return TASK_OK;
}
