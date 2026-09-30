#include "robot_tasks.h"
#include "steps.h"
#include "proto.h"

/* ANTI 反恐(第二个任务):带内视觉锁到 QR 选环色的靶(同心环)→ 射击位稳住 → 激光。
 * 激光+相机同体共轴 → 画面中心锁到十环即已上靶,off_x=0(不用补相机↔爪偏)。
 * 射击距离靠靶像素高(自选 ~450mm),垂直被安装锁死不用调。 */

#define LASER_ON_MS       2000u   /* 触发保持(靶面自动评分) */

int task_anti_run(int target_color)
{
    if (run_aborted()) return TASK_ABORT;

    proto_send_scene(SCENE_ANTI);
    /* 1 找目标=慢慢经过时锁 d2 色靶(赛题图中三靶上下排列；车头朝图左时沿车体横轴扫);真没看到才
     * 左右走补扫,to=0 **一直扫到出现、不弃站**(垫底,正常别走到这)。
     *   找到后视觉锁靶(激光+相机同体共轴,环心进画面中心=已上靶,off_x=0) */
    if (!step_sweep(PF_OBJ, CLS_TARGET, target_color, 0, 0)) return TASK_ABORT;   /* to=0 不限时 */
    if (!step_align(CLS_TARGET, target_color, 0)) return TASK_ABORT;   /* to=0 对到成为止 */

    /* 2 对准后复用节点停稳：刹车、编码器连续静止、本段航向清零、等待。
     * 只能证明轮计数暂未变化；打靶角度/余晃仍须实车验收。 */
    if (!step_prepare_leg()) return TASK_ABORT;

    /* 3 开枪(亮 LASER_ON_MS 后自动灭) */
    if (!step_fire(LASER_ON_MS)) return TASK_ABORT;

    return TASK_OK;
}
