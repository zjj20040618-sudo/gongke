#ifndef APP_ROUTE_TEST_PLAN_H
#define APP_ROUTE_TEST_PLAN_H

#include <stdint.h>

/* Mode31 has its own 16-stage task trial. Active31 R1=585/pv250;
 * select31 starts QR reception but only g starts movement. R1 still/yaw-fix
 * must finish and a legal QR tuple must be cached before R2 can start.
 * Mode31 R2 uses configured BACK then body-back100/body-left100 diagonal with held
 * heading, before the retained RIGHT90. Remaining crossing BACK550/v300
 * preserves westward total100+550=650; southward total is configured BACK+100.
 * These are unmeasured trial distances. Restore pre-cross yaw after crossing,
 * then FWD/v40 until tilt2.0deg,
 * stopped IMU zero then BACK190/route_v (no post-contact nudges),
 * then RIGHT90/FORWARD745/RIGHT90/FORWARD795/RIGHT90. Its old 2450/2125 road-only tail is REPLACED
 * by ball search/X-only, first NEW selected ball coordinate deploys
 * rack3400@5000 alongside search (not before entry RIGHT90),
 * ball pre-grab LEFT0 (retained but skipped)/down32000@10000/grip2100/wait2/up32000/direct+180,
 * request the bucket only AFTER completed first180; no post-turn LEFT15,
 * ball rank54 required before paired180; bucket rank1BACK/rank3FWD/rank2wait,
 * only |cx-X165|<15 latches continuous fine v20, final155..175 or seen-loss>300ms/down20000/release1150/
 * after1s_ready1500/wait2_total/up20000/retract3400@5000,
 * return+180/stable400/direct_target_no_lateral_offset, selected target search_route_v/until_x375/fine30/X265_band262..268/fire2s,
 * target-to-corner, RIGHT92 and hostage search_route_v/X185-or-seen-loss>300ms/
 * no pre-grab lateral offset/open1150/extend3400@5000/grip2100/retract3000@5000/rank/forward-to-end_hold_grip.
 * Explicit v before g overrides ALL ordinary forward/back and task search
 * using the mode31 RAM speed, including the final return. Without a v command,
 * use150 for both route halves and target search. Reselect31 keeps RAM;
 * test_init/power cycle restores these defaults. R2 diagonal uses its configured cap.
 * Independent pv RAM slot controls pre-task left/right (boot250).
 * From the ball/bucket stage onward, optional lateral offsets use the task seed
 * and lateral feedforward is0, with moving yaw hold retained. Mode31 stays automatic.
 *31 hostage pre-grab lateral parameter is0 and must not dispatch a distance job;
 * crossing, board contact and visual fine speeds stay fixed. Mode31/43 turns
 * both use the original coarse angular command. End profile is retained.
 * Visual Y adjustment is temporarily disabled in vision_align_test.h; cy stays logged.
 * Ball/bucket/hostage workpoints remain TEMPORARY, not calibrated grab points.
 * Ball/bucket/hostage: stopped alignment or explicit seen-then-lost>300ms trigger;
 * loss never means alignment. First legal NEW selected coordinate is required,
 * then brake and confirm still250ms before the action; never-seen cannot trigger.
 * Hostage: extend and
 * grip2100 then retract3000, no vertical movement/release; legitimate rank gates final route.
 * No bucket-anchor detour,
 * manual-d wait or scan. Standalone42 retains its independent reset recipe.
 * Lift uses axis1 nr=DIR1/down, nl=DIR0/up at10000pulses/s. Emitted counts
 * are open-loop, not position sensing or homing; manual stop cancels the chain.
 * Mode31 before-cross right uses+90, hostage right uses+92, left remains-90;
 *43 seeds the same goals in private RAM; independent turns remain unchanged.
 * The legacy 15-node table is retained for34, including FWD80 and its
 * bucket/manual-d tail; independent36/37 retain their existing FWD80.
 * Crossing/board-contact translation does not correct yaw. Modes31/43
 * restores the captured pre-cross heading before a fresh stopped contact zero.
 * After contact, stop, re-zero and settle before BACK190 resumes yaw hold.
 * Mode31 left strafes use a private lkp4.5 trial seed and lff+0.01 toward
 * the vehicle rear. The04:30 log showed R1 yaw+6.35deg and the user measured
 * about20mm forward drift; this is a SMALL candidate, not a full slip model.
 * RIGHT765 uses private xkp4.5;43/44 retain3.0 and ordinary ykp is unchanged.
 *31 endpoint/turn-end minimum angular command is0.40 (previous0.30);
 * endpoint cap0.60/still400 stay unchanged;31 end tolerance is0.3.
 * Historical43 seeds (NOT active31's zero-FF task tail): target search/corner use
 * body-right target-search2%/target-to-corner9.5%, hostage search5.5% and final exit7% feedforward;
 * ball forward coarse search uses trial1.5% body-right.
 *31/43 board freezes relative pitch/roll zero after300ms stationary
 * wait; after a300ms startup guard, tilt2.0deg must persist100ms in new frames.
 * it is a user-observed contact trial, not guaranteed squaring or homing.
 *31 logs a10s missing-trigger timeout then continues, never successful contact;
 *43/44 retain their missing-trigger abort.
 * Trial target-to-corner:31 ranks615/445/370;43/44 colors560/410/335. Hostage first-seen
 * rank1/2/3 selects1415/1315/1215 from its grab stop, never QR shape.
 * Legacy34 still owns its old2450/2125 road-only corridors.
 * Routes reuse the existing distance/yaw-fix executor and route RAM tuning;
 *31 normal travel after accepted QR uses yaw>2.0 through the post-cross long
 * strafe (stage6 inclusive), then yaw>1.0;43 retains1.5 throughout.31 task search/fine yaw>1.0
 * brakes and restores the original heading; crossing/contact remain exempt,
 * and resumes the SAME signed remaining distance/task. No mid-leg zero/reset;
 * correction rotation does not consume the remaining straight distance.
 *31 motion/turn timeouts brake, log reached=0, then advance; mid-yaw timeouts
 * retain the original leg/task and rearm after400ms travel, never bypass images.
 * Manual stops, invalid sensors and diagonal geometry errors remain terminal.
 *43/44 keep their prior timeout policy. Crossing31 alone multiplies body-left
 * front/rear wheel TARGET speeds by1.15;44 retains its separate1.10 trial.
 * The table below retains43's545/780/182 seeds; active31 copies/overrides
 * its own legs without changing43 RAM or44's physical-node recipe.
 * manual/mode32 tuning and the formal mission are not changed here. */
/* ===== 31 手动调参入口（下面的数值以当前代码为准） =====
 * 单位：MM=毫米，V_MMS=毫米/秒，DEG=度，MS=毫秒，PPS=脉冲/秒，US=舵机脉宽微秒。
 * 只改数字，保留 u/f 后缀；参数改完重新编译、烧录。已发送的蓝牙 RAM 参数会覆盖默认值。
 * 当前路线：左移 -> 后退 -> 后退/左移斜走 -> 右转 -> 倒车越障 -> 纠角/靠板
 * -> 后退190 -> 右转 -> 前进775 -> 右转 -> 前进775 -> 右转 -> 找球/抓放/打靶/人质。
 * 两段前进分别调：第一段 EXIT_FORWARD_MM=745，第二段 TASK_ENTRY_FORWARD_MM=795。
 * 标有“旧参数/43”的不是31当前路线的调参入口，不要看到 BACK/182 就直接改。
 * 球、桶、人质的目标像素在 vision_align_test.h 的 VAT_ROUTE31_*_X_PX。
 */
#define ROUTE_TEST_MODE 31
#define ROUTE_TEST_V_MMS 100.0f /* Keep legacy34/36/37 defaults independent. */
#define ROUTE31_STRAIGHT_V_MMS 150.0f /*31全程普通前进/后退默认速度；找目标巡航、回终点也用它。越障/靠板/微调不受影响。 */
#define ROUTE31_LATERAL_V_MMS 250.0f /*31出发左平移速度；蓝牙 pv 可以覆盖，与普通直走速度分开。 */
#define ROUTE31_HEADING_KP_SEED 10.0f /* 普通直走/找目标时的航向比例增益 ykp，不是侧向串动补偿。 */
#define ROUTE31_START_LEFT_MM 590u /* R1：出发左平移距离。43的旧距离545另行保留。 */
#define ROUTE31_TASK_ENTRY_FORWARD_MM 805u /* 越障后第二段前进：第二次右转后，前进到进入排爆区的转角。 */
#define ROUTE31_TASK_ENTRY_BACK_MM ROUTE31_TASK_ENTRY_FORWARD_MM /* 旧名字的距离别名；当前31实际执行前进，不是后退。 */
/* Only31's bottom-right approach. All distances are positive body-axis mm.
 * Map south=body-back and west=body-left BEFORE the retained RIGHT90.
 * Tune one at a time; changing diagonal-left also changes total westward
 * travel unless CORNER_CROSS is adjusted separately.43/44 stay610/620. */
#define ROUTE31_CORNER_BACK_MM 480u       /* R2：斜走之前，纯后退的距离。当前31不使用下面旧的 R2_BACK_MM=610。 */
#define ROUTE31_CORNER_DIAG_BACK_MM 100u  /* 紧接R2的斜走：后退分量，不是斜线总长度。 */
#define ROUTE31_CORNER_DIAG_LEFT_MM 100u  /* 同一次斜走的左移分量；车身不自转。 */
#define ROUTE31_CORNER_CROSS_MM 580u      /* 右转后倒车越障距离；与前面的斜走左移100合计该轴650。 */
#define ROUTE31_CORNER_DIAG_V_MMS 100.0f  /*31越障前左后斜走合速度上限；原50动力不足，单独提高到100试验。 */
#define ROUTE31_CORNER_DIAG_ORTH_KP 0.5f  /* 斜走横向轨迹修正增益，单位(mm/s)/mm；未实测标定。 */
#define ROUTE31_CORNER_DIAG_ORTH_TOL_MM 12.0f /* 斜走终点沿线/垂线容差；几何错误会停。 */
#define ROUTE31_CORNER_DIAG_MAX_MS 6000u  /* 斜走总时限，包括纠角暂停；超时报告未到位后进入下一步。 */
#define ROUTE31_TASK_V_MMS ROUTE31_STRAIGHT_V_MMS /* 后半程直走/找目标跟随全局默认150；蓝牙 v 仍可覆盖。 */
#define ROUTE31_TASK_LATERAL_V_MMS 300.0f /* 保留原有任务区可选平移速度；当前球/人质的可选平移距离为0，不执行。 */
#define ROUTE31_CROSS_LEFT_SCALE 1.2f /* 只在倒车越障时：按车头判断的左前/左后目标轮速乘1.15，即加15%。 */
#define ROUTE43_STRAIGHT_V_MMS 100.0f /* Keep historical43 default independent from active31 v150. */
#define ROUTE43_LATERAL_V_MMS 100.0f /* Independent historical43 default; do not inherit31 pv250. */
/* Legacy reverse-crossing recipe for34/36/37;31/43 use the private table below. */
#define ROUTE_CROSS_BACK_MM       650u
#define ROUTE31_CROSS_BACK_MM     620u /*43/44旧越障距离；31当前实际用 CORNER_CROSS_MM=550。 */
#define ROUTE_CROSS_BACK_V_MMS    300.0f /* 倒车越障速度；独立于31普通直走默认150。共享越障初值。 */
#define ROUTE_POST_CROSS_ALIGN_MM  80u
#define ROUTE_POST_CROSS_ALIGN_V_MMS 20.0f
#define ROUTE_POST_CROSS_CLEAR_MM 190u /* 靠板并停稳清零后，后退离板的距离；31速度用全局直走值。 */
#define ROUTE_POST_CROSS_CLEAR_V_MMS 100.0f /* 旧独立路线速度；31的后退离板速度不读这个值。 */
#define ROUTE_TEST_CROSS_V_MMS ROUTE_CROSS_BACK_V_MMS
#define ROUTE_TEST_ACC_MMS2 700.0f /* 路线加速度，单位mm/s^2；共享初值。 */
#define ROUTE_TEST_DEC_MMS2 350.0f /* 路线减速度，单位mm/s^2；共享初值。 */
#define ROUTE_TEST_FORWARD_FF_SEED (-6.25f / 1000.0f) /* 前半程前进侧向速度补偿初值：负值向车身右侧；31任务区固定零补偿。 */
#define ROUTE_TEST_LEFT_FF_SEED 0.0f /* route v100 not measured; lff sets RAM trial */
#define ROUTE_TEST_RIGHT_FF_SEED 0.0f /* do not silently copy manual v300 calibration */
#define ROUTE_TEST_HEADING_KP_SEED 0.3f /* user prefers less physical drift, 2026-10-05 */
#define ROUTE31_LEFT_YAW_KP_SEED 4.5f /*31左平移保持车头方向的增益 lkp，不是普通直走 ykp。 */
#define ROUTE43_LEFT_YAW_KP_SEED 3.0f /* Preserve43/44 legacy lateral profile. */
#define ROUTE31_LEFT_FF_SEED 0.010f /*31左平移串动补偿：正值向车尾加1%的纵向速度；实走效果需测试。 */
#define ROUTE31_EXIT_RIGHT_FF_SEED ROUTE31_LEFT_FF_SEED /* 旧长右平移补偿；31该路段已改右转+前进，当前不作用于前进。 */
#define ROUTE31_STAGES 16u
#define ROUTE31_PAIR_STAGE            9u
#define ROUTE31_PREDEPLOY_STAGE       8u /*43 legacy predeploy before LEFT90;31 waits for first selected ball. */
#define ROUTE31_RETURN180_STAGE      10u
#define ROUTE31_TARGET_STAGE         11u
#define ROUTE31_TARGET_CORNER_STAGE  12u
#define ROUTE31_HOSTAGE_TURN_STAGE   13u
/* Continuous arc trial is disabled: restore distance stop/yaw-fix, in-place
 * RIGHT90, then the finite body-forward/left diagonal below before task3. */
#define ROUTE31_ARC_ENABLE          0
#define ROUTE31_ARC_TARGET_DEG       89.5f /* 此处连续右弯角度；不改其他90/180转向和43/44。 */
#define ROUTE31_HOSTAGE_CORNER_DIAG_MM 110u /* 右转后左前斜走：斜线总长110mm，不是两个分量各110。 */
#define ROUTE31_HOSTAGE_CORNER_DIAG_V_MMS 200.0f /* 本段默认合速度200；显式蓝牙v较低时仍限速，其他直走默认150不改。 */
#define ROUTE31_HOSTAGE_CORNER_DIAG_ORTH_TOL_MM 5.0f /* 垂直于斜线的编码器投影容差。 */
#define ROUTE31_HOSTAGE_CORNER_DIAG_ALONG_TOL_MM 20.0f /* 越点采样/刹车余量，不提前20mm停车；非实车到位精度。 */
#define ROUTE31_HOSTAGE_STAGE        14u
#define ROUTE31_HOSTAGE_EXIT_STAGE   15u
#define ROUTE31_GREEN_TO_CORNER_MM  470u /*31: green only, prior435 minus15; red/blue independent and unchanged. */
#define ROUTE31_RED_TO_CORNER_MM    560u
#define ROUTE31_BLUE_TO_CORNER_MM   385u
/* Active31 TARGET54/task2 first-seen-rank mapping, copied before RX closes.
 * rank0 waits stationary after laserOFF; never fall back to QR color/rank1.
 * First-seen order vs physical stations is still a hardware verification item.
 *43/44 retain the legacy color distances above. */
#define ROUTE31_TARGET_RANK1_MM 560u /* 恢复弧转前560，再减30；按首次出现编号，不按颜色。 */
#define ROUTE31_TARGET_RANK2_MM 395u /* 恢复弧转前395，再减30。 */
#define ROUTE31_TARGET_RANK3_MM 340u /* 恢复弧转前340，再减30。 */

/* User trial: no additional in-place heading repair after target fire.
 * Primary turns, moving heading hold and normal road endpoints stay unchanged.
 * Set1 only to retain the historical correction candidate's host coverage. */
#ifndef ROUTE31_TARGET_STOP_YAW_ENABLE
#define ROUTE31_TARGET_STOP_YAW_ENABLE 0 /*31打靶停车后不额外原地纠角；移动中航向保持和超角度纠偏仍在。 */
#endif
#define ROUTE31_HOSTAGE_HOLD_MS       0u /* 人质额外观察等待为0；停稳、抓取及有效编号门控仍保留。 */
#define ROUTE31_HOSTAGE_RANK1_MM   1415u /* 抓到1号人质后，到终点的前进距离；速度用全局直走值。 */
#define ROUTE31_HOSTAGE_RANK2_MM   1355u /* 抓到2号人质后，到终点的前进距离。 */
#define ROUTE31_HOSTAGE_RANK3_MM   1215u /* 抓到3号人质后，到终点的前进距离。 */
#define ROUTE43_HOSTAGE_RANK1_MM   1415u /* Deferred43 preserves its previous final distances. */
#define ROUTE43_HOSTAGE_RANK2_MM   1315u
#define ROUTE43_HOSTAGE_RANK3_MM   1215u
#define ROUTE31_LASER_MS           2000u /*31对准靶子后激光亮2秒；独立35的持续亮逻辑不由此控制。 */
#define ROUTE31_TARGET_SEARCH_V_MMS ROUTE31_STRAIGHT_V_MMS
#define ROUTE31_TARGET_FINE_V_MMS    30.0f /*31靶子前后精调速度，不随全局 v 改动；35另有速度。 */
#define ROUTE31_TARGET_CX             265 /*31靶子的目标X像素；下面上下界自动跟随中心，允许正负3。 */
#define ROUTE31_TARGET_LOW_CX         (ROUTE31_TARGET_CX - 3)
#define ROUTE31_TARGET_HIGH_CX        (ROUTE31_TARGET_CX + 3)
#define ROUTE31_FINE_ENTER_CX         (ROUTE31_TARGET_CX + 110) /* 只用于靶子：首次X<=此值进入精调，进入后不退回粗搜。 */
#define ROUTE43_TARGET_CX             250 /* Preserve43/44's prior target workpoint. */
#define ROUTE43_TARGET_LOW_CX         247
#define ROUTE43_TARGET_HIGH_CX        253
#define ROUTE43_FINE_ENTER_CX         360
/* 以下任务区串动补偿是43等历史初值：当前31任务区实际固定零侧向补偿。
 * 仅修改这些FF数字不会改变31当前找球/靶/人质、靶后前进或回终点的轨迹。
 * 31出发阶段仍可使用上面的普通前进FFF与左平移LFF，两者不是原地旋转。 */
#define ROUTE31_BALL_SEARCH_RIGHT_FF_RATIO 0.015f /*43等历史找球补偿初值：向车身右侧15mm/m；当前31不用。 */
#define ROUTE31_TARGET_SEARCH_RIGHT_FF_RATIO 0.020f /*43等历史找靶补偿初值：向车身右侧20mm/m；当前31不用。 */
#define ROUTE43_TARGET_SEARCH_RIGHT_FF_RATIO ROUTE31_TARGET_SEARCH_RIGHT_FF_RATIO
#define ROUTE31_HOSTAGE_SEARCH_RIGHT_FF_RATIO 0.055f /*43等历史找人质补偿初值：向右55mm/m；当前31不用。 */
#define ROUTE43_HOSTAGE_SEARCH_RIGHT_FF_RATIO ROUTE31_HOSTAGE_SEARCH_RIGHT_FF_RATIO
#define ROUTE31_CORNER_RIGHT_FF_RATIO 0.095f /*43等历史靶后到转角补偿初值：向右95mm/m；不是航向角补偿，当前31不用。 */
#define ROUTE43_CORNER_RIGHT_FF_RATIO ROUTE31_CORNER_RIGHT_FF_RATIO
#define ROUTE31_HOSTAGE_EXIT_RIGHT_FF_RATIO 0.07f /*43等历史回终点补偿初值：向右70mm/m；当前31不用。 */
#define ROUTE43_HOSTAGE_EXIT_RIGHT_FF_RATIO ROUTE31_HOSTAGE_EXIT_RIGHT_FF_RATIO
#define ROUTE31_LIFT_AXIS             1
#define ROUTE31_LIFT_PPS          10000u /* 升降步进脉冲频率；共享31/43。发出步数不等于确认无丢步。 */
#define ROUTE31_LIFT_DOWN_STEPS   32000u /* 抓球前下降步数。 */
#define ROUTE31_LIFT_BALL_UP_STEPS 32000u /* 抓球后抬升步数。 */
#define ROUTE31_LIFT_BUCKET_DOWN_STEPS 20000u /* 对准桶后，放球前下降步数。 */
#define ROUTE31_LIFT_BUCKET_UP_STEPS 20000u /* 放球后抬升步数。 */
#define ROUTE31_BALL_GRIP_WAIT_MS   2000u /* 夹紧球后的舵机等待，不是观察占位。 */
#define ROUTE31_BUCKET_RELEASE_WAIT_MS 2000u /* 松爪后的总等待；第1秒会改到预备脉宽1500。 */
#define ROUTE31_BUCKET_LIFT_WAIT_MS    0u /* Extra post-lift observation pause removed. */
#define ROUTE31_BALL_GRIP_US        2100u /* 抓球夹紧的舵机脉宽，不是占空比或角度。 */
#define ROUTE31_HOSTAGE_GRIP_US     2100u /* 抓人质夹紧的舵机脉宽；到终点保持夹紧。 */
#define ROUTE43_BALL_GRIP_US        ROUTE31_BALL_GRIP_US
#define ROUTE43_HOSTAGE_GRIP_US     ROUTE31_HOSTAGE_GRIP_US
#define ROUTE31_CLAW_READY_US       1500u /* 放球松爪1秒后，爪子变为此预备脉宽；不是在人质转角才动作。 */
#define ROUTE31_BUCKET_RELEASE_READY_MS 1000u /* From actual release1150; original total wait2 remains. */
#define ROUTE31_BALL_PREGRAB_LEFT_MM    0u /* 对准球后、抓球前左移；0表示跳过。要恢复这一处平移可改此数值。 */
#define ROUTE31_BALL_TO_BUCKET_LEFT_MM 15u /* 旧首次180后左移参数；当前31直接请求桶并跳过此处，仅改数字不会恢复。 */
#define ROUTE31_BALL_TO_BUCKET_LEFT_V_MMS 80.0f /* 旧180后小平移速度上限；该动作目前在31被跳过。 */
#define ROUTE31_PAIR_LEFT_MM          40u /* Historical macro; current31/43 seed BALL_TO_BUCKET_LEFT_MM instead. */
#define ROUTE31_RETURN_LEFT_MM        20u /* Historical macro; current31/43 seed RETURN_RIGHT_MM instead. */
#define ROUTE31_RETURN_RIGHT_MM        0u /* 第二次180后右平移距离：当前0跳过；设为正数会恢复此动作和纠角，速度上限80。 */
#define ROUTE31_RACK_AXIS              0
#define ROUTE31_RACK_EXTEND_DIR        0 /* 齿条伸出方向=原始 nl/DIR0；不要随意与收回方向互换。 */
#define ROUTE31_RACK_RETRACT_DIR       1 /* 齿条收回方向=原始 nr/DIR1；不改变升降轴方向。 */
#define ROUTE31_RACK_PPS            5000u /* 齿条伸出/收回脉冲频率；升降频率另设为10000。 */
#define ROUTE31_RACK_EXTEND_STEPS    3400u /* 首次看到目标球后伸出，以及放球后全收回的步数。 */
#define ROUTE31_HOSTAGE_RACK_EXTEND_STEPS 3400u /* 抓人质前伸出步数；抓后只收回下面设定的3000步。 */
#define ROUTE43_RACK_EXTEND_STEPS    ROUTE31_RACK_EXTEND_STEPS
#define ROUTE31_HOSTAGE_PREGRAB_LEFT_MM     0u /* 对准人质后、抓前左移；0跳过。恢复时还会重新收图确认X。 */
#define ROUTE43_HOSTAGE_PREGRAB_LEFT_MM    30u /* Preserve43/44's historical left offset independently of active31. */
#define ROUTE31_HOSTAGE_RACK_RETRACT_STEPS 3000u /* 抓紧人质后部分收回步数；不松爪。 */
#define ROUTE31_HOSTAGE_GRIP_SETTLE_MS     250u /* 人质夹紧后等待；没有实体夹紧反馈。 */
#define ROUTE31_RACK_SETTLE_MS        250u /* 齿条动作结束后的稳定等待。 */
#define ROUTE31_POST_CROSS_ALIGN_MM 40u /* 旧定距靠板参数；当前31按倾角触发停止，不靠此距离。 */
#define ROUTE31_BOARD_CONTACT_V_MMS 40.0f /* 靠雪弗板速度：前轮不驱动，后两轮向前推；不受全局 v 影响。 */
#define ROUTE31_BOARD_CONTACT_TILT_DEG 1.5f /* 靠板前停稳校零后，俯仰/横滚变化的绝对值触发阈值；不是航向yaw。 */
#define ROUTE43_BOARD_CONTACT_TILT_DEG ROUTE31_BOARD_CONTACT_TILT_DEG
#define ROUTE31_BOARD_CONTACT_FRAMES 2u /* 靠板确认所需独立有效IMU帧数；重复轮询不算新帧。 */
#define ROUTE31_BOARD_CONTACT_ZERO_WAIT_MS 1000u /* 旧分支停稳校零等待；当前31/43由下面 BOARD_ZERO_WAIT_MS 控制。 */
#define ROUTE31_BOARD_ZERO_WAIT_MS 300u /* 靠板前轮子停稳后，再静置300ms，建立新的倾角零点。 */
#define ROUTE31_STABLE_MS 400u /* 纠角/转弯到位后连续稳定时间；核心判定，单位ms。 */
#define ROUTE31_BOARD_CONTACT_START_GUARD_MS 300u /* 靠板起步后300ms内不以动态倾角误判接触。 */
#define ROUTE31_BOARD_CONTACT_CONFIRM_MS 100u /* 靠板倾角阈值需持续100ms，且有新IMU帧。 */
#define ROUTE31_BOARD_CONTACT_MAX_MS 10000u /*31靠板时限10秒；超时报告未确认后继续，不当作实体靠板成功。 */
#define ROUTE31_BOARD_NUDGE_BACK_MM    10u /* Historical macro; current31/43 skip post-contact nudges. */
#define ROUTE31_BOARD_NUDGE_FORWARD_MM 15u /* Historical macro; current31/43 skip this leg. */
#define ROUTE31_R2_BACK_MM          610u /*43旧R2距离；31实际R2请改上面的 CORNER_BACK_MM。 */
#define ROUTE31_EXIT_RIGHT_MM      745u /* 旧越障后长右平移距离；当前31已替换成右转+前进，不再读取此距离。 */
#define ROUTE31_EXIT_FORWARD_MM    680u /* 越障后第一段前进：按当前795减50；第二段 TASK_ENTRY_FORWARD_MM 独立保留795。 */
#define ROUTE31_EXIT_BACK_MM       ROUTE31_EXIT_FORWARD_MM /* Historical distance alias; no active31 BACK on this stage. */
#define ROUTE43_EXIT_RIGHT_MM      780u /* Preserve43's shared-table seed;31 overrides its live leg. */
#define ROUTE31_EXIT_YAW_KP_SEED   4.5f /* 旧长横移 xkp；当前31该段是前进，实际用普通直走 ykp。 */
#define ROUTE43_EXIT_YAW_KP_SEED   3.0f /* Preserve43 RAM seed. */
#define ROUTE31_ENTRY_BACK_MM      780u /*43旧后退入口距离；31当前第二段前进请改 TASK_ENTRY_FORWARD_MM。 */
/* 31 ordinary endpoints: restore continuous correction to the ORIGINAL heading.
 *31 either side may settle with abs(error)<0.3 and400ms still, without crossing
 * zero or pulse/gap cycles.31/43 use minimum angular command0.40/0.30rad/s
 * and a0.60rad/s cap after insufficient progress.
 * Mode31 accepts after2s even
 * if still outside tolerance, reporting the remaining error before re-zero.
 * Other modes and primary90/180 turns keep their12s failure budgets.
 * This is a trial velocity command, not measured torque/angle accuracy.
 * Manual stop and all other turn/XY/mode profiles remain unchanged. */
#define ROUTE31_POST_YAW_TOL_DEG   0.3f /*31纠角结束要求：误差绝对值严格小于0.3度，并稳定400ms。 */
#define ROUTE43_POST_YAW_TOL_DEG   0.4f
#define ROUTE31_POST_YAW_MIN_W     0.40f /*31纠角最小角速度命令rad/s；不是PWM/实测扭矩。 */
#define ROUTE31_POST_YAW_MAX_W     0.60f /*31纠角上限；400ms进展不足时加强到此档。 */
#define ROUTE43_POST_YAW_MIN_W     0.30f /* Preserve43/44 correction floor. */
#define ROUTE31_POST_YAW_MAX_MS    2000u /*31纠角时限2秒；到时继续并记录剩余误差，不代表已经纠正。 */
#define ROUTE31_MID_YAW_LIMIT_DEG 1.0f /* 后半程移动中误差严格超过1度：刹停纠角，同段继续。 */
#define ROUTE43_MID_YAW_LIMIT_DEG 1.5f /* Preserve43/44. */
#define ROUTE31_YAW_RETRY_RUN_MS 400u /* 中途纠角超时后，同一段先走400ms再允许重试；不继承到下一段。 */
#define ROUTE31_EARLY_MID_YAW_LIMIT_DEG 2.0f /* 接受QR后到越障后第一段前进结束：移动中严格超过2度才纠。越障/靠板除外。 */
#define ROUTE31_EARLY_MID_YAW_LAST_STAGE 6u /* 2度档最后一个逻辑节点是第一段前进；第二段开始用上面的1度档。 */
#define ROUTE31_PRE_CROSS_MID_YAW_LIMIT_DEG ROUTE31_EARLY_MID_YAW_LIMIT_DEG /* Compatibility name; no43/44 change. */
#define ROUTE31_RIGHT_TARGET_DEG 90.0f /*31普通右转角度，包括越障前和越障后新加的三个右转；人质入口单独设定。 */
#define ROUTE31_HOSTAGE_RIGHT_TARGET_DEG 89.5f /* 人质转角右转目标角度；不影响独立20转弯测试。 */
#define ROUTE43_RIGHT_TARGET_DEG 90.0f /* Historical macro; current43 seeds the per-stage31 goals. */
#define ROUTE31_LEFT_TARGET_DEG (-90.0f) /* 旧路线左转目标角度；31当前三个新转角均用右转。 */
#define ROUTE31_RETURN_TARGET_DEG 180.0f /*43旧的第二次回转初值；当前31不使用182，请勿改错。 */
#define ROUTE31_ACTIVE_RETURN_TARGET_DEG 180.0f /*31放完球后的第二次回转角度；首次抓球后回转仍为180。 */
/* Route31 +/-90 and +180: remove the earlier2x coarse boost for a slow trial.
 *43 uses the same seed; its private tv/tkp remain adjustable while stopped.
 * These are trial speed scheduling thresholds, not
 * measured angle compensation. Do not scale endpoint yaw-fix or standalone turns. */
#define ROUTE31_TURN_SPEED_SCALE 1.0f /*31主要90/180转弯的粗转速度倍率；不改变纠角速度。 */
#define ROUTE43_TURN_SPEED_SCALE ROUTE31_TURN_SPEED_SCALE
#define ROUTE31_TURN_MIN_W       0.40f /*31主转弯末端慢修正的最小角速度rad/s。 */
#define ROUTE43_TURN_MIN_W       0.30f /* Preserve43; standalone/44 retain0.18. */
#define ROUTE31_TURN_TOL_DEG     0.3f /*31主转弯到位容差：误差绝对值<=0.3度；普通纠角另要求严格<0.3。 */
#define ROUTE43_TURN_TOL_DEG     0.4f /* Preserve43/44 turn arrival. */
#define ROUTE31_TURN_SLOW_DEG 15.0f /* 转弯速度分档的临近目标阈值；不是最终转弯目标角度。 */
#define ROUTE31_TURN_FAST_DEG 25.0f /* 转弯速度分档的远离目标阈值；与最终90/180目标分开。 */
/* Legacy bucket/manual-d route for34 only. */
#define ROUTE_TEST_STAGES 15u
#define ROUTE_TEST_ALIGN_STAGE 7u
#define ROUTE_TEST_BACK_STAGE 8u

typedef struct {
    uint8_t mode;
    uint16_t distance_mm; /* positive slot value; direction comes from mode */
    float speed_mms;
    const char *name;
    uint8_t heading_hold; /* 0: no yaw correction; 1: normal route RAM gain */
} RouteTestLeg;

/* 此表是43历史种子，不是31最终执行表。31实际执行时会在 test.c 中覆盖
 * R1、R2、越障、两个775前进、转向等参数；请优先改上面标明生效的宏。
 * 表内mode：15前进，16后退，17左平移，18右平移，20右转，30左转，22回转。
 * Shared historical seeds for43. Active31 copies its road legs before overriding
 *585/775, and replaces stage6/7/8 by RIGHT90+FORWARD, RIGHT90+FORWARD, RIGHT90.
 * The two internal turns do not renumber task roles or change43/44 seeds.
 * Tail speeds/FF and paired offsets are scoped in its executor. */
static const RouteTestLeg s_route31_plan[ROUTE31_STAGES] = {
    { 17u,  545u, ROUTE43_LATERAL_V_MMS, "START_LEFT", 1u },
    { 16u, ROUTE31_R2_BACK_MM, ROUTE43_STRAIGHT_V_MMS, "BACK_TO_3RD", 1u },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT90_BEFORE_CROSS", 1u },
    { 16u, ROUTE31_CROSS_BACK_MM, ROUTE_CROSS_BACK_V_MMS, "BACK_FULL_OBSTACLE_ROAD", 0u },
    { 15u, 0u, ROUTE31_BOARD_CONTACT_V_MMS, "POST_CROSS_FORWARD_TILT_CONTACT", 0u },
    { 16u, ROUTE_POST_CROSS_CLEAR_MM, ROUTE43_STRAIGHT_V_MMS, "POST_CROSS_BACK_CLEAR", 1u },
    { 18u, ROUTE43_EXIT_RIGHT_MM, ROUTE43_LATERAL_V_MMS, "EXIT_RIGHT", 1u },
    { 16u, ROUTE31_ENTRY_BACK_MM, ROUTE43_STRAIGHT_V_MMS, "BACK_TO_TASK_CORNER", 1u },
    { 30u,    0u, ROUTE_TEST_V_MMS, "LEFT90_TO_TASKS", 1u },
    { 41u,    0u, ROUTE43_STRAIGHT_V_MMS, "BALL_BUCKET_X_GRAB_RELEASE", 1u },
    { 22u,    0u, ROUTE_TEST_V_MMS, "RETURN180_AFTER_BUCKET", 1u },
    { 35u,    0u, ROUTE43_STRAIGHT_V_MMS, "SELECTED_TARGET_X_FIRE2S", 1u },
    { 15u,    0u, ROUTE43_STRAIGHT_V_MMS, "TARGET_TO_CORNER_UNMEASURED", 1u },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT_TO_HOSTAGE", 1u },
    { 40u,    0u, ROUTE43_STRAIGHT_V_MMS, "HOSTAGE_X_GRAB_RANK", 1u },
    { 15u,    0u, ROUTE43_STRAIGHT_V_MMS, "HOSTAGE_RANK_TO_END_HOLD_GRIP", 1u }
};

static const RouteTestLeg s_route_test_plan[ROUTE_TEST_STAGES] = {
    { 17u,  530u, ROUTE_TEST_V_MMS, "START_LEFT", 1u },
    { 16u,  650u, ROUTE_TEST_V_MMS, "BACK_TO_3RD", 1u },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT90_BEFORE_CROSS", 1u },
    { 16u, ROUTE_CROSS_BACK_MM, ROUTE_CROSS_BACK_V_MMS, "BACK_FULL_OBSTACLE_ROAD", 0u },
    { 15u, ROUTE_POST_CROSS_ALIGN_MM, ROUTE_POST_CROSS_ALIGN_V_MMS, "POST_CROSS_FORWARD_ALIGN", 0u },
    { 16u, ROUTE_POST_CROSS_CLEAR_MM, ROUTE_POST_CROSS_CLEAR_V_MMS, "POST_CROSS_BACK_CLEAR", 1u },
    { 30u,    0u, ROUTE_TEST_V_MMS, "LEFT92_TO_BUCKET_VIEW", 1u },
    {  0u,    0u, 50.0f, "BUCKET_X500_ALIGN", 1u },
    { 16u,    0u, ROUTE_TEST_V_MMS, "MANUAL_BACK_D", 1u },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT90_AFTER_MANUAL_BACK", 1u },
    { 15u,  780u, ROUTE_TEST_V_MMS, "FWD_TO_TASK_CORNER", 1u },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT90_TO_TASKS", 1u },
    { 15u, 2450u, ROUTE_TEST_V_MMS, "TASK_CORRIDOR_NO_TASKS", 1u },
    { 20u,    0u, ROUTE_TEST_V_MMS, "RIGHT90_TO_RESCUE", 1u },
    { 15u, 2125u, ROUTE_TEST_V_MMS, "RESCUE_TO_FINAL_NO_TASK", 1u }
};

#endif
