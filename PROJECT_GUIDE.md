# 反恐电控工程导览与调参索引

更新：2026-10-01。当前共用工作工程为 `C:/Users/15119/gongkesai/gongke`（GitHub `zjj20040618-sudo/gongke`）；不要与桌面交接包或旧 `jiejie` 仓库混编。Keil 从 [MDK-ARM/jiejie.uvprojx](MDK-ARM/jiejie.uvprojx) 打开，工程名不代表仍使用旧仓库。

视觉接收侧新增二进制适配；完整接口与未完成项见 [VISION_INTEGRATION.md](VISION_INTEGRATION.md)。下面的 2026-09-30 编译/ASCII 检查条目是历史记录，不是新协议的整机验收。

这是一张**找代码的地图**，不是“全部参数已经调好”的证明。`TODO`、`0` 和标注为“种子”的值都要按实车测试填写。当前正式整场有配置闸门，不能因工程能编译就直接上车跑。

## 从哪儿开始读

```text
Src/main.c + Src/freertos.c  启动/任务调度
          ↓
App/robot.c               初始化、三路串口分流、蓝牙服务
          ├─ App/test.c               蓝牙台架测试（与整场互斥）
          └─ App/mission.c            正式整场状态机、QR 与路线
                    ├─ App/task_eod.c / task_anti.c / task_rescue.c
                    └─ App/steps.c + auto_steps.c  可复用动作/越障
                              ↓
             App/motion.c → control.c → board_pins.c（底盘）
             App/arm.c（齿条、丝杆、爪舵机）
             App/proto.c / imu.c（视觉帧、姿态输入）
```

第一次阅读建议顺序：`robot.c` → `mission.c` → 对应的 `task_*.c` → `steps.c` → `motion.c/control.c`；要调某个模块再看下表，不必先翻完 `test.c` 的全部模式。

## 每个自写文件负责什么

同名 `.h` 主要放对外接口/类型；算法和参数通常在 `.c`。本表覆盖 `App/` 的全部自写模块。

| 文件 | 职责 | 要找的参数/入口 |
| --- | --- | --- |
| [robot.c](App/robot.c) / [robot.h](App/robot.h) | 全系统初始化；USART2 视觉、USART3 蓝牙、UART4 IMU 接收分流；周期任务入口与诊断回传 | `FW_BUILD_ID`、`robot_init()`、`robot_diag_report()`；固件号需随发布版本人工更新 |
| [mission.c](App/mission.c) / [mission.h](App/mission.h) | 正式整场顺序、QR 三目标校验、路线腿和启动闸门 | `QR_*`、`R_*`、`ROUTE_FWD_V_MMS`、`ROUTE_STRAFE_V_MMS`、`CROSS_*`、`CAL_*_READY`；`mission_config_missing()` 会指出缺项 |
| [task_eod.c](App/task_eod.c) | 排爆：对球、预降、伸爪抓球、抬升、两次 180°、对桶放球、双轴按步数回起点 | `EOD_BALL_PRELOWER_STEPS`、`EOD_BALL_EXTEND_STEPS`、`EOD_LIFT_STEPS`、`EOD_LOWER_STEPS`（全待实测） |
| [task_anti.c](App/task_anti.c) | 反恐：对选定颜色靶、用节点停稳判据确认、激光射击 | `LASER_ON_MS`；停稳时序复用 `steps.c::step_prepare_leg()`，站位在 `steps.c` |
| [task_rescue.c](App/task_rescue.c) | 救援：对选定形状人质、预降、伸爪抓取、抬升并保持夹持；底盘回程由 `mission.c` 做 | `RESCUE_PRELOWER_STEPS`、`RESCUE_EXTEND_STEPS`、`RESCUE_LIFT_STEPS`（全待实测）；不放下、不做排爆式复位 |
| [robot_tasks.h](App/robot_tasks.h) | 三个任务的接口与返回码 | 只改接口，不放行程数值；**不要改名为 `task.h`**，会与 FreeRTOS 头文件撞名 |
| [steps.c](App/steps.c) / [steps.h](App/steps.h) | 共用步骤：视觉扫/对位、节点停稳、直走/横移/旋转、爪与两轴动作、激光；正式抓放前停稳、阶段回传 | `s_stand[4]`、`X_ALIGN_*`、`X_DEPTH_*`、`SWEEP_*`、`s_nav_w_kp_deg`、`s_orth_kp`、`ROT_*`、`ARM_STEP_INTERVAL_MS`、`AXIS1_*_DIR`、`step_arm_prepare/run/release()`；正式 180°已修连续航向方向歧义，但仍未集成模式22的速度参数或完成实车验收 |
| [auto_steps.c](App/auto_steps.c) / [auto_steps.h](App/auto_steps.h) | 越障：定速、锁向、pitch 峰峰值两阶段判定 | `X_RISE_DEG`、`X_FLAT_DEG`、`X_WIN_N`；越障速度/物理防跑飞时长在 `mission.c` |
| [motion.c](App/motion.c) / [motion.h](App/motion.h) | 麦轮运动学、前进/横移里程、位姿估计、启停加减速剖面 | `M_WHEEL_R_MM`、`M_A_HALF_MM`、`s_profile` (`acc/dec`)；`M_GEAR_RATIO` **保持 1.0**，不可再乘 30 |
| [control.c](App/control.c) / [control.h](App/control.h) | 四轮编码器测速和每轮速度闭环、刹车、开环单轮驱动 | `CTRL_ENCODER_CPR` 在 `.h`；`s_kp/s_ki/s_lp_alpha/s_dead_min` 在 `.c`，可先蓝牙 RAM 临时调 |
| [board_pins.c](App/board_pins.c) / [board_pins.h](App/board_pins.h) | 电机/编码器索引和极性、PWM/方向脚、STBY、激光、蓝牙 TX | `s_motor_inv`、`s_encoder_inv` 和 pin/TIM 数组；仅在单轮正向及编码器符号复核后改 |
| [arm.c](App/arm.c) / [arm.h](App/arm.h) | 两路步进 STEP/DIR 脉冲、爪舵机 PWM | `CLAW_OPEN_US`、`CLAW_CLOSE_US`、STEP/DIR pin 数组；当前机械挡块**没有**回零电信号 |
| [imu.c](App/imu.c) / [imu.h](App/imu.h) | IMU 串口帧解析、连续 yaw/pitch/roll、链路有效性、分段航向软件零点 | `IMU_LINK_TIMEOUT_MS`、协议字段；方向/零点须结合实车数据查，不靠改常数猜 |
| [proto.c](App/proto.c) / [proto.h](App/proto.h) | 当前视觉 AA55/CRC16 二进制接收、QR/OBJ 类别转换；保留旧 ASCII 解析用于回归，二进制模式尚无切场景命令 | `proto_set_binary_mode`、`CLS_*`、`LAB_*`、`ProtoFrame`；按 `VISION_INTEGRATION.md` 核对映射与目标工作点 |
| [test.c](App/test.c) / [test.h](App/test.h) | 蓝牙台架模式 1–29、`g` 控制、数据回传、RAM 调参；**不是**正式整场路线 | 执行器/命令解析在 `.c`；命令见下文，测试 `d/v` 不会写入 `mission.c` |
| [test_config.h](App/test_config.h) | 台架命令长度、模式上限、采样周期、默认速度/距离/补偿种子及上电安全开关集中入口 | `BENCH_AUTO=0` 必须保持；只改变台架默认，不会自动改变正式路线；右转 90°仍复用 `turn_profile.h` |
| [turn_profile.h](App/turn_profile.h) | 已做过落地测试的右转 90° 参数组，供测试模式20与正式 90°分支共用 | `TURN90_*`；模式22 的 180°只是候选，正式通用 180°参数在 `steps.c` |

`Src/main.c`/`Src/freertos.c` 是启动和任务周期，`Src/gpio.c`/`Src/tim.c`/`Src/usart.c` 与根目录 `jiejie.ioc` 是 CubeMX 外设配置；`Drivers/`、`Middlewares/` 是库。改引脚、定时器、串口时要同时核对 `.ioc`、生成代码和 `App/board_pins.c`/`arm.c`，不要只改其中一个。

## 要调什么，去哪里改

| 调试内容 | 台架/RAM 临时入口 | 最终归属（填实测值后全量重编） |
| --- | --- | --- |
| 单轮方向、编码器正负、轮位 | `test.c` 模式7–10、13，蓝牙 `diag` | `board_pins.c` 的电机/编码器映射与极性；`control.h` 的 `CTRL_ENCODER_CPR` |
| 轮速环 | 蓝牙 `kp/ki/lp/dead` → `param`，断电丢失 | `control.c` 的四个默认种子参数 |
| 底盘直走/横移距离、速度 | 模式15–18 的 `v`/`d`；`ykp/okp/acc/dec` 为 RAM-only；`lff/rff/fff` **只在指定模式和速度生效** | 轮径/运动学在 `motion.c`，航向/正交修正在 `steps.c`，正式路线段长/速度在 `mission.c`；测试 `d` 不会自动写回 |
| 起停加减速 | 蓝牙 `acc/dec` → `param`，仅 RAM | `motion.c` 的 `s_profile`；正式任务要求非零且实测确认 |
| 90°/180°车身自转 | 模式20/22 日志与实体角度 | 90°看 `turn_profile.h`；正式通用 180°看 `steps.c` 的 `ROT_*`；不能直接把模式22完成当正式 180°验收 |
| QR/球/靶/人质/桶视觉 | 核对手机/视觉串口原始帧及 `diag`；视觉识别算法由队友负责 | `proto.h` 类别/标签，`mission.c` QR 三元组，`steps.c` 的 `s_stand[4]`、对位与扫描参数 |
| 越障 | 先采 IMU pitch 与实车通过数据 | `auto_steps.c` 的 `X_RISE_DEG/X_FLAT_DEG`；`mission.c` 的 `CROSS_V_MMS/CROSS_TMO_MS` |
| 爪开合与两步进方向 | 蓝牙 `su<脉宽>`、`co/cc`；24/25 有限步不回，26/27 有限步等待 2 秒反走，步进方向/步数用 `nl5`（DIR0）或 `nr5`（DIR1）；28 舵机等待 2 秒回前一脉宽，29 舵机保持 | `arm.c` 爪脉宽与轴引脚、`steps.c` 方向/步间隔；`l/r` 只是测试命令别名，不代表已确认机械方向；勿用机械挡块撞停找零 |
| 球抓放 | 装车后逐段量下降、伸出、抬升、降桶和回程 | `task_eod.c` 的四个 `EOD_*` 步数；回程按已走步数反向，不是传感器绝对回零 |
| 人质抓取 | 装车后逐段量下降、伸出、抬升 | `task_rescue.c` 的三个 `RESCUE_*` 步数；抓后保持夹持，不放人/复位 |
| 激光 | 先验证编码器停稳、实物余晃与实际照射 | `task_anti.c` 的亮灯时间，停稳判据及目标站位在 `steps.c` |

当前已知的调试命令可在蓝牙发 `?` 让固件自己列出；发 `param` 查 RAM 参数，发 `diag` 查固件号与链路。**只有在空闲/BOOT 时**才可选号与调参；`g` 的含义随模式变化，上电不会自动转轮（`BENCH_AUTO=0`）。不要把示例命令当作允许未经架空/落地风险检查就启动机械。

## 配置状态与安全边界

- 正式整场入口是 `mission.c::mission_start()`；`mission_config_missing()` 会拒绝仍为 0 的路线、扫描、球/人质行程等关键数值，还要四个 `CAL_*_READY` 人工确认。不要为了“能跑”直接把闸门改成 1。
- 目前球/人质关键行程仍为 0。2026-09-30 正式抓球、对桶放球、救援抓人质的视觉对位后，已接入现有 `step_prepare_leg()`：先刹车、观察编码器静止 250 ms、IMU 本段软件清零、再等 750 ms，失败或中止就不发机械动作。各抓放动作回传 `REC type=ARM` 阶段和 `ARM_PULSE/ARM_CLAW` 命令记录；`COMMAND_SENT`、`ALL_PULSES_SENT` **仅表示控制命令/脉冲发出**，`physical_unverified=1` 明确表示没有位置传感器证明到位。中途停止后的机械位置不确定，须人工重新确认起点。
- 抓放阶段记录版 Keil 全量重编日志 `MDK-ARM/rebuild_arm_stage_trace_2026-09-30.txt` 为 0 Error / 0 Warning；主机端 `tests/arm_task_flow_test.c` 检查排爆、救援顺序及两个中止点，共 4 例通过。但当前配置闸门仍关闭，链接 map 将正式任务函数裁掉；这只是源码/编译检查，**不是已烧录或实车抓放验收**。填入实测行程后须重新全量重编，确认 map 中正式任务已链接，再做实体测试。
- 2026-09-30 软件复核修复了 `g` 后立刻 `a` 的启动竞态：原 `MissionTask` 醒来会再次调用 `run_reset()`，可能擦掉已收到的急停。现在只在初始化和**接受启动请求之前**清旧标志；启动锁存后立刻检查中止，不再清零。整场 `a` 先置中止并立即下发底盘刹车，蓝牙回 `OK ABORT_REQUEST brake_commanded; physical_stop_unverified`，不谎报物理已停稳。越障循环也改为**先检查急停/防跑飞时限，再发本拍速度**。`tests/obstacle_abort_test.c` 主机 5 例通过；尚未烧录。上车首次整场联调需专门做一次安全架空 `g`→立即 `a` 验证。当前所有标定闸门仍保持关闭，勿为此短测擅自放行整场。
- 反恐视觉对靶后也改为复用 `step_prepare_leg()`，不再只盲等 1 秒就发射。在工作工程根目录运行 `tests/run_host_tests.ps1`，可重复检查排爆/救援 4 例、反恐 2 例、越障中止/时限 5 例、实际 `mission.c` 启动瞬间急停 1 例、QR 三位映射 27 个合法/7 个非法组合、视觉 ASCII 解析 9 行和源码顺序 4 项。启动急停主机测试只注入“启动请求已接受”状态，**正式参数闸门原样关闭**；这些仍非实车验收。
- 2026-09-30 解析复核修复 `proto.c`：数字字段格式错误的 QR/OBJ/ERR 行原会以 `PF_NONE` 被计作已接受，现在默认 `PF_UNKNOWN` 并计入拒收，不传给业务回调。此修复初版固件号 `20260930-PROTO-REJECT-FIX`；Keil 全量日志 `MDK-ARM/rebuild_proto_reject_fix_2026-09-30.txt` 为 0 Error / 0 Warning，未烧录。视觉队友仍须给实际 `QR,d1,d2,d3` 与四类 `OBJ` 样帧，确认标签、置信度、相机中心/尺寸、帧率与场景切换后首帧时序；协议测试不能替代联调。
- 2026-09-30 框架整理把 `test.c` 内台架常量原值移入 `test_config.h`，不动已验证的速度/控制律；同步修正 `steps.h` 曾误称 `step_sweep` 可扫 QR 的过时说明。有限 `to` 的视觉对位以前可能在等目标帧时无限卡住，现匹配帧等待会按时限退出，纵向移动也只使用剩余时限；正式任务当前传 `to=0`，行为不变。新增 `tests/align_timeout_test.c` 主机 1 例通过；全套主机回归通过，Keil 全量日志 `MDK-ARM/rebuild_framework_audit_2026-09-30.txt` 为 0 Error / 0 Warning。HEX SHA-256 仍为 `DB040A88C642C19B39564851EAC190CD94D2EADCA83013CC4071C06DABDA9AA8`（当前配置闸门使这段正式视觉流程被链接器裁掉），未烧录或实车验收。
- 正式普通路线原来一份 `ROUTE_V_MMS` 混用直走与横移，现分成 `ROUTE_FWD_V_MMS`、`ROUTE_STRAFE_V_MMS`，均继续置 0 并纳入启动闸门。你已有的“直走 `v200`、平移 `v300`”是分开设置的依据，但不等于后半程所有路段已验收，故本次不擅自填正式速度。该修改初版固件号 `20260930-ROUTE-SPEED-SPLIT`；Keil 全量日志 `MDK-ARM/rebuild_route_speed_split_2026-09-30.txt` 为 0 Error / 0 Warning，尚未烧录。主机测试另查 3 个后半程直行调用、6 个横移调用都引用对应速度及双闸门。
- 正式通用 `step_rotate_deg(±180)` 原用 0–360° yaw 的最短角差，目标刚好 180°时容易因起转前约 1°噪声选择反方向。现改用 IMU 连续航向减起始航向，保持“正 180 就正向、负 180 就反向”；主机假 IMU 的 3 个首拍方向测试通过。**这里只修角度口径，未套用模式22的速度/超时参数，也未实车带球验证。**当前最新固件号 `20260930-TURN180-CONTINUOUS`；Keil 全量日志 `MDK-ARM/rebuild_turn180_continuous_2026-09-30.txt` 为 0 Error / 0 Warning。正式任务参数仍为 0，map 将 `step_rotate_deg`/三个任务函数裁掉，当前 HEX 不包含这段正式控制代码；填实测参数放行后必须重编确认已链接。
- 台架模式 24–27 限单次 50 步，发 `nl1..nl50` 指 DIR0、`nr1..nr50` 指 DIR1，旧 `n-5`/`n5` 分别等价 `nl5`/`nr5`。方向、轴号、实际位移及挡块余量都须先目视确认；26/27 完整走完后等待 2 秒，再反走同样的**命令步数**。途中按 `g/a/0` 立即停止后续脉冲并取消自动回程；无电气零点，暂停、卡住或断电后从人工标记重新确认起点。
- 模式 28/29 先用 `u1000..u1800` 设置目标，再 `g` 发舵机 PWM 指令。28 等 2 秒回到动作前**指令脉宽**；29 保持。等待中 `g/a/0` 取消回程但保持当前 PWM，不是切电或保证舵机机械立即停住。先不装爪/连杆测，小范围渐进；`su` 为原有即时手动指令。
- 改代码后全量 Rebuild，核对日志、固件识别号、HEX 哈希；上车仍要分别验证方向、行程、碰撞包络，编译通过不代表硬件正确。
