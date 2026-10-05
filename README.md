# 反恐电控 STM32F407 共用工程

更新：2026-10-06。电控固件标识 `20261006-XY38-41-SLOW-T10`；配套视觉仍保留已审核`7a2ab10`源码（App2.1.2、9564模型、USER长按退出、01物体/53三码/63四阶段）。源码、模型、工程依赖与回归测试一并保存在本仓库，不包含个人设置、令牌和编译产物。

先读 [README_本次更新.md](README_本次更新.md) 了解本次改动；视觉队友接着读 [CONTROL_TO_VISION.md](CONTROL_TO_VISION.md)。视觉侧说明保存在 [VISION_TO_CONTROL.md](VISION_TO_CONTROL.md)；双端已有53/63软件配套，源码版本不代表设备已部署。

- 电控工程：[MDK-ARM/jiejie.uvprojx](MDK-ARM/jiejie.uvprojx)。`App/` 为自写控制代码，`Src/`、`Inc/` 为外设/RTOS入口。
- 视觉工程：`03_扫码与物体识别联合App工程/qr_object_app/`，保留队友本轮源码与9564模型，不另改识别算法。
- 结构与调参入口：[PROJECT_GUIDE.md](PROJECT_GUIDE.md)、[MOTION_YAW_TUNING.md](MOTION_YAW_TUNING.md)。
- 协议与共同待办：[VISION_CONTROL_PROTOCOL.md](VISION_CONTROL_PROTOCOL.md)、[VISION_CONTROL_TODO.md](VISION_CONTROL_TODO.md)。
- 电控/机构待测：[CONTROL_TUNING_TODO.md](CONTROL_TUNING_TODO.md)、[ACTUATOR_TEAMMATE_HANDOFF.md](ACTUATOR_TEAMMATE_HANDOFF.md)。
- 协作边界：[COLLABORATION.md](COLLABORATION.md)；拉取源码不等于重新编译、烧录或安装相机App。

本次新增38球、39桶、40人质、41球→180°→桶的独立扫码/XY对齐，不抓取、不亮激光。临时工作点及真实画幅须先核对，不能因源码有数值当作已标定。31为独立12节点道路试跑，R1后须合法QR才进R2，去旧桶/手发d节点；新任务链尚未接入。34保留旧15节点桶对位/等新d配方，不再与31同表；36无QR/桶十节点、37仅越障三段。32仍独立旧路线/无臂任务联调，33只收诊断；已有上电舵机PWM不等于舵机断电。具体操作见本次更新说明。

全套主机回归及Keil全量重编已通过，Keil为0 Error / 0 Warning；只证明软件构建与合成输入行为。用户已反馈补共GND后QR正常，08:33回显日志是历史故障；新四入口仍未烧录/实机对位验证。正式整场闸门仍关闭，不能据此宣称赛道、真实串口或抓放已验收。

主机入口：`tests/run_host_tests.ps1`（本机GCC/Python在 `D:/mingw64/bin/`，换电脑需配置对应路径）。编译后HEX在 `MDK-ARM/jiejie/jiejie.hex`，不随源码提交；更新设备前须核对新构建及固件标识。
