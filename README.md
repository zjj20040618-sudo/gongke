# 反恐电控 STM32F407 共用工程

更新：2026-10-06。本次发布电控 `20261006-XY-STEP3-X20-Y30`，上传成功以GitHub远端提交为准，未烧录；独立38～41采用X20/Y30短步→刹停→新图复看，XY移动w=0，Y动后纠回本任务初始航向再联合复核。39免QR不变，41的180°仍保留原22。远端main`066d346`已含审核过的视觉`5d5e812`（App2.1.16），本地合并态已同步；电控STEP3保持本机提交0e8cc85，不主动改视觉算法/模型，7a2ab10仅为上轮历史基准。原01/53/60/63/61/62格式和任务映射兼容；新增人质54站位尚未接入电控，不能宣称全功能对齐。源码不包含个人设置、令牌和编译产物。

先读 [README_本次更新.md](README_本次更新.md) 了解本次改动；视觉队友接着读 [CONTROL_TO_VISION.md](CONTROL_TO_VISION.md)。视觉侧说明保存在 [VISION_TO_CONTROL.md](VISION_TO_CONTROL.md)；双端已有53/63软件配套，源码版本不代表设备已部署。

- 电控工程：[MDK-ARM/jiejie.uvprojx](MDK-ARM/jiejie.uvprojx)。`App/` 为自写控制代码，`Src/`、`Inc/` 为外设/RTOS入口。
- 视觉工程：`08_MaixCAM2扫码与物体识别App工程/maixcam2_qr_object_app/`，当前唯一视觉工程为MaixCAM2，使用9767 MUD及NPU/VNPU模型；旧03 MaixCAM工程已删除。
- 结构与调参入口：[PROJECT_GUIDE.md](PROJECT_GUIDE.md)、[MOTION_YAW_TUNING.md](MOTION_YAW_TUNING.md)。
- 协议与共同待办：[VISION_CONTROL_PROTOCOL.md](VISION_CONTROL_PROTOCOL.md)、[VISION_CONTROL_TODO.md](VISION_CONTROL_TODO.md)。
- 电控/机构待测：[CONTROL_TUNING_TODO.md](CONTROL_TUNING_TODO.md)、[ACTUATOR_TEAMMATE_HANDOFF.md](ACTUATOR_TEAMMATE_HANDOFF.md)。
- 协作边界：[COLLABORATION.md](COLLABORATION.md)；拉取源码不等于重新编译、烧录或安装相机App。

独立38球、39桶、40人质、41球→180°→桶的XY对齐，不抓取、不亮激光。只有39免扫码，39→g直接新请求桶；38/40/41仍扫码。临时工作点及真实画幅须先核对，不能因源码有数值当作已标定。31为独立12节点道路试跑，R1后须合法QR才进R2，去旧桶/手发d节点；新任务链尚未接入。34保留旧15节点桶对位/等新d配方，不再与31同表；36无QR/桶十节点、37仅越障三段。32仍独立旧路线/无臂任务联调，33只收诊断；已有上电舵机PWM不等于舵机断电。具体操作见本次更新说明。

本轮短步候选完整主机回归exit0（含普通/fast-math引擎及真实蓝牙专项），Keil全量重编0 Error / 0 Warning，当前HEX校验见本次更新说明；主机仍有既有motion.c未用th警告。07:47旧NO-YAWFIX人质包确认的640×480仅属于该轮，不证明新视觉部署画幅；新OBJECT实际由模型输入决定，若高度320，38/39/41的球420/桶400工作点会IMAGE_GEOMETRY停车，须用新实际01重新核对，不能自动缩放。新增54接收/锁存/三条返回路线待实现，不将QR形状当站位或默认站位1。正式整场闸门仍关闭，未烧录或实机验收。

主机入口：`tests/run_host_tests.ps1`（本机GCC/Python在 `D:/mingw64/bin/`，换电脑需配置对应路径）。编译后HEX在 `MDK-ARM/jiejie/jiejie.hex`，不随源码提交；更新设备前须核对新构建及固件标识。
