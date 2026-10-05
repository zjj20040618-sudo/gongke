# 反恐电控 STM32F407 共用工程

更新：2026-10-05。电控固件标识 `20261005-ROUTE34-NOQR`（保留QR53接收）；配套视觉审核基准 `5ce9f65`（App 2.1.0、9564 模型、01物体/53三码/63四阶段）。源码、模型、工程依赖与回归测试一并保存在本仓库，不包含个人设置、令牌和编译产物。

先读 [README_本次更新.md](README_本次更新.md) 了解本次改动；视觉队友接着读 [CONTROL_TO_VISION.md](CONTROL_TO_VISION.md)。视觉侧说明保存在 [VISION_TO_CONTROL.md](VISION_TO_CONTROL.md)，其中“MCU尚不支持53/63”是该视觉提交当时的评估，已被本轮电控更新替代。

- 电控工程：[MDK-ARM/jiejie.uvprojx](MDK-ARM/jiejie.uvprojx)。`App/` 为自写控制代码，`Src/`、`Inc/` 为外设/RTOS入口。
- 视觉工程：`03_扫码与物体识别联合App工程/qr_object_app/`，保留队友本轮源码与9564模型，不另改识别算法。
- 结构与调参入口：[PROJECT_GUIDE.md](PROJECT_GUIDE.md)、[MOTION_YAW_TUNING.md](MOTION_YAW_TUNING.md)。
- 协议与共同待办：[VISION_CONTROL_PROTOCOL.md](VISION_CONTROL_PROTOCOL.md)、[VISION_CONTROL_TODO.md](VISION_CONTROL_TODO.md)。
- 电控/机构待测：[CONTROL_TUNING_TODO.md](CONTROL_TUNING_TODO.md)、[ACTUATOR_TEAMMATE_HANDOFF.md](ACTUATOR_TEAMMATE_HANDOFF.md)。
- 协作边界：[COLLABORATION.md](COLLABORATION.md)；拉取源码不等于重新编译、烧录或安装相机App。

模式31只走路线、包含越障路，并在R1后等待合法三任务QR才进入R2；不运行抓球/打靶/救援。新增34共用31整条12步配方与运动参数，但不请求/等待QR，34→g即可独立验证越障与路线，再g/a/0取消。模式32是无机械臂单向任务联调，使用独立旧路线种子，工作点和桶锚距离必须先标定。模式33只接收诊断，不输出行走/激光/步进/夹爪动作；既有上电舵机PWM不等于舵机断电。

全套主机回归及Keil全量重编已通过，Keil为0 Error / 0 Warning；只证明软件构建与合成输入行为。用户08:33日志报告的是前一QR53-RXGATE固件，只收到请求回显，ACK/QR结果尚未接通。新增34须重新编译/烧录，尚无34实机验收。正式整场闸门仍关闭，不能据此宣称赛道、真实串口或抓放验收通过。

主机入口：`tests/run_host_tests.ps1`（本机GCC/Python在 `D:/mingw64/bin/`，换电脑需配置对应路径）。编译后HEX在 `MDK-ARM/jiejie/jiejie.hex`，不随源码提交；更新设备前须核对新构建及固件标识。
