# 反恐电控 STM32F407 工程

2026-10-04 工作版本：本仓库包含反恐排爆救援机器人的电控工程、所需库源码、视觉 App 与联调资料。当前固件号为 `20261004-VISION-DIAG33`，不是设备已烧录证明。

- 每次上传改了什么、验证了什么、队友需要做什么：先读 [README_本次更新.md](README_本次更新.md)
- 打开 Keil 工程：[MDK-ARM/jiejie.uvprojx](MDK-ARM/jiejie.uvprojx)
- 看代码结构、每个文件职责、调参入口：[PROJECT_GUIDE.md](PROJECT_GUIDE.md)
- 两人拉取/提交/推送、提交编号与标签怎么辨认：[COLLABORATION.md](COLLABORATION.md)
- 视觉—电控共同待办、责任分工和队友回复格式：[VISION_CONTROL_TODO.md](VISION_CONTROL_TODO.md)
- 用户休息时队友接手电控测试、数值填写和记录：[CONTROL_TUNING_TODO.md](CONTROL_TUNING_TODO.md)
- 视觉采用二进制，蓝牙保持 ASCII；自动切模式的双端实现见 [VISION_CONTROL_PROTOCOL.md](VISION_CONTROL_PROTOCOL.md)，类别映射与未完成项见 [VISION_INTEGRATION.md](VISION_INTEGRATION.md)。桶/两种人质、真实 UART 和工作点仍待联调。
- 自写业务代码在 `App/`；CubeMX 生成的启动、外设与任务入口在 `Src/`、`Inc/` 和根目录 `jiejie.ioc`。

模式31只走已登记的路线；32在R1后等合法QR，单向联调球/桶、靶、人质，抓放以10秒停车占位，不调用往返补扫；33只静止核对扫码和目标回传，操作见 [VISION_TEAM_HANDOFF.md](VISION_TEAM_HANDOFF.md)。当前正式任务闸门仍关闭，视觉工作点、越障效果和机械行程尚待实测；Keil 构建成功不代表整场可以开跑。

最近一次功能代码90192ea的独立快照已通过主机回归及Keil全量编译：0 Error / 0 Warning；版本、HEX校验值及未验证范围见 [README_本次更新.md](README_本次更新.md)。候选002是历史记录，不是新版33。编译产物不随源码提交；需要更新电控程序时重新编译或使用经核对的交接HEX，拉取代码不会自动更新单片机。纯文档上传无需因此重编/烧录。

历史：2026-09-30 分享快照的 HEX SHA-256 为 `DB040A88C642C19B39564851EAC190CD94D2EADCA83013CC4071C06DABDA9AA8`，不适用于当前模式32。

主机回归入口是 `tests/run_host_tests.ps1`，目前使用 `D:/mingw64/bin/gcc.exe`；其他电脑运行前需将脚本里的编译器路径改为本机 GCC 路径。
