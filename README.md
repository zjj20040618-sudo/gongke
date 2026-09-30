# 反恐电控 STM32F407 工程

2026-09-30 分享快照：本目录只包含反恐排爆救援机器人电控工程、所需库源码与测试资料。

- 打开 Keil 工程：[MDK-ARM/jiejie.uvprojx](MDK-ARM/jiejie.uvprojx)
- 看代码结构、每个文件职责、调参入口：[PROJECT_GUIDE.md](PROJECT_GUIDE.md)
- 2026-10-01 视觉接收改为队友现有二进制，蓝牙保持 ASCII；接口、测试和未完成项见 [VISION_INTEGRATION.md](VISION_INTEGRATION.md)。这是接收侧对接，自动切模式、桶/两种人质映射和夹爪站位仍待联调。
- 自写业务代码在 `App/`；CubeMX 生成的启动、外设与任务入口在 `Src/`、`Inc/` 和根目录 `jiejie.ioc`。

当前工程尚有正式路线、视觉站位、越障和机械行程等待实测的占位值；Keil 构建成功不代表整场可以开跑。调试前先看导览里的“配置状态与安全边界”。

分享快照已在独立导出目录完成 Keil 全量编译：0 Error / 0 Warning；主机回归全部通过。编译生成的 HEX SHA-256 为 `DB040A88C642C19B39564851EAC190CD94D2EADCA83013CC4071C06DABDA9AA8`。编译产物不随源码包分发，请安装 Keil MDK 及 STM32F4 设备包后打开工程并重新编译。

主机回归入口是 `tests/run_host_tests.ps1`，目前使用 `D:/mingw64/bin/gcc.exe`；其他电脑运行前需将脚本里的编译器路径改为本机 GCC 路径。
