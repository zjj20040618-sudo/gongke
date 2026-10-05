# 视觉二进制对接

更新：2026-10-05，视觉App2.1.2更新物体标签对比度/摆放，协议沿用2.1.0。**01物体完整字段、53三码无二维码框、63四阶段均不变。** 最新字节定义、模型SHA与反馈事项见 [VISION_TO_CONTROL.md](VISION_TO_CONTROL.md)。旧02/52停止发送，旧51带框QR只作为接收器历史回归fixture。

## 文件与维护边界

| 文件 | 职责 |
| --- | --- |
| `qr_object_app/protocol.py` | 60/63请求分片、61ACK、62包装、01物体/53QR组包 |
| `control_session.py` / `task_selection.py` | 请求去重/旧号、ACK门、任务映射与当前帧最高分筛选 |
| `main.py` / `hardware.py` | 新ACK后采集、QRCode锁存、UART尾包续写与原始日志 |
| `config.py` / `app.yaml` / 上一级`build_packages.py` | 9564十类、串口/字体参数、2.1.2版本及安装清单 |
| `App/proto.c` / `proto.h` | 当前MCU只解析01/51，仅发60；本轮不修改 |
| `App/mission_trial.c` / `steps.c` / `test.c` | 电控流程/筛选/静止诊断；本轮不修改 |
| `qr_object_app/tests/test_mcu_protocol.py` | 当前真实视觉组包→真实App/proto.c兼容/拒绝回放 |
| `tests/test_vision_control_main.py` | 真实主循环的ACK/采集/任务/空帧顺序 |

## 模型ID与电控业务编号

| 模型ID / 本项目语义 | MCU CLS / LAB |
| --- | --- |
| 0扁圆/腰鼓 | 2 / 5 |
| 1圆柱 | 2 / 3 |
| 2圆台/圆锥 | 2 / 4 |
| 3蓝球、4红球、5绿球 | 0 / 2、0 / 0、0 / 1 |
| 6红靶、7蓝靶、8绿靶 | 1 / 0、1 / 2、1 / 1 |
| 9黑桶 | 3 / 0（label占位） |

模型ID、QR数字与任务编号是不同编号，不能直接等同。训练别名腰鼓/圆锥须赛题实物验证，不推广为几何名称等价。

任务顺序球1→桶4→靶2→人质3。63仅返回指定类别一个；无目标为空01帧，画幅仍真实。通用60/OBJECT保留所选三任务加本帧桶，用于诊断。QR53有三码后逐帧锁存重发，目标坐标不锁存。

## 当前已验证与待电控处理

- 真实C解析器支持恢复的01十类物体/空帧；不支持53三码。无论裸帧或62包装，新QR均被拒；测试的预期拒绝不是“新协议联调通过”。
- 当前MCU只发60，球→桶不换63任务，无法让视觉仅输出当前阶段类别；电控须新增请求发送与四阶段调用、短QR接收。保持60用于诊断，并按相同请求/ACK/结果关联规则审核。
- UART本机全write不证明MCU采用；采集实际RX/TX原始HEX、ACK编号和电控diag/业务日志交叉检查。
- 未新增Keil编译、相机部署、MCU烧录、实体UART或运动验收；禁止借用旧日志/旧包证明本版。

复现命令与本轮测试数量见 [README_本次更新.md](README_本次更新.md)。历史 `tests/test_vision_binary_replay.py` 的QR51 fixture只验证旧MCU接收能力，不代表新writer发51。

后续维护 [VISION_CONTROL_TODO.md](VISION_CONTROL_TODO.md) 原VC编号。电控创建自己的 `CONTROL_TO_VISION.md`，写源码/固件/烧录和实测反馈。涉及路线、PID、框架或运动闸门先按 [COLLABORATION.md](COLLABORATION.md) 审核；本轮不扩大范围。
