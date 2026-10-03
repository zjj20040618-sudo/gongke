# 视觉二进制对接（更新于2026-10-04）

## 当前结论

视觉最新输出及核对记录见 [VISION_TO_CONTROL.md](VISION_TO_CONTROL.md)，电控反馈写入其维护的 `CONTROL_TO_VISION.md`；创建前参考 [VISION_TEAM_HANDOFF.md](VISION_TEAM_HANDOFF.md)。共同验收仍使用原VC编号。

电控 USART2 与视觉使用 `AA 55` 二进制帧；USART3 蓝牙仍收发 ASCII。已审核采用视觉9541十类模型版本及QR选目标逻辑；电控配套接收ID0腰鼓、ID2圆锥、ID9黑桶，其余类别编号和线格式不变。两种人质使用队友训练别名，仍须用赛题实物验证，不等于几何名称通用等价。

这是双端软件实现，不是整机验收。当前独立模式32调用模式确认链，旧正式任务闸门仍关闭，上电自动运动仍禁用；未新增烧录、实体UART或抓放测试。完整控制协议及32实际调用点见 [VISION_CONTROL_PROTOCOL.md](VISION_CONTROL_PROTOCOL.md)。

## 文件与修改入口

| 文件 | 职责 |
| --- | --- |
| `qr_object_app/protocol.py` / `control_session.py`（视觉目录内） | 目标/QR组包、模式命令、ACK和请求编号会话 |
| `qr_object_app/config.py`（视觉目录内） | 9541十类模型、UART配置；启动IDLE，OBJECT画幅读取模型输入 |
| `App/proto.c` / `proto.h` | 帧同步、长度与 CRC 校验、QR 三目标检查、模型类别转换 |
| `App/robot.c` | 启动时选择二进制接收，注册回调；蓝牙 `diag` 输出接收统计 |
| `App/test.c` / `test.h` | 模式33：静止请求QR→OBJECT，限频ASCII回传选定目标及新旧状态 |
| `App/steps.c` | 按当前任务类别和标签筛选，防止同帧其他目标覆盖所需目标 |
| `App/mission_trial.c` | 32单向路程、QR选择、视觉对位与10秒机械占位；不调用旧往返补扫 |
| `tests/test_vision_binary_replay.py` | 真实视觉组包函数 → 真实电控解析器的主机回放 |

当前固件标识：`20261004-VISION-DIAG33`。USART2 电控 TX=PD5、RX=PD6，视觉当前 TX=A19、RX=A18，115200 波特率；连接时 TX 对 RX 并共地，具体板端电压及针脚需双方现场核对。

## 目标/QR内层格式（以实际组包代码为准）

多字节数值为小端。公共结构为 `AA 55 + body + CRC16小端`；CRC 是 CCITT-FALSE，初始 `FFFF`、多项式 `1021`，只覆盖 body。

下面01/51是目标与QR的内层body定义。受电控控制的运行使用62封装请求号及内层body，配合60命令、61确认；不发送嵌套AA55/CRC。旧01/51裸帧仅用于独立兼容回放，不能用于32自动联调，外层格式见控制协议。

- 目标 body：`type:u8=01, seq:u16, count:u8, img_w:u16, img_h:u16, capture_ms:u16, inference_ms:u16, vision_ms:u16`，随后每目标 `class:u8, score:u16, cx:u16, cy:u16, w:u16, h:u16`。上限10目标，旧裸帧长度`18+11*count`；正式62封装整帧`23+11*count`。
- QR body：`type:u8=51, seq:u16, count:u8`，每QR是`UTF8长度:u8 + payload + cx:u16, cy:u16, w:u16, h:u16`。电控仅接受一个QR，payload恰好为三个`1..3` ASCII数字，依次球色、靶色、人质形状；缺项/多项/非法字符不算成功。合法三位码旧裸帧20字节，正式62封装整帧25字节。
- 二进制分片接收，100 ms 的不完整帧字节间隔用于丢弃残帧，不是任务超时。CRC、长度、类别、置信度范围及目标几何字段检查完成后才交业务层。
- 连续重复 seq 不重复消费；允许 seq 从 65535 回到 0。这不是可靠传输/重发协议，也不能识别所有乱序或重启情形。
- score 的 `0..1000` 转为旧业务 `0..100`。本次没有新增质量阈值；视觉当前检测阈值是 0.35。
- 同类多个目标暂选置信度最高者，同分取先出现者；并非视觉跟踪锁定，实体联调需确认是否符合站位需求。

## 模型 ID 与电控语义

| 视觉 ID / 名称 | 电控 CLS / LAB | 状态 |
| --- | --- | --- |
| 0 / oblate | 2 / 5（人质 / 腰鼓） | 按队友训练别名适配，须核对实物 |
| 1 / cylinder | 2 / 3（人质 / 圆柱） | 已接收映射，未实车验收 |
| 2 / truncated_cone | 2 / 4（人质 / 圆锥） | 按队友训练别名适配，须核对实物 |
| 3 / blue_ball | 0 / 2（球 / 蓝） | 已接收映射 |
| 4 / red_ball | 0 / 0（球 / 红） | 已接收映射 |
| 5 / green_ball | 0 / 1（球 / 绿） | 已接收映射 |
| 6 / red_target | 1 / 0（靶 / 红） | 已接收映射 |
| 7 / blue_target | 1 / 2（靶 / 蓝） | 已接收映射 |
| 8 / green_target | 1 / 1（靶 / 绿） | 已接收映射 |
| 9 / black_barrel | 3 / 0（桶 / 固定标签） | 接收已适配；真实识别/桶口工作点待验 |

类别仅允许0～9；10及以上使整包拒收，不会冒充任务目标。QR 第三位仍是 `1圆柱/2圆锥/3腰鼓`，不能改成模型 ID。

## 还需要双方补齐的四件事

进度、责任人、后续问题与验收记录统一更新 [VISION_CONTROL_TODO.md](VISION_CONTROL_TODO.md)，本节只是接口摘要，不作为第二份状态表。

1. **模式控制实机验证**：IDLE/QR/OBJECT命令、实际模式ACK、请求关联及确认后的新取图结果已有软件实现；32调用已接入。视觉须部署完整当前App，双方核对真实收发和切换时序。USER仅用于UART接管前的独立试验，不能靠人工切键宣称自动链已验收。
2. **桶和两种人质的实物证据**：软件ID映射已补；用真实赛题圆锥/腰鼓/桶核对识别与框选位置，不把队友标签或主机回放当识别准确率。
3. **夹爪工作点与站距**：以模式33收到的实际img_w/img_h为准，不沿用旧480×320假设。旧正式 `steps.c::s_stand[4]` 的 `cx=160` 只是种子；32使用BOOT阶段RAM `vsg1/vsg2` 和 `bcx/tcx/hcx/kcx`，当前命令上限仍479，若模型画幅或所需工作点不兼容，先配套修改并审核，不能硬填。方向及工作点须实测；32不做未标定的纵深粗调，相机中心不等于夹爪点。
4. **实体链路**：上车前核对串口接线、电平、波特率，用真实 QR 和各类目标逐一验证 `diag` 统计、选定目标、断帧和目标丢失。必须再验停稳后机械动作、急停和完整路线，随后才考虑开放 `CAL_VISION_READY`。

手机 `diag` 增加 ASCII 行 `VISION wire=BINARY crc_bad=... bad=... gap=... unmapped=... duplicate=...`。`accepted` 表示解析通过，不等于抓取到位；`obj` 统计目标包数，不是单个目标数量。

## 可重复的软件验证

在仓库根目录运行：

```powershell
python -B tests/test_vision_binary_replay.py
& ./tests/run_host_tests.ps1
```

第一项使用实际视觉组包代码与实际 `App/proto.c`，覆盖27个合法QR、十类映射、桶与三任务混包、空包、实际画幅边界、重复/坏帧、断包恢复、回绕、命令/确认与请求关联，共21组。完整回归另含2组视觉主循环、3组诊断IRQ/停止契约及18次C测试；32覆盖68次停止注入，33覆盖17项运行中写命令拦截和四阶段×三停键。均为合成主机数据，不证明设备串口时序、识别率或机械可靠性。

默认 GCC 为 `D:/mingw64/bin/gcc.exe`；二进制回放可用环境变量 `EOD_HOST_CC` 指向本机编译器，原 PowerShell 回归修改脚本里的 `$compiler`。Keil 使用 `MDK-ARM/jiejie.uvprojx`；构建结果须看本次日志，不借用 README 的 09-30 快照证明新代码已编译。

最新交接包以包内VERSION.json、SHA256SUMS.txt和本次Keil日志为准，操作见 [VISION_TEAM_HANDOFF.md](VISION_TEAM_HANDOFF.md)。下述旧候选编译记录不作为新版33的证明；没有自动烧录，旧正式闸门仍关闭。

历史：2026-10-01接收侧构建日志 `MDK-ARM/rebuild_vision_binary_2026-10-01.txt` 为0/0，HEX SHA-256 `FD720654CA5E65AA38E9D68AD75C403CDE9816B9B052EF5D398BA4942C0D1A87`；它不是当前32候选。

## 两个人怎么协作

开工前查看本地改动与远端新提交，电控侧按 [COLLABORATION.md](COLLABORATION.md) 审核差异后再快进拉取；电控主要改 `App/`，视觉主要改自己的 App 目录。每次只提交自己实际修改的文件，写清功能，再 push；另一人审核采用后才成为配套本地版本。若有本地修改或分叉，先看差异协调，不强推、不丢弃对方改动。涉及 `protocol.py/config.py` 的变化先同步这份映射与回放测试。现有 G0 表格是待确认材料，不是已经双方签字通过的联调记录。
