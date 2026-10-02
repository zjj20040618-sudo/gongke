# 视觉二进制结果格式与接收映射（原始接入2026-10-01）

## 当前结论

2026-10-02 最新双端控制、ACK与请求绑定格式见 [VISION_CONTROL_PROTOCOL.md](VISION_CONTROL_PROTOCOL.md)，源码固件号 `20261002-VISION-CONTROL`。视觉目录布局、模型及识别算法保留；本轮修改其主循环/模式/协议/UART控制接口。电控USART2收发二进制，USART3蓝牙仍ASCII。下面01/51为旧结果body说明，正式控制结果须用62外壳，不裸发旧包代替新接口。

最初接入只是接收侧，本轮已补双向软件；仍不是整机验收。正式标定闸门关闭、上电自动运动禁用，未烧录或做实体UART/抓放测试。本轮构建/回归证据见最新双向协议说明，下面09-30/10-01构建为历史记录。

## 文件与修改入口

| 文件 | 职责 |
| --- | --- |
| `qr_object_app/protocol.py`（视觉目录内） | 现有目标帧、QR 帧组包；本次测试直接调用它 |
| `qr_object_app/config.py`（视觉目录内） | 模型类别、画幅、UART 配置；保持原样 |
| `App/proto.c` / `proto.h` | 帧同步、长度与 CRC 校验、QR 三目标检查、模型类别转换 |
| `App/robot.c` | 启动时选择二进制接收，注册回调；蓝牙 `diag` 输出接收统计 |
| `App/steps.c` | 按当前任务类别和标签筛选，防止同帧其他目标覆盖所需目标 |
| `tests/test_vision_binary_replay.py` | 真实视觉组包函数 → 真实电控解析器的主机回放 |

最初接收版固件标识：`20261001-VISION-BINARY-RX`（历史）。USART2 电控 TX=PD5、RX=PD6，视觉当前 TX=A19、RX=A18，115200 波特率；连接时 TX 对 RX 并共地，具体板端电压及针脚需双方现场核对。

## 帧格式（以队友现有组包代码为准）

多字节数值为小端。公共结构为 `AA 55 + body + CRC16小端`；CRC 是 CCITT-FALSE，初始 `FFFF`、多项式 `1021`，只覆盖 body。

- 目标 body：`type:u8=01, seq:u16, count:u8, img_w:u16, img_h:u16, capture_ms:u16, inference_ms:u16, vision_ms:u16`，随后每目标 `class:u8, score:u16, cx:u16, cy:u16, w:u16, h:u16`。当前上限 10 目标，整帧长度 `18+11*count`。
- QR body：`type:u8=51, seq:u16, count:u8`，每 QR 是 `UTF8长度:u8 + payload + cx:u16, cy:u16, w:u16, h:u16`。电控仅接受一个 QR，payload 必须恰好为三个 `1..3` 的 ASCII 数字，依次是球色、靶色、人质形状；缺项、多项或非法字符不会标记 QR 成功。合法三位码整帧 20 字节。
- 二进制分片接收，100 ms 的不完整帧字节间隔用于丢弃残帧，不是任务超时。CRC、长度、类别、置信度范围及目标几何字段检查完成后才交业务层。
- 连续重复 seq 不重复消费；允许 seq 从 65535 回到 0。这不是可靠传输/重发协议，也不能识别所有乱序或重启情形。
- score 的 `0..1000` 转为旧业务 `0..100`。本次没有新增质量阈值；视觉当前检测阈值是 0.35。
- 同类多个目标暂选置信度最高者，同分取先出现者；并非视觉跟踪锁定，实体联调需确认是否符合站位需求。

## 模型 ID 与电控语义

| 视觉 ID / 名称 | 电控 CLS / LAB | 状态 |
| --- | --- | --- |
| 0 / oblate | 不映射 | 不能凭名字认作腰鼓，待确认 |
| 1 / cylinder | 2 / 3（人质 / 圆柱） | 已接收映射，未实车验收 |
| 2 / truncated_cone | 不映射 | 圆台是否等于比赛指定圆锥，待确认 |
| 3 / blue_ball | 0 / 2（球 / 蓝） | 已接收映射 |
| 4 / red_ball | 0 / 0（球 / 红） | 已接收映射 |
| 5 / green_ball | 0 / 1（球 / 绿） | 已接收映射 |
| 6 / red_target | 1 / 0（靶 / 红） | 已接收映射 |
| 7 / blue_target | 1 / 2（靶 / 蓝） | 已接收映射 |
| 8 / green_target | 1 / 1（靶 / 绿） | 已接收映射 |
| 桶 | 需要 CLS=3，当前模型无类别 | 阻塞排爆放桶视觉对准 |

未知映射计入 `unmapped`，不会冒充可用任务目标。QR 第三位仍是 `1圆柱/2圆锥/3腰鼓`，不能改成模型 ID。

## 还需要双方补齐的四件事

进度、责任人、后续问题与验收记录统一更新 [VISION_CONTROL_TODO.md](VISION_CONTROL_TODO.md)，本节只是接口摘要，不作为第二份状态表。

1. **自动切识别模式实机验收**：本轮两端已实现IDLE/QR/OBJECT请求、ACK及请求绑定首帧；等待确认期间不运动，匹配合法目标后才执行任务。人工USER不能代替自动验收；重启后不能续用请求号，须停机清链路/重启两端。格式及调用点见最新双向协议文档。
2. **桶和两种人质**：确认桶怎么识别，以及 oblate/truncated_cone 对应实物是否正确；不擅自映射。
3. **夹爪工作点与站距**：目标帧直接使用实际画幅坐标，当前 OBJECT 为 480×320，不缩成旧 320 宽。`steps.c::s_stand[4]` 中的旧 `cx=160` 只是种子，站距/目标像素高度仍待测。相机中心不等于夹爪对准点；球、靶、人质、桶分别在真实正确站位量 `cx_stand_px/h_stand_px/d_stand_mm`，不要猜“偏四分之一”。
4. **实体链路**：上车前核对串口接线、电平、波特率，用真实 QR 和各类目标逐一验证 `diag` 统计、选定目标、断帧和目标丢失。必须再验停稳后机械动作、急停和完整路线，随后才考虑开放 `CAL_VISION_READY`。

手机 `diag` 增加 ASCII 行 `VISION wire=BINARY crc_bad=... bad=... gap=... unmapped=... duplicate=...`。`accepted` 表示解析通过，不等于抓取到位；`obj` 统计目标包数，不是单个目标数量。

## 可重复的软件验证

在仓库根目录运行：

```powershell
python -B tests/test_vision_binary_replay.py
& ./tests/run_host_tests.ps1
```

第一项使用实际视觉组包代码与实际 `App/proto.c`，原有11组加本轮7组控制/ACK/绑定/相机控制逻辑检查，共18组。另运行 `python -B tests/test_vision_control_main.py` 的2组真实相机主循环模拟设备检查。PowerShell项检查实际 `steps.c` 清槽、握手等待/中止及原有电控回归；均为合成主机证据，不证明设备串口时序、识别率或机械可靠性。

默认 GCC 为 `D:/mingw64/bin/gcc.exe`；二进制回放可用环境变量 `EOD_HOST_CC` 指向本机编译器，原 PowerShell 回归修改脚本里的 `$compiler`。Keil 使用 `MDK-ARM/jiejie.uvprojx`；构建结果须看本次日志，不借用 README 的 09-30 快照证明新代码已编译。

本次已在桌面快捷方式对应的 `D:/zjj/UV4/UV4.exe` 全量重编（ARMCLANG V6.24），日志 `MDK-ARM/rebuild_vision_binary_2026-10-01.txt` 为 **0 Error / 0 Warning**。HEX SHA-256 为 `FD720654CA5E65AA38E9D68AD75C403CDE9816B9B052EF5D398BA4942C0D1A87`；没有烧录。构建输出/日志被 Git 忽略，不随源码上传。解析接收已链接；正式任务仍受配置闸门影响，不能拿这个 HEX 当作已开放整场的版本。

## 两个人怎么协作

开工前查看本地改动与远端新提交，电控侧按 [COLLABORATION.md](COLLABORATION.md) 审核差异后再快进拉取；电控主要改 `App/`，视觉主要改自己的 App 目录。每次只提交自己实际修改的文件，写清功能，再 push；另一人审核采用后才成为配套本地版本。若有本地修改或分叉，先看差异协调，不强推、不丢弃对方改动。涉及 `protocol.py/config.py` 的变化先同步这份映射与回放测试。现有 G0 表格是待确认材料，不是已经双方签字通过的联调记录。
