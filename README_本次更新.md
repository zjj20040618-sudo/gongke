# 本次上传修改说明

更新：2026-10-05。视觉App **2.1.0**；基于main的32d0522，仅修改视觉、构建、测试和协议文档。**App/ 电控代码未动；未部署相机、未烧录MCU、未运行电机或验收实机。**

## 改了什么

- 接收新 `0x63`：task1球→task4黑桶→task2靶→task3人质，每帧只回当前指定类别最高分一个。桶只接受task4/digit0，持续发当前帧ID9；目标丢失发真实画幅空包，不缓存旧坐标。通用60/OBJECT保留33诊断的所选三任务加桶。
- 按用户选择恢复物体 `0x01` 的ID、置信度、中心XY、框宽高、画幅和耗时，靶也保留旧字段；电控业务可仅取靶X。前版02停止发送。
- 按用户最后要求，QR改 **`0x53` 三位ASCII任务码，无二维码完整框**，没有位置、尺寸或字符串长度字段；未扫码count0，有码count1且恰好三码。前版52状态和旧51带框格式均不发送。
- 成功新QR请求清码；同号重试不清。二维码消失后仍发本轮锁存码至换模式，避免一次结果丢失。三码重发不表示相机又看到二维码。
- 相同请求完整内容重试原ACK；同号改内容失败并暂停业务，较小旧号不能退回旧任务；失败ACK报告实际模式。
- UART保留未写尾部、只续suffix，ACK全写后才新取图/发送业务。RX/ACK打印HEX与计数，业务限频，完整write明确不等于MCU收到。
- 修复短写组合问题：旧尾包与新结果不再共用seq；每个新帧构造时分配序号，防止新空帧被MCU当重复丢掉。
- 采用补充基线5766cb8的9564 MUD/CVI及字体5；仅提取这些视觉项，未合入该分支无关改动。模型CVI SHA256：`4F7AF93A509145F3786BC8143A09E2C108E741DF19AD07C9A1861198A37E833D`。
- 构建脚本从app.yaml动态读取版本/清单、从config读取模型并核对MUD引用，修复旧1.1.0/9541硬编码。同步方向文件、协议/集成/App说明及原VC待办。

## 本轮验证：111项通过

| 入口 | 数量 | 证据范围 |
| --- | --- | --- |
| App目录unittest discover | 80 | 模式/二维码/27组合、任务63/会话、UART短写、构建ZIP、真实C解析兼容/拒绝及短写主循环组合 |
| 根test_vision_control_main.py | 7 | 真实主循环ACK门、四阶段、丢桶空包、QRCode重发/重试/重置、失败与旧号 |
| 根test_vision_binary_replay.py | 21 | 当前物体writer与历史QR51 fixture→真实C解析器；不是当前QR53兼容证明 |
| 根test_vision_diag_contract.py | 3 | 电控诊断回调无动作/无IRQ发送、停止优先等已有契约 |

另有protocol头/CRC烟测、CVI与5766cb8的Git blob一致、MUD字段一致（仅去末尾多余空行）、文档链接、无冲突标记及git diff --check检查。真实C回放使用现有GCC4.9.2与 `-fuse-ld=bfd`，未安装工具。

重点组合回归：实际main+实际UartLink短写旧目标seq0、新空帧seq1，真实App/proto.c接受两帧、duplicate=0；多轮短写时旧请求尾包不成为新请求结果，新ACK完整之后才采新任务，最新坐标被接收。所有这些均为电脑端合成数据，**不是实体UART或识别率证明**。

复现（仓库根；测试需完整仓库和GCC，不仅App源码ZIP）：

```powershell
$env:PYTHONIOENCODING = 'utf-8'
$env:EOD_HOST_CC = 'E:\setup\devc++\Dev-Cpp\MinGW64\bin\gcc.exe'
& 'E:\setup\anaconda\envs\yolo_train\python.exe' -B -m unittest discover -s '03_扫码与物体识别联合App工程/qr_object_app/tests' -q
& 'E:\setup\anaconda\envs\yolo_train\python.exe' -B tests/test_vision_control_main.py
& 'E:\setup\anaconda\envs\yolo_train\python.exe' -B tests/test_vision_binary_replay.py
& 'E:\setup\anaconda\envs\yolo_train\python.exe' -B tests/test_vision_diag_contract.py
& 'E:\setup\anaconda\envs\yolo_train\python.exe' -B '03_扫码与物体识别联合App工程/build_packages.py'
```

## 包与设备

本地生成：

- `03_扫码与物体识别联合App工程/dist/maix-qr_object_switch-v2.1.0.zip`
- `03_扫码与物体识别联合App工程/dist/qr_object_switch_source_v2.1.0.zip`
- 同目录 `SHA256SUMS.txt` 为本次构建校验值；安装包16文件、源码包26文件，逐文件与源内容比对。

dist按既有规则忽略，不提交二次产物；云端上传源码与模型，队友可按脚本重建。旧包保留，但不能凭文件名把旧1.1.0、2.0.0当本版。

## 电控队友下一步（VC-12）

1. 先读 [VISION_TO_CONTROL.md](VISION_TO_CONTROL.md) 第3～5节。当前仓库 `App/proto.c` 能接恢复的01，但**不接53、仅发60**。本轮真实回放确认短QR裸帧/62包裹均拒绝。用户文件提到的队友本地TASKSELECT63版本未在此源码确认，不能当已配套。
2. 电控实现纯三码53接收、63发送与task1→4→2→3调用；桶阶段必须换请求号。保留通用60诊断及61/62会话校验，审核并编译/烧录。
3. 电控创建维护 `CONTROL_TO_VISION.md`，记录提交、固件/烧录状态、回放与实测日志。先无运动静止确认三码、单类坐标、ACK与空帧，再安排对位/机构。

源码已上传、App已部署、MCU已烧录、实机已验收必须分别记录；本轮只完成源码、软件验证和本地打包，不开正式运动闸门。旧交接包/Keil日志仅历史，不能冒充本轮验收。
