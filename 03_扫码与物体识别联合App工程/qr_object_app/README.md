# 扫码与物体识别联合 App

## 使用

1. 在 MaixVision 打开本文件夹，连接 MaixCAM Pro，运行 `main.py`。
2. 默认IDLE待机，等待电控UART请求。电控控制QR/OBJECT开始、停止及切换；先回ACK，再持续发携带本轮请求号的结果。正式联调无需USER键。
3. 首次UART接管前，USER可独立切QR/OBJECT，输出旧裸帧供单项调试；接管后忽略USER，不能手动覆盖电控模式。
4. 两端须使用本轮配套代码。重启/断链后先中止流程、清链路并重启两端，不自动续赛。完整字节格式、调用点及队友待办见根目录 [VISION_CONTROL_PROTOCOL.md](../../VISION_CONTROL_PROTOCOL.md)；模型/算法未改，桶未补。

| 模式 | 实际采集像素 | 功能 |
|---|---:|---|
| IDLE | 不取新图 | 不推理、不发业务结果，仍轮询UART |
| QR | 1600×900 | 解码新图，自动控制时发送62外壳内的51结果；空包不算扫码成功 |
| OBJECT | 480×320 | YOLO26识别9类，自动控制时发送62外壳内的01结果 |

类别ID固定：0扁圆、1圆柱、2圆台、3蓝球、4红球、5绿球、6红靶、7蓝靶、8绿靶。

## 串口

MaixCAM Pro `A19/UART1_TX -> STM32 RX`，`A18/UART1_RX <- STM32 TX`，两板GND相连，115200波特。

## 文件对应关系

- `main.py`：总流程，相当于main.c。
- `config.py`：全部可调参数，相当于config.h。
- `user_button.py`：USER键消抖和事件标志。
- `mode_controller.py`：模式状态机和分辨率切换。
- `qr_reader.py`：二维码解码。
- `object_detector.py`：9302 YOLO26九类模型。
- `protocol.py`：STM32数据包和CRC。
- `control_session.py`：请求/实际模式ACK、重复命令、结果请求绑定；在主循环调用。
- `hardware.py`：UART初始化。
- `ui.py`：画框、FPS、耗时。

按键回调只设置标志，真正切换在主循环完成，防止按键线程与摄像头抢资源。`cap`是取图耗时；`work`在扫码模式是解码耗时、物体模式是推理耗时；`uart`是写入串口缓冲区耗时，不是单片机应答往返耗时；`loop`是整圈耗时。

