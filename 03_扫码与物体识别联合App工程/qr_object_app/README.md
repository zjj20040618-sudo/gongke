# 扫码与物体识别联合 App

## 使用

1. 在 MaixVision 打开本文件夹，连接 MaixCAM Pro，运行 `main.py`。
2. 默认二维码模式，左上角显示 `QR SCAN 1600x900`。
3. 短按机器顶部左侧 **USER** 键，切到物体模式，显示 `OBJECT YOLO26 480x320`。
4. 再短按 USER 返回扫码。顶部右侧 **RESET** 只重启板子，不切模式。

| 模式 | 实际采集像素 | 功能 |
|---|---:|---|
| QR | 1600×900 | 扫描中心区域二维码，UART发送0x51帧 |
| OBJECT | 480×320 | YOLO26识别9类，UART发送0x01帧 |

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
- `hardware.py`：UART初始化。
- `ui.py`：画框、FPS、耗时。

按键回调只设置标志，真正切换在主循环完成，防止按键线程与摄像头抢资源。`cap`是取图耗时；`work`在扫码模式是解码耗时、物体模式是推理耗时；`uart`是写入串口缓冲区耗时，不是单片机应答往返耗时；`loop`是整圈耗时。

