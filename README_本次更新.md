# 本次本地更新说明

更新：2026-10-05。修改前基线 `a81f4c5`，视觉App **2.1.3**。已将相机日志改为MC本机自动记录，不再要求用户复制终端。用户要求先本地测试，暂不上传源码或日志。

## 改动与不变项

- 新增 `qr_object_app/uart_log.py`：启动UART即创建持久目录 `/root/vision_uart_logs` 下唯一会话文件。后台写入RX全部字节、WRITE驱动报告的片段、TX整帧意图/完成状态，文件不按帧采样。
- 256条队列、0.5秒flush、1MiB分片、16MiB每轮、64MiB目录；明确标记记录缺口。限额/磁盘错误停止日志但不停止通信，不删除旧文件。强杀/突然断电可能损失尾部。
- `hardware.py`只增加诊断记录，UART短写仍只续尾部；`main.py`正常退出关闭串口并收尾日志；`app.yaml`纳入新模块并升版本。识别算法、ACK门控、业务包字节、模型9564、任务顺序不变；没有改电控App/Src/Inc。
- 电脑日志工具保留为下载后分析器，支持MC日志中的WRITE碎片与LOG GAP；零字节write不会错误截断已知拼帧。
- 同步本说明、App README及VISION_TO_CONTROL.md；CONTROL_TO_VISION.md作为输入未改。用户config.py字号2的未提交差异保留、不暂存；本机安装包采用当前配置。

## 本轮验证

**131项通过**：相机App103项（含7项本机日志新增测试）、真实主循环7项、真实Python53组包与电控C解析器回放5项、电脑分析器16项。日志测试使用临时目录与合成帧，不是实机记录。

验证覆盖完整/短写/零写保持原始字节、所有业务帧记录、磁盘错误不阻断通信、队列溢出、配额不删除旧日志、分片和独立会话、GUI导入保存及CRC边界。

运行入口：

```powershell
# 在视觉工程目录，GCC按已有主机测试配置加入PATH
python -B -m unittest discover -s qr_object_app/tests
# 在仓库根
python -B -m unittest discover -s tests -p test_vision_control_main.py
python -B -m unittest discover -s tests -p test_vision_qr53_replay.py
# 在串口日志工具目录
python -B -m unittest discover -s tests
```

本机已生成并逐文件/ZIP校验 `dist/maix-qr_object_switch-v2.1.3.zip`（17个App成员，含uart_log.py；日志/电脑工具不入包）。
SHA-256：`0fbe4d12b0ebf568aef8e3ed1c160b1ae69cc99c0ce554c5628961065644fe39`。

## 未验证与下一步

设备 **未部署、未启动日志、未进行实机测试**。用户提供设备IP且授权MaixVision，但本轮没有可调用的Windows电脑操控接口；SSH公钥认证失败，未确认密码，因此未继续尝试默认凭据。不能把已打包说成已部署，也不能把MaixVision连接视为SSH登录已确认。

1. 通过MaixVision运行整个更新后的qr_object_app，或安装2.1.3完整包。不能只传main.py。
2. DEVICE出现 `[UART FILE] saving /root/vision_uart_logs/...`，再做33静止诊断，保持运动/激光安全隔离。
3. 正常停止App并等待收尾；从设备文件管理器下载本轮所有同前缀分片，或配置合法SSH后由Codex读取。手机仍导出同轮BLE状态。
4. Codex核对两端真实证据、审查隐私后再上传指定云端，更新方向交接文件。不自动上传原始个人日志。

操作细节：[App README的MC自动日志节](03_扫码与物体识别联合App工程/qr_object_app/README.md)。本机write全长不证明MCU收到/接受；相机日志不能代替电控的VW/VR/ACK接收状态。
