# MC2 App入口

2026-10-09已按队友MCU f976afb统一编号格式：球task1和人质task3都用62/54，各自只允许模型ID3/4/5和0/1/2。编号1～3本轮锁定，坐标仍只发指定目标当前新图，漏检发空包。MC2 UART2保持/dev/ttyS2、115200、B0 TX/B1 RX。完整规则与155项回归结果见上一级README.md“MC2球/人质编号统一为62/54”。

用户已授权发布完整08源码至共用gongke/main，不打包、部署或烧录；用户app.yaml未提交改动不纳入，首次发布清单沿用已提交原版本。旧dist安装包不含新球编号，运行当前整个源码文件夹。MCU须含f976afb兼容接收逻辑，电脑回放已验证，不等于实体串口或返回动作已验收。

在MaixVision中打开本文件夹，运行main.py。完整操作步骤、参数来源、串口接线、验证状态和清理记录见上一级README.md。

config.py集中调参数；mode_controller.py切相机模式；object_detector.py加载模型；
qr_reader.py扫码；task_selection.py筛选目标；protocol.py组包；
hardware.py收发；uart_log.py保存断电清除的日志；ui.py和touch_inspector.py负责画面与触摸。

不要只上传main.py，也不要漏掉两份axmodel。
