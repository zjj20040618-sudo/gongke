# MaixCAM2 扫码与物体识别测试工程

独立测试工程，沿用04电控仓库的扫码、任务筛选、按键、触屏和二进制协议，适配MaixCAM2。原MC工程及电控代码没有修改。本工程未配置远程仓库。

## 2026-10-09：人工扫码后恢复识别任务
修复现场截图中request=6、TASK:3/DIGIT:3、QR已锁存123却停在“MCU PAUSED: NEW REQUEST REQUIRED”的情况。MCU接管后，USER切到不同模式只暂停该模式不匹配期间的结果；手动返回原请求模式即可恢复。原请求为OBJECT时，在人工QR预览中确认合法三码，下一轮自动返回原OBJECT任务，不必等更大的请求号。扫码不改写原63的任务/目标，不将53包绑定到OBJECT请求。

保留原请求已完成的61 ACK及球/人质首次出现历史；人工预览不新增编号、不发送业务包，恢复后重新暖机并只发新图。未完成ACK、非法/空二维码、模式加载失败及IDLE仍不会放行业务；新请求仍隔离旧任务并重新编号。原QR请求自动预加载OBJECT后仍只发62/53，坐标须等MCU新的OBJECT请求，线格式与类别映射不变。

验证：已提交参数基线163项电脑测试全通过，无错误或跳过；用户手调配置下162项行为测试全通过，仅排除原有固定默认参数比较。配置文件及应用清单未修改、未提交；保留用户ROI 0.25、中心(0.44,0.34)和置信度0.50等调整。新增真实main/UartLink对固定f976afb MCU解析器回放，验证球/人质两组人工扫码后仍用原request恢复、保持历史编号、隔离双缓冲暖机与QR包。未部署或实机验收，旧dist不包含修复。

复测：先在电控端停车；MaixVision重新运行当前完整maixcam2_qr_object_app。电控下发OBJECT任务后USER切QR，应显示SCAN QR TO RESUME OBJECT；合法三码确认后应打印QR task=... auto OBJECT，恢复本轮62/01、62/54。返回模式会恢复任务数据，人工切模式不是电机急停。

## 2026-10-09：MC2球/人质编号统一为62/54

用户已反馈迁入前本工程在MC2可运行。首次迁入的小球编号采用55，与队友2026-10-09发布的MCU提交f976afb不兼容；本次采用最小补丁统一为54，不替换队友旧MC视觉工程或本机MCU源码。保留已有的人质54字节、MC2平台检查、9767模型、相机/显示配置和UART2。config.py、hardware.py、app.yaml及模型与本次修改前SHA256一致，用户改过的应用信息不覆盖。

新63/task1请求完整ACK后开始记球的首次出现顺序1～3；同帧新颜色从左到右，目标首次出现即可返回编号，不要求三球同时入镜。新请求清零，同号重试保留，漏检/移动/重现不重新编号。只有模型ID3蓝/4红/5绿参与；每种颜色一个站位，不区分同色不同实物。未见目标编号为0，不代表第1个或抓取成功。

坐标仍按原62/01，只含所请求颜色的新图坐标，漏检立即发新空包。另发18字节62/54历史顺序包，与本帧01共用seq：`AA 55 | 62 | request:u16 | 09 00 | 54 | seq:u16 | target_model_id:u8 | target_rank:u8 | seen_count:u8 | slots:3*u8 | CRC:u16`。所有多字节字段小端，CRC16-CCITT-FALSE覆盖62到slots；slots前seen_count个模型ID互不重复，空位FF。球task1只允许ID3/4/5，人质task3只允许ID0/1/2，目标必须匹配当前63请求；不再发送55。靶/桶/通用诊断/扫码交接/人工预览均不发54。屏幕及日志仍显示BALL ORDER/TARGET#。

串口仍为`/dev/ttyS2`、115200，TX=B0/UART2_TX、RX=B1/UART2_RX，未移入旧MC的UART1配置。60/63请求、61 ACK、62请求绑定、01坐标、53三码和人质54字段不变；完整ACK后才发结果。同号重试保留编号、新请求清零。软件兼容目标为队友MCU f976afb（或保持该接收契约的后续版本），其中proto_target_rank_get读取当前请求的历史编号，不代表已经对准、抓取或完成返回路线。本机未合并该MCU版本，不修改电控或烧录。

155项电脑测试全部通过，无失败、错误或跳过。原迁入版145项通过，但新增队友接收回放在修改前失败，验证55确实不兼容；改54后全通过。覆盖三种球的全部六种排列、原人质54字节、固定CRC、请求/ACK/类别域门控、重试/重置、双缓冲暖机、目标丢失、同seq01与54独立接收、短写旧尾包先于新ACK、序号回绕及非法包恢复。test_teammate_rank_protocol.py从Git固定提交f976afb导出真实C解析器及其测试到自动清理的临时编译目录，执行MC2真实main/UartLink发送回放；没有复制旧MC运行代码进MC2，也没有替换本机App/proto.c。旧本地解析器仍不支持54，既有回放仅证明后续01不受影响；兼容结论以f976afb专项为准。

原地修改源码和现有说明，不另存备份、不打包、不部署。用户随后授权上传共用gongke/main；发布完整08源码和协议交接，不把独立仓库目录上传成缺失内容的Git子模块。只提交本任务代码，用户app.yaml未提交改动不纳入；远端首次加入应用清单时采用独立仓库已提交的原版本。旧dist包不含新球编号，复测须MaixVision打开当前完整maixcam2_qr_object_app，不能只传main.py。用户的原MC2可运行反馈不等于新编号实机验收；先停车，确认MCU固件含f976afb接收逻辑，再核对新task1请求、完整61 ACK、62/01、62/54及MCU读取到的编号。

## 1. 先运行

1. 给MC2接好供电，取下镜头盖，连接MaixVision。先让小车停止运动再测试。
2. 在MaixVision打开本工程下整个 `maixcam2_qr_object_app` 文件夹，不是只打开main.py。
3. 运行main.py。启动应出现 `[DEVICE] id=maixcam2`、`[MODE] QR 1920x1280` 和UART信息。
4. USER短按切到物体识别，再短按回扫码；长按1.5秒退出。物体模式能点屏幕上的框开关文字信息；点空处恢复默认。
5. 观察 `[YOLO26]` 的实际模型输入宽高、`[PERF]` 的fps/各阶段耗时和UART TX/RX。没有实体MC2运行日志前，不算上板验收完成。

如需要装成独立App，在MaixVision的应用安装入口选择：
`dist/maix-mc2_qr_object_test-v1.0.0.zip`。不要把源码压缩包当成安装包。

两种模式不是同时运行：扫码用灰度图CPU解码；识别用RGB图和YOLO26模型双缓冲。新请求/切模式后丢弃首轮流水线结果，随后把检测结果配到对应输入图，不拿上一任务的坐标凑包。

## 2. 串口接线

MC2 **B0 / UART2_TX** 接 MCU **RX**；MC2 **B1 / UART2_RX** 接 MCU **TX**；GND接GND。
如果沿用原电控PD5/PD6的USART2接线，则B0接PD6，B1接PD5。MC2的UART2和STM32的USART2是两端各自的编号，并不要求编号相同。

设备路径 `/dev/ttyS2`，波特率 **115200、8N1、3.3V TTL**。不要接RS232电平或把5V接入信号脚；不要把两端TX接一起。

接线依据：[Sipeed UART文档](https://wiki.sipeed.com/maixpy/doc/zh/peripheral/uart.html)，MC2映射表B0/B1对应UART2。

## 3. 沿用哪些参数

`maixcam2_qr_object_app/config.py` 类似C的config.h；每个可调参数旁都有注释。

- 开机QR；扫码1920×1280，30fps请求值，ROI比例0.20，中心(0.44,0.35)，切换后丢2帧，读取超时2000ms。
- 合法三码1帧确认；识别置信度0.35；模型双缓冲True；显示旧框保留200ms，但旧框不进入UART。
- 扫码相机缓冲请求1，物体请求2。**MC2驱动可能增加内部缓冲数**，不能把请求1当作硬件一定只留1帧。
- 物体输入宽高读取模型，不由config中480×320的显示回退值决定。MUD没有写宽高，因此本次没有把旧模型尺寸冒充新模型尺寸。
- 标签顺序严格核对；0扁圆物体（赛题腰鼓）、1圆柱、2圆台、3蓝球、4红球、5绿球、6红靶、7蓝靶、8绿靶、9黑桶。保留参考工程的历史Cone/圆锥别名，物体实体仍按圆台理解，不改变MCU编号。

MC2首次初始化和切换使用公共Camera API，不继承旧MC的GC4653/MMF专用假设。相机、镜头或安装位置换了，**电控像素工作点必须重新核对**；不能照搬旧相机的x/y，也不能为了兼容伪造图像宽高。

## 4. 与MCU联调

握手及坐标字段不改，球/人质共用62内层54顺序包：AA55帧头，CRC16-CCITT，小端多字节字段。

- MCU发0x60：新request_id + mode，0停止、1扫码、2识别。视觉回0x61 ACK；ACK完整写入后才能发本轮结果。
- MCU发0x63：新request_id + task_id + digit。task1球、task2靶、task3人质、task4黑桶；前三任务digit取1/2/3，桶取0。
- 视觉0x53回三位ASCII任务码；0x01回本帧目标及真实图幅、中心cx/cy、框宽高、分数和耗时；电控接管时结果封进0x62并绑定request_id。
- 0x54回传首次出现历史顺序：task1球ID3/4/5，task3人质ID0/1/2；两组不能混用，目标须匹配当前请求。返回编号不是颜色/形状类别、QR数字或当前帧坐标。
- 不再发0x55。MCU必须使用f976afb或兼容接收实现，不能只更新相机就视为双端已部署。
- 独立扫码成功后自动切识别；如MCU持有QR请求，只继续报码，等待MCU的新识别请求才返回坐标。
- 未扫码时USER短按能查看所有类别。MCU接管后只在预览模式与原请求不一致期间暂停回传；返回原请求模式恢复。人工QR扫码确认合法三码可自动返回已ACK的原OBJECT任务，预览不新增历史编号，不把二维码包发到物体请求。
- 没找到指定目标就发新空包，不复用旧坐标。软件write完成不等于MCU已经收到，必须核对MCU记录。

例如331：球选蓝球ID3，靶选蓝靶ID7，人质选圆柱ID1。三个数字属于三个不同任务，不是三次都选择同一种物体。

换MC2后，先在静止状态验证扫码、61/62收发、单任务筛选、实际图幅、同帧cx/cy、空包、新seq和发送间隔。电控已有300ms目标断流保护；达不到间隔时先排查推理/显示/串口，不重复旧帧骗过保护。不据电脑测试放行实车抓取。

## 5. 模型和源码来源

参考：`../04_电控/gongke/03_扫码与物体识别联合App工程/qr_object_app`，
分支vision-task-filter-20261006，提交 **0ffa353**（本次复制时工作区干净）。

新模型来源：`../../data/maixhub-model-convert-maixcam2-9767`。
三个文件已复制到App目录并比对SHA256，没有修改原模型：

```text
model_9767.mud           B5A9AACEEBB51BBBFE7B0DBAB23C4F39389B6619503087DE26BED56F9CA9EB71
model_9767_npu.axmodel   5CC4FD6CDF17B07638DFD4AE3EF5CB5CBB174AF82988A6276B0AF44955A99B52
model_9767_vnpu.axmodel  0998E5EFDF5C3D569ECBFE188601926984A53CCC2C2305553A744F0D52BA6D6A
```

MUD的type=axmodel、model_type=yolo26、10个中文标签，与配置一致。程序加载MUD，NPU/VNPU权重必须在同目录。固件需提供nn.YOLO26；缺少时给明确提示，不自动刷机。MC的cvimodel不能用于MC2。

源码阅读顺序：config.py、main.py、mode_controller.py、object_detector.py，再看qr_reader.py、task_selection.py、control_session.py、protocol.py。frame_pair.py解释模型双缓冲的图像配对；hardware.py负责串口重连/部分写；ui.py和touch_inspector.py只负责显示。

## 6. 验证与下一步

2026-10-09：**127项电脑测试全部通过，无跳过**。包含UART2引脚/路径、模型引用/类别、USER模式往返、相机失败回滚、双缓冲图像配对、任务隔离、部分写/重连、RAM日志和显示/触屏回归；其中8项通过相邻04仓库真实App/proto.c回放，包括27种任务码组合和真实二进制组包。

安装包24个文件及源码包43个文件均逐项与源文件比对，ZIP完整性通过；三个模型复制哈希一致。Git对模型禁用自动换行转换，保持原始文件字节。原参考仓库保持干净，线协议相关模块与参考源码一致。模拟Maix API不证明固件兼容、真实帧率、识别率、实体UART或车辆动作；**实体MC2尚未部署/实测**。

复验（PowerShell，先进入本工程目录）：

```powershell
Set-Location -LiteralPath 'D:\Users\BELLATOR\Desktop\projecting\工科\project\04_电控\gongke\08_MaixCAM2扫码与物体识别App工程'
& 'E:\setup\anaconda\envs\yolo_train\python.exe' -B -m unittest discover -s .\maixcam2_qr_object_app\tests -q
& 'E:\setup\anaconda\envs\yolo_train\python.exe' -B .\build_packages.py
```

打包仅需电脑Python标准库，不依赖训练环境。上面的Python路径只复用现成解释器；MC2在设备自身MaixPy中运行。真实MCU回放测试只读相邻04仓库和既有GCC；缺少时明确跳过，不自动安装或改电控。

下一步：在MC2运行完整App，记录[DEVICE]/[MODE]/[YOLO26]/[PERF]，验证USER往返和MCU模式请求；根据实际图幅及安装位置重测电控工作点。失败回滚只需停止本测试App并使用原工程，本次没有替换原App或改系统设置。

## 7. 生成文件与清理记录

- `maixcam2_qr_object_app/*.py`、`app.yaml`、`app.png`、三个模型和两级README：工程交付，保留。
- `maixcam2_qr_object_app/tests/` 与 `build_packages.py`：可复用回归和打包工具，保留，不是一次性报告。
- `dist/*.zip`、`dist/SHA256SUMS.txt`：本轮生成、校验后保留用于上板；可用build_packages.py重建，Git忽略，不当作唯一源码备份。
- 电脑测试临时目录：由TemporaryDirectory退出时自动清理。使用Python -B，不生成本轮pycache。没有另建临时报表或.backup。
- 设备UART日志：每次App启动建新会话，只写已验证的tmpfs/ramfs，优先/dev/shm/vision_uart_logs；断电清除。无RAM挂载则报警并禁用日志，不回退SD卡。单会话4MB、目录16MB限额，避免填满内存；重启App不等于断电，旧会话可能仍在。
- 需要留作故障证据的设备日志：断电前自行导出并记录去向；不要自动删除唯一证据。收尾只清确认由本任务产生且无引用的中间文件，不清原模型、训练数据、原工程或用户修改。
