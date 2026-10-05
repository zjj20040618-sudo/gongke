# 扫码与十类识别 App 2.1.2

更新：2026-10-05。检测框外优先显示带黑底白字的类别、置信度和中心坐标，避开彩色框；空间不足时在框内使用黑底白字保证可读。结果字号、QR显示、模型9564、四阶段筛选和串口01/53/63不变。电控已集成53/63软件解析；真实UART与实机显示仍待确认。

## 1. 怎么运行

1. MaixVision连接MaixCAM Pro，打开整个 `qr_object_app`，运行整个项目，不只传main.py。
2. 或安装上一级 `dist/maix-qr_object_switch-v2.1.2.zip`。App ID仍为 `qr_object_switch`，安装会替换同ID旧App；需要保留旧设备应用时先自行备份。
3. 默认IDLE；`UART_ENABLED=True`。电控发请求才识别，正式流程不需按USER。接管前USER短按QR/OBJECT、长按1.5秒退出；接管后忽略按键。
4. 115200、8N1、共地、3.3V：A19/TX→MCU PD6/RX，MCU PD5/TX→A18/RX。实际引脚按板型核对。

只拉Git不会更新相机/MCU。不要混装旧1.1.0安装包、本版源码和未适配电控固件。

## 2. 运行流程

1. 电控发60/QR，视觉回ACK后扫码；锁存首个合法三位1..3任务码。
2. 视觉通过53回三码。码消失后继续回锁存值；新QR请求才清空，同号重试不清空。
3. 电控依次发63：**球task1→桶task4→靶task2→人质task3**，每次换请求号。63自动进入OBJECT，每帧只发指定类别最高分一个，没找到发空帧。
4. 黑桶task4/digit0每帧持续发ID9，不缓存丢失桶坐标。球/靶/人质阶段不附带桶；画面可显示其他类别。
5. 完成/中止发60/IDLE，停止取图、推理及业务结果。视觉不控制电机或猜抓球是否完成。

| 请求 | digit1 | digit2 | digit3 |
| --- | --- | --- | --- |
| task1球 | 红球ID4 | 绿球ID5 | 蓝球ID3 |
| task2靶 | 红靶ID6 | 绿靶ID8 | 蓝靶ID7 |
| task3人质 | 圆柱ID1 | 圆锥ID2 | 腰鼓ID0 |

桶只接受task4/digit0→ID9。digit是数值字节，不是ASCII。例QR123：球4→桶9→靶8→人质0。

通用60/OBJECT保留33诊断的“三任务加桶”，不是四阶段请求。未扫QR时只选桶。相同类别取最高置信度，同分先出现者；每帧重选，不跟踪或沿用旧框。

## 3. 配置和数据

- `MODEL_FILE=model_9564.mud`，对应9564.cvimodel，模型不变。`BOX_TEXT_SCALE` 控制结果基础字号；QR和物体共用，并按实际屏幕尺寸补偿源图缩小。坐标宽度不够时分x/y两行，QR三码/颜色/形状分行，长行测量后缩字防止出界。物体文字使用黑底白字并优先放框外。旧9541/9302保留但不运行、不打入包。
- 类别顺序：0扁圆物体/本项目腰鼓，1圆柱，2圆台/本项目圆锥，3蓝球，4红球，5绿球，6红靶，7蓝靶，8绿靶，9黑桶。训练别名须实物核验。
- 阈值0.35；OBJECT用模型实际画幅，不猜480×320；QR1600×900，中心区域扫码。
- 目标01发ID、置信度×1000、中心X/Y、框宽高、真实画幅及处理耗时。靶也保留旧字段，电控业务可只消费X。坐标像素、左上原点，不是毫米。
- QR53只含seq、count、三个ASCII数字（有码）；无字符串长度、无二维码位置/大小。未扫码count0。
- 外层62绑定request_id；ACK61全写后才能取新图/发结果。UART短写只续未写尾部，失败暂停待ACK业务。

详细字节字段、CRC、固定向量及电控适配要求见 [VISION_TO_CONTROL.md 第4～5节](../../VISION_TO_CONTROL.md)。不支持的二维码格式不能靠补零框假装兼容。

终端RX打印原始HEX；ACK完整日志；业务HEX默认每10帧。`[PERF] sent_ids`是筛入发送包的ID；`[UART TX] complete`仅证明本机write全长，**不证明MCU收到或接受**。

## 4. 验证与构建

主机验证需要完整gongke仓库（C回放依赖根App/tests），不是仅解压App源码包。在上一级 `03_扫码与物体识别联合App工程` 运行：

```powershell
python -B -m unittest discover -s qr_object_app/tests -v
python -B build_packages.py
```

仓库根的 `tests/test_vision_control_main.py` 检查真实主循环ACK/采集/结果顺序。新版真实C回放入口和本轮结果见 [README_本次更新.md](../../README_本次更新.md)，历史回放不代表新QR兼容。

下一步：电控补53解析与63请求，部署配套程序后先做无运动静止UART检查，再测工作点/动作。源码已上传、包已生成、设备已部署、实机已通过是四件不同的事。

显示实现依据：[Sipeed Display API](https://wiki.sipeed.com/maixpy/api/maix/display.html) 的屏幕尺寸/FIT_CONTAIN及 [Image API](https://en.wiki.sipeed.com/maixpy/api/maix/image.html) 的string_size测量。软件布局测试不代替实际相机屏幕和MaixVision观感确认。
