# 本次上传修改说明

更新：2026-10-05；视觉App **2.1.1**，基于已发布5ce9f65。仅改显示及对应测试/文档；**串口、模型、阈值、任务筛选、电控源码均未改变**。

## 用户要求与实现

- 物体框旁增加中心坐标 `x=…、y=…`。计算仍是 `x+w//2, y+h//2`，与现有回包一致；宽度不足时分成x/y两行，不修改目标坐标。
- 结果基础字号从5增至6；QR三码、颜色/形状分短行。用实际屏幕宽高补偿1600×900源图缩小，使QR结果和物体坐标使用相同基础字号，不只改一个固定scale。
- 字符测量和绘制用相同thickness；长行单独缩字、文本块夹紧边界，标题行距按测量高度布局。首轮QR不需要加载物体模型来计算字号。
- App版本2.1.1，安装ID不变；旧包保留，新版需重新运行整个项目或安装新版包。

## 验证与安装包

软件测试共126项通过：App 95项（含新增显示15项）、实际主循环7项、二进制/C协议回放21项、诊断契约3项。显示覆盖奇数框中心点、窄图分行、QR/物体缩屏字号、边缘/长文字及标题避让；测试不接相机或电机。

```powershell
& 'E:\setup\anaconda\envs\yolo_train\python.exe' -B -m unittest discover -s '03_扫码与物体识别联合App工程/qr_object_app/tests' -q
& 'E:\setup\anaconda\envs\yolo_train\python.exe' -B tests/test_vision_control_main.py
& 'E:\setup\anaconda\envs\yolo_train\python.exe' -B '03_扫码与物体识别联合App工程/build_packages.py'
```

本地包：`03_扫码与物体识别联合App工程/dist/maix-qr_object_switch-v2.1.1.zip`；源码包和SHA256SUMS.txt同目录。构建时逐文件核对源内容；dist按既有规则忽略，不上传二次产物，云端提供源码/模型和重建脚本。

**尚未部署相机或实机确认字号、未烧录电控/运行电机。** API依据见 [App README](03_扫码与物体识别联合App工程/qr_object_app/README.md) 末尾官方链接。布局模拟和软件回放不等于实机画面或实体UART验收。

## 对电控的影响

无新增字段。物体01、纯三位QR53、控制60/63、ACK61/结果62均不变。当前MCU原有53/63适配缺口仍在，要求见 [VISION_TO_CONTROL.md](VISION_TO_CONTROL.md)，不因UI更新关闭VC-12。

队友可拉取新版，不需为屏幕坐标/字号修改解析器；原协议适配仍按原计划审核/编译/烧录。用户只需更新完整视觉App，确认屏幕坐标与字号效果。
