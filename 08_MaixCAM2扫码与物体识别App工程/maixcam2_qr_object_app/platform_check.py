"""MC2平台保护；无需修改系统配置，检测不符就给出明确错误。"""


def check_device():
    from maix import sys
    device = str(sys.device_id()).lower()
    print("[DEVICE] id={}".format(device))
    if device != "maixcam2":
        raise RuntimeError("本测试App仅供MaixCAM2，当前device_id=" + device)
