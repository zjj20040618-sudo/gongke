"""二维码解码模块：图像输入，结构化结果输出。"""
from maix import image
import math
import config
from touch_inspector import screen_to_image

def task_text_cn(payload):
    if len(payload) != 3 or any(ch not in "123" for ch in payload):
        return "非赛题任务码"
    return "排爆:{} 靶:{} 人质:{}".format(config.QR_COLOR_NAMES_CN[payload[0]], config.QR_COLOR_NAMES_CN[payload[1]], config.QR_SHAPE_NAMES_CN[payload[2]])

def task_text_ascii(payload):
    if len(payload) != 3 or any(ch not in "123" for ch in payload):
        return "RAW"
    return "B:{} T:{} S:{}".format(config.QR_COLOR_CODES[payload[0]], config.QR_COLOR_CODES[payload[1]], config.QR_SHAPE_CODES[payload[2]])

def center_roi(img, center=None):
    fraction = float(config.QR_ROI_FRACTION)
    center = center or (config.QR_ROI_CENTER_X, config.QR_ROI_CENTER_Y)
    if not math.isfinite(fraction) or not 0 < fraction <= 1:
        raise ValueError("QR_ROI_FRACTION must be in (0, 1]")
    if any(not math.isfinite(float(value)) or not 0 <= float(value) <= 1 for value in center):
        raise ValueError("QR ROI center must be in [0, 1]")
    if fraction == 1.0:
        return []
    width, height = img.width(), img.height()
    roi_w = min(width, max(2, int(width * fraction) // 2 * 2))
    roi_h = min(height, max(2, int(height * fraction) // 2 * 2))
    x = max(0, min(width - roi_w, int(width * float(center[0]) - roi_w / 2 + .5)))
    y = max(0, min(height - roi_h, int(height * float(center[1]) - roi_h / 2 + .5)))
    return [x, y, roi_w, roi_h]

class QrReader:
    def __init__(self):
        try:
            self.decoder = image.QRCodeDecoderType.QRCODE_DECODER_TYPE_ZBAR
        except Exception:
            self.decoder = None
        self.last_error = None
        self.center = (config.QR_ROI_CENTER_X, config.QR_ROI_CENTER_Y)

    def roi(self, img):
        return center_roi(img, self.center)

    def move_to(self, img, tap, screen_size):
        """点触定位中心；使用实际图幅/黑边映射，框始终留在图像内。"""
        point = screen_to_image(tap, screen_size, (img.width(), img.height()))
        if point is None:
            return False
        old_roi = self.roi(img)
        new_roi = center_roi(img, (point[0] / img.width(), point[1] / img.height()))
        if new_roi == old_roi or not new_roi:
            return False
        x, y, w, h = new_roi
        self.center = ((x + w / 2) / img.width(), (y + h / 2) / img.height())
        print("[QR ROI] center_x={:.6f} center_y={:.6f} roi={} actual={}x{}".format(
            *self.center, new_roi, img.width(), img.height()))
        return True

    def decode(self, img):
        try:
            roi = self.roi(img)
            codes = img.find_qrcodes(roi) if self.decoder is None else img.find_qrcodes(roi, decoder_type=self.decoder)
            self.last_error = None
        except Exception as exc:
            message = str(exc)
            if message != self.last_error:
                print("[QR] decode error:", message)
                self.last_error = message
            return []
        results = []
        # 返回全部解码结果供任务确认检查；QR_MAX只限制画框，不能隐藏不同码冲突。
        for code in codes:
            payload = code.payload()
            results.append({"payload": payload, "text": task_text_cn(payload), "text_ascii": task_text_ascii(payload), "x": int(code.x()), "y": int(code.y()), "w": int(code.w()), "h": int(code.h()), "corners": code.corners()})
        return results
