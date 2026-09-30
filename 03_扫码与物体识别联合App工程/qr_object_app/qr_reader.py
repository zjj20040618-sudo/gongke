"""二维码解码模块：图像输入，结构化结果输出。"""
from maix import image
import config

def task_text_cn(payload):
    if len(payload) != 3 or any(ch not in "123" for ch in payload):
        return "非赛题任务码"
    return "排爆:{} 靶:{} 人质:{}".format(config.QR_COLOR_NAMES_CN[payload[0]], config.QR_COLOR_NAMES_CN[payload[1]], config.QR_SHAPE_NAMES_CN[payload[2]])

def task_text_ascii(payload):
    if len(payload) != 3 or any(ch not in "123" for ch in payload):
        return "RAW"
    return "B:{} T:{} S:{}".format(config.QR_COLOR_CODES[payload[0]], config.QR_COLOR_CODES[payload[1]], config.QR_SHAPE_CODES[payload[2]])

def center_roi(img):
    fraction = float(config.QR_ROI_FRACTION)
    if fraction >= 1.0:
        return []
    width, height = img.width(), img.height()
    roi_w = max(2, int(width * fraction) // 2 * 2)
    roi_h = max(2, int(height * fraction) // 2 * 2)
    return [(width - roi_w) // 2, (height - roi_h) // 2, roi_w, roi_h]

class QrReader:
    def __init__(self):
        try:
            self.decoder = image.QRCodeDecoderType.QRCODE_DECODER_TYPE_ZBAR
        except Exception:
            self.decoder = None
        self.last_error = None

    def decode(self, img):
        try:
            roi = center_roi(img)
            codes = img.find_qrcodes(roi) if self.decoder is None else img.find_qrcodes(roi, decoder_type=self.decoder)
            self.last_error = None
        except Exception as exc:
            message = str(exc)
            if message != self.last_error:
                print("[QR] decode error:", message)
                self.last_error = message
            return []
        results = []
        for code in codes[:config.QR_MAX]:
            payload = code.payload()
            results.append({"payload": payload, "text": task_text_cn(payload), "text_ascii": task_text_ascii(payload), "x": int(code.x()), "y": int(code.y()), "w": int(code.w()), "h": int(code.h()), "corners": code.corners()})
        return results

