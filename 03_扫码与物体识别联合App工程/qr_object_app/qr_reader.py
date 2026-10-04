# 扫码模块：对当前图像的中心区域解码，把SDK二维码对象整理成字典列表。
# 字典保存payload文本、显示文字、框坐标和四角；文字翻译不会改变原二维码内容。
# 这里可以读出非赛题码，是否接受为三位任务码还由TaskSelection.observe判断。

"""二维码解码模块：图像输入，结构化结果输出。"""
from maix import image
import config

# 功能：把三位任务码转成屏幕用中文说明。
# 参数：payload：二维码内容字符串。
# 返回：中文说明字符串；格式不合要求则返回提示文字。
def task_text_cn(payload):
    if len(payload) != 3 or any(ch not in "123" for ch in payload):
        return "非赛题任务码"
    return "排爆:{} 靶:{} 人质:{}".format(config.QR_COLOR_NAMES_CN[payload[0]], config.QR_COLOR_NAMES_CN[payload[1]], config.QR_SHAPE_NAMES_CN[payload[2]])

# 功能：生成可画在图上的ASCII任务说明。
# 参数：payload：二维码内容字符串。
# 返回：英文缩写字符串，非法码显示RAW。
def task_text_ascii(payload):
    if len(payload) != 3 or any(ch not in "123" for ch in payload):
        return "RAW"
    return "B:{} T:{} S:{}".format(config.QR_COLOR_CODES[payload[0]], config.QR_COLOR_CODES[payload[1]], config.QR_SHAPE_CODES[payload[2]])

# 功能：按配置计算居中的扫码区域。
# 参数：img：当前图像。
# 返回：[左,上,宽,高]列表；比例>=1时返回[]供SDK按全图处理。
def center_roi(img):
    fraction = float(config.QR_ROI_FRACTION)
    if fraction >= 1.0:
        return []
    width, height = img.width(), img.height()
    # //是向下整除；先除2再乘2把尺寸限制为偶数，max(2,...)保证至少2像素。
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

    # 功能：读取本帧二维码，异常时报告并返回空结果。
    # 参数：img：当前图像。
    # 返回：二维码字典列表，最多QR_MAX个；未发现或解码异常时为空列表。
    def decode(self, img):
        try:
            roi = center_roi(img)
            codes = img.find_qrcodes(roi) if self.decoder is None else img.find_qrcodes(roi, decoder_type=self.decoder)
            self.last_error = None
        except Exception as exc:
            message = str(exc)
            # 只在错误内容变化时打印，避免每帧相同错误刷屏；解码成功会清旧错误。
            if message != self.last_error:
                print("[QR] decode error:", message)
                self.last_error = message
            return []
        results = []
        for code in codes[:config.QR_MAX]:
            payload = code.payload()
            # append把新字典加到列表末尾；坐标来自本次SDK结果，不取之前画框缓存。
            results.append({"payload": payload, "text": task_text_cn(payload), "text_ascii": task_text_ascii(payload), "x": int(code.x()), "y": int(code.y()), "w": int(code.w()), "h": int(code.h()), "corners": code.corners()})
        return results

