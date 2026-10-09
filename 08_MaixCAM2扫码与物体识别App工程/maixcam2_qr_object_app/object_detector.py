"""YOLO26 十类检测：输入一帧，返回物体列表和本次 detect 调用耗时。"""
import os
from maix import image, nn, time
import config
from model_metadata import validate_model

class Detection:
    def __init__(self, raw):
        self.x, self.y, self.w, self.h = int(raw.x), int(raw.y), int(raw.w), int(raw.h)
        self.score, self.class_id = float(raw.score), int(raw.class_id)

class ObjectDetector:
    def __init__(self):
        if not hasattr(nn, "YOLO26"):
            raise RuntimeError("当前MC2固件没有nn.YOLO26，请安装支持YOLO26的MaixPy固件")
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), config.MODEL_FILE)
        if not os.path.exists(model_path):
            raise RuntimeError("找不到模型文件: " + model_path)
        validate_model(model_path, config.CLASS_NAMES_CN)  # 先核对MC2格式和两份模型，不猜类别。
        self.model = nn.YOLO26(model=model_path, dual_buff=config.DUAL_BUFFER)
        # tuple 相当于不再修改的数组；比较顺序防止新模型套用旧类别表。
        labels = tuple(str(label).strip() for label in self.model.labels)
        if labels != config.CLASS_NAMES_CN:
            self.close()
            raise RuntimeError("模型标签与 config.py 顺序不一致: " + repr(labels))
        self.input_width = self.model.input_width()
        self.input_height = self.model.input_height()
        if self.input_width <= 0 or self.input_height <= 0:
            self.close()
            raise RuntimeError("模型返回无效输入尺寸")
        if self.model.input_format() != image.Format.FMT_RGB888:
            self.close()
            raise RuntimeError("本工程使用 RGB888，模型输入格式不匹配")
        print("[YOLO26] loaded={} classes={} input={}x{} dual_buffer={}".format(
            model_path, len(labels), self.input_width, self.input_height, config.DUAL_BUFFER))

    def detect(self, img):
        started = time.ticks_ms()
        objects = [Detection(item) for item in self.model.detect(img, conf_th=config.CONF_THRESHOLD)]
        if any(item.class_id < 0 or item.class_id >= len(config.CLASS_NAMES) for item in objects):
            raise RuntimeError("模型返回了配置范围外的 class_id，停止以免坐标和类别错配")
        objects.sort(key=lambda item: item.score, reverse=True)
        # 挑任务类别前不能截断，否则高分干扰物会挤掉黑桶或指定目标。
        return objects, time.ticks_ms() - started

    def close(self):
        self.model = None
