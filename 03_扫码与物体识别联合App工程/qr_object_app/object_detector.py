"""YOLO26九类检测：输入一帧，输出Detection列表和推理毫秒数。"""
import os
from maix import nn, time
import config

class Detection:
    def __init__(self, raw):
        self.x, self.y, self.w, self.h = int(raw.x), int(raw.y), int(raw.w), int(raw.h)
        self.score, self.class_id = float(raw.score), int(raw.class_id)

class ObjectDetector:
    def __init__(self):
        if not hasattr(nn, "YOLO26"):
            raise RuntimeError("当前MaixPy固件没有nn.YOLO26，请升级固件")
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), config.MODEL_FILE)
        if not os.path.exists(model_path):
            raise RuntimeError("找不到模型文件: " + model_path)
        self.model = nn.YOLO26(model=model_path, dual_buff=config.DUAL_BUFFER)
        print("[YOLO26] loaded:", model_path)

    def detect(self, img):
        started = time.ticks_ms()
        objects = [Detection(item) for item in self.model.detect(img, conf_th=config.CONF_THRESHOLD)]
        objects.sort(key=lambda item: item.score, reverse=True)
        return objects[:config.MAX_OBJECTS], time.ticks_ms() - started

    def close(self):
        self.model = None

