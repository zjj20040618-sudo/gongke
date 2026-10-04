# 检测模块：把Maix SDK输出统一为Detection对象，再返回给选择模块和画框模块。
# Detection像记录一只物体的小结构体，x/y是框左上角，w/h是框大小，score是置信度。
# ObjectDetector保存真正的模型对象；加载阶段要核对类别顺序、输入格式和真实画幅。

"""YOLO26 十类检测：输入一帧，返回物体列表和本次 detect 调用耗时。"""
import os
from maix import image, nn, time
import config

class Detection:
    # 功能：把SDK原始检测记录转换为本工程的字段。
    # 参数：raw：含坐标、分数、类别的SDK对象。
    # 返回：None（没有显式return时默认返回None）。
    def __init__(self, raw):
        self.x, self.y, self.w, self.h = int(raw.x), int(raw.y), int(raw.w), int(raw.h)
        self.score, self.class_id = float(raw.score), int(raw.class_id)

class ObjectDetector:
    def __init__(self):
        if not hasattr(nn, "YOLO26"):
            raise RuntimeError("当前固件没有 nn.YOLO26，需要 MaixPy 4.12.5 或更新版本")
        # __file__是当前源码路径；dirname取文件夹，join拼模型名，不依赖终端在哪个目录运行。
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), config.MODEL_FILE)
        if not os.path.exists(model_path):
            raise RuntimeError("找不到模型文件: " + model_path)
        self.model = nn.YOLO26(model=model_path, dual_buff=config.DUAL_BUFFER)
        # tuple 相当于不再修改的数组；比较顺序防止新模型套用旧类别表。
        # tuple(...for...)逐项转成去首尾空白的标签元组；要连顺序一起一致，不能只比较类别数量。
        labels = tuple(str(label).strip() for label in self.model.labels)
        if labels != config.CLASS_NAMES_CN:
            self.close()
            raise RuntimeError("模型标签与 config.py 顺序不一致: " + repr(labels))
        self.input_width = self.model.input_width()
        self.input_height = self.model.input_height()
        if self.model.input_format() != image.Format.FMT_RGB888:
            self.close()
            raise RuntimeError("本工程使用 RGB888，模型输入格式不匹配")
        print("[YOLO26] loaded={} classes={} input={}x{} dual_buffer={}".format(
            model_path, len(labels), self.input_width, self.input_height, config.DUAL_BUFFER))

    # 功能：对当前图像推理，检查类别，并把框按分数从高到低排列。
    # 参数：img：当前摄像头图像。
    # 返回：(Detection列表, 本次调用耗时毫秒)。
    # 理解：返回完整检测列表给task.select；先截断会让高分干扰物挤掉指定目标。
    def detect(self, img):
        started = time.ticks_ms()
        objects = [Detection(item) for item in self.model.detect(img, conf_th=config.CONF_THRESHOLD)]
        if any(item.class_id < 0 or item.class_id >= len(config.CLASS_NAMES) for item in objects):
            raise RuntimeError("模型返回了配置范围外的 class_id，停止以免坐标和类别错配")
        # lambda是短匿名函数，这里每个元素取score作为排序依据；reverse=True表示降序。
        objects.sort(key=lambda item: item.score, reverse=True)
        # 挑任务类别前不能截断，否则高分干扰物会挤掉黑桶或指定目标。
        return objects, time.ticks_ms() - started

    # 功能：清除模型引用，便于释放模型占用的资源。
    # 返回：None（没有显式return时默认返回None）。
    def close(self):
        self.model = None

