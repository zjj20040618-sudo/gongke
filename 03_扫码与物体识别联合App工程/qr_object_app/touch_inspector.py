"""触屏逐个开关物体信息；不参与任务筛选、检测或UART组包。"""


def screen_to_image(point, screen_size, image_size):
    """匹配Display.show默认FIT_CONTAIN：小图居中不放大，大图等比缩小。"""
    sw, sh = screen_size
    iw, ih = image_size
    if min(sw, sh, iw, ih) <= 0:
        return None
    scale = min(1.0, sw / iw, sh / ih)
    x = (point[0] - (sw - iw * scale) / 2) / scale
    y = (point[1] - (sh - ih * scale) / 2) / scale
    return (x, y) if 0 <= x < iw and 0 <= y < ih else None


def _box(obj):
    return (obj.x, obj.y, obj.w, obj.h)


def _iou(a, b):
    x, y = max(a[0], b[0]), max(a[1], b[1])
    w = max(0, min(a[0] + a[2], b[0] + b[2]) - x)
    h = max(0, min(a[1] + a[3], b[1] + b[3]) - y)
    intersection = w * h
    return intersection / max(1, a[2] * a[3] + b[2] * b[3] - intersection)


class ObjectInspector:
    def __init__(self, display_size, enabled=True):
        self.display_size = display_size
        self.device = None
        self.pressed = False
        self.reset()
        if enabled:
            try:
                from maix import touchscreen
                self.device = touchscreen.TouchScreen()
                if not self.device.is_opened():
                    raise RuntimeError("touchscreen not opened")
                print("[TOUCH] tap object: toggle info; tap empty image: restore defaults")
            except Exception as exc:
                self.close()
                print("[TOUCH] disabled; detection/UART continue:", exc)

    @property
    def selected(self):
        return bool(self.overrides)

    @property
    def status(self):
        if self.last_touched is None:
            return None
        return "ID:{} INFO:{} / TAP:TOGGLE".format(
            self.last_touched["class_id"], "ON" if self.last_touched["visible"] else "OFF")

    def reset(self):
        self.overrides = []  # 每个字典记一个物体的显示开关；不是整类物体的开关。
        self.last_touched = None

    def poll(self):
        """非阻塞、每轮最多8事件；长按/拖动不重复选择，保留快速点击。"""
        tap = None
        if self.device is None:
            return tap
        try:
            for _ in range(8):
                if not self.device.available(timeout=0):
                    break
                x, y, pressed = self.device.read()
                if pressed and not self.pressed:
                    tap = (x, y)
                self.pressed = bool(pressed)
        except Exception as exc:
            self.close()
            self.reset()
            tap = None  # 读到按下后驱动又报错，也不让残留点击改变默认显示。
            print("[TOUCH] read failed; restore defaults, UART continues:", exc)
        return tap

    def choose(self, objects, image_size, tap=None, default_visible=None):
        """objects用于点选全部框；default_visible指定默认显示信息的物体。

        手动模式默认显示全部信息，任务模式只默认显示任务目标。
        用同类别和框重叠匹配相邻帧；只返回本帧对象，丢失即清除其开关。
        """
        defaults = objects if default_visible is None else default_visible
        default_ids = {id(obj) for obj in defaults}
        matched = {}  # 键是本帧列表下标，值是该物体独立的开关字典。
        for state in self.overrides:
            candidates = [(index, _iou(state["box"], _box(obj)))
                          for index, obj in enumerate(objects)
                          if index not in matched and obj.class_id == state["class_id"]]
            if candidates:
                index, overlap = max(candidates, key=lambda item: item[1])
                if overlap >= 0.2:
                    state["box"] = _box(objects[index])
                    matched[index] = state
        self.overrides = list(matched.values())
        if not any(state is self.last_touched for state in self.overrides):
            self.last_touched = None
        if tap is not None:
            point = screen_to_image(tap, self.display_size, image_size)
            if point is not None:
                x, y = point
                hits = [(index, obj) for index, obj in enumerate(objects) if obj.w > 0 and obj.h > 0 and
                        obj.x <= x < obj.x + obj.w and obj.y <= y < obj.y + obj.h]
                if hits:
                    # 重叠框优先较小框，其次置信度；不修改原检测列表。
                    index, obj = min(hits, key=lambda item: (item[1].w * item[1].h, -item[1].score))
                    state = matched.get(index)
                    if state is None:
                        state = {"class_id": obj.class_id, "box": _box(obj),
                                 "visible": id(obj) in default_ids}
                        matched[index] = state
                        self.overrides.append(state)
                    state["visible"] = not state["visible"]
                    self.last_touched = state
                else:
                    self.reset()  # 点画面空白恢复本模式的默认显示；黑边不操作。
                    matched = {}
        return [obj for index, obj in enumerate(objects)
                if (matched[index]["visible"] if index in matched else id(obj) in default_ids)]

    def close(self):
        if self.device is not None:
            try:
                self.device.close()
            except Exception as exc:
                print("[TOUCH] close failed:", exc)
            self.device = None
