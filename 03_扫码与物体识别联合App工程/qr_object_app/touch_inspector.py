"""触屏只选显示对象；不参与任务筛选、检测或UART组包。"""


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
                print("[TOUCH] tap object: inspect; tap empty image: show all")
            except Exception as exc:
                self.close()
                print("[TOUCH] disabled; detection/UART continue:", exc)

    @property
    def selected(self):
        return self.class_id is not None

    @property
    def status(self):
        if not self.selected:
            return None
        return "ID:{} {} / TAP EMPTY: ALL".format(self.class_id, "LOST" if self.lost else "SELECTED")

    def reset(self):
        self.class_id = self.box = None
        self.lost = False

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
            print("[TOUCH] read failed; show all, UART continues:", exc)
        return tap

    def choose(self, objects, image_size, tap=None):
        if tap is not None:
            point = screen_to_image(tap, self.display_size, image_size)
            if point is not None:
                x, y = point
                hits = [obj for obj in objects if obj.w > 0 and obj.h > 0 and
                        obj.x <= x < obj.x + obj.w and obj.y <= y < obj.y + obj.h]
                self.reset()
                if hits:
                    # 重叠框优先较小框，其次置信度；不修改原检测列表。
                    obj = min(hits, key=lambda obj: (obj.w * obj.h, -obj.score))
                    self.class_id, self.box = obj.class_id, _box(obj)
                    return [obj]
        if not self.selected:
            return objects
        if not self.lost:
            candidates = [(obj, _iou(self.box, _box(obj))) for obj in objects
                          if obj.class_id == self.class_id]
            if candidates:
                obj, overlap = max(candidates, key=lambda item: item[1])
                if overlap >= 0.2:
                    self.box = _box(obj)
                    return [obj]
        # 丢失后不自动跳到另一个同类物体，也不沿用旧坐标；重新点选。
        self.lost = True
        return []

    def close(self):
        if self.device is not None:
            try:
                self.device.close()
            except Exception as exc:
                print("[TOUCH] close failed:", exc)
            self.device = None
