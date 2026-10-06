"""只给屏幕保留短时漏检的框；缓存对象不能拿去筛选任务或组串口包。"""
from touch_inspector import _box, _iou


class DisplayBox:
    """复制检测数据，类似把C结构体的成员复制到另一个结构体。"""
    def __init__(self, obj, default_info):
        self.class_id, self.score = obj.class_id, obj.score
        self.x, self.y, self.w, self.h = obj.x, obj.y, obj.w, obj.h
        self.default_info = default_info
        self.held = False  # True表示本帧没检测到，屏幕暂留旧框。


class DisplayCache:
    def __init__(self, hold_ms=200):
        self.hold_ms = max(0, int(hold_ms))
        self.reset()

    def reset(self):
        self.entries = []
        self.image_size = None

    def update(self, objects, default_info, image_size, now_ms):
        """命中立即更新；漏检只保留至最后命中后hold_ms，不sleep、不续期。

        同类且位置重叠的框视为同一个显示物体，各框只匹配一次。
        时间倒退或分辨率变化时丢弃旧缓存，避免错画坐标。
        """
        if image_size != self.image_size or any(now_ms < seen for _, seen in self.entries):
            self.reset()
        self.image_size = image_size
        old = [(obj, seen) for obj, seen in self.entries if now_ms - seen < self.hold_ms]
        used = set()
        fresh = []
        default_ids = {id(obj) for obj in default_info}
        for obj in objects:
            candidates = [(index, _iou(_box(obj), _box(previous)))
                          for index, (previous, _) in enumerate(old)
                          if index not in used and previous.class_id == obj.class_id]
            if candidates:
                index, overlap = max(candidates, key=lambda item: item[1])
                if overlap >= 0.2:
                    used.add(index)
            # 本帧对象不被画框缓存修改，UART继续使用原始objects。
            fresh.append((DisplayBox(obj, id(obj) in default_ids), now_ms))
        for index, (obj, seen) in enumerate(old):
            if index not in used:
                obj.held = True
                fresh.append((obj, seen))  # 漏检不改seen；持续漏检一定会到期。
        self.entries = fresh
        boxes = [obj for obj, _ in fresh]
        return boxes, [obj for obj in boxes if obj.default_info]
