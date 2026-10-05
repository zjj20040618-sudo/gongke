"""扫码锁存和当前帧筛选；独立测试选三任务，60诊断加桶，63选单任务。

给 C 语言读者：本类相当于保存任务状态的结构体；不会保存物体坐标。
扫码必须连续确认，检测目标则每帧重新选择，防止沿用已经消失的物体。
"""
import math
import config


class TaskSelection:
    # 本项目定义：1红、2绿、3蓝。人质：1圆柱、2圆锥、3腰鼓。
    # 已确认的训练别名：圆台体对应本项目圆锥，扁圆物体对应本项目腰鼓。
    BALL_IDS = {"1": 4, "2": 5, "3": 3}
    TARGET_IDS = {"1": 6, "2": 8, "3": 7}
    HOSTAGE_IDS = {"1": 1, "2": 2, "3": 0}
    BLACK_BARREL_ID = 9

    def __init__(self):
        self.reset()

    def reset(self):
        self.payload = None
        self.class_ids = ()
        self.candidate = None
        self.candidate_count = 0
        self.message = "WAIT TASK"

    @staticmethod
    def valid_payload(payload):
        return isinstance(payload, str) and len(payload) == 3 and all(ch in "123" for ch in payload)

    def observe_qrs(self, qrs):
        """检查完整的新扫码帧；仅在首次连续确认任务时返回 True。

        同帧重复的相同任务码算一帧；不同码冲突、空帧和非法码打断计数。
        集合 set 用于去重，不能先只取第一个码，否则会忽略冲突。
        """
        if self.payload is not None:
            return False
        payloads = [qr.get("payload") for qr in qrs]
        if not payloads or any(not self.valid_payload(value) for value in payloads):
            self.candidate, self.candidate_count = None, 0
            self.message = "WAIT VALID TASK"
            return False
        unique = set(payloads)
        if len(unique) != 1:
            self.candidate, self.candidate_count = None, 0
            self.message = "QR CONFLICT"
            return False
        payload = payloads[0]
        if payload != self.candidate:
            self.candidate, self.candidate_count = payload, 1
        else:
            self.candidate_count += 1
        self.message = "TASK:{} {}/{}".format(payload, self.candidate_count, config.TASK_CONFIRM_FRAMES)
        if self.candidate_count < config.TASK_CONFIRM_FRAMES:
            return False
        self.observe(payload)
        self.message = "TASK:{} LOCKED".format(self.payload)
        return True

    def observe(self, payload):
        # 只负责锁存已确认的码；相机主循环通过 observe_qrs 做连续帧确认。
        if not self.valid_payload(payload):
            return False
        if self.payload is not None:
            return payload == self.payload  # 本轮首个合法任务码锁存，不被后续另一张码覆盖。
        self.payload = payload
        self.class_ids = (self.BALL_IDS[payload[0]], self.TARGET_IDS[payload[1]], self.HOSTAGE_IDS[payload[2]])
        print("[TASK] code={} ball={} target={} hostage={} barrel=9".format(payload, *self.class_ids))
        return True

    @classmethod
    def class_id_for_request(cls, task_id, digit):
        if task_id == 4 and digit == 0:
            return cls.BLACK_BARREL_ID
        maps = {1: cls.BALL_IDS, 2: cls.TARGET_IDS, 3: cls.HOSTAGE_IDS}
        if task_id not in maps or digit not in (1, 2, 3):
            raise ValueError("invalid task_id/qr_digit")
        return maps[task_id][str(digit)]

    def select(self, objects, requested_class_id=None, include_barrel=True, img_w=None, img_h=None):
        # 与电控一样按类别挑置信度最高的一个；仅处理本帧，不保存坐标。
        # 电控63自带目标类别，不依赖相机是否已扫码；独立模式只选三码所需物体。
        if requested_class_id is None:
            allowed = self.class_ids + ((self.BLACK_BARREL_ID,) if include_barrel else ())
        else:
            allowed = (requested_class_id,)
        best = {}
        for obj in objects:
            try:
                class_id, score = obj.class_id, float(obj.score)
                if type(class_id) is not int or class_id not in allowed or not math.isfinite(score) or not config.CONF_THRESHOLD <= score <= 1.0:
                    continue
                if img_w is not None and img_h is not None:
                    x, y, w, h = int(obj.x), int(obj.y), int(obj.w), int(obj.h)
                    if x < 0 or y < 0 or w <= 0 or h <= 0 or x + w > img_w or y + h > img_h:
                        continue
                if class_id not in best or score > best[class_id].score:
                    best[class_id] = obj
            except (TypeError, ValueError, AttributeError, OverflowError):
                continue
        return [best[class_id] for class_id in allowed if class_id in best][:config.MAX_OBJECTS]
