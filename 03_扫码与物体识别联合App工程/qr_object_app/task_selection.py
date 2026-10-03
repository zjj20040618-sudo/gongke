"""三位二维码选择三类任务目标；黑桶不依赖任务码，持续上报当前坐标。"""
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

    def observe(self, payload):
        if not isinstance(payload, str) or len(payload) != 3 or any(ch not in "123" for ch in payload):
            return False
        if self.payload is not None:
            return payload == self.payload  # 本轮首个合法任务码锁存，不被后续另一张码覆盖。
        self.payload = payload
        self.class_ids = (self.BALL_IDS[payload[0]], self.TARGET_IDS[payload[1]], self.HOSTAGE_IDS[payload[2]])
        print("[TASK] code={} ball={} target={} hostage={} barrel=9".format(payload, *self.class_ids))
        return True

    def select(self, objects):
        # 与电控一样按类别挑置信度最高的一个；仅处理本帧，不保存坐标。
        allowed = self.class_ids + (self.BLACK_BARREL_ID,)
        best = {}
        for obj in objects:
            if obj.class_id in allowed and (obj.class_id not in best or obj.score > best[obj.class_id].score):
                best[obj.class_id] = obj
        return [best[class_id] for class_id in allowed if class_id in best][:config.MAX_OBJECTS]
