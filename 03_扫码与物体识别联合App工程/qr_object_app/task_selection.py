# 任务选择器：把三位二维码转换为三个模型类别，随后从每帧检测中挑目标。
# 这与图像检测不同：检测负责找框，选择负责判断哪种球/靶/人质是本轮需要的。
# dict通过类别ID保存本帧最好框；tuple保存本轮允许的类别，黑桶额外固定加入。

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

    # 功能：开始新扫码轮次时清空已锁定任务。
    # 返回：None（没有显式return时默认返回None）。
    # 理解：payload=None表示还未选任务；class_ids=()是空元组，不是含一个空元素。
    def reset(self):
        self.payload = None
        self.class_ids = ()

    # 功能：检查并锁存本轮首个合法三位码。
    # 参数：payload：二维码内容字符串，例如"123"。
    # 返回：True=与本轮选择一致或刚锁存成功；False=非法或不同于已锁存码。
    def observe(self, payload):
        # isinstance检查类型；any在生成的各项条件中只要有一项True就为True。
        # 这里同时要求字符串、长度3、每位为1/2/3，避免只用第一位或接受坏码。
        if not isinstance(payload, str) or len(payload) != 3 or any(ch not in "123" for ch in payload):
            return False
        if self.payload is not None:
            return payload == self.payload  # 本轮首个合法任务码锁存，不被后续另一张码覆盖。
        self.payload = payload
        # 用三张映射表分别查三位字符：例如123选择ID4红球、ID8绿靶、ID0腰鼓。
        self.class_ids = (self.BALL_IDS[payload[0]], self.TARGET_IDS[payload[1]], self.HOSTAGE_IDS[payload[2]])
        print("[TASK] code={} ball={} target={} hostage={} barrel=9".format(payload, *self.class_ids))
        return True

    # 功能：从本帧检测结果挑指定球、靶、人质和黑桶，各类取最高分。
    # 参数：objects：本帧Detection对象列表。
    # 返回：筛选后的列表；某类没出现就不包含它，可能返回空列表。
    # 理解：best每次都是新字典，不保存上一帧坐标；没有扫码也可选择ID9黑桶。
    def select(self, objects):
        # 与电控一样按类别挑置信度最高的一个；仅处理本帧，不保存坐标。
        # (值,)才是单元素元组，末尾逗号不能省；+在这里拼元组，不是数值加法。
        allowed = self.class_ids + (self.BLACK_BARREL_ID,)
        best = {}
        for obj in objects:
            # in检查成员是否存在；同类比较score，只把更高分框覆盖到best对应键。
            if obj.class_id in allowed and (obj.class_id not in best or obj.score > best[obj.class_id].score):
                best[obj.class_id] = obj
        return [best[class_id] for class_id in allowed if class_id in best][:config.MAX_OBJECTS]
