"""任务区的首次识别顺序：只保存类别编号，不保存旧坐标。

给 C 语言读者：order 是一个列表，类似长度最多为 3 的数组。
列表下标从 0 开始，对外序号从 1 开始，所以查找后要加 1。
每轮每种形状/颜色只有一个站位；本模块不区分两个同类的实物。
这是首次有效出现顺序，不保证就是物理摆放次序；漏检和安装朝向须实测。
"""
from task_selection import TaskSelection


class EncounterOrder:
    CLASS_IDS = ()
    LOG_NAME = "ORDER"

    def __init__(self):
        self.selector = TaskSelection()
        self.reset()

    def reset(self, target_class_id=None):
        """新请求清零；None 表示不在本轮排序任务中。"""
        if target_class_id is not None and (type(target_class_id) is not int
                                           or target_class_id not in self.CLASS_IDS):
            raise ValueError("invalid {} target".format(self.LOG_NAME.lower()))
        self.target_class_id = target_class_id
        self.order = []

    @property
    def active(self):
        return self.target_class_id is not None

    @property
    def target_rank(self):
        # 0 表示尚未看到目标，不能让电控当成第 1 个或已经抓到。
        return self.order.index(self.target_class_id) + 1 if self.target_class_id in self.order else 0

    def observe(self, objects, img_w, img_h):
        """只接收配帧后的新检测结果；HOLD 显示框不能传进来。

        每类先挑有效的最高分框。不同帧按首次出现时间编号；
        同一帧首次看到多类时按框中心从左到右，同 x 再按 y、类别编号。
        已编号的类别不再排序，漏检、移动、重现都不会改变原序号。
        """
        if not self.active:
            return
        unseen = []
        for class_id in self.CLASS_IDS:
            if class_id not in self.order:
                unseen.extend(self.selector.select(objects, class_id, include_barrel=False,
                                                  img_w=img_w, img_h=img_h))
        unseen.sort(key=lambda obj: (obj.x + obj.w // 2, obj.y + obj.h // 2, obj.class_id))
        for obj in unseen:
            self.order.append(obj.class_id)
            print("[{}] rank={} class={} target_rank={}".format(
                self.LOG_NAME, len(self.order), obj.class_id, self.target_rank))


class HostageOrder(EncounterOrder):
    CLASS_IDS = (0, 1, 2)  # 扁圆物体、圆柱体、圆台体；不是站位序号。
    LOG_NAME = "HOSTAGE"


class BallOrder(EncounterOrder):
    CLASS_IDS = (3, 4, 5)  # 蓝球、红球、绿球；不是站位序号或QR数字。
    LOG_NAME = "BALL"
