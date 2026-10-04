# 电控会话：保存谁在控制模式、正在响应哪次请求以及上一次ACK。
# class定义类型；self是当前对象，self.request_id的写法类似你Python笔记中的self.color。
# 与C结构体成员相似，属性存状态；方法还能操作这些属性，调用时Python会自动传self。

"""Main-loop-only control. No Maix dependency, so transitions can be host-tested."""
from protocol import build_ack_packet, bind_result

class ControlSession:
    MODES = ("IDLE", "QR", "OBJECT")

    # 功能：创建会话并保存模式控制器。
    # 参数：modes：ModeController对象。
    # 返回：None（没有显式return时默认返回None）。
    # 理解：__init__在ControlSession(modes)创建对象时自动调用。
    def __init__(self, modes):
        self.modes = modes
        self.request_id = None
        self.last_command = None
        self.last_ack = None
        self.remote_owned = False

    # 功能：处理一次模式请求，生成确认包。
    # 参数：request_id：请求编号；mode：0待机/1扫码/2物体。
    # 返回：(ACK字节串, 是否为新请求)：两项元组。
    # 理解：第二项True并不表示成功；status=0/1才区分本次切换结果。
    def apply(self, request_id, mode):
        command = (request_id, mode)
        # 元组比较要两项都相等；同请求重试直接回旧ACK，不重新载模型或清任务码。
        if command == self.last_command:
            return self.last_ack, False  # lost ACK retry must not reload model or clear fresh results
        self.remote_owned = True
        # 先让本轮结果失效；只有enter成功才登记新编号，防失败后仍发送旧模式结果。
        self.request_id = None  # a failed switch must never report old results under new request
        status = 0
        try:
            if mode not in range(3) or (self.last_command and request_id == self.last_command[0]):
                raise ValueError("invalid mode or reused request")
            self.modes.enter(self.MODES[mode])
            self.request_id = request_id
        except Exception as exc:
            status = 1
            print("[CONTROL] switch failed:", exc)
        # 条件表达式：满足if条件取前面的值，否则取else值；index把模式名转换回编号。
        actual_mode = self.MODES.index(self.modes.mode) if self.modes.mode in self.MODES else 0
        self.last_command = command
        self.last_ack = build_ack_packet(request_id, actual_mode, status)
        print("[CONTROL] request={} actual_mode={} status={}".format(request_id, self.MODES[actual_mode], status))
        return self.last_ack, True

    # 功能：按当前会话决定禁止发送、裸帧发送或加请求外壳。
    # 参数：packet：原始目标/QR字节包。
    # 返回：None或可发送的字节包。
    # 理解：电控接管后必须带当前请求号；接管前独立测试仍可发送裸帧。
    def result(self, packet):
        if self.modes.mode == "IDLE" or (self.remote_owned and self.request_id is None):
            return None
        return bind_result(packet, self.request_id) if self.remote_owned else packet
