"""Main-loop-only control. No Maix dependency, so transitions can be host-tested."""
from protocol import build_ack_packet, bind_result
from task_selection import TaskSelection

class ControlSession:
    MODES = ("IDLE", "QR", "OBJECT")

    def __init__(self, modes):
        self.modes = modes
        self.request_id = None
        self.last_command = None
        self.last_ack = None
        self.remote_owned = False
        self.task_id = self.qr_digit = self.target_class_id = None
        self.acknowledged = False
        self.manual_override = False
        self.qr_handoff = False  # 已自动打开OBJECT，但电控仍持有原QR请求。

    def auto_object_after_qr(self):
        """预加载OBJECT；不擅自改变电控请求模式、编号或ACK。

        电控QR请求期间仍只回锁存53；坐标须等新OBJECT/63请求。
        独立模式则直接回三码所选物体。
        """
        if self.manual_override or self.modes.mode != "QR":
            return False
        if self.remote_owned and (not self.acknowledged or self.request_id is None):
            return False
        self.modes.enter("OBJECT")
        self.qr_handoff = self.remote_owned
        return True

    def manual_toggle(self):
        """USER只切视觉预览；接管后手动查看不能产生本轮任务结果。

        先切摄像头，再改变控制状态；切换失败保留原会话。
        已接管时保持请求号用于拒绝旧重试，必须收到更大新请求号才恢复业务。
        这不是MCU急停，实车应先从电控停止运动再手动诊断。
        """
        self.modes.toggle()
        self.qr_handoff = False
        if self.remote_owned:
            self.manual_override = True
            self.acknowledged = False

    def ready_for_capture(self):
        return (self.manual_override or not self.remote_owned
                or (self.request_id is not None and self.acknowledged))

    def apply(self, request_id, mode, task_id=None, qr_digit=None):
        command = (request_id, mode, task_id, qr_digit)
        if command == self.last_command:
            if self.manual_override:
                actual = self.MODES.index(self.modes.mode) if self.modes.mode in self.MODES else 0
                # 不发原来成功的缓存ACK：现在是人工诊断，不再执行该请求。
                return build_ack_packet(request_id, actual, 1), False
            return self.last_ack, False  # lost ACK retry must not reload model or clear fresh results
        if self.last_command and request_id < self.last_command[0]:
            actual = self.MODES.index(self.modes.mode) if self.modes.mode in self.MODES else 0
            print("[CONTROL] stale request={} current={} rejected".format(request_id, self.last_command[0]))
            return build_ack_packet(request_id, actual, 1), False
        self.remote_owned = True
        self.qr_handoff = False
        self.request_id = None  # a failed switch must never report old results under new request
        self.acknowledged = False
        status = 0
        try:
            if not 1 <= request_id <= 65535 or mode not in range(3) or (self.last_command and request_id == self.last_command[0]):
                raise ValueError("invalid mode or reused request")
            target = None
            if task_id is not None or qr_digit is not None:
                if mode != 2:
                    raise ValueError("task request must enter OBJECT")
                target = TaskSelection.class_id_for_request(task_id, qr_digit)
            self.modes.enter(self.MODES[mode])
            self.task_id, self.qr_digit, self.target_class_id = task_id, qr_digit, target
            self.request_id = request_id
            self.manual_override = False
        except Exception as exc:
            status = 1
            print("[CONTROL] switch failed:", exc)
        actual_mode = self.MODES.index(self.modes.mode) if self.modes.mode in self.MODES else 0
        self.last_command = command
        self.last_ack = build_ack_packet(request_id, actual_mode, status)
        print("[CONTROL] request={} actual_mode={} task={} digit={} class={} status={}".format(request_id, self.MODES[actual_mode], task_id, qr_digit, self.target_class_id, status))
        return self.last_ack, True

    def ack_sent(self, packet):
        if not self.manual_override and packet == self.last_ack and self.request_id is not None:
            self.acknowledged = True

    def result(self, packet):
        if self.manual_override or self.modes.mode == "IDLE" or (self.remote_owned and (self.request_id is None or not self.acknowledged)):
            return None
        # 手动/异常状态下也不得把QR结果绑定到OBJECT请求，或反过来。
        expected_type = 0x53 if self.modes.mode == "QR" or self.qr_handoff else 0x01
        if packet is None or len(packet) < 3:
            return None
        if packet[2] == 0x54:
            # 球task1/人质task3共用54；不得串用类别域或当前请求的目标。
            class_ids = (3, 4, 5) if self.task_id == 1 else (0, 1, 2)
            if (not self.remote_owned or self.task_id not in (1, 3) or self.modes.mode != "OBJECT"
                    or self.qr_handoff or len(packet) != 13 or packet[5] != self.target_class_id
                    or packet[5] not in class_ids):
                return None
        elif packet[2] != expected_type:
            return None
        return bind_result(packet, self.request_id) if self.remote_owned else packet
