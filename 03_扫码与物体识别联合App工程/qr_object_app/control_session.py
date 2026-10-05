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

    def apply(self, request_id, mode, task_id=None, qr_digit=None):
        command = (request_id, mode, task_id, qr_digit)
        if command == self.last_command:
            return self.last_ack, False  # lost ACK retry must not reload model or clear fresh results
        if self.last_command and request_id < self.last_command[0]:
            actual = self.MODES.index(self.modes.mode) if self.modes.mode in self.MODES else 0
            print("[CONTROL] stale request={} current={} rejected".format(request_id, self.last_command[0]))
            return build_ack_packet(request_id, actual, 1), False
        self.remote_owned = True
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
        except Exception as exc:
            status = 1
            print("[CONTROL] switch failed:", exc)
        actual_mode = self.MODES.index(self.modes.mode) if self.modes.mode in self.MODES else 0
        self.last_command = command
        self.last_ack = build_ack_packet(request_id, actual_mode, status)
        print("[CONTROL] request={} actual_mode={} task={} digit={} class={} status={}".format(request_id, self.MODES[actual_mode], task_id, qr_digit, self.target_class_id, status))
        return self.last_ack, True

    def ack_sent(self, packet):
        if packet == self.last_ack and self.request_id is not None:
            self.acknowledged = True

    def result(self, packet):
        if self.modes.mode == "IDLE" or (self.remote_owned and (self.request_id is None or not self.acknowledged)):
            return None
        return bind_result(packet, self.request_id) if self.remote_owned else packet
