"""Main-loop-only control. No Maix dependency, so transitions can be host-tested."""
from protocol import build_ack_packet, bind_result

class ControlSession:
    MODES = ("IDLE", "QR", "OBJECT")

    def __init__(self, modes):
        self.modes = modes
        self.request_id = None
        self.last_command = None
        self.last_ack = None
        self.remote_owned = False

    def apply(self, request_id, mode):
        command = (request_id, mode)
        if command == self.last_command:
            return self.last_ack, False  # lost ACK retry must not reload model or clear fresh results
        self.remote_owned = True
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
        actual_mode = self.MODES.index(self.modes.mode) if self.modes.mode in self.MODES else 0
        self.last_command = command
        self.last_ack = build_ack_packet(request_id, actual_mode, status)
        print("[CONTROL] request={} actual_mode={} status={}".format(request_id, self.MODES[actual_mode], status))
        return self.last_ack, True

    def result(self, packet):
        if self.modes.mode == "IDLE" or (self.remote_owned and self.request_id is None):
            return None
        return bind_result(packet, self.request_id) if self.remote_owned else packet
