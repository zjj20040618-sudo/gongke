"""Independent ISR and scheduling contracts for receive-only vision mode33.

Included in run_host_tests.ps1; this does not flash or drive hardware.
The C router fixture covers runtime state transitions. These checks protect the
integration boundary that a router-only fixture cannot exercise.
"""

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def c_code(text):
    """Mask comments and literals so braces/calls in messages are irrelevant."""
    tokens = re.compile(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'')
    return tokens.sub(lambda m: "".join("\n" if c == "\n" else " " for c in m[0]), text)


def functions(text):
    code = c_code(text)
    result = {}
    definitions = re.finditer(
        r"(?m)^[ \t]*(?:static\s+)?[A-Za-z_]\w*(?:\s+\*?\s*[A-Za-z_]\w*)*"
        r"\s+([A-Za-z_]\w*)\s*\([^;{}]*\)\s*\{",
        code,
    )
    for match in definitions:
        start = match.end()
        depth = 1
        end = start
        while depth and end < len(code):
            depth += (code[end] == "{") - (code[end] == "}")
            end += 1
        if depth:
            raise AssertionError(f"Unclosed function: {match[1]}")
        result[match[1]] = code[start:end - 1]
    return result


class VisionDiagIntegrationContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = {
            name: (ROOT / "App" / f"{name}.c").read_text(encoding="utf-8")
            for name in ("robot", "test", "proto", "steps")
        }
        cls.bodies = {}
        for text in cls.source.values():
            cls.bodies.update(functions(text))

    def test_frame_fanout_preserves_mission_consumer(self):
        init = self.bodies["robot_init"]
        registration = re.search(r"proto_set_on_frame\((\w+)\)", init)
        self.assertIsNotNone(registration, "No vision frame consumer registered")
        callback = self.bodies[registration[1]]
        self.assertRegex(callback, r"\bsteps_feed_frame\s*\(")
        self.assertRegex(callback, r"\btest_vision_feed_frame\s*\(")
        # The fan-out must feed mission frames even when the diagnostic is off.
        self.assertNotRegex(callback, r"\b(?:if|switch|return)\b")

    def test_frame_callback_call_graph_has_no_actions_or_tx(self):
        pending = ["robot_vision_frame", "test_vision_feed_frame"]
        visited = set()
        forbidden = re.compile(
            r"^(?:arm_|motion_|ctrl_|uart.*tx|HAL_UART_Transmit|bp_debug_send|"
            r"bp_laser_set|proto_send_scene|proto_service|send$|.*printf$|osDelay$)"
        )
        while pending:
            name = pending.pop()
            if name in visited:
                continue
            visited.add(name)
            self.assertIn(name, self.bodies)
            for called in re.findall(r"\b([A-Za-z_]\w*)\s*\(", self.bodies[name]):
                self.assertIsNone(forbidden.match(called), f"ISR path {name} calls {called}")
                if called in self.bodies:
                    pending.append(called)
        self.assertIn("steps_feed_frame", visited)

    def test_pending_stop_is_processed_before_diagnostic_progress(self):
        service = self.bodies["robot_bt_service"]
        self.assertLess(service.index("test_feed("), service.index("test_poll("))
        poll = self.bodies["test_poll"]
        self.assertLess(poll.index("flush_line("), poll.index("vision_diag_poll("))
        router = self.bodies["run_cmd"]
        self.assertLess(router.index("vision_diag_stop("), router.index("mode_g("))
        self.assertLess(router.index("vision_diag_stop("), router.index("cmd_select("))

    def test_boot_qr_request_follows_rx_setup_without_starting_motion(self):
        init = self.bodies["robot_init"]
        self.assertIn("proto_qr_begin(", init)
        self.assertLess(init.index("proto_set_on_frame("), init.index("uart_rx_ensure_all("))
        self.assertLess(init.index("test_init("), init.index("proto_qr_begin("))
        self.assertLess(init.index("uart_rx_ensure_all("), init.index("proto_qr_begin("))
        self.assertNotRegex(init, r"\b(?:mode_start|mission_start|route_seq_g|motion_vel_set(?:_precise)?)\s*\(")

    def test_scan_ok_notification_is_in_default_task_not_rx_isr(self):
        service = self.bodies["robot_bt_service"]
        self.assertIn("proto_qr_take_notice(", service)
        self.assertLess(service.index("proto_service("), service.index("proto_qr_take_notice("))
        self.assertLess(service.index("proto_qr_take_notice("), service.index("test_poll("))
        notice = self.bodies["proto_qr_take_notice"]
        self.assertNotRegex(notice, r"\b(?:bp_debug_send|send|printf|snprintf|HAL_UART_Transmit|osDelay)\s*\(")


if __name__ == "__main__":
    unittest.main(verbosity=2)
