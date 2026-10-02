"""Real camera main loop with fake devices: command/ACK/capture/result ordering only."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

APP = Path(__file__).resolve().parents[1] / "03_扫码与物体识别联合App工程/qr_object_app"
sys.path.insert(0, str(APP))
import config
from protocol import build_control_packet, build_ack_packet, bind_result, build_qr_packet, build_object_packet

class MainLoopTests(unittest.TestCase):
    def run_loop(self, fail_object=False):
        sent, captures = [], []
        commands = iter(build_control_packet(i, mode) for i, mode in ((1, 1), (2, 2), (3, 0)))
        loops = [0]
        def need_exit():
            loops[0] += 1
            return loops[0] > 3
        class Serial:
            def read(self, **kwargs):
                self_args = {"len": 256, "timeout": 0}
                if kwargs != self_args:
                    raise AssertionError("UART read must be bounded and nonblocking")
                return next(commands)
        class Modes:
            toggle_calls = 0
            def __init__(self, cam):
                self.mode = None
                self.detector = SimpleNamespace(detect=lambda img: ([], 0))
                cam.modes = self
            def enter(self, mode):
                if fail_object and mode == "OBJECT":
                    raise RuntimeError("synthetic load error")
                self.mode = mode
            def toggle(self):
                Modes.toggle_calls += 1
            def close(self):
                pass
        class Camera:
            def __init__(self, *args):
                pass
            def read(self):
                captures.append(self.modes.mode)
                return SimpleNamespace(width=lambda: 480, height=lambda: 320)
        class Button:
            def take_toggle_request(self):
                return True  # USER must not override automatic ownership
            def close(self):
                pass
        def send(serial, packet):
            if packet is None:
                return False
            sent.append(packet)
            return True
        def noop(*args):
            pass
        replacements = {
            "maix": SimpleNamespace(app=SimpleNamespace(need_exit=need_exit),
                camera=SimpleNamespace(Camera=Camera), display=SimpleNamespace(),
                image=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888=0)),
                time=SimpleNamespace(ticks_ms=lambda: 0, sleep_ms=noop)),
            "hardware": SimpleNamespace(init_uart=lambda: Serial(), send_packet=send),
            "mode_controller": SimpleNamespace(ModeController=Modes),
            "qr_reader": SimpleNamespace(QrReader=lambda: SimpleNamespace(decode=lambda img: [])),
            "ui": SimpleNamespace(draw_header=noop, draw_objects=noop, draw_qrs=noop),
            "user_button": SimpleNamespace(UserButton=Button),
        }
        with patch.dict(sys.modules, replacements), patch.object(config, "DISPLAY_ENABLED", False):
            spec = importlib.util.spec_from_file_location("test_camera_main", APP / "main.py")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            module.main()
        self.assertEqual(Modes.toggle_calls, 0)
        return sent, captures

    def test_ack_precedes_new_capture_and_idle_stops_processing(self):
        sent, captures = self.run_loop()
        self.assertEqual(captures, ["QR", "OBJECT"])
        self.assertEqual(sent, [build_ack_packet(1, 1), bind_result(build_qr_packet(0, []), 1),
            build_ack_packet(2, 2), bind_result(build_object_packet(1, [], 480, 320, 0, 0, 0), 2),
            build_ack_packet(3, 0)])

    def test_failed_switch_does_not_send_old_mode_as_new_task(self):
        sent, captures = self.run_loop(fail_object=True)
        self.assertEqual(captures, ["QR"])
        self.assertEqual(sent, [build_ack_packet(1, 1), bind_result(build_qr_packet(0, []), 1),
            build_ack_packet(2, 1, 1), build_ack_packet(3, 0)])

if __name__ == "__main__":
    unittest.main(verbosity=2)
