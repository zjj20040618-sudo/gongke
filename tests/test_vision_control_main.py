"""Real main loop, fake devices: ACK gates capture and four tasks use fresh frames."""
import importlib.util
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

APP = Path(__file__).resolve().parents[1] / "03_扫码与物体识别联合App工程/qr_object_app"
sys.path.insert(0, str(APP))
import config
from protocol import build_control_packet, build_task_packet, build_ack_packet, bind_result, build_qr_packet, build_object_packet


class MainLoopTests(unittest.TestCase):
    def run_loop(self, commands, qr_results=(), detections=(), writes=(), fail_object=False):
        sent, captures, events, modes_entered = [], [], [], []
        incoming, decoded, detected, write_results = map(iter, (commands, qr_results, detections, writes))
        loops = [0]
        def need_exit():
            loops[0] += 1
            return loops[0] > len(commands)
        class Serial:
            def read(self, **kwargs):
                if kwargs != {"len": 256, "timeout": 0}:
                    raise AssertionError("UART read must be bounded and nonblocking")
                return next(incoming)
        class Modes:
            toggle_calls = 0
            def __init__(self, cam):
                self.mode = None
                self.detector = SimpleNamespace(detect=lambda img: (next(detected, []), 7))
                cam.modes = self
            def enter(self, mode):
                modes_entered.append(mode)
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
                events.append(("capture", self.modes.mode))
                return SimpleNamespace(width=lambda: 640, height=lambda: 480)
        class Button:
            def take_toggle_request(self):
                return True  # USER cannot override remote ownership.
            def close(self):
                pass
        def send(serial, packet):
            if packet is None:
                return False
            complete = next(write_results, True)
            events.append(("write", packet[2], complete))
            if complete:
                sent.append(packet)
            return complete
        def noop(*args):
            pass
        replacements = {
            "maix": SimpleNamespace(app=SimpleNamespace(need_exit=need_exit),
                camera=SimpleNamespace(Camera=Camera), display=SimpleNamespace(),
                image=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888=0)),
                time=SimpleNamespace(ticks_ms=lambda: 0, sleep_ms=noop)),
            "hardware": SimpleNamespace(init_uart=lambda: Serial(), send_packet=send),
            "mode_controller": SimpleNamespace(ModeController=Modes),
            "qr_reader": SimpleNamespace(QrReader=lambda: SimpleNamespace(decode=lambda img: next(decoded, []))),
            "ui": SimpleNamespace(draw_header=noop, draw_objects=noop, draw_qrs=noop),
            "user_button": SimpleNamespace(UserButton=Button),
        }
        with patch.dict(sys.modules, replacements), patch.object(config, "DISPLAY_ENABLED", False):
            spec = importlib.util.spec_from_file_location("test_camera_main", APP / "main.py")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            module.main()
        self.assertEqual(Modes.toggle_calls, 0)
        return sent, captures, events, modes_entered

    @staticmethod
    def qr(payload):
        return [{"payload": payload, "text": payload, "x": 10, "y": 20, "w": 30, "h": 40}]

    def test_ack_precedes_new_capture_and_idle_stops_processing(self):
        commands = [build_control_packet(i, mode) for i, mode in ((1, 1), (2, 2), (3, 0))]
        sent, captures, events, _ = self.run_loop(commands)
        self.assertEqual(captures, ["QR", "OBJECT"])
        self.assertEqual(events[0], ("write", 0x61, True))
        self.assertEqual(sent, [build_ack_packet(1, 1), bind_result(build_qr_packet(0, None), 1),
            build_ack_packet(2, 2), bind_result(build_object_packet(1, [], 640, 480, 0, 7, 0), 2),
            build_ack_packet(3, 0)])

    def test_failed_switch_pauses_results_and_reports_actual_mode(self):
        commands = [build_control_packet(i, mode) for i, mode in ((1, 1), (2, 2), (3, 0))]
        sent, captures, _, _ = self.run_loop(commands, fail_object=True)
        self.assertEqual(captures, ["QR"])
        self.assertEqual(sent, [build_ack_packet(1, 1), bind_result(build_qr_packet(0, None), 1),
            build_ack_packet(2, 1, 1), build_ack_packet(3, 0)])

    def test_qr_code_repeats_after_disappearance_retry_preserves_new_request_resets(self):
        commands = [build_control_packet(1, 1), b"", build_control_packet(1, 1), b"",
                    build_control_packet(2, 1), b"", build_control_packet(3, 0)]
        sent, captures, _, entered = self.run_loop(commands,
            [self.qr("123"), [], self.qr("321"), [], self.qr("222"), []])
        qr_packets = [packet for packet in sent if packet[2] == 0x62]
        self.assertEqual([packet[11:-2] for packet in qr_packets], [b"123"] * 4 + [b"222"] * 2)
        self.assertTrue(all(len(packet) == 16 and packet[7] == 0x53 for packet in qr_packets))
        self.assertEqual(captures, ["QR"] * 6)
        self.assertEqual(entered, ["IDLE", "QR", "QR", "IDLE"])

    def test_four_tasks_single_target_and_missing_bucket_never_reuses_coordinates(self):
        obj = lambda cid, score, x: SimpleNamespace(class_id=cid, score=score, x=x, y=30, w=20, h=40)
        objects = [obj(9, .9, 90), obj(4, .6, 40), obj(4, .95, 45), obj(8, .8, 80), obj(0, .9, 10), obj(3, .99, 30)]
        commands = [build_control_packet(1, 1), build_task_packet(2, 1, 1), build_task_packet(3, 4, 0),
                    b"", build_task_packet(4, 2, 2), build_task_packet(5, 3, 3), build_control_packet(6, 0)]
        sent, captures, _, _ = self.run_loop(commands, [self.qr("123")], [objects, objects, [], objects, objects])
        packets = [packet for packet in sent if packet[2] == 0x62 and packet[7] == 0x01]
        self.assertEqual([struct.unpack_from("<H", p, 3)[0] for p in packets], [2, 3, 3, 4, 5])
        self.assertEqual([p[10] for p in packets], [1, 1, 0, 1, 1])
        self.assertEqual([p[21] if p[10] else None for p in packets], [4, 9, None, 8, 0])
        self.assertEqual(struct.unpack_from("<H", packets[0], 24)[0], 55)
        self.assertEqual(struct.unpack_from("<HH", packets[2], 11), (640, 480))
        self.assertEqual(captures, ["QR"] + ["OBJECT"] * 5)

    def test_incomplete_ack_blocks_capture_until_full_write(self):
        commands = [build_control_packet(1, 1), b"", b"", build_control_packet(2, 0)]
        sent, captures, events, _ = self.run_loop(commands, [self.qr("123")], writes=[False, False, True, True, True])
        self.assertEqual(captures, ["QR"])
        self.assertEqual(events[:4], [("write", 0x61, False), ("write", 0x61, False), ("write", 0x61, True), ("capture", "QR")])
        self.assertEqual([packet[2] for packet in sent], [0x61, 0x62, 0x61])

    def test_stale_qr_request_does_not_return_to_qr_or_restore_old_task(self):
        bucket = SimpleNamespace(class_id=9, score=.9, x=20, y=30, w=10, h=10)
        red = SimpleNamespace(class_id=4, score=.9, x=40, y=50, w=10, h=10)
        sent, captures, _, _ = self.run_loop([build_control_packet(1, 1), build_task_packet(2, 1, 1),
            build_control_packet(1, 1), build_control_packet(3, 0)], [self.qr("123")], [[bucket, red], [bucket, red]])
        self.assertEqual(captures, ["QR", "OBJECT", "OBJECT"])
        self.assertIn(build_ack_packet(1, 2, 1), sent)
        packets = [p for p in sent if p[2] == 0x62 and p[7] == 0x01]
        self.assertEqual([p[21] for p in packets], [4, 4])

    def test_same_request_changed_content_and_bad_task_pause_until_fresh_request(self):
        commands = [build_control_packet(1, 1), build_task_packet(1, 4, 0), build_task_packet(2, 4, 1),
                    build_task_packet(3, 4, 0), build_control_packet(4, 0)]
        sent, captures, _, _ = self.run_loop(commands)
        self.assertEqual(captures, ["QR", "OBJECT"])
        self.assertIn(build_ack_packet(1, 1, 1), sent)
        self.assertIn(build_ack_packet(2, 1, 1), sent)


if __name__ == "__main__":
    unittest.main(verbosity=2)
