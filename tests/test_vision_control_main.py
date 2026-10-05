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
    def run_loop(self, commands, qr_results=(), detections=(), writes=(), fail_object=False,
                 start_mode="IDLE", manual_toggles=None, displayed=None, taps=()):
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
                self.enter("OBJECT" if self.mode == "QR" else "QR")
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
                return True if manual_toggles is None else loops[0] in manual_toggles
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
        def draw_objects(img, objects, *args, **kwargs):
            if displayed is not None:
                displayed.append([obj.class_id for obj in kwargs["info_objects"]])
        from touch_inspector import ObjectInspector
        touch_points = iter(taps)
        def inspector_factory(display_size, enabled):
            inspector = ObjectInspector(display_size, enabled=False)
            inspector.poll = lambda: next(touch_points, None)
            return inspector
        replacements = {
            "maix": SimpleNamespace(app=SimpleNamespace(need_exit=need_exit),
                camera=SimpleNamespace(Camera=Camera), display=SimpleNamespace(),
                image=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888=0)),
                time=SimpleNamespace(ticks_ms=lambda: 0, sleep_ms=noop)),
            "hardware": SimpleNamespace(init_uart=lambda: Serial(), send_packet=send),
            "mode_controller": SimpleNamespace(ModeController=Modes),
            "qr_reader": SimpleNamespace(QrReader=lambda: SimpleNamespace(decode=lambda img: next(decoded, []))),
            "ui": SimpleNamespace(draw_header=noop, draw_objects=draw_objects, draw_qrs=noop),
            "user_button": SimpleNamespace(UserButton=Button),
            "touch_inspector": SimpleNamespace(ObjectInspector=inspector_factory),
        }
        with patch.dict(sys.modules, replacements), patch.multiple(config, DISPLAY_ENABLED=False, START_MODE=start_mode):
            spec = importlib.util.spec_from_file_location("test_camera_main", APP / "main.py")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            module.main()
        if manual_toggles is None:
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
        commands = [build_control_packet(1, 1), b"", build_control_packet(1, 1), b"", b"",
                    build_control_packet(2, 1), b"", b"", b"", build_control_packet(3, 0)]
        sent, captures, _, entered = self.run_loop(commands,
            [self.qr("123")] * 3 + [[], self.qr("321")] + [self.qr("222")] * 3 + [[]])
        qr_packets = [packet for packet in sent if packet[2] == 0x62]
        self.assertEqual([packet[11:-2] for packet in qr_packets], [b""] * 2 + [b"123"] * 3 + [b""] * 2 + [b"222"] * 2)
        self.assertTrue(all(len(packet) in (13, 16) and packet[7] == 0x53 for packet in qr_packets))
        self.assertEqual(captures, ["QR"] * 9)
        self.assertEqual(entered, ["IDLE", "QR", "QR", "IDLE"])

    def test_standalone_confirms_then_automatically_filters_three_classes(self):
        obj = lambda cid, score, x: SimpleNamespace(class_id=cid, score=score, x=x, y=20, w=30, h=40)
        candidates = [obj(cid, .8, 10) for cid in range(10)] + [obj(3, .95, 100)]
        displayed = []
        sent, captures, _, entered = self.run_loop([b""] * 5, [self.qr("331")] * 3,
            [candidates, []], start_mode="QR", manual_toggles=(), displayed=displayed)
        self.assertEqual(captures, ["QR"] * 3 + ["OBJECT"] * 2)
        self.assertEqual(entered, ["QR", "OBJECT"])
        self.assertEqual(sent[:3], [build_qr_packet(0, None), build_qr_packet(1, None), build_qr_packet(2, "331")])
        self.assertEqual(sent[3], build_object_packet(3, [candidates[-1], candidates[7], candidates[1]], 640, 480, 0, 7, 0))
        self.assertEqual(sent[4], build_object_packet(4, [], 640, 480, 0, 7, 0))
        self.assertEqual(displayed, [[1, 7, 3], []])  # 信息沿检测顺序，UART仍按任务顺序。

    def test_default_idle_user_enters_qr_and_unconfirmed_object_mode_sends_no_coordinates(self):
        objects = [SimpleNamespace(class_id=9, score=.9, x=10, y=20, w=30, h=40)]
        displayed = []
        sent, captures, _, entered = self.run_loop([b""] * 3, [[]], [objects, objects],
            manual_toggles=(1, 2), displayed=displayed)
        self.assertEqual(entered, ["IDLE", "QR", "OBJECT"])
        self.assertEqual(captures, ["QR", "OBJECT", "OBJECT"])
        self.assertEqual([p[5] for p in sent], [0, 0, 0])
        self.assertEqual(displayed, [[9], [9]])

    def test_manual_start_identifies_all_ten_classes_without_qr(self):
        objects = [SimpleNamespace(class_id=cid, score=.9, x=cid * 40, y=100, w=30, h=40)
                   for cid in range(10)]
        displayed = []
        sent, captures, _, _ = self.run_loop([b""], detections=[objects],
            start_mode="OBJECT", manual_toggles=(), displayed=displayed)
        self.assertEqual(displayed, [list(range(10))])
        self.assertEqual(captures, ["OBJECT"])
        self.assertEqual(sent, [build_object_packet(0, [], 640, 480, 0, 7, 0)])

    def test_touch_other_and_target_toggle_info_without_changing_uart(self):
        red = SimpleNamespace(class_id=4, score=.9, x=100, y=100, w=40, h=40)
        bucket = SimpleNamespace(class_id=9, score=.9, x=300, y=100, w=40, h=40)
        displayed = []
        # 640x480源图缩到480x320屏幕，左右黑边各约26.7像素。
        sent, _, _, _ = self.run_loop([build_task_packet(1, 4, 0)] + [b""] * 4,
            detections=[[red, bucket]] * 5, displayed=displayed,
            taps=[None, (107, 80), (107, 80), (240, 80), (240, 80)])
        self.assertEqual(displayed, [[9], [4, 9], [9], [], [9]])
        self.assertEqual([p for p in sent if p[2] == 0x62],
            [bind_result(build_object_packet(seq, [bucket], 640, 480, 0, 7, 0), 1)
             for seq in range(5)])

    def test_mcu_takes_over_manual_all_classes_and_restores_task_defaults(self):
        objects = [SimpleNamespace(class_id=cid, score=.9, x=cid * 40, y=100, w=30, h=40)
                   for cid in range(10)]
        displayed = []
        self.run_loop([b"", build_task_packet(1, 1, 1)], detections=[objects, objects],
            start_mode="OBJECT", manual_toggles=(), displayed=displayed)
        self.assertEqual(displayed, [list(range(10)), [4]])

    def test_user_return_to_qr_clears_old_task_before_confirming_another(self):
        objects = [SimpleNamespace(class_id=cid, score=.8, x=10, y=20, w=30, h=40) for cid in range(10)]
        sent, captures, _, entered = self.run_loop([b""] * 9,
            [self.qr("331")] * 3 + [[]] + [self.qr("123")] * 3,
            [objects, objects], start_mode="QR", manual_toggles=(5,))
        self.assertEqual(captures, ["QR"] * 3 + ["OBJECT"] + ["QR"] * 4 + ["OBJECT"])
        self.assertEqual(entered, ["QR", "OBJECT", "QR", "OBJECT"])
        self.assertEqual(sent[4], build_qr_packet(4, None))
        self.assertEqual(sent[-1], build_object_packet(8, [objects[4], objects[8], objects[0]], 640, 480, 0, 7, 0))

    def test_remote_request_wins_over_pending_standalone_auto_switch(self):
        sent, captures, _, entered = self.run_loop([b""] * 3 + [build_control_packet(1, 1), b""],
            [self.qr("331")] * 3 + [self.qr("123"), self.qr("123")], start_mode="QR", manual_toggles=())
        self.assertEqual(captures, ["QR"] * 5)
        self.assertEqual(entered, ["QR", "QR"])
        self.assertEqual(sent[-2:], [bind_result(build_qr_packet(3, None), 1), bind_result(build_qr_packet(4, None), 1)])

    def test_standalone_auto_load_failure_keeps_locked_qr_and_does_not_retry_forever(self):
        sent, captures, _, entered = self.run_loop([b""] * 5, [self.qr("331")] * 3 + [[], []],
            start_mode="QR", manual_toggles=(), fail_object=True)
        self.assertEqual(captures, ["QR"] * 5)
        self.assertEqual(entered, ["QR", "OBJECT"])
        self.assertEqual(sent[-2:], [build_qr_packet(3, "331"), build_qr_packet(4, "331")])

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
