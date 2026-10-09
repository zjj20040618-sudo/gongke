"""Actual main + UartLink short writes, then replay bytes through App/proto.c."""
import contextlib
import importlib.util
import io
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
import config
from protocol import build_control_packet, build_task_packet
from utils import crc16_ccitt


def load_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def detection(class_id, x=10):
    return SimpleNamespace(class_id=class_id, score=.9, x=x, y=20, w=30, h=40)


class UartMainTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Reuse the existing compiler/replay harness without importing its test
        # class into this module's discovery namespace.
        replay_module = load_file("uart_main_mcu_replay", APP / "tests/test_mcu_protocol.py")
        replay_class = replay_module.CurrentMcuProtocolTests
        replay_class.setUpClass()
        cls.addClassCleanup(replay_class.doClassCleanups)
        cls.mcu = replay_class()

    def run_loop(self, commands, detections, write_counts, dual_buffer=False):
        incoming, detected, counts = map(iter, (commands, detections, write_counts))
        turn, captures = [0], []

        def need_exit():
            turn[0] += 1
            return turn[0] > len(commands)

        class Serial:
            def __init__(self):
                self.wire = bytearray()
                self.chunks = []
                self.calls = []

            def read(self, **kwargs):
                if kwargs != {"len": 256, "timeout": 0}:
                    raise AssertionError("UART read must remain bounded and nonblocking")
                data = next(incoming)
                if isinstance(data, Exception):
                    raise data
                return data

            def write(self, data):
                count = next(counts, len(data))
                if not 0 <= count <= len(data):
                    raise AssertionError("test write count exceeds current suffix")
                self.calls.append((turn[0], data, count))
                if count:
                    chunk = data[:count]
                    self.wire.extend(chunk)
                    self.chunks.append((turn[0], chunk))
                return count

        serial = Serial()

        class Modes:
            def __init__(self, cam):
                self.mode = None
                self.detector = SimpleNamespace(detect=lambda img: (next(detected), 7))
                cam.modes = self

            def enter(self, mode):
                self.mode = mode

            def close(self):
                pass

        class Camera:
            def __init__(self, *args, **kwargs):
                pass

            def read(self, **kwargs):
                captures.append((turn[0], len(serial.wire), self.modes.mode))
                return Frame()

        class Frame:
            def width(self): return 640
            def height(self): return 480
            def copy(self): return Frame()

        noop = lambda *args, **kwargs: None
        fake_maix = SimpleNamespace(
            app=SimpleNamespace(need_exit=need_exit), camera=SimpleNamespace(Camera=Camera),
            display=SimpleNamespace(), image=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888=0)),
            time=SimpleNamespace(ticks_ms=lambda: 0, sleep_ms=noop),
            err=SimpleNamespace(), pinmap=SimpleNamespace(), uart=SimpleNamespace())
        replacements = {
            "maix": fake_maix,
            "mode_controller": SimpleNamespace(ModeController=Modes),
            "qr_reader": SimpleNamespace(QrReader=lambda: SimpleNamespace(decode=lambda img: [], roi=lambda img: [])),
            "ui": SimpleNamespace(draw_header=noop, draw_objects=noop, draw_qrs=noop, make_qr_preview=noop),
            "user_button": SimpleNamespace(UserButton=lambda: SimpleNamespace(
                take_toggle_request=lambda: False, close=noop)),
        }
        with patch.dict(sys.modules, replacements), \
             patch.multiple(config, DISPLAY_ENABLED=False, START_MODE="IDLE", UART_TRACE=False,
                 UART_WRITE_ATTEMPTS=8, UART_TRACE_EVERY_N_FRAMES=10, DUAL_BUFFER=dual_buffer), \
             contextlib.redirect_stdout(io.StringIO()):
            hardware = load_file("uart_main_hardware", APP / "hardware.py")
            with patch.dict(sys.modules, {"hardware": hardware}), \
                 patch.object(hardware, "init_uart", return_value=hardware.UartLink(serial)):
                main = load_file("uart_main_loop", APP / "main.py")
                main.main()
        return serial, captures

    def test_rx_exception_then_new_qr_request_still_acks_and_reports_qr(self):
        serial, captures = self.run_loop(
            [build_task_packet(1, 1, 1), OSError("temporary RX"),
             build_control_packet(2, 1), b"", build_control_packet(2, 1)],
            [[detection(4)], [detection(4)]], [])
        frames = self.frames(serial.wire)
        self.assertEqual([mode for _, _, mode in captures], ["OBJECT", "OBJECT", "QR", "QR", "QR"])
        self.assertEqual([packet[2] for packet in frames],
                         [0x61, 0x62, 0x62, 0x62, 0x62, 0x61, 0x62, 0x62, 0x61, 0x62])
        self.assertEqual([packet[7] for packet in frames if packet[2] == 0x62],
                         [0x01, 0x54, 0x01, 0x54, 0x53, 0x53, 0x53])
        qr_results = [packet for packet in frames if packet[2] == 0x62
                      and struct.unpack_from("<H", packet, 3)[0] == 2]
        self.assertEqual(len(qr_results), 3)
        self.assertTrue(all(packet[7] == 0x53 for packet in qr_results))
        timeline = [(0, "!11")]
        for turn in range(1, 6):
            if turn == 3:
                timeline.append((turn * 10 - 1, "@1"))
            timeline += [(turn * 10, chunk) for at, chunk in serial.chunks if at == turn]
        _, stats = self.mcu.replay(timeline)
        self.assertEqual(stats[3], 0)  # 当前真实C解析器的binary_bad计数。

    def frames(self, wire):
        frames, offset = [], 0
        while offset < len(wire):
            self.assertEqual(wire[offset:offset + 2], b"\xaa\x55")
            opcode = wire[offset + 2]
            self.assertIn(opcode, (0x61, 0x62))
            size = 9 if opcode == 0x61 else 9 + struct.unpack_from("<H", wire, offset + 5)[0]
            packet = bytes(wire[offset:offset + size])
            self.assertEqual(len(packet), size, "test ended with an unfinished UART tail")
            self.assertEqual(struct.unpack_from("<H", packet, size - 2)[0], crc16_ccitt(packet[2:-2]))
            frames.append(packet)
            offset += size
        return frames

    def test_dual_buffer_fresh_request_and_empty_frame_reach_actual_mcu(self):
        serial, _ = self.run_loop([build_task_packet(1, 1, 1), b"",
            build_task_packet(2, 4, 0), b"", b""],
            [[], [detection(4, 100)], [detection(4, 999)], [detection(9, 200)], []],
            [], dual_buffer=True)
        frames = self.frames(serial.wire)
        self.assertEqual([p[2] for p in frames], [0x61, 0x62, 0x62, 0x61, 0x62, 0x62])
        results = [p for p in frames if p[2] == 0x62 and p[7] == 0x01]
        self.assertEqual([p[10] for p in results], [1, 1, 0])
        self.assertEqual([p[21] for p in results if p[10]], [4, 9])
        self.assertEqual([p[10:16] for p in frames if p[2] == 0x62 and p[7] == 0x54],
                         [bytes((4, 1, 1, 4, 255, 255))])
        timeline = [(0, "!11")]
        for turn in range(1, 6):
            if turn == 3: timeline.append((turn * 10 - 1, "!40"))
            timeline += [(turn * 10, chunk) for chunk_turn, chunk in serial.chunks if chunk_turn == turn]
        lines, stats = self.mcu.replay(timeline)
        self.assertEqual([line for line in lines if line.startswith("OBJ,")],
            ["OBJ,0,0,115,40,30,40,90,0,640,480", "OBJ,3,0,215,40,30,40,90,1,640,480"])
        self.assertEqual(stats[:3], (3, 0, 3))
        self.assertEqual(stats[7], 0)

    def test_old_target_tail_and_new_empty_frame_have_unique_seq_and_both_reach_real_mcu(self):
        serial, captures = self.run_loop([build_control_packet(1, 2), b""],
            [[detection(9)], []], [9, 3, 0])
        frames = self.frames(serial.wire)
        self.assertEqual([packet[2] for packet in frames], [0x61, 0x62, 0x62])
        results = frames[1:]
        self.assertEqual([(struct.unpack_from("<H", packet, 8)[0], packet[10])
            for packet in results], [(0, 1), (1, 0)])
        self.assertEqual([capture[0] for capture in captures], [1, 2])
        lines, stats = self.mcu.replay([(0, "@2")] + [(turn * 10, chunk) for turn, chunk in serial.chunks])
        self.assertEqual([line for line in lines if line.startswith("OBJ,")],
            ["OBJ,3,0,25,40,30,40,90,0,640,480"])
        self.assertEqual(stats[:3], (2, 0, 2), "both target and empty object frames must be accepted")
        self.assertEqual(stats[7], 0, "new empty frame must not be mistaken for a duplicate")

    def test_repeated_short_writes_drain_old_tail_then_ack_before_fresh_capture_and_latest_target(self):
        # 靶任务不产生54；保留原来的单坐标长尾/新ACK时间线压力测试。
        commands = [build_task_packet(1, 2, 1), build_task_packet(2, 4, 0)] + [b""] * 5
        serial, captures = self.run_loop(commands,
            [[detection(6)], [detection(9, 100)], [detection(9, 200)], [detection(9, 300)]],
            [9, 3, 0, 5, 0, 4, 0, 22, 2, 0, 7, 4, 0, 6, 0])
        frames = self.frames(serial.wire)
        self.assertEqual([packet[2] for packet in frames], [0x61, 0x62, 0x61, 0x62, 0x62])
        self.assertEqual([struct.unpack_from("<H", packet, 3)[0] for packet in frames], [1, 1, 2, 2, 2])
        self.assertEqual([capture[0] for capture in captures], [1, 5, 6, 7])
        self.assertEqual(captures[1][1], sum(len(packet) for packet in frames[:3]),
            "first new-task capture must follow complete old tail and new ACK")
        results = [packet for packet in frames if packet[2] == 0x62]
        self.assertEqual([struct.unpack_from("<H", packet, 8)[0] for packet in results], [0, 1, 3])
        self.assertEqual(struct.unpack_from("<H", results[-1], 24)[0], 315,
            "the newest captured target must eventually reach the UART")
        timeline = [(0, "@2")]
        for turn in range(1, len(commands) + 1):
            if turn == 2:
                timeline.append((turn * 10 - 1, "@2"))
            timeline += [(turn * 10, chunk) for chunk_turn, chunk in serial.chunks if chunk_turn == turn]
        lines, stats = self.mcu.replay(timeline)
        objects = [line for line in lines if line.startswith("OBJ,")]
        self.assertEqual(objects, ["OBJ,3,0,115,40,30,40,90,1,640,480", "OBJ,3,0,315,40,30,40,90,3,640,480"])
        self.assertEqual(stats[:4], (2, 0, 2, 0))
        self.assertEqual(stats[7], 0)

    def test_ball_coordinate_and_rank_tails_finish_before_bucket_ack_and_capture(self):
        # 首轮01短写，追加54时先完整排旧01；54又短写，次轮先排54再ACK。
        serial, captures = self.run_loop([build_task_packet(1, 1, 1), build_task_packet(2, 4, 0)],
            [[detection(4)], [detection(9, 100)]], [9, 3, 0, 31, 3, 0])
        frames = self.frames(serial.wire)
        self.assertEqual([p[2] for p in frames], [0x61, 0x62, 0x62, 0x61, 0x62])
        self.assertEqual([p[7] for p in frames if p[2] == 0x62], [0x01, 0x54, 0x01])
        self.assertEqual([struct.unpack_from('<H', p, 3)[0] for p in frames], [1, 1, 1, 2, 2])
        self.assertEqual(frames[1][8:10], frames[2][8:10])
        self.assertEqual(frames[2][10:16], bytes((4, 1, 1, 4, 255, 255)))
        self.assertEqual(captures[1][1], sum(len(p) for p in frames[:4]))
        timeline = [(0, '!11')]
        for turn in (1, 2):
            if turn == 2:
                timeline.append((19, '!40'))
            timeline += [(turn * 10, chunk) for at, chunk in serial.chunks if at == turn]
        lines, stats = self.mcu.replay(timeline)
        self.assertEqual([line for line in lines if line.startswith('OBJ,')],
                         ['OBJ,0,0,25,40,30,40,90,0,640,480',
                          'OBJ,3,0,115,40,30,40,90,1,640,480'])
        self.assertEqual(stats[:4], (2, 0, 2, 0))
        self.assertEqual(stats[7], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
