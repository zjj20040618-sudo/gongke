"""Real Maix packet writer -> real STM32 parser, synthetic host evidence only."""
import itertools
import os
from pathlib import Path
import random
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "03_扫码与物体识别联合App工程/qr_object_app"))
from protocol import (build_object_packet, build_qr_packet, build_control_packet,
                      build_ack_packet, bind_result, CommandReceiver)
from control_session import ControlSession
from utils import crc16_ccitt


def detection(class_id, score=0.875, x=10, y=20, w=30, h=40):
    return SimpleNamespace(class_id=class_id, score=score, x=x, y=y, w=w, h=h)


def qr(sequence, payload="123"):
    return build_qr_packet(sequence, [{"payload": payload, "x": 100, "y": 50, "w": 20, "h": 30}])


def objects(sequence, items, width=480, height=320):
    return build_object_packet(sequence, items, width, height, 8, 14, 25)


def recalculate_crc(packet):
    return bytes(packet[:-2]) + struct.pack("<H", crc16_ccitt(packet[2:-2]))


class BinaryReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="eod-binary-replay-")
        cls.executable = Path(cls.temporary.name) / "replay.exe"
        compiler = os.environ.get("EOD_HOST_CC", "D:/mingw64/bin/gcc.exe")
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-I" + str(ROOT / "App"), "-I" + str(ROOT / "tests/stubs"),
                        str(ROOT / "tests/vision_binary_replay.c"), str(ROOT / "App/proto.c"),
                        "-o", str(cls.executable)], check=True, capture_output=True)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def replay(self, chunks):
        if isinstance(chunks, bytes):
            chunks = [(0, chunks)]
        lines = []
        for tick, chunk in chunks:
            if isinstance(chunk, str):
                lines.append("{} {}\n".format(tick, chunk))
                continue
            for offset in range(0, len(chunk), 500):
                lines.append("{} {}\n".format(tick, chunk[offset:offset + 500].hex()))
        output = subprocess.run([str(self.executable)], input="".join(lines).encode(),
                                capture_output=True, check=True).stdout.decode().splitlines()
        stats = tuple(map(int, output[-1].split(",")[1:]))
        return output[:-1], stats

    def test_control_ack_then_fresh_qr_not_ack_alone(self):
        frames, _ = self.replay([(0, "@1"), (1, "?"),
            (2, bind_result(qr(1), 1)), (3, build_ack_packet(1, 1)), (4, "?"),
            (5, bind_result(build_qr_packet(2, []), 1)), (6, "?"),
            (7, bind_result(qr(3), 1))])
        self.assertEqual(frames, ["TX," + build_control_packet(1, 1).hex(),
            "STATUS,0", "STATUS,0", "STATUS,1", "QR,1,2,3,3"])

    def test_control_stale_legacy_wrong_kind_and_stop(self):
        frames, _ = self.replay([(0, "@2"), (1, build_ack_packet(1, 2)),
            (2, objects(1, [detection(4)])), (3, bind_result(qr(1), 1)),
            (4, bind_result(objects(2, [detection(4)]), 1)),
            (5, "@2"), (6, bind_result(objects(3, [detection(4)]), 1)),
            (7, build_ack_packet(1, 2)), (8, "?"), (9, build_ack_packet(2, 2)),
            (10, bind_result(objects(4, []), 2)), (11, "?"),
            (12, "@0"), (13, bind_result(objects(5, [detection(4)]), 2)),
            (14, build_ack_packet(3, 0)), (15, "?")])
        self.assertEqual([f for f in frames if f.startswith("OBJ,")],
                         ["OBJ,0,0,25,40,30,40,88,2,480,320"])
        self.assertEqual([f for f in frames if f.startswith("STATUS")],
                         ["STATUS,0", "STATUS,1", "STATUS,1"])
        self.assertFalse(any(f.startswith("QR,") for f in frames))

    def test_control_retry_exact_id_and_negative_ack(self):
        command = "TX," + build_control_packet(1, 1).hex()
        frames, _ = self.replay([(0, "@1"), (499, "~"), (500, "~"),
            (501, build_ack_packet(99, 1)), (502, "?"),
            (503, build_ack_packet(1, 0, 1)), (504, "?"), (1000, "~"),
            (1001, bind_result(qr(1), 1)), (1002, "@0"),
            (1003, build_ack_packet(2, 0)), (1004, "?")])
        self.assertEqual(frames, [command, command, "STATUS,0", "STATUS,-1",
            "TX," + build_control_packet(2, 0).hex(), "STATUS,1"])

    def test_control_bad_crc_length_and_wrong_actual_mode(self):
        corrupt = bytearray(build_ack_packet(1, 1)); corrupt[-1] ^= 1
        bad_length = recalculate_crc(b"\xaa\x55\x62\x01\x00\xff\xff\x00\x00")
        frames, stats = self.replay([(0, "@1"), (1, bytes(corrupt) + bad_length),
            (200, build_ack_packet(1, 2)), (201, "?"),
            (202, bind_result(qr(9), 1))])
        self.assertEqual(frames[-1], "STATUS,-1")
        self.assertFalse(any(f.startswith("QR,") for f in frames))
        self.assertGreaterEqual(stats[3], 1)

    def test_control_invalid_qr_is_fresh_but_not_task_success(self):
        frames, _ = self.replay([(0, "@1"), (1, build_ack_packet(1, 1)),
            (2, bind_result(qr(1, "12x"), 1)), (3, "?"),
            (4, bind_result(qr(2, "x" * 255), 1)), (5, bind_result(qr(3), 1))])
        self.assertEqual(frames, ["TX," + build_control_packet(1, 1).hex(),
            "STATUS,1", "QR,1,2,3,3"])

    def test_camera_command_parser_fragment_noise_and_crc(self):
        receiver = CommandReceiver()
        command = build_control_packet(7, 2)
        corrupt = bytearray(command); corrupt[-1] ^= 1
        self.assertEqual(receiver.feed(b"noise" * 1000 + bytes(corrupt) + command[:4]), [])
        self.assertLessEqual(len(receiver.buffer), 7)
        self.assertEqual(receiver.feed(command[4:] + build_control_packet(8, 0)), [(7, 2), (8, 0)])
        self.assertEqual(receiver.feed(build_control_packet(0, 1)), [])

    def test_camera_apply_idempotent_stop_and_failure(self):
        class Modes:
            mode = "IDLE"
            calls = 0
            fail = False
            def enter(self, mode):
                self.calls += 1
                if self.fail:
                    raise RuntimeError("synthetic model load failure")
                self.mode = mode
        modes = Modes(); control = ControlSession(modes)
        ack, changed = control.apply(1, 2)
        self.assertTrue(changed)
        self.assertEqual(ack, build_ack_packet(1, 2))
        self.assertEqual(control.result(objects(1, [])), bind_result(objects(1, []), 1))
        self.assertEqual(control.apply(1, 2), (ack, False))
        self.assertEqual(modes.calls, 1)
        self.assertTrue(control.apply(2, 2)[1])
        self.assertEqual(control.result(objects(2, [])), bind_result(objects(2, []), 2))
        self.assertEqual(control.apply(3, 0)[0], build_ack_packet(3, 0))
        self.assertIsNone(control.result(qr(3)))
        modes.fail = True
        self.assertEqual(control.apply(4, 1)[0], build_ack_packet(4, 0, 1))
        self.assertIsNone(control.result(qr(4)))
        self.assertEqual(control.apply(4, 1)[1], False)

    def test_all_27_qr_codes_fragmented_and_concatenated(self):
        packets = [qr(i, "".join(digits)) for i, digits in enumerate(itertools.product("123", repeat=3))]
        stream = b"".join(packets)
        frames, stats = self.replay([(0, stream[i:i + 3]) for i in range(0, len(stream), 3)])
        self.assertEqual(frames, ["QR,{},{},{},{}".format(*digits, i)
                                 for i, digits in enumerate(itertools.product("123", repeat=3))])
        self.assertEqual(stats[:3], (27, 27, 0))

    def test_class_mapping_actual_pixels_confidence_and_unknown_shapes(self):
        frames, stats = self.replay(objects(7, [detection(i) for i in range(9)]))
        mapping = [(2, 3), (0, 2), (0, 0), (0, 1), (1, 0), (1, 2), (1, 1)]
        self.assertEqual(frames, ["OBJ,{},{},25,40,30,40,88,7,480,320".format(*pair) for pair in mapping])
        self.assertEqual(stats[6], 2)  # model IDs 0 and 2 deliberately unmapped

    def test_same_class_uses_best_not_last_and_duplicates_do_not_count_twice(self):
        packet = objects(8, [detection(4, 0.9, x=30), detection(4, 0.6), detection(4, 0.9, x=60)])
        frames, stats = self.replay(packet + packet)
        self.assertEqual(frames, ["OBJ,0,0,45,40,30,40,90,8,480,320"])
        self.assertEqual(stats[7], 1)

    def test_empty_object_packet_and_sequence_wrap(self):
        frames, stats = self.replay(objects(65535, []) + qr(0))
        self.assertEqual(frames, ["QR,1,2,3,0"])
        self.assertEqual(stats[:3], (2, 1, 1))

    def test_invalid_qr_payloads_never_dispatch(self):
        payloads = ["", "12", "1234", "120", "1x3", "１２３", "x" * 255]
        stream = b"".join(qr(i, payload) for i, payload in enumerate(payloads)) + qr(99)
        frames, stats = self.replay(stream)
        self.assertEqual(frames, ["QR,1,2,3,99"])
        self.assertEqual(stats[4], len(payloads))

    def test_crc_corruption_noise_truncation_then_valid(self):
        bad = bytearray(qr(1)); bad[-1] ^= 1
        frames, stats = self.replay(b"noise\xaa" + bytes(bad) + qr(2)[:8] + qr(3) + qr(4))
        self.assertEqual(frames, ["QR,1,2,3,3", "QR,1,2,3,4"])
        self.assertGreaterEqual(stats[3], 2)

    def test_corrupt_counts_and_types_recover(self):
        bad_type = bytearray(qr(1)); bad_type[2] = 0x99
        bad_count = bytearray(objects(2, [])); bad_count[5] = 11
        qr_count = bytearray(qr(3)); qr_count[5] = 2
        frames, stats = self.replay(bytes(bad_type) + bytes(bad_count) + bytes(qr_count) + qr(4))
        self.assertEqual(frames, ["QR,1,2,3,4"])
        self.assertGreaterEqual(stats[4], 3)

    def test_invalid_geometry_confidence_and_class_drop_whole_frame(self):
        invalid = [detection(255), detection(4, 1.1), detection(4, w=0),
                   detection(4, x=480), detection(4, w=481)]
        stream = b"".join(objects(i, [detection(4), item]) for i, item in enumerate(invalid)) + qr(9)
        frames, stats = self.replay(stream)
        self.assertEqual(frames, ["QR,1,2,3,9"])
        self.assertEqual(stats[4], len(invalid))

    def test_gap_reset_and_tick_wrap(self):
        partial = qr(1, "x" * 255)[:7]
        frames, stats = self.replay([(0xfffffff0, partial), (0x100, qr(2))])
        self.assertEqual(frames, ["QR,1,2,3,2"])
        self.assertEqual(stats[5], 1)

    def test_deterministic_noise_then_gap_recovers(self):
        rng = random.Random(20261001)
        chunks = []
        for i in range(30):
            noise = bytes(rng.randrange(256) for _ in range(100))
            chunks.extend([(i * 1000, noise), (i * 1000 + 200, qr(i))])
        frames, _ = self.replay(chunks)
        self.assertEqual(frames, ["QR,1,2,3,{}".format(i) for i in range(30)])

    def test_crc_known_vector_and_valid_crc_unknown_payload_not_accepted(self):
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29b1)
        malformed = bytearray(qr(0)); malformed[7] = ord("x")
        frames, _ = self.replay(recalculate_crc(malformed) + qr(1))
        self.assertEqual(frames, ["QR,1,2,3,1"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
