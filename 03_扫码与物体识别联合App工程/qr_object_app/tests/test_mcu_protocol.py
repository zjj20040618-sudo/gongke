"""Real Python packets to real App/proto.c; no UART or device evidence."""
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

APP_DIR = Path(__file__).resolve().parents[1]
ROOT = APP_DIR.parents[1]
sys.path.insert(0, str(APP_DIR))
from protocol import (bind_result, build_ack_packet, build_control_packet,
                      build_object_packet, build_qr_packet)


def detection(class_id, score=0.875, x=10):
    return SimpleNamespace(class_id=class_id, score=score, x=x, y=20, w=30, h=40)


class CurrentMcuProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="maix-task63-replay-")
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.executable = Path(cls.temporary.name) / "replay.exe"
        compiler = os.environ.get("EOD_HOST_CC", "E:/setup/devc++/Dev-Cpp/MinGW64/bin/gcc.exe")
        result = subprocess.run([compiler, "-std=c11", "-fuse-ld=bfd", "-Wall", "-Wextra", "-Werror",
            "-I" + str(ROOT / "App"), "-I" + str(ROOT / "tests/stubs"),
            str(ROOT / "tests/vision_binary_replay.c"), str(ROOT / "App/proto.c"),
            "-o", str(cls.executable)], capture_output=True)
        if result.returncode:
            raise RuntimeError(result.stderr.decode(errors="replace"))

    def replay(self, chunks):
        if isinstance(chunks, bytes):
            chunks = [(0, chunks)]
        lines = []
        for tick, chunk in chunks:
            if isinstance(chunk, str):
                lines.append("{} {}\n".format(tick, chunk))
            else:
                for offset in range(0, len(chunk), 500):
                    lines.append("{} {}\n".format(tick, chunk[offset:offset + 500].hex()))
        output = subprocess.run([str(self.executable)], input="".join(lines).encode(),
            capture_output=True, check=True).stdout.decode().splitlines()
        return output[:-1], tuple(map(int, output[-1].split(",")[1:]))

    def test_object01_all_ten_model_classes_and_full_geometry(self):
        packet = build_object_packet(7, [detection(i) for i in range(10)], 320, 320, 8, 14, 25)
        frames, stats = self.replay([(0, packet[:5]), (0, packet[5:])])
        mapping = ((2, 5), (2, 3), (2, 4), (0, 2), (0, 0), (0, 1), (1, 0), (1, 2), (1, 1), (3, 0))
        self.assertEqual(frames, ["OBJ,{},{},25,40,30,40,88,7,320,320".format(*pair) for pair in mapping])
        self.assertEqual(stats, (1, 0, 1, 0, 0, 0, 0, 0))

    def test_bound_object01_empty_frame_and_old_request_guard(self):
        first = build_object_packet(42, [detection(4), detection(9)], 320, 320)
        empty = build_object_packet(43, [], 320, 320)
        frames, stats = self.replay([(0, "@2"), (1, build_ack_packet(1, 2)),
            (2, bind_result(first, 1)), (3, bind_result(empty, 1)), (4, "?"),
            (5, "@2"), (6, build_ack_packet(2, 2)), (7, bind_result(first, 1)), (8, "?")])
        self.assertEqual([row for row in frames if row.startswith("OBJ,")], [
            "OBJ,0,0,25,40,30,40,88,42,320,320",
            "OBJ,3,0,25,40,30,40,88,42,320,320"])
        self.assertEqual([row for row in frames if row.startswith("STATUS,")], ["STATUS,1", "STATUS,0"])
        self.assertEqual(stats[:3], (2, 0, 2))

    def test_qr53_bound_frames_are_explicitly_rejected_by_current_mcu(self):
        frames, stats = self.replay([(0, "@1"), (1, build_ack_packet(1, 1)),
            (2, bind_result(build_qr_packet(1, "123"), 1)),
            (3, bind_result(build_qr_packet(2, None), 1)), (4, "?")])
        self.assertEqual(frames, ["TX," + build_control_packet(1, 1).hex(), "STATUS,0"])
        self.assertEqual(stats[:5], (0, 0, 0, 0, 2))

    def test_qr53_bare_rejection_does_not_poison_next_object01(self):
        packet = build_object_packet(3, [detection(9)], 320, 320)
        frames, stats = self.replay(build_qr_packet(1, "123") + build_qr_packet(2, None) + packet)
        self.assertEqual(frames, ["OBJ,3,0,25,40,30,40,88,3,320,320"])
        self.assertEqual(stats[:3], (1, 0, 1))
        self.assertGreaterEqual(stats[4], 2)

    def test_bad_object_crc_rejected_then_valid_frame_recovers(self):
        packet = build_object_packet(8, [detection(8)], 320, 320)
        damaged = bytearray(packet)
        damaged[-1] ^= 1
        frames, stats = self.replay(bytes(damaged) + packet)
        self.assertEqual(frames, ["OBJ,1,1,25,40,30,40,88,8,320,320"])
        self.assertEqual(stats[:4], (1, 0, 1, 1))


if __name__ == "__main__":
    unittest.main(verbosity=2)
