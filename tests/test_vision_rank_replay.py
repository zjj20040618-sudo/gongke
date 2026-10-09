"""Actual camera main/producers -> actual MCU 54 parser, synthetic host only.

Fake devices come from the existing main-loop fixture; neither a real UART nor
physical ordering/grabbing is tested. BALL task1 and HOSTAGE task3 use54 in
separate class domains without changing the existing01 coordinate stream.
"""
import contextlib
import io
import itertools
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest

import test_vision_control_main as main_support
from protocol import (build_task_packet, build_ack_packet, build_control_packet,
                      build_hostage_order_packet, build_ball_order_packet,
                      build_object_packet, bind_result)
from control_session import ControlSession
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]


def obj(cid, x=10, y=20, score=.9):
    return SimpleNamespace(class_id=cid, score=score, x=x, y=y, w=30, h=40)


class VisionRankReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="eod-rank-replay-")
        cls.executable = Path(cls.temporary.name) / "rank.exe"
        compiler = os.environ.get("EOD_HOST_CC", "D:/mingw64/bin/gcc.exe")
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-I" + str(ROOT / "App"), "-I" + str(ROOT / "tests/stubs"),
                        str(ROOT / "tests/proto_rank_test.c"), "-o", str(cls.executable)],
                       check=True, capture_output=True)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def run_camera(self, commands, **kwargs):
        with contextlib.redirect_stdout(io.StringIO()):
            return main_support.MainLoopTests().run_loop(commands, **kwargs)[0]

    def replay(self, chunks):
        lines = ["{} {}\n".format(tick, chunk if isinstance(chunk, str) else chunk.hex())
                 for tick, chunk in chunks]
        completed = subprocess.run([str(self.executable), "--replay"],
            input="".join(lines).encode("ascii"), capture_output=True, check=True)
        output = completed.stdout.decode("ascii").splitlines()
        ranks = [tuple(map(int, line.split(",")[1:])) for line in output if line.startswith("RANK,")]
        counts = [tuple(map(int, line.split(",")[1:])) for line in output if line.startswith("COUNTERS,")]
        return ranks, counts

    def replay_camera(self, sent, task=3, digit=2, close=False):
        chunks, current, tick = [], None, 0
        for packet in sent:
            if packet[2] == 0x61:
                request = struct.unpack_from("<H", packet, 3)[0]
                if request != current:
                    chunks.append((tick, "!{}{}".format(task, digit)))
                    current = request
            chunks.append((tick, packet))
            if packet[2] == 0x62 and packet[7] == 0x54:
                self.assertEqual(len(packet), 18)
                self.assertEqual(struct.unpack_from("<H", packet, 5)[0], 9)
                chunks.append((tick, "?"))
            tick += 1
        if close:
            chunks += [(tick, "#"), (tick, "?")]
        if not chunks:
            chunks = [(0, "!{}{}".format(task, digit)), (1, "?")]
        return self.replay(chunks)

    def test_all_shapes_six_temporal_orders_same_sequence_01_54(self):
        selected = {1: 1, 2: 2, 3: 0}
        for digit, model in selected.items():
            for order in itertools.permutations((0, 1, 2)):
                with self.subTest(digit=digit, order=order):
                    sent = self.run_camera([build_task_packet(1, 3, digit)] + [b""] * 2,
                        detections=[[obj(cid)] for cid in order])
                    ranks, counts = self.replay_camera(sent, digit=digit)
                    wanted = order.index(model) + 1
                    self.assertEqual([r[6] for r in ranks],
                        [0 if i < wanted else wanted for i in range(1, 4)])
                    self.assertEqual(ranks[-1], (1, 3, digit, 1, 2, model, wanted, 3, *order))
                    self.assertEqual(counts[-1][:4], (1, 0, 3, 0))
                    self.assertEqual(counts[-1][4:7], (0, 0, 0))
                    coords = [p for p in sent if p[2] == 0x62 and p[7] == 1]
                    rank_packets = [p for p in sent if p[2] == 0x62 and p[7] == 0x54]
                    self.assertEqual([p[8:10] for p in coords], [p[8:10] for p in rank_packets])

    def test_same_frame_left_to_right_order_not_qr_or_target_class_order(self):
        for order in itertools.permutations((0, 1, 2)):
            with self.subTest(order=order):
                sent = self.run_camera([build_task_packet(1, 3, 2)],
                    detections=[[obj(cid, x=70 * i + 10) for i, cid in enumerate(order)]])
                ranks, counts = self.replay_camera(sent)
                self.assertEqual(ranks[-1][6:], (order.index(2) + 1, 3, *order))
                self.assertEqual(counts[-1][:3], (1, 0, 1))

    def test_target_lost_or_moved_does_not_change_latched_rank_or_emit_old_coords(self):
        sent = self.run_camera([build_task_packet(1, 3, 2)] + [b""] * 5,
            detections=[[obj(1)], [obj(2)], [], [obj(0)], [obj(2, 300), obj(1, 500)], []])
        ranks, counts = self.replay_camera(sent, close=True)
        self.assertEqual([r[6] for r in ranks[:-1]], [0, 2, 2, 2, 2, 2])
        self.assertEqual(ranks[-2][7:], (3, 1, 2, 0))
        self.assertEqual(counts[-2][0], 2)  # Only the two actually visible target coordinates.
        self.assertEqual(ranks[-1], (0,) * 11)
        self.assertEqual(counts[-1][2:7], (6, 0, 0, 0, 0))

    def test_retry_preserves_prefix_new_request_resets_it(self):
        sent = self.run_camera([build_task_packet(1, 3, 2), build_task_packet(1, 3, 2),
            build_task_packet(2, 3, 2), b""],
            detections=[[obj(1)], [obj(2)], [obj(2)], [obj(0)]])
        ranks, counts = self.replay_camera(sent)
        self.assertEqual([r[3] for r in ranks], [1, 1, 2, 2])
        self.assertEqual([r[6] for r in ranks], [0, 2, 1, 1])
        self.assertEqual(ranks[-1][7:], (2, 2, 0, 255))
        self.assertEqual(counts[-1][2:7], (4, 0, 0, 0, 0))

    def test_actual_dual_buffer_ignores_old_task_warmup(self):
        sent = self.run_camera([build_task_packet(1, 3, 2), b"",
            build_task_packet(2, 3, 2), b""],
            detections=[[obj(0), obj(1)], [obj(2)], [obj(1)], [obj(2)]], dual_buffer=True)
        ranks, counts = self.replay_camera(sent)
        self.assertEqual([r[6] for r in ranks], [1, 1])
        self.assertEqual([r[7:] for r in ranks], [(1, 2, 255, 255)] * 2)
        self.assertEqual(counts[-1][2:7], (2, 0, 0, 0, 0))

    def test_partial_ack_and_lost_rank_write_never_invent_missing_rank(self):
        sent = self.run_camera([build_task_packet(1, 3, 2)] + [b""] * 3,
            detections=[[obj(2)], [], [obj(0)]], writes=[False, True, True, False, True, True, True, True])
        ranks, counts = self.replay_camera(sent)
        self.assertEqual([r[6] for r in ranks], [1, 1])
        self.assertEqual(counts[-1][2], 2)
        self.assertEqual(counts[-1][4:7], (0, 0, 0))

    def test_session_requires_ack_and_matching_rank_domain(self):
        class Modes:
            mode = "IDLE"
            def enter(self, mode): self.mode = mode
        rank = build_hostage_order_packet(1, 2, [2])
        for task, digit in ((1, 1), (2, 1), (3, 2), (4, 0)):
            with self.subTest(task=task), contextlib.redirect_stdout(io.StringIO()):
                session = ControlSession(Modes())
                ack, _ = session.apply(1, 2, task, digit)
                self.assertIsNone(session.result(rank))
                session.ack_sent(ack)
                self.assertEqual(session.result(rank), bind_result(rank, 1) if task == 3 else None)
        with contextlib.redirect_stdout(io.StringIO()):
            session = ControlSession(Modes())
            ack, _ = session.apply(1, 2)
            session.ack_sent(ack)
            self.assertIsNone(session.result(rank))
        with self.assertRaises(ValueError):
            build_hostage_order_packet(1, 4, [4])
        with self.assertRaises(ValueError):
            build_ball_order_packet(1, 2, [2])
        for task, digit in ((1, 1), (2, 1), (3, 2), (4, 0)):
            with self.subTest(ball_task=task), contextlib.redirect_stdout(io.StringIO()):
                session = ControlSession(Modes())
                ack, _ = session.apply(1, 2, task, digit)
                ball_rank = build_ball_order_packet(1, 4, [3, 4])
                self.assertIsNone(session.result(ball_rank))
                session.ack_sent(ack)
                self.assertEqual(session.result(ball_rank), bind_result(ball_rank, 1) if task == 1 else None)

    def test_wrong_hostage54_refused_while_real_ball01_still_delivers(self):
        rank = bind_result(build_hostage_order_packet(8, 2, [2]), 1)
        for digit, model in ((1, 4), (2, 5), (3, 3)):
            with self.subTest(digit=digit):
                chunks = [(0, "!1{}".format(digit)), (1, build_ack_packet(1, 2)),
                    (2, rank), (3, "?"),
                    (4, bind_result(build_object_packet(8, [obj(model)], 640, 480), 1)), (5, "?")]
                ranks, counts = self.replay(chunks)
                self.assertEqual(ranks, [(0,) * 11] * 2)
                self.assertEqual(counts[0], (0, 0, 0, 0, 0, 0, 1, 0))
                self.assertEqual(counts[1], (1, 0, 0, 0, 0, 0, 1, 1))

    def test_ball_all_colors_six_temporal_orders_real_camera_and_parser(self):
        for digit, model in ((1, 4), (2, 5), (3, 3)):
            for order in itertools.permutations((3, 4, 5)):
                with self.subTest(digit=digit, order=order):
                    sent = self.run_camera([build_task_packet(1, 1, digit)] + [b""] * 2,
                        detections=[[obj(cid)] for cid in order])
                    ranks, counts = self.replay_camera(sent, task=1, digit=digit)
                    wanted = order.index(model) + 1
                    self.assertEqual([r[6] for r in ranks],
                        [0 if i < wanted else wanted for i in range(1, 4)])
                    self.assertEqual(ranks[-1], (1, 1, digit, 1, 2, model, wanted, 3, *order))
                    self.assertEqual(counts[-1][:4], (1, 0, 3, 0))
                    self.assertEqual(counts[-1][4:7], (0, 0, 0))
                    coords = [p for p in sent if p[2] == 0x62 and p[7] == 1]
                    rank_packets = [p for p in sent if p[2] == 0x62 and p[7] == 0x54]
                    self.assertEqual([p[8:10] for p in coords], [p[8:10] for p in rank_packets])

    def test_ball_real_producer_unknown_late_rank_wrap_and_request_mix(self):
        chunks = [(0, "!11"), (1, build_ack_packet(1, 2)),
            (2, bind_result(build_ball_order_packet(65534, 4, [3]), 1)), (3, "?"),
            (4, bind_result(build_object_packet(65535, [obj(4)], 640, 480), 1)), (5, "?"),
            (6, bind_result(build_ball_order_packet(65535, 4, [3, 4]), 1)), (7, "?"),
            (8, bind_result(build_ball_order_packet(0, 4, [3, 4, 5]), 1)), (9, "?"),
            (10, "!11"), (11, build_ack_packet(2, 2)),
            (12, bind_result(build_ball_order_packet(1, 4, [4]), 1)), (13, "?"),
            (14, bind_result(build_hostage_order_packet(1, 2, [2]), 2)), (15, "?"),
            (16, bind_result(build_ball_order_packet(1, 4, [4]), 2)), (17, "?"),
            (18, "#"), (19, bind_result(build_ball_order_packet(2, 4, [4]), 2)), (20, "?")]
        ranks, counts = self.replay(chunks)
        self.assertEqual([r[6] for r in ranks], [0, 0, 2, 2, 0, 0, 1, 0])
        self.assertEqual(ranks[3], (1, 1, 1, 1, 0, 4, 2, 3, 3, 4, 5))
        self.assertEqual(counts[3][0], 1)
        self.assertEqual(counts[3][2], 3)
        self.assertEqual(ranks[-2], (1, 1, 1, 2, 1, 4, 1, 1, 4, 255, 255))
        self.assertEqual(ranks[-1], (0,) * 11)

    def test_real_producer_preack_stale_request_duplicate_and_wrap(self):
        rank = build_hostage_order_packet(65535, 2, [1, 2])
        chunks = [(0, "!32"), (1, bind_result(rank, 1)), (2, "?"),
            (3, build_ack_packet(1, 2)), (4, bind_result(rank, 1)), (5, "?"),
            (6, bind_result(rank, 1)), (7, bind_result(build_hostage_order_packet(0, 2, [1, 2, 0]), 1)),
            (8, "?"), (9, "!32"), (10, build_ack_packet(2, 2)),
            (11, bind_result(build_hostage_order_packet(1, 2, [2]), 1)), (12, "?"),
            (13, bind_result(build_hostage_order_packet(1, 2, [2]), 2)), (14, "?")]
        ranks, counts = self.replay(chunks)
        self.assertEqual([r[0] for r in ranks], [0, 1, 1, 0, 1])
        self.assertEqual(ranks[2], (1, 3, 2, 1, 0, 2, 2, 3, 1, 2, 0))
        self.assertEqual(ranks[-1], (1, 3, 2, 2, 1, 2, 1, 1, 2, 255, 255))
        self.assertEqual(counts[-1][:7], (0, 0, 3, 1, 0, 0, 2))
        self.assertEqual(counts[-1][-1], 0)  # Historical54 cannot supply coordinate freshness.

    def test_documented_exact_wire_vectors_and_fragmented_delivery(self):
        for order, expected in (([2], "aa55620700090054010002010102ffffc6aa"),
                                ([1, 2], "aa5562070009005401000202020102ff05e3"),
                                ([1, 0, 2], "aa5562070009005401000203030100023067")):
            with self.subTest(order=order):
                packet = bind_result(build_hostage_order_packet(1, 2, order), 7)
                self.assertEqual(packet.hex(), expected)
                chunks = [(i, "!32") for i in range(7)] + [(8, build_ack_packet(7, 2))]
                chunks += [(9, bytes([byte])) for byte in packet] + [(10, "?")]
                ranks, counts = self.replay(chunks)
                self.assertEqual(ranks[-1][3:8], (7, 1, 2, order.index(2) + 1, len(order)))
                self.assertEqual(counts[-1][:7], (0, 0, 1, 0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
