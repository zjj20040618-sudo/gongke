"""MC2 real main/UART against the immutable teammate MCU f976afb parser.

Export only tracked C test inputs into an automatically cleaned temporary build
directory. Never replace the working MCU files or infer hardware acceptance.
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

import test_uart_main as fixture
from protocol import (bind_result, build_ack_packet, build_ball_order_packet,
                      build_hostage_order_packet, build_object_packet, build_task_packet)

MCU_REVISION = "f976afb8edd373feb30ca8834b219a16d59ac687"
ROOT = Path(__file__).resolve().parents[3]


class TeammateRankProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="mc2-teammate-rank-")
        cls.addClassCleanup(cls.temporary.cleanup)
        build = Path(cls.temporary.name)
        for name in ("App/proto.c", "App/proto.h", "tests/stubs/main.h",
                     "tests/proto_rank_test.c"):
            source = subprocess.run(["git", "show", MCU_REVISION + ":" + name],
                                    cwd=ROOT, capture_output=True, check=True).stdout
            target = build / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(source)
        compiler = os.environ.get("EOD_HOST_CC", "E:/setup/devc++/Dev-Cpp/MinGW64/bin/gcc.exe")
        cls.executable = build / "rank.exe"
        subprocess.run([compiler, "-std=c11", "-fuse-ld=bfd", "-Wall", "-Wextra", "-Werror",
                        "-I" + str(build / "App"), "-I" + str(build / "tests/stubs"),
                        str(build / "tests/proto_rank_test.c"), "-o", str(cls.executable)],
                       capture_output=True, check=True)
        fixture.UartMainTests.setUpClass()
        cls.addClassCleanup(fixture.UartMainTests.doClassCleanups)
        cls.harness = fixture.UartMainTests()

    def replay(self, chunks):
        lines = ["{} {}\n".format(tick, data if isinstance(data, str) else data.hex())
                 for tick, data in chunks]
        result = subprocess.run([str(self.executable), "--replay"],
                                input="".join(lines).encode("ascii"),
                                capture_output=True, check=True)
        output = result.stdout.decode("ascii").splitlines()
        ranks = [tuple(map(int, line.split(",")[1:])) for line in output
                 if line.startswith("RANK,")]
        counts = [tuple(map(int, line.split(",")[1:])) for line in output
                  if line.startswith("COUNTERS,")]
        return ranks, counts

    def run_camera(self, commands, detections, writes=(), **kwargs):
        with contextlib.redirect_stdout(io.StringIO()):
            serial, captures = self.harness.run_loop(commands, detections, writes, **kwargs)
        packets = self.harness.frames(serial.wire)
        requests = {struct.unpack_from("<H", cmd, 3)[0]: (cmd[5], cmd[6])
                    for cmd in commands if cmd and cmd[2] == 0x63}
        chunks = []
        current = None
        for tick, packet in enumerate(packets):
            request = struct.unpack_from("<H", packet, 3)[0]
            if packet[2] == 0x61 and request != current:
                task, digit = requests[request]
                chunks.append((tick, "!{}{}".format(task, digit)))
                current = request
            chunks.append((tick, packet))
            if packet[2] == 0x62 and packet[7] in (0x54, 0x55):
                chunks.append((tick, "?"))
        chunks.append((len(packets), "?"))
        ranks, counts = self.replay(chunks)
        return packets, ranks, counts, captures

    def test_teammate_parser_own_regressions(self):
        result = subprocess.run([str(self.executable)], capture_output=True, check=True)
        self.assertIn(b"request-bound54 rank:", result.stdout)

    def test_mc2_all_ball_colors_and_six_orders_are_received(self):
        for digit, target in ((1, 4), (2, 5), (3, 3)):
            for order in itertools.permutations((3, 4, 5)):
                with self.subTest(digit=digit, order=order):
                    packets, ranks, counts, _ = self.run_camera(
                        [build_task_packet(1, 1, digit), b"", b""],
                        [[fixture.detection(cid)] for cid in order])
                    self.assertEqual(ranks[-1], (1, 1, digit, 1, 2, target,
                                                order.index(target) + 1, 3, *order))
                    self.assertEqual(counts[-1][:7], (1, 0, 3, 0, 0, 0, 0))
                    coords = [p for p in packets if p[2] == 0x62 and p[7] == 1]
                    orders = [p for p in packets if p[2] == 0x62 and p[7] == 0x54]
                    self.assertEqual([p[8:10] for p in coords], [p[8:10] for p in orders])

    def test_mc2_hostage54_bytes_and_receiver_are_preserved(self):
        for digit, target in ((1, 1), (2, 2), (3, 0)):
            packets, ranks, counts, _ = self.run_camera(
                [build_task_packet(1, 3, digit)],
                [[fixture.detection(2, 300), fixture.detection(0, 150),
                  fixture.detection(1, 20)]])
            self.assertEqual(ranks[-1], (1, 3, digit, 1, 0, target,
                                        (1, 0, 2).index(target) + 1, 3, 1, 0, 2))
            self.assertEqual(counts[-1][:7], (1, 0, 1, 0, 0, 0, 0))
            self.assertTrue(all(p[7] in (1, 0x54) for p in packets if p[2] == 0x62))

    def test_retry_loss_and_new_request_keep_numbering_contract(self):
        _, ranks, counts, _ = self.run_camera(
            [build_task_packet(1, 1, 1), build_task_packet(1, 1, 1), b"",
             build_task_packet(2, 1, 2), b""],
            [[fixture.detection(5)], [fixture.detection(4)], [],
             [fixture.detection(5)], []])
        self.assertEqual([r[3] for r in ranks[:-1]], [1, 1, 1, 2, 2])
        self.assertEqual([r[6] for r in ranks[:-1]], [0, 2, 2, 1, 1])
        self.assertEqual(ranks[-1][7:], (1, 5, 255, 255))
        self.assertEqual(counts[-1][:7], (2, 0, 5, 0, 0, 0, 0))

    def test_dual_buffer_and_short_ack_do_not_report_old_or_unacked_rank(self):
        for commands, detections, writes, kwargs, expected in (
            ([build_task_packet(1, 1, 1), b"", build_task_packet(2, 1, 1), b""],
             [[fixture.detection(5), fixture.detection(3)], [fixture.detection(4)],
              [fixture.detection(5)], [fixture.detection(4)]], (), {"dual_buffer": True}, 2),
            ([build_task_packet(1, 1, 1), b"", b""],
             [[fixture.detection(4)], []], (3, 0, 6, 3, 0), {}, 2)):
            _, ranks, counts, _ = self.run_camera(commands, detections, writes, **kwargs)
            self.assertTrue(all(r[0] == 1 and r[6] == 1 for r in ranks))
            self.assertEqual(counts[-1][2:7], (expected, 0, 0, 0, 0))

    def test_partial_old_rank_tail_precedes_new_ack_and_does_not_contaminate(self):
        packets, ranks, counts, captures = self.run_camera(
            [build_task_packet(1, 1, 1), build_task_packet(2, 1, 2), b""],
            [[fixture.detection(4)], [fixture.detection(5)]],
            (9, 34, 3, 0, 5, 0, 10, 9))
        self.assertEqual([p[2] if p[2] == 0x61 else p[7] for p in packets],
                         [0x61, 1, 0x54, 0x61, 1, 0x54])
        self.assertEqual([r[3] for r in ranks[:-1]], [1, 2])
        self.assertEqual([r[6] for r in ranks], [1, 1, 1])
        self.assertEqual(counts[-1][:7], (2, 0, 2, 0, 0, 0, 0))
        self.assertEqual([row[0] for row in captures], [1, 3])

    def test_receiver_ack_request_domain_crc_and_sequence_are_independent(self):
        order = bind_result(build_ball_order_packet(0, 4, [4]), 1)
        coord = bind_result(build_object_packet(0, [fixture.detection(4)], 640, 480), 1)
        bad_crc = order[:-1] + bytes([order[-1] ^ 1])
        ranks, counts = self.replay([
            (0, "!11"), (1, order), (1, "?"),  # Before ACK.
            (2, build_ack_packet(1, 2)), (3, bad_crc), (3, "?"),
            (4, bind_result(build_ball_order_packet(0, 4, [4]), 2)), (4, "?"),
            (5, bind_result(build_hostage_order_packet(0, 1, [1]), 1)), (5, "?"),
            (6, order), (6, "?"), (7, coord), (7, "?"), (8, "#"), (8, "?")])
        self.assertTrue(all(r == (0,) * 11 for r in ranks[:4]))
        self.assertEqual(ranks[4:6], [(1, 1, 1, 1, 0, 4, 1, 1, 4, 255, 255)] * 2)
        self.assertEqual(counts[4][0], 0)  # Rank alone is not a coordinate callback.
        self.assertEqual(counts[4][-1], 0)  # Nor coordinate freshness.
        self.assertEqual(counts[5][0], 1)  # Same-seq01 still accepted after54.
        self.assertEqual(counts[5][-1], 1)
        self.assertEqual(ranks[-1], (0,) * 11)

    def test_unknown_zero_and_sequence_wrap_are_received(self):
        ranks, counts = self.replay([
            (0, "!11"), (0, build_ack_packet(1, 2)),
            (1, bind_result(build_ball_order_packet(65535, 4, [5]), 1)), (1, "?"),
            (2, bind_result(build_ball_order_packet(0, 4, [5, 4]), 1)), (2, "?")])
        self.assertEqual([r[6] for r in ranks], [0, 2])
        self.assertEqual(ranks[-1][4:], (0, 4, 2, 2, 5, 4, 255))
        self.assertEqual(counts[-1][:7], (0, 0, 2, 0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
