"""Current camera producer/session -> real MCU parser; host-only, no UART."""
import contextlib
import io
import itertools
import struct
import unittest

import test_vision_binary_replay as replay_support
from protocol import (build_ack_packet, build_control_packet, build_task_packet,
                      build_qr_packet, build_object_packet, bind_result, CommandReceiver)
from control_session import ControlSession
from task_selection import TaskSelection
from utils import crc16_ccitt


def bound_body(body, request=1):
    outer = struct.pack("<BHH", 0x62, request, len(body)) + body
    return b"\xaa\x55" + outer + struct.pack("<H", crc16_ccitt(outer))


class CurrentVisionReplayTests(unittest.TestCase):
    setUpClass = classmethod(replay_support.BinaryReplayTests.setUpClass.__func__)
    tearDownClass = classmethod(replay_support.BinaryReplayTests.tearDownClass.__func__)
    replay = replay_support.BinaryReplayTests.replay

    def test_all_27_qr53_choices_and_fragmented_delivery(self):
        for digits in itertools.product("123", repeat=3):
            payload = "".join(digits)
            packet = bind_result(build_qr_packet(42, payload), 1)
            with self.subTest(payload=payload):
                frames, stats = self.replay([(0, "@1"), (1, build_ack_packet(1, 1))]
                    + [(2, bytes([byte])) for byte in packet] + [(3, "?")])
                self.assertEqual(frames, ["TX," + build_control_packet(1, 1).hex(),
                    "QR,{},{},{},42".format(*digits), "STATUS,1"])
                self.assertEqual(stats, (1, 1, 0, 0, 0, 0, 0, 0))

    def test_qr_requires_matching_ack_and_three_actual_digits(self):
        frames, stats = self.replay([(0, "@1"),
            (1, bind_result(build_qr_packet(1, "123"), 1)),
            (2, build_ack_packet(2, 1)), (3, "?"),
            (4, build_ack_packet(1, 1)),
            (5, bind_result(build_qr_packet(2, None), 1)), (6, "?"),
            (7, bound_body(b"\x53\x03\x00\x01" + b"120")),
            (8, bound_body(b"\x53\x04\x00\x01" + b"12")),
            (9, bound_body(b"\x53\x05\x00\x01" + b"1234")),
            (10, bound_body(b"\x53\x06\x00\x02" + b"123")),
            (11, bind_result(build_qr_packet(7, "321"), 1))])
        self.assertEqual([f for f in frames if f.startswith("QR,")], ["QR,3,2,1,7"])
        self.assertEqual([f for f in frames if f.startswith("STATUS,")], ["STATUS,0", "STATUS,1"])
        self.assertEqual(stats[:5], (1, 1, 0, 0, 4))

    def test_crc_gap_duplicate_and_old_request_do_not_become_scan_success(self):
        packet = bind_result(build_qr_packet(10, "123"), 1)
        corrupt = bytearray(packet)
        corrupt[-1] ^= 1
        frames, stats = self.replay([(0, "@1"), (1, build_ack_packet(1, 1)),
            (2, bytes(corrupt)), (3, packet[:8]), (200, packet[8:]),
            (201, packet), (202, packet), (203, "#"), (204, "@1"),
            (205, build_ack_packet(2, 1)), (206, packet), (207, "?"),
            (208, bind_result(build_qr_packet(10, "231"), 2))])
        self.assertEqual([f for f in frames if f.startswith("QR,")], ["QR,1,2,3,10", "QR,2,3,1,10"])
        self.assertEqual([f for f in frames if f.startswith("STATUS,")], ["STATUS,0"])
        self.assertEqual(stats[0:4], (2, 2, 0, 1))
        self.assertEqual(stats[5], 1)
        self.assertEqual(stats[7], 1)

    def test_local_end_ignores_stream_without_sending_camera_stop(self):
        ball = replay_support.detection(4)
        frames, stats = self.replay([(0, "!11"), (1, build_ack_packet(1, 2)),
            (2, bind_result(build_object_packet(1, [ball], 320, 320), 1)),
            (3, "#"), (4, build_ack_packet(1, 2)),
            (5, bind_result(build_object_packet(2, [ball], 320, 320), 1)),
            (600, "~"), (601, "?"), (602, "!40"),
            (603, build_ack_packet(2, 2)),
            (604, bind_result(build_object_packet(3, [ball], 320, 320), 1)),
            (605, bind_result(build_object_packet(1, [replay_support.detection(9)], 320, 320), 2))])
        self.assertEqual([f for f in frames if f.startswith("TX,")],
            ["TX," + build_task_packet(1, 1, 1).hex(), "TX," + build_task_packet(2, 4, 0).hex()])
        self.assertEqual([f for f in frames if f.startswith("OBJ,")],
            ["OBJ,0,0,25,40,30,40,88,1,320,320", "OBJ,3,0,25,40,30,40,88,1,320,320"])
        self.assertIn("STATUS,0", frames)
        self.assertEqual(stats[:5], (2, 0, 2, 0, 0))

    def test_all_qr_choices_drive_real_camera_four_request_handshake(self):
        class Modes:
            mode = "IDLE"
            def enter(self, name):
                self.mode = name

        for a, b, c in itertools.product(range(1, 4), repeat=3):
            with self.subTest(qr=(a, b, c)), contextlib.redirect_stdout(io.StringIO()):
                session = ControlSession(Modes())
                receiver = CommandReceiver()
                chunks = [(0, "@1")]
                self.assertEqual(receiver.feed(build_control_packet(1, 1)), [(1, 1)])
                ack, changed = session.apply(1, 1)
                self.assertTrue(changed)
                self.assertIsNone(session.result(build_qr_packet(1, "{}{}{}".format(a, b, c))))
                session.ack_sent(ack)
                chunks += [(1, ack), (2, session.result(build_qr_packet(1, "{}{}{}".format(a, b, c)))), (3, "#")]
                expected_objects = []
                for offset, (task, digit) in enumerate(((1, a), (4, 0), (2, b), (3, c)), start=2):
                    command = build_task_packet(offset, task, digit)
                    decoded = receiver.feed(command[:3]) + receiver.feed(command[3:])
                    self.assertEqual(decoded, [(offset, 2, task, digit)])
                    ack, changed = session.apply(*decoded[0])
                    self.assertTrue(changed)
                    self.assertIsNone(session.result(build_object_packet(1, [], 320, 320)))
                    retry, changed = session.apply(*decoded[0])
                    self.assertFalse(changed)
                    self.assertEqual(retry, ack)
                    session.ack_sent(ack)
                    objects = [replay_support.detection(i) for i in range(10)]
                    chosen = TaskSelection().select(objects, session.target_class_id)
                    self.assertEqual(len(chosen), 1)
                    cls = task - 1
                    label = 0 if task == 4 else digit + 2 if task == 3 else digit - 1
                    expected_objects.append("OBJ,{},{},25,40,30,40,88,1,320,320".format(cls, label))
                    tick = offset * 10
                    result = session.result(build_object_packet(1, chosen, 320, 320))
                    chunks += [(tick, "!{}{}".format(task, digit)), (tick + 1, ack),
                        (tick + 2, result), (tick + 3, "#"),
                        (tick + 4, session.result(build_object_packet(2, chosen, 320, 320)))]
                frames, stats = self.replay(chunks)
                self.assertEqual([f for f in frames if f.startswith("OBJ,")], expected_objects)
                self.assertEqual([f for f in frames if f.startswith("QR,")], ["QR,{},{},{},1".format(a, b, c)])
                self.assertEqual(len([f for f in frames if f.startswith("TX,")]), 5)
                self.assertEqual(stats, (5, 1, 4, 0, 0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
