"""球编号沿用人质首次出现规则；模型类别和站位编号不能混用。"""
import contextlib
import io
import itertools
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hostage_order import BallOrder
from protocol import build_ball_order_packet, build_hostage_order_packet, bind_result
from control_session import ControlSession
from utils import crc16_ccitt


def ball(cid, x=10, score=.9):
    return SimpleNamespace(class_id=cid, score=score, x=x, y=20, w=30, h=40)


class BallOrderTests(unittest.TestCase):
    def setUp(self):
        silence = contextlib.redirect_stdout(io.StringIO())
        silence.__enter__()
        self.addCleanup(silence.__exit__, None, None, None)

    def test_all_six_orders_and_each_requested_color(self):
        for order in itertools.permutations((3, 4, 5)):
            for target in order:
                tracker = BallOrder()
                tracker.reset(target)
                for cid in order:
                    tracker.observe([ball(cid)], 640, 480)
                tracker.observe([ball(cid, 500 - n * 100) for n, cid in enumerate(order)], 640, 480)
                self.assertEqual(tracker.order, list(order))
                self.assertEqual(tracker.target_rank, order.index(target) + 1)

    def test_same_frame_left_to_right_best_box_and_no_foreign_or_invalid_classes(self):
        tracker = BallOrder()
        tracker.reset(4)
        tracker.observe([ball(3, 200), ball(4, 400), ball(4, 50, .95), ball(5, 100),
                         ball(0), ball(6), ball(9), ball(3, -10, .99)], 640, 480)
        self.assertEqual(tracker.order, [4, 5, 3])
        self.assertEqual(tracker.target_rank, 1)

    def test_rank_latches_across_loss_and_resets_only_for_new_task(self):
        tracker = BallOrder()
        tracker.reset(4)
        tracker.observe([ball(5)], 640, 480)
        self.assertEqual(tracker.target_rank, 0)
        tracker.observe([ball(4)], 640, 480)
        tracker.observe([], 640, 480)
        self.assertEqual(tracker.target_rank, 2)
        tracker.reset(4)
        self.assertEqual(tracker.order, [])
        tracker.reset()
        tracker.observe([ball(4)], 640, 480)
        self.assertFalse(tracker.active)
        self.assertEqual(tracker.order, [])
        for target in (0, 1, 2, 6, True):
            with self.assertRaises(ValueError):
                tracker.reset(target)

    def test_packet_body_wrapper_crc_and_unused_slots(self):
        for order in ([], [5], [5, 4], [5, 4, 3], [4]):
            packet = build_ball_order_packet(0x1234, 4, order)
            rank = order.index(4) + 1 if 4 in order else 0
            self.assertEqual(len(packet), 13)
            self.assertEqual(struct.unpack('<BHBBB3B', packet[2:-2]),
                (0x54, 0x1234, 4, rank, len(order), *(order + [255] * (3 - len(order)))))
            bound = bind_result(packet, 7)
            self.assertEqual(len(bound), 18)
            self.assertEqual(bound[2:7], b'\x62\x07\x00\x09\x00')
            for item in (packet, bound):
                self.assertEqual(struct.unpack('<H', item[-2:])[0], crc16_ccitt(item[2:-2]))

    def test_invalid_orders_rejected_and_hostage_packet_unchanged(self):
        for target, order in ((2, []), (True, []), (4, [4, 4]), (4, [0]),
                              (4, [False]), (4, [3, 4, 5, 3])):
            with self.assertRaises(ValueError):
                build_ball_order_packet(0, target, order)
        self.assertEqual(bind_result(build_hostage_order_packet(1, 2, [1, 0, 2]), 7).hex(),
                         'aa5562070009005401000203030100023067')

    def test_golden_packets_for_mcu_first_second_and_third_ball(self):
        for order, expected in (([4], 'aa55620700090054010004010104ffff8795'),
                ([5, 4], 'aa5562070009005401000402020504ff8218'),
                ([5, 3, 4], 'aa5562070009005401000403030503048403')):
            self.assertEqual(bind_result(build_ball_order_packet(1, 4, order), 7).hex(), expected)

    def test_only_acked_ball_task_can_send_ball_order(self):
        modes = SimpleNamespace(mode='IDLE')
        modes.enter = lambda mode: setattr(modes, 'mode', mode)
        modes.toggle = lambda: modes.enter('QR')
        session = ControlSession(modes)
        packet = build_ball_order_packet(0, 4, [4])
        self.assertIsNone(session.result(packet))
        for request, mode, task, digit in ((1, 1, None, None), (2, 2, None, None),
                (3, 2, 2, 1), (4, 2, 3, 1), (5, 2, 4, 0), (6, 2, 1, 1)):
            ack, _ = session.apply(request, mode, task, digit)
            self.assertIsNone(session.result(packet))
            session.ack_sent(ack)
            self.assertEqual(session.result(packet), bind_result(packet, request) if task == 1 else None)
            if mode == 1:
                session.auto_object_after_qr()
                self.assertIsNone(session.result(packet))
        self.assertIsNone(session.result(build_hostage_order_packet(0, 1, [1])))
        session.manual_toggle()
        self.assertIsNone(session.result(packet))

    def test_shared54_checks_target_class_domain_and_packet_length(self):
        modes = SimpleNamespace(mode='IDLE')
        modes.enter = lambda mode: setattr(modes, 'mode', mode)
        session = ControlSession(modes)
        for request, task, digit, selected, other in (
                (1, 1, 1, build_ball_order_packet(0, 4, [4]),
                 (build_ball_order_packet(0, 5, [5]), build_hostage_order_packet(0, 1, [1]))),
                (2, 3, 2, build_hostage_order_packet(0, 2, [2]),
                 (build_hostage_order_packet(0, 1, [1]), build_ball_order_packet(0, 4, [4])))):
            ack, _ = session.apply(request, 2, task, digit)
            self.assertIsNone(session.result(selected))
            session.ack_sent(ack)
            self.assertEqual(session.result(selected), bind_result(selected, request))
            for wrong in (*other, selected[:-1], selected + b'\x00'):
                self.assertIsNone(session.result(wrong))

    def test_legacy55_is_not_emitted_or_bound(self):
        modes = SimpleNamespace(mode='IDLE')
        modes.enter = lambda mode: setattr(modes, 'mode', mode)
        session = ControlSession(modes)
        ack, _ = session.apply(1, 2, 1, 1)
        session.ack_sent(ack)
        legacy = bytearray(build_ball_order_packet(1, 4, [4]))
        legacy[2] = 0x55
        legacy[-2:] = struct.pack('<H', crc16_ccitt(legacy[2:-2]))
        self.assertIsNone(session.result(legacy))
        self.assertEqual(build_ball_order_packet(1, 4, [4])[2], 0x54)


if __name__ == '__main__':
    unittest.main()
