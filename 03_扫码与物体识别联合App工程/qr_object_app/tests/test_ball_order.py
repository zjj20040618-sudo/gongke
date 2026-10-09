"""球首次出现序号：独立于QR颜色，与人质复用54线格式。"""
import contextlib
import io
import itertools
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hostage_order import BallOrder, HostageOrder
from control_session import ControlSession
from protocol import build_ball_order_packet, build_hostage_order_packet, bind_result
from utils import crc16_ccitt


def obj(cid, x=10, score=.9, y=20, w=30, h=40):
    return SimpleNamespace(class_id=cid, score=score, x=x, y=y, w=w, h=h)


class BallOrderTests(unittest.TestCase):
    def setUp(self):
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)
        self.order = BallOrder()

    def test_all_six_encounter_orders_for_each_qr_color(self):
        for encounter in itertools.permutations((3, 4, 5)):
            for target in (3, 4, 5):
                with self.subTest(encounter=encounter, target=target):
                    self.order.reset(target)
                    for seen, cid in enumerate(encounter, 1):
                        self.order.observe([obj(cid)], 640, 480)
                        expected = encounter.index(target) + 1 if target in encounter[:seen] else 0
                        self.assertEqual(self.order.target_rank, expected)
                    self.assertEqual(self.order.order, list(encounter))

    def test_lost_reappearing_or_moving_ball_does_not_reorder(self):
        self.order.reset(4)
        for objects in ([obj(5)], [obj(4)], [], [obj(3, 0), obj(4, 0), obj(5, 500)]):
            self.order.observe(objects, 640, 480)
        self.assertEqual(self.order.order, [5, 4, 3])
        self.assertEqual(self.order.target_rank, 2)
        self.order.observe([], 640, 480)
        self.assertEqual(self.order.target_rank, 2)

    def test_same_frame_sort_uses_all_ball_classes_and_best_valid_box(self):
        self.order.reset(4)
        self.order.observe([obj(4, 400), obj(3, 50), obj(5, 200), obj(4, 0, .5)], 640, 480)
        self.assertEqual(self.order.order, [3, 5, 4])
        self.assertEqual(self.order.target_rank, 3)
        self.order.observe([obj(4, 0), obj(3, 500)], 640, 480)
        self.assertEqual(self.order.order, [3, 5, 4])

    def test_new_ball_left_of_old_ball_is_appended_not_inserted(self):
        self.order.reset(4)
        self.order.observe([obj(3, 400)], 640, 480)
        self.order.observe([obj(4, 50), obj(5, 0)], 640, 480)
        self.assertEqual(self.order.order, [3, 5, 4])

    def test_invalid_nonball_and_bool_classes_do_not_consume_rank(self):
        self.order.reset(4)
        self.order.observe([obj(0), obj(1), obj(9), obj(3, score=.1), obj(4, score=float('nan')),
                            obj(5, score=1.2), obj(3, x=-1), obj(4, x=620), obj(5, w=0), obj(True)],
                           640, 480)
        self.assertEqual(self.order.order, [])
        self.assertEqual(self.order.target_rank, 0)

    def test_target_class_group_is_validated_and_reset_is_independent(self):
        for target in (None, 3, 4, 5):
            self.order.reset(target)
        for target in (0, 1, 2, 6, 9, True, 3.0):
            with self.subTest(target=target), self.assertRaises(ValueError):
                self.order.reset(target)
        self.order.reset(4)
        self.order.observe([obj(4)], 640, 480)
        hostage = HostageOrder()
        hostage.reset(2)
        hostage.observe([obj(2)], 640, 480)
        self.order.reset()
        self.order.observe([obj(3)], 640, 480)
        self.assertFalse(self.order.active)
        self.assertEqual(self.order.order, [])
        self.assertEqual(hostage.order, [2])

    def test_same_center_tie_is_deterministic_not_input_order(self):
        self.order.reset(4)
        self.order.observe([obj(5), obj(4), obj(3)], 640, 480)
        self.assertEqual(self.order.order, [3, 4, 5])

    def test_packets_keep_existing_nine_byte_inner_layout_crc_and_u16_sequence(self):
        for order in ([], [3], [3, 4], [5, 3, 4], [4]):
            with self.subTest(order=order):
                rank = order.index(4) + 1 if 4 in order else 0
                packet = build_ball_order_packet(0x1234, 4, order)
                self.assertEqual(len(packet), 13)
                self.assertEqual(struct.unpack('<BHBBB3B', packet[2:-2]),
                                 (0x54, 0x1234, 4, rank, len(order), *(order + [255] * (3 - len(order)))))
                bound = bind_result(packet, 7)
                self.assertEqual(len(bound), 18)
                self.assertEqual(bound[2:7], b'\x62\x07\x00\x09\x00')
                self.assertEqual(struct.unpack('<H', bound[-2:])[0], crc16_ccitt(bound[2:-2]))
        self.assertEqual(struct.unpack_from('<H', build_ball_order_packet(65536, 4, [4]), 3)[0], 0)

    def test_packets_reject_cross_group_duplicate_bool_and_too_many_slots(self):
        for target, order in ((0, []), (True, []), (None, []), (4, [3, 3]),
                              (4, [3, 4, 5, 3]), (4, [0]), (4, [False])):
            with self.subTest(target=target, order=order), self.assertRaises(ValueError):
                build_ball_order_packet(0, target, order)
        with self.assertRaises(ValueError):
            build_hostage_order_packet(0, 4, [4])

    def test_golden_ball_rank_packets_for_mcu_reference(self):
        vectors = (([4], 'aa55620700090054010004010104ffff8795'),
                   ([3, 4], 'aa5562070009005401000402020304ff22aa'),
                   ([3, 5, 4], 'aa556207000900540100040303030504821b'))
        for order, packet_hex in vectors:
            self.assertEqual(bind_result(build_ball_order_packet(1, 4, order), 7),
                             bytes.fromhex(packet_hex))

    def test_ack_request_target_group_and_manual_pause_gate_rank(self):
        modes = SimpleNamespace(mode='IDLE')
        modes.enter = lambda mode: setattr(modes, 'mode', mode)
        modes.toggle = lambda: modes.enter('QR')
        control = ControlSession(modes)
        packet = build_ball_order_packet(0, 4, [3, 4])
        mismatched = build_ball_order_packet(0, 5, [3, 5])
        hostage = build_hostage_order_packet(0, 2, [2])
        self.assertIsNone(control.result(packet))
        for request, mode, task, digit in ((1, 1, None, None), (2, 2, None, None),
                                          (3, 2, 4, 0), (4, 2, 2, 1),
                                          (5, 2, 3, 2), (6, 2, 1, 1)):
            ack, _ = control.apply(request, mode, task, digit)
            self.assertIsNone(control.result(packet))
            control.ack_sent(ack)
            self.assertEqual(control.result(packet), bind_result(packet, request) if task == 1 else None)
            self.assertIsNone(control.result(mismatched))
            self.assertEqual(control.result(hostage), bind_result(hostage, request) if task == 3 else None)
        control.manual_toggle()
        self.assertIsNone(control.result(packet))


if __name__ == '__main__':
    unittest.main(verbosity=2)
