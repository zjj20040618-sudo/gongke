"""无需设备：验证首次顺序锁定、协议字节和请求门控。"""
import contextlib
import io
import itertools
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hostage_order import HostageOrder
from control_session import ControlSession
from protocol import build_hostage_order_packet, bind_result
from utils import crc16_ccitt


def obj(cid, x=10, score=.9, y=20, w=30, h=40):
    return SimpleNamespace(class_id=cid, score=score, x=x, y=y, w=w, h=h)


class HostageOrderTests(unittest.TestCase):
    def setUp(self):
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)
        self.order = HostageOrder()

    def test_all_six_encounter_orders_for_all_three_targets(self):
        for encounter in itertools.permutations((0, 1, 2)):
            for target in (0, 1, 2):
                with self.subTest(encounter=encounter, target=target):
                    self.order.reset(target)
                    for seen, cid in enumerate(encounter, 1):
                        self.order.observe([obj(cid)], 640, 480)
                        expected = encounter.index(target) + 1 if target in encounter[:seen] else 0
                        self.assertEqual(self.order.target_rank, expected)
                    self.assertEqual(self.order.order, list(encounter))

    def test_target_second_then_third_appears_without_renumbering(self):
        self.order.reset(2)
        for objects in ([obj(1)], [obj(2)], [], [obj(0), obj(2, 0), obj(1, 500)]):
            self.order.observe(objects, 640, 480)
        self.assertEqual(self.order.order, [1, 2, 0])
        self.assertEqual(self.order.target_rank, 2)
        self.order.observe([], 640, 480)
        self.assertEqual(self.order.target_rank, 2)

    def test_same_frame_new_shapes_use_left_to_right_best_valid_box(self):
        self.order.reset(2)
        self.order.observe([obj(2, 400), obj(1, 50), obj(0, 200), obj(2, 0, .5)], 640, 480)
        self.assertEqual(self.order.order, [1, 0, 2])
        self.order.observe([obj(2, 0), obj(0, 100), obj(1, 500)], 640, 480)
        self.assertEqual(self.order.order, [1, 0, 2])

    def test_existing_ranks_precede_new_shapes_even_if_new_shape_is_left(self):
        self.order.reset(2)
        self.order.observe([obj(1, 400)], 640, 480)
        self.order.observe([obj(2, 50), obj(0, 0)], 640, 480)
        self.assertEqual(self.order.order, [1, 0, 2])

    def test_invalid_or_nonhostage_detections_do_not_consume_a_rank(self):
        self.order.reset(2)
        invalid = [obj(9), obj(4), obj(1, score=.1), obj(2, score=float('nan')),
                   obj(0, score=1.2), obj(1, x=-1), obj(2, x=620), obj(0, w=0), obj(True)]
        self.order.observe(invalid, 640, 480)
        self.assertEqual(self.order.order, [])
        self.assertEqual(self.order.target_rank, 0)

    def test_ties_are_deterministic_and_input_order_independent(self):
        self.order.reset(2)
        self.order.observe([obj(2), obj(1), obj(0)], 640, 480)
        self.assertEqual(self.order.order, [0, 1, 2])

    def test_reset_starts_fresh_and_inactive_ignores_objects(self):
        self.order.reset(2)
        self.order.observe([obj(2)], 640, 480)
        self.order.reset(1)
        self.assertEqual(self.order.target_rank, 0)
        self.order.observe([obj(0)], 640, 480)
        self.assertEqual(self.order.order, [0])
        self.order.reset()
        self.order.observe([obj(1)], 640, 480)
        self.assertFalse(self.order.active)
        self.assertEqual(self.order.order, [])

    def test_packet_layout_empty_progress_and_complete_crc(self):
        for order, rank in (([], 0), ([1], 0), ([1, 2], 2), ([1, 2, 0], 2), ([2], 1)):
            with self.subTest(order=order):
                packet = build_hostage_order_packet(0x1234, 2, order)
                self.assertEqual(len(packet), 13)
                self.assertEqual(packet[:2], b'\xaa\x55')
                self.assertEqual(struct.unpack('<BHBBB3B', packet[2:-2]),
                                 (0x54, 0x1234, 2, rank, len(order), *(order + [255] * (3 - len(order)))))
                self.assertEqual(struct.unpack('<H', packet[-2:])[0], crc16_ccitt(packet[2:-2]))
                bound = bind_result(packet, 7)
                self.assertEqual(len(bound), 18)
                self.assertEqual(bound[2:7], b'\x62\x07\x00\x09\x00')
                self.assertEqual(bound[7:-2], packet[2:-2])
                self.assertEqual(struct.unpack('<H', bound[-2:])[0], crc16_ccitt(bound[2:-2]))

    def test_invalid_packet_state_is_rejected(self):
        for target, order in ((3, []), (True, []), (None, []), (2, [1, 1]),
                              (2, [0, 1, 2, 0]), (2, [3]), (2, [False])):
            with self.subTest(target=target, order=order), self.assertRaises(ValueError):
                build_hostage_order_packet(0, target, order)

    def test_golden_rank_packets_for_mcu_reference(self):
        expected = (([2], 'aa55620700090054010002010102ffffc6aa'),
                    ([1, 2], 'aa5562070009005401000202020102ff05e3'),
                    ([1, 0, 2], 'aa5562070009005401000203030100023067'))
        for order, hex_packet in expected:
            self.assertEqual(bind_result(build_hostage_order_packet(1, 2, order), 7),
                             bytes.fromhex(hex_packet))

    def test_uart_short_write_finishes_previous_packet_before_order(self):
        import test_hardware as uart_tests
        serial = uart_tests.FakeSerial((3, 0))
        link = uart_tests.hardware.UartLink(serial)
        from protocol import build_object_packet
        coordinates = bind_result(build_object_packet(0, [], 640, 480), 7)
        packet = bind_result(build_hostage_order_packet(0, 2, [2]), 7)
        self.assertFalse(link.send(coordinates))
        self.assertTrue(link.send(packet))
        self.assertEqual(bytes(serial.accepted), coordinates + packet)

    def test_order_packets_require_acked_rescue_request_not_qr_or_other_tasks(self):
        modes = SimpleNamespace(mode='IDLE')
        modes.enter = lambda mode: setattr(modes, 'mode', mode)
        modes.toggle = lambda: modes.enter('QR')
        control = ControlSession(modes)
        packet = build_hostage_order_packet(0, 2, [2])
        self.assertIsNone(control.result(packet))
        for request, mode, task, digit in ((1, 1, None, None), (2, 2, None, None),
                                          (3, 2, 1, 1), (4, 2, 3, 2)):
            ack, _ = control.apply(request, mode, task, digit)
            self.assertIsNone(control.result(packet))
            control.ack_sent(ack)
            self.assertEqual(control.result(packet), bind_result(packet, request) if task == 3 else None)
            if mode == 1:
                control.auto_object_after_qr()
                self.assertIsNone(control.result(packet))
        control.manual_toggle()
        self.assertIsNone(control.result(packet))
