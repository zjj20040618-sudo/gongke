"""真实视觉主循环：task1才编号全部球，01仍只发QR选择颜色。"""
import itertools
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_vision_control_main as main_tests
from protocol import build_task_packet, build_control_packet


def obj(cid, x=10):
    return SimpleNamespace(class_id=cid, score=.9, x=x, y=20, w=30, h=40)


def packets(sent, opcode):
    return [p for p in sent if p[2] == 0x62 and p[7] == opcode]


def orders(sent):
    return [struct.unpack('<BHBBB3B', packet[7:-2]) for packet in packets(sent, 0x54)]


class BallOrderMainTests(unittest.TestCase):
    def run_loop(self, *args, **kwargs):
        return main_tests.MainLoopTests().run_loop(*args, **kwargs)

    def test_each_qr_color_and_six_orders_send_rank_not_color_position(self):
        for digit, target in ((1, 4), (2, 5), (3, 3)):
            for encounter in itertools.permutations((3, 4, 5)):
                with self.subTest(digit=digit, encounter=encounter):
                    sent, _, _, _ = self.run_loop([build_task_packet(1, 1, digit)] + [b''] * 2,
                                                  detections=[[obj(cid)] for cid in encounter])
                    self.assertEqual(orders(sent)[-1][2:], (target, encounter.index(target) + 1, 3, *encounter))
                    positions = packets(sent, 0x01)
                    self.assertEqual([p[10] for p in positions], [int(cid == target) for cid in encounter])
                    self.assertEqual([p[21] for p in positions if p[10]], [target])

    def test_target_third_and_loss_keep_rank_but_not_old_coordinates(self):
        sent, _, _, _ = self.run_loop([build_task_packet(1, 1, 1)] + [b''] * 4,
            detections=[[obj(3)], [obj(5)], [obj(4)], [], [obj(4, 300), obj(3, 500)]])
        self.assertEqual([r[3] for r in orders(sent)], [0, 0, 3, 3, 3])
        self.assertEqual(orders(sent)[-1][4:], (3, 3, 5, 4))
        positions = packets(sent, 0x01)
        self.assertEqual([p[10] for p in positions], [0, 0, 1, 0, 1])
        self.assertEqual([struct.unpack_from('<H', p, 8)[0] for p in positions], list(range(5)))
        self.assertEqual([r[1] for r in orders(sent)], list(range(5)))
        self.assertEqual(struct.unpack_from('<H', positions[-1], 24)[0], 315)

    def test_same_frame_sort_observes_all_balls_not_only_requested_one(self):
        sent, _, _, _ = self.run_loop([build_task_packet(1, 1, 1)],
            detections=[[obj(4, 400), obj(3, 50), obj(5, 200), obj(0, 0), obj(9, 0)]])
        self.assertEqual(orders(sent)[0][2:], (4, 3, 3, 3, 5, 4))
        self.assertEqual(packets(sent, 0x01)[0][10], 1)
        self.assertEqual(packets(sent, 0x01)[0][21], 4)

    def test_retry_keeps_new_request_resets_ball_then_hostage_and_bucket_change_group(self):
        sent, _, _, _ = self.run_loop([build_task_packet(1, 1, 1), build_task_packet(1, 1, 1),
            build_task_packet(2, 1, 1), build_task_packet(3, 3, 2), build_task_packet(4, 4, 0),
            build_control_packet(5, 0)],
            detections=[[obj(3)], [obj(4)], [obj(4)], [obj(2), obj(3)], [obj(4), obj(9)]])
        self.assertEqual([r[2:4] for r in orders(sent)], [(4, 0), (4, 2), (4, 1), (2, 1)])
        self.assertEqual([struct.unpack_from('<H', p, 3)[0] for p in packets(sent, 0x54)], [1, 1, 2, 3])
        self.assertEqual(orders(sent)[-1][4:], (1, 2, 255, 255))

    def test_dual_buffer_warmup_and_previous_request_detections_do_not_rank(self):
        sent, _, _, _ = self.run_loop([build_task_packet(1, 1, 1), b'',
            build_task_packet(2, 1, 1), b''],
            detections=[[obj(3), obj(5)], [obj(4)], [obj(3)], [obj(4)]], dual_buffer=True)
        self.assertEqual([r[3] for r in orders(sent)], [1, 1])
        self.assertEqual([r[4:] for r in orders(sent)], [(1, 4, 255, 255)] * 2)

    def test_partial_ack_blocks_capture_ranking_and_failed_order_does_not_renumber(self):
        sent, captures, _, _ = self.run_loop([build_task_packet(1, 1, 1)] + [b''] * 3,
            detections=[[obj(4)], [], [obj(3)]], writes=[False, True, True, False, True, True, True, True])
        self.assertEqual(len(captures), 3)
        self.assertEqual([r[3] for r in orders(sent)], [1, 1])

    def test_qr_handoff_general_diagnostic_manual_and_standalone_do_not_rank(self):
        for commands, kwargs in (([build_control_packet(1, 2), b''], {'detections': [[obj(4)], [obj(5)]]}),
                                 ([b''] * 2, {'qr_results': [main_tests.MainLoopTests.qr('123')],
                                             'detections': [[obj(4)]], 'start_mode': 'QR'}),
                                 ([build_control_packet(1, 1), b''],
                                  {'qr_results': [main_tests.MainLoopTests.qr('123')],
                                   'detections': [[obj(4)]]})):
            with self.subTest(commands=commands):
                sent, _, _, _ = self.run_loop(commands, **kwargs)
                self.assertEqual(orders(sent), [])
        sent, _, _, _ = self.run_loop([build_task_packet(1, 1, 1), b'', b''],
            detections=[[obj(4)], [obj(3)]], qr_results=[[]], manual_toggles=(2, 3))
        self.assertEqual(len(orders(sent)), 1)

    def test_invalid_new_task_stops_rank_and_stale_task_does_not_reset_current_rank(self):
        sent, _, _, _ = self.run_loop([build_task_packet(2, 1, 1), build_task_packet(1, 3, 2),
                                       build_task_packet(3, 1, 0), b''],
            detections=[[obj(3)], [obj(4)]])
        self.assertEqual([r[3] for r in orders(sent)], [0, 2])
        self.assertEqual([struct.unpack_from('<H', p, 3)[0] for p in packets(sent, 0x54)], [2, 2])


if __name__ == '__main__':
    unittest.main(verbosity=2)
