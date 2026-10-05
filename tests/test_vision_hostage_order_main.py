"""真实主循环：进入人质区才排序；完整检测用于顺序，01仍筛目标。"""
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


def orders(sent):
    return [struct.unpack('<BHBBB3B', packet[7:-2])
            for packet in sent if packet[2] == 0x62 and packet[7] == 0x54]


class HostageOrderMainTests(unittest.TestCase):
    def run_loop(self, *args, **kwargs):
        return main_tests.MainLoopTests().run_loop(*args, **kwargs)

    def test_target_third_and_missing_target_keeps_rank_but_no_old_coordinates(self):
        sent, _, _, _ = self.run_loop([build_task_packet(1, 3, 2)] + [b''] * 4,
            detections=[[obj(1)], [obj(0)], [obj(2)], [], [obj(2, 300), obj(1, 500)]])
        rows = orders(sent)
        self.assertEqual([r[3] for r in rows], [0, 0, 3, 3, 3])
        self.assertEqual([r[1] for r in rows], list(range(5)))
        self.assertEqual(rows[-1][4:], (3, 1, 0, 2))
        positions = [p for p in sent if p[2] == 0x62 and p[7] == 1]
        self.assertEqual([p[10] for p in positions], [0, 0, 1, 0, 1])
        self.assertEqual([struct.unpack_from('<H', p, 8)[0] for p in positions], list(range(5)))
        self.assertEqual(struct.unpack_from('<H', positions[-1], 24)[0], 315)

    def test_target_second_before_and_after_third_is_seen(self):
        sent, _, _, _ = self.run_loop([build_task_packet(1, 3, 2)] + [b''] * 2,
            detections=[[obj(1)], [obj(2)], [obj(0), obj(2)]])
        self.assertEqual([r[3] for r in orders(sent)], [0, 2, 2])
        self.assertEqual(orders(sent)[-1][4:], (3, 1, 2, 0))

    def test_target_first_does_not_require_three_shapes_in_one_picture(self):
        sent, _, _, _ = self.run_loop([build_task_packet(1, 3, 2), b''],
            detections=[[obj(2)], [obj(1)]])
        self.assertEqual([r[3] for r in orders(sent)], [1, 1])

    def test_retry_preserves_order_new_request_resets_it_other_task_stops_it(self):
        sent, _, _, _ = self.run_loop([build_task_packet(1, 3, 2), build_task_packet(1, 3, 2),
            build_task_packet(2, 3, 2), build_task_packet(3, 4, 0), build_control_packet(4, 0)],
            detections=[[obj(1)], [obj(2)], [obj(2)], [obj(0)]])
        self.assertEqual([r[3] for r in orders(sent)], [0, 2, 1])
        self.assertEqual([struct.unpack_from('<H', p, 3)[0] for p in sent
                          if p[2] == 0x62 and p[7] == 0x54], [1, 1, 2])

    def test_dual_buffer_ignores_warmup_and_old_task_detection(self):
        sent, _, _, _ = self.run_loop([build_task_packet(1, 3, 2), b'',
            build_task_packet(2, 3, 2), b''],
            detections=[[obj(0), obj(1)], [obj(2)], [obj(1)], [obj(2)]], dual_buffer=True)
        self.assertEqual([r[3] for r in orders(sent)], [1, 1])
        self.assertEqual([r[4:] for r in orders(sent)], [(1, 2, 255, 255)] * 2)

    def test_ack_partial_blocks_ranking_and_short_order_write_resends_latched_rank(self):
        sent, captures, _, _ = self.run_loop([build_task_packet(1, 3, 2)] + [b''] * 3,
            detections=[[obj(2)], [], [obj(0)]], writes=[False, True, True, False, True, True, True, True])
        self.assertEqual(len(captures), 3)
        self.assertEqual([r[3] for r in orders(sent)], [1, 1])

    def test_manual_override_and_standalone_diagnostics_do_not_report_order(self):
        sent, _, _, _ = self.run_loop([build_task_packet(1, 3, 2), b'', b''],
            detections=[[obj(2)], [obj(1)]], qr_results=[[]], manual_toggles=(2, 3))
        self.assertEqual(len(orders(sent)), 1)
        standalone, _, _, _ = self.run_loop([b''] * 2, qr_results=[main_tests.MainLoopTests.qr('332')],
            detections=[[obj(2)]], start_mode='QR')
        self.assertTrue(all(p[2] != 0x54 for p in standalone))


if __name__ == '__main__':
    unittest.main(verbosity=2)
