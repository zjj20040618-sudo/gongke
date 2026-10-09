"""真实main、真实UART短写、真实C解析器回放；设备/模型用替身。"""
import struct
import unittest
import test_uart_main as fixture
from protocol import build_task_packet, build_control_packet


class BallOrderMainTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.UartMainTests.setUpClass()
        cls.addClassCleanup(fixture.UartMainTests.doClassCleanups)
        cls.harness = fixture.UartMainTests()

    def run_loop(self, commands, detections, writes=(), **kwargs):
        serial, captures = self.harness.run_loop(commands, detections, writes, **kwargs)
        packets = self.harness.frames(serial.wire)
        orders = [struct.unpack('<BHBBB3B', p[7:-2]) for p in packets
                  if p[2] == 0x62 and p[7] == 0x54 and p[10] in (3, 4, 5)]
        coordinates = [p for p in packets if p[2] == 0x62 and p[7] == 1]
        return serial, captures, packets, orders, coordinates

    def test_target_third_loss_preserves_rank_but_empty_coordinates_and_same_seq(self):
        serial, _, _, orders, positions = self.run_loop([build_task_packet(1, 1, 1)] + [b''] * 4,
            [[fixture.detection(5)], [fixture.detection(3)], [fixture.detection(4)], [],
             [fixture.detection(4, 300), fixture.detection(5, 500)]])
        self.assertEqual([r[3] for r in orders], [0, 0, 3, 3, 3])
        self.assertEqual(orders[-1][4:], (3, 5, 3, 4))
        self.assertEqual([r[1] for r in orders], list(range(5)))
        self.assertEqual([struct.unpack_from('<H', p, 8)[0] for p in positions], list(range(5)))
        self.assertEqual([p[10] for p in positions], [0, 0, 1, 0, 1])
        self.assertEqual([p[21] for p in positions if p[10]], [4, 4])
        self.assertEqual(struct.unpack_from('<H', positions[-1], 24)[0], 315)
        lines, stats = self.harness.mcu.replay([(0, '@2')] +
            [(turn * 10, data) for turn, data in serial.chunks] + [(100, '?')])
        self.assertIn('STATUS,1', lines)
        self.assertEqual([row for row in lines if row.startswith('OBJ,')],
            ['OBJ,0,0,25,40,30,40,90,2,640,480', 'OBJ,0,0,315,40,30,40,90,4,640,480'])
        self.assertEqual(stats[3], 0)  # 真实C的CRC/帧格式仍正确。
        self.assertGreater(stats[4], 0)  # 回放使用通用60 OBJECT；63编号接收由f976afb专项证明。

    def test_same_frame_all_colors_ordered_left_to_right_for_each_requested_color(self):
        for digit, target, rank in ((1, 4, 3), (2, 5, 1), (3, 3, 2)):
            _, _, _, orders, positions = self.run_loop([build_task_packet(1, 1, digit)],
                [[fixture.detection(4, 300), fixture.detection(3, 150), fixture.detection(5, 20)]])
            self.assertEqual(orders[0][2:], (target, rank, 3, 5, 3, 4))
            self.assertEqual(positions[0][10], 1)
            self.assertEqual(positions[0][21], target)

    def test_retry_keeps_order_new_request_resets_other_task_and_idle_stop_ball_order(self):
        _, _, packets, orders, _ = self.run_loop([build_task_packet(1, 1, 1),
            build_task_packet(1, 1, 1), build_task_packet(2, 1, 1),
            build_task_packet(3, 3, 2), build_control_packet(4, 0)],
            [[fixture.detection(5)], [fixture.detection(4)], [fixture.detection(4)],
             [fixture.detection(2), fixture.detection(3)]])
        self.assertEqual([r[3] for r in orders], [0, 2, 1])
        self.assertEqual([struct.unpack_from('<H', p, 3)[0] for p in packets
                          if p[2] == 0x62 and p[7] == 0x54 and p[10] in (3, 4, 5)], [1, 1, 2])
        self.assertEqual(len([p for p in packets if p[2] == 0x62 and p[7] == 0x54 and p[10] in (0, 1, 2)]), 1)

    def test_dual_buffer_warmup_and_previous_request_detections_do_not_number(self):
        _, _, _, orders, _ = self.run_loop([build_task_packet(1, 1, 1), b'',
            build_task_packet(2, 1, 1), b''],
            [[fixture.detection(5), fixture.detection(3)], [fixture.detection(4)],
             [fixture.detection(5)], [fixture.detection(4)]], dual_buffer=True)
        self.assertEqual([r[3] for r in orders], [1, 1])
        self.assertEqual([r[4:] for r in orders], [(1, 4, 255, 255)] * 2)

    def test_short_ack_blocks_capture_then_short_coordinate_tail_precedes_ball_order(self):
        serial, captures, packets, orders, positions = self.run_loop(
            [build_task_packet(1, 1, 1)] + [b''] * 2,
            [[fixture.detection(4)], []], [3, 0, 6, 3, 0])
        self.assertEqual([row[0] for row in captures], [2, 3])
        self.assertEqual([p[2] if p[2] == 0x61 else p[7] for p in packets],
                         [0x61, 1, 0x54, 1, 0x54])
        self.assertEqual([r[3] for r in orders], [1, 1])
        self.assertEqual([p[10] for p in positions], [1, 0])
        self.assertEqual(bytes(serial.wire), b''.join(packets))

    def test_manual_preview_pauses_order_return_resumes_and_general_request_has_no_order(self):
        _, _, _, orders, _ = self.run_loop([build_task_packet(1, 1, 1), b'', b''],
            [[fixture.detection(4)], [fixture.detection(5)]], manual_toggles=(2, 3))
        self.assertEqual(len(orders), 2)
        self.assertEqual([r[3] for r in orders], [1, 1])
        self.assertEqual(orders[-1][4:], (2, 4, 5, 255))
        _, _, _, orders, _ = self.run_loop([build_control_packet(1, 2)],
            [[fixture.detection(3), fixture.detection(4), fixture.detection(5)]])
        self.assertEqual(orders, [])

    def test_partial_old_order_completes_before_new_ack_and_new_order_resets(self):
        _, captures, packets, orders, positions = self.run_loop(
            [build_task_packet(1, 1, 1), build_task_packet(2, 1, 2), b''],
            [[fixture.detection(4)], [fixture.detection(5)]], [9, 34, 3, 0, 5, 0, 10, 9])
        self.assertEqual([row[0] for row in captures], [1, 3])
        self.assertEqual([p[2] if p[2] == 0x61 else p[7] for p in packets],
                         [0x61, 1, 0x54, 0x61, 1, 0x54])
        self.assertEqual([struct.unpack_from('<H', p, 3)[0] for p in packets], [1, 1, 1, 2, 2, 2])
        self.assertEqual([r[2:] for r in orders], [(4, 1, 1, 4, 255, 255), (5, 1, 1, 5, 255, 255)])
        self.assertEqual([r[1] for r in orders], [0, 1])
        self.assertEqual([p[21] for p in positions], [4, 5])


if __name__ == '__main__':
    unittest.main()
