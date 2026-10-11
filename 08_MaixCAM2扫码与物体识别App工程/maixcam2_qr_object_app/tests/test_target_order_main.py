"""真实main/UartLink与真实旧C接收器；不改接收器来伪称靶54兼容。"""
import contextlib
import io
import itertools
import struct
import unittest
import test_uart_main as fixture
import config
from boot_session import boot_packet, wrap_packet
from protocol import build_task_packet, build_control_packet
from test_boot_session import C1, C2, S1, S2, frames as boot_frames
from utils import crc16_ccitt


def target(class_id, x=10):
    obj = fixture.detection(class_id, x)
    obj.score = (config.CONF_THRESHOLD + 1.0) / 2.0
    return obj


class TargetOrderMainTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.UartMainTests.setUpClass()
        cls.addClassCleanup(fixture.UartMainTests.doClassCleanups)
        cls.harness = fixture.UartMainTests()

    def run_loop(self, commands, detections, writes=(), **kwargs):
        serial, captures = self.harness.run_loop(commands, detections, writes, **kwargs)
        packets = self.harness.frames(serial.wire)
        orders = [struct.unpack("<BHBBB3B", p[7:-2]) for p in packets
                  if p[2] == 0x62 and p[7] == 0x54 and p[10] in (6, 7, 8)]
        coordinates = [p for p in packets if p[2] == 0x62 and p[7] == 1]
        return serial, captures, packets, orders, coordinates

    def test_target_third_loss_keeps_number_empty01_and_same_frame_sequence(self):
        serial, _, _, orders, coordinates = self.run_loop(
            [build_task_packet(1, 2, 1)] + [b""] * 4,
            [[target(8)], [target(7)], [target(6)], [],
             [target(6, 300), target(8, 500)]])
        self.assertEqual([row[3] for row in orders], [0, 0, 3, 3, 3])
        self.assertEqual(orders[-1][4:], (3, 8, 7, 6))
        self.assertEqual([row[1] for row in orders], list(range(5)))
        self.assertEqual([struct.unpack_from("<H", p, 8)[0] for p in coordinates], list(range(5)))
        self.assertEqual([p[10] for p in coordinates], [0, 0, 1, 0, 1])
        self.assertEqual([p[21] for p in coordinates if p[10]], [6, 6])
        self.assertEqual(struct.unpack_from("<H", coordinates[-1], 24)[0], 315)
        lines, stats = self.harness.mcu.replay([(0, "@2")] +
            [(turn * 10, data) for turn, data in serial.chunks] + [(100, "?")])
        self.assertIn("STATUS,1", lines)
        score = int((config.CONF_THRESHOLD + 1.0) / 2.0 * 100 + .5)
        self.assertEqual([row for row in lines if row.startswith("OBJ,")],
                         ["OBJ,1,0,25,40,30,40,{},2,640,480".format(score),
                          "OBJ,1,0,315,40,30,40,{},4,640,480".format(score)])
        self.assertEqual(stats[3], 0)
        self.assertGreater(stats[4], 0)  # 当前C解析器通用60也拒54；不是CRC损坏。

    def test_each_task2_digit_and_six_first_appearance_orders_produce_correct_rank(self):
        for digit, selected in ((1, 6), (2, 8), (3, 7)):
            for order in itertools.permutations((6, 7, 8)):
                with self.subTest(digit=digit, order=order):
                    _, _, _, orders, coordinates = self.run_loop(
                        [build_task_packet(1, 2, digit), b"", b""],
                        [[target(class_id)] for class_id in order])
                    self.assertEqual(orders[-1][2:],
                                     (selected, order.index(selected) + 1, 3, *order))
                    self.assertEqual([p[8:10] for p in coordinates],
                                     [struct.pack("<H", row[1]) for row in orders])
                    self.assertEqual([p[21] for p in coordinates if p[10]], [selected])

    def test_same_frame_all_targets_use_centers_not_color_or_qr_digit_order(self):
        for digit, selected, rank in ((1, 6, 3), (2, 8, 1), (3, 7, 2)):
            _, _, _, orders, coordinates = self.run_loop([build_task_packet(1, 2, digit)],
                [[target(6, 300), target(7, 150), target(8, 20)]])
            self.assertEqual(orders[0][2:], (selected, rank, 3, 8, 7, 6))
            self.assertEqual(coordinates[0][10], 1)
            self.assertEqual(coordinates[0][21], selected)

    def test_retry_keeps_number_new_request_resets_other_task_and_idle_stop_target54(self):
        _, _, packets, orders, _ = self.run_loop(
            [build_task_packet(1, 2, 1), build_task_packet(1, 2, 1),
             build_task_packet(2, 2, 1), build_task_packet(3, 1, 1),
             build_control_packet(4, 0)],
            [[target(8)], [target(6)], [target(6)], [target(4), target(6)]])
        self.assertEqual([row[3] for row in orders], [0, 2, 1])
        self.assertEqual([struct.unpack_from("<H", p, 3)[0] for p in packets
                          if p[2] == 0x62 and p[7] == 0x54 and p[10] in (6, 7, 8)], [1, 1, 2])
        self.assertEqual(len([p for p in packets if p[2] == 0x62
                              and p[7] == 0x54 and p[10] in (3, 4, 5)]), 1)

    def test_dual_buffer_warmup_and_previous_request_detections_do_not_allocate_rank(self):
        _, _, _, orders, _ = self.run_loop(
            [build_task_packet(1, 2, 1), b"", build_task_packet(2, 2, 1), b""],
            [[target(8), target(7)], [target(6)], [target(8)], [target(6)]], dual_buffer=True)
        self.assertEqual([row[3] for row in orders], [1, 1])
        self.assertEqual([row[4:] for row in orders], [(1, 6, 255, 255)] * 2)

    def test_short_ack_blocks_capture_and_coordinate_tail_precedes_order_same_seq(self):
        serial, captures, packets, orders, coordinates = self.run_loop(
            [build_task_packet(1, 2, 1)] + [b""] * 2,
            [[target(6)], []], [3, 0, 6, 3, 0])
        self.assertEqual([row[0] for row in captures], [2, 3])
        self.assertEqual([p[2] if p[2] == 0x61 else p[7] for p in packets],
                         [0x61, 1, 0x54, 1, 0x54])
        self.assertEqual([row[3] for row in orders], [1, 1])
        self.assertEqual([p[10] for p in coordinates], [1, 0])
        self.assertEqual([p[8:10] for p in coordinates],
                         [struct.pack("<H", row[1]) for row in orders])
        self.assertEqual(bytes(serial.wire), b"".join(packets))

    def test_manual_preview_pauses_numbering_return_preserves_history_general_has_no54(self):
        _, _, _, orders, _ = self.run_loop([build_task_packet(1, 2, 1), b"", b""],
            [[target(8)], [target(6)]], manual_toggles=(2, 3))
        self.assertEqual([row[3] for row in orders], [0, 2])
        self.assertEqual(orders[-1][4:], (2, 8, 6, 255))
        for command in (build_control_packet(1, 2), build_task_packet(1, 4, 0)):
            _, _, _, orders, _ = self.run_loop([command],
                [[target(6), target(7), target(8), target(9)]])
            self.assertEqual(orders, [])

    def test_manual_valid_qr_auto_resume_preserves_target_request_and_order(self):
        qr = {"payload": "123", "text": "123", "x": 0, "y": 0, "w": 20, "h": 20}
        serial, captures, _, orders, coordinates = self.run_loop(
            [build_task_packet(6, 2, 1), b"", b""],
            [[target(8)], [target(6)]], manual_toggles=(2,), qr_frames=([qr],))
        self.assertEqual([row[2] for row in captures], ["OBJECT", "QR", "OBJECT"])
        self.assertEqual([row[3] for row in orders], [0, 2])
        self.assertEqual(orders[-1][4:], (2, 8, 6, 255))
        self.assertTrue(all(struct.unpack_from("<H", p, 3)[0] == 6 for p in coordinates))
        self.assertFalse(any(at == 2 for at, _ in serial.chunks))

    def test_partial_old_order_finishes_before_new_ack_and_new_order_is_reset(self):
        _, captures, packets, orders, coordinates = self.run_loop(
            [build_task_packet(1, 2, 1), build_task_packet(2, 2, 2), b""],
            [[target(6)], [target(8)]], [9, 34, 3, 0, 5, 0, 10, 9])
        self.assertEqual([row[0] for row in captures], [1, 3])
        self.assertEqual([p[2] if p[2] == 0x61 else p[7] for p in packets],
                         [0x61, 1, 0x54, 0x61, 1, 0x54])
        self.assertEqual([struct.unpack_from("<H", p, 3)[0] for p in packets], [1, 1, 1, 2, 2, 2])
        self.assertEqual([row[2:] for row in orders],
                         [(6, 1, 1, 6, 255, 255), (8, 1, 1, 8, 255, 255)])
        self.assertEqual([row[1] for row in orders], [0, 1])
        self.assertEqual([p[21] for p in coordinates], [6, 8])

    def test_new_boot_session_resets_target_order_sequence_and_old_request_authority(self):
        commands = [boot_packet(0x64, C1), boot_packet(0x67, C1, S1),
                    wrap_packet(build_task_packet(1, 2, 1), C1, S1), b"",
                    boot_packet(0x64, C2), boot_packet(0x67, C2, S2),
                    wrap_packet(build_task_packet(1, 2, 1), C1, S1),
                    wrap_packet(build_task_packet(1, 2, 1), C2, S2),
                    boot_packet(0x67, C2, S2)]
        nonces = iter((S1, S2))
        serial, captures = self.harness.run_loop(commands,
            [[target(7)], [target(6)], [target(8)], [target(6)], [target(6)]], (),
            boot_enabled=True, boot_random=lambda size: next(nonces))
        packets = boot_frames(serial.wire)
        for packet in packets:
            self.assertEqual(struct.unpack("<H", packet[-2:])[0], crc16_ccitt(packet[2:-2]))
        results = [p for p in packets if p[2] == 0x66 and p[21] == 0x62]
        orders = [struct.unpack("<BHBBB3B", p[26:-2]) for p in results if p[26] == 0x54]
        self.assertEqual([row[1] for row in orders], [0, 1, 2, 0, 1])
        self.assertEqual([row[3] for row in orders], [0, 2, 2, 1, 1])
        self.assertEqual([row[4:] for row in orders],
                         [(1, 7, 255, 255), (2, 7, 6, 255), (3, 7, 6, 8),
                          (1, 6, 255, 255), (1, 6, 255, 255)])
        self.assertEqual([row[0] for row in captures], [3, 4, 5, 8, 9])


class TargetOrderLegacyReceiverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Reuse immutable f976afb production C parser. No MCU source rewrite.
        module = fixture.load_file("target_legacy_rank_replay", fixture.APP / "tests/test_teammate_rank_protocol.py")
        replay = module.TeammateRankProtocolTests
        replay.setUpClass()
        cls.addClassCleanup(replay.doClassCleanups)
        cls.receiver = replay()

    def test_real_old_task2_receiver_rejects_target54_without_breaking01(self):
        for digit, selected in ((1, 6), (2, 8), (3, 7)):
            for order in itertools.permutations((6, 7, 8)):
                with self.subTest(digit=digit, order=order), contextlib.redirect_stdout(io.StringIO()):
                    packets, ranks, counts, _ = self.receiver.run_camera(
                        [build_task_packet(1, 2, digit), b"", b""],
                        [[target(class_id)] for class_id in order])
                self.assertTrue(all(rank == (0,) * 11 for rank in ranks))
                self.assertEqual(counts[-1][:3], (1, 0, 0))
                self.assertEqual(counts[-1][3:6], (0, 0, 0))
                self.assertGreaterEqual(counts[-1][6], 3)
                self.assertEqual(counts[-1][-1], 1)
                coordinates = [p for p in packets if p[2] == 0x62 and p[7] == 1]
                orders = [p for p in packets if p[2] == 0x62 and p[7] == 0x54]
                self.assertEqual([p[8:10] for p in coordinates], [p[8:10] for p in orders])
                self.assertEqual([p[21] for p in coordinates if p[10]], [selected])


if __name__ == "__main__":
    unittest.main()
