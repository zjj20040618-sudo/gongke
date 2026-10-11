"""靶子首次出现编号；型号6/7/8、颜色号和出现序号不可混用。"""
import contextlib
import io
import itertools
import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import config
from hostage_order import TargetOrder
from protocol import (bind_result, build_ball_order_packet,
                      build_hostage_order_packet, build_target_order_packet,
                      build_task_packet, build_ack_packet)
from control_session import ControlSession
from utils import crc16_ccitt


def target(class_id, x=10, score=None, **geometry):
    score = (config.CONF_THRESHOLD + 1.0) / 2.0 if score is None else score
    fields = dict(class_id=class_id, score=score, x=x, y=20, w=30, h=40)
    fields.update(geometry)
    return SimpleNamespace(**fields)


def modes():
    value = SimpleNamespace(mode="IDLE")
    value.enter = lambda mode: setattr(value, "mode", mode)
    value.toggle = lambda: value.enter("QR" if value.mode == "OBJECT" else "OBJECT")
    return value


class TargetOrderTests(unittest.TestCase):
    def setUp(self):
        silence = contextlib.redirect_stdout(io.StringIO())
        silence.__enter__()
        self.addCleanup(silence.__exit__, None, None, None)

    def test_all_six_orders_and_each_requested_target_latch_first_appearance(self):
        for order in itertools.permutations((6, 7, 8)):
            for selected in order:
                with self.subTest(order=order, selected=selected):
                    tracker = TargetOrder()
                    tracker.reset(selected)
                    for class_id in order:
                        tracker.observe([target(class_id)], 640, 480)
                    tracker.observe([target(class_id, 500 - n * 100)
                                     for n, class_id in enumerate(order)], 640, 480)
                    self.assertEqual(tracker.order, list(order))
                    self.assertEqual(tracker.target_rank, order.index(selected) + 1)

    def test_same_frame_orders_best_box_by_center_then_y_then_class(self):
        tracker = TargetOrder()
        tracker.reset(6)
        score = (config.CONF_THRESHOLD + 1.0) / 2.0
        tracker.observe([target(6, 400, score), target(6, 50, 1.0),
                         target(7, 200), target(8, 100), target(0),
                         target(4), target(9), target(7, -10, 1.0)], 640, 480)
        self.assertEqual(tracker.order, [6, 8, 7])
        self.assertEqual(tracker.target_rank, 1)
        tracker.reset(6)
        tracker.observe([target(8, 100), target(7, 100), target(6, 100, y=40)], 640, 480)
        self.assertEqual(tracker.order, [7, 8, 6])

    def test_low_score_invalid_or_clipped_boxes_cannot_allocate_number(self):
        tracker = TargetOrder()
        tracker.reset(6)
        tracker.observe([target(6, score=config.CONF_THRESHOLD - .01),
                         target(6, score=float("nan")), target(6, score=1.01),
                         target(6, x=-1), target(6, x=620), target(6, y=460),
                         target(6, w=0), target(6, h=-1),
                         target(True), target("6"), target(6, score="invalid")], 640, 480)
        self.assertEqual(tracker.order, [])
        self.assertEqual(tracker.target_rank, 0)
        tracker.observe([target(6, score=config.CONF_THRESHOLD)], 640, 480)
        self.assertEqual(tracker.order, [6])

    def test_loss_and_reappearance_keep_rank_reset_and_inactive_clear_it(self):
        tracker = TargetOrder()
        tracker.reset(6)
        tracker.observe([target(8)], 640, 480)
        self.assertEqual(tracker.target_rank, 0)
        tracker.observe([target(6)], 640, 480)
        tracker.observe([], 640, 480)
        tracker.observe([target(6, 600), target(8)], 640, 480)
        self.assertEqual(tracker.order, [8, 6])
        self.assertEqual(tracker.target_rank, 2)
        tracker.reset(6)
        self.assertEqual(tracker.order, [])
        self.assertEqual(tracker.target_rank, 0)
        tracker.reset()
        tracker.observe([target(6)], 640, 480)
        self.assertFalse(tracker.active)
        self.assertEqual(tracker.order, [])
        for selected in (0, 3, 5, 9, True, "6"):
            with self.assertRaises(ValueError):
                tracker.reset(selected)

    def test_packet_body_binding_crc_rank_unknown_and_ff_unused_slots(self):
        for order in ([], [8], [8, 6], [8, 6, 7], [6]):
            packet = build_target_order_packet(0x1234, 6, order)
            rank = order.index(6) + 1 if 6 in order else 0
            self.assertEqual(len(packet), 13)
            self.assertEqual(struct.unpack("<BHBBB3B", packet[2:-2]),
                             (0x54, 0x1234, 6, rank, len(order),
                              *(order + [255] * (3 - len(order)))))
            bound = bind_result(packet, 7)
            self.assertEqual(len(bound), 18)
            self.assertEqual(bound[2:7], b"\x62\x07\x00\x09\x00")
            for item in (packet, bound):
                self.assertEqual(struct.unpack("<H", item[-2:])[0],
                                 crc16_ccitt(item[2:-2]))

    def test_all_six_packet_orders_have_correct_rank_for_all_three_models(self):
        for order in itertools.permutations((6, 7, 8)):
            for selected in order:
                body = build_target_order_packet(0xffff, selected, order)[2:-2]
                self.assertEqual(struct.unpack("<BHBBB3B", body),
                                 (0x54, 0xffff, selected, order.index(selected) + 1,
                                  3, *order))

    def test_handoff_json_fixed_examples_match_actual_writer_and_crc(self):
        manifest = json.loads((Path(__file__).resolve().parents[2] /
                               "target_order_protocol.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["task_id"], 2)
        self.assertEqual(manifest["digit_to_model"], {"1": 6, "2": 8, "3": 7})
        self.assertEqual(manifest["model_to_color"], {"6": "red", "7": "blue", "8": "green"})
        self.assertEqual((manifest["unknown_rank"], manifest["empty_slot"]), (0, 255))
        self.assertFalse(manifest["hardware_verified"])
        fixed = {
            "request_blue": "aa556301000203b7c6",
            "ack": "aa55610100020057b2",
            "unknown": "aa556201000900540000070000ffffff995c",
            "blue_second": "aa5562010009005401000702030807068946",
        }
        actual = {
            "request_blue": build_task_packet(1, 2, 3),
            "ack": build_ack_packet(1, 2),
            "unknown": bind_result(build_target_order_packet(0, 7, []), 1),
            "blue_second": bind_result(build_target_order_packet(1, 7, [8, 7, 6]), 1),
        }
        self.assertEqual({item["name"] for item in manifest["examples"]}, set(fixed))
        for example in manifest["examples"]:
            packet = bytes.fromhex(example["hex"])
            self.assertEqual(packet.hex(), fixed[example["name"]])
            self.assertEqual(packet, actual[example["name"]])
            self.assertEqual(len(packet), example["length"])
            self.assertEqual(struct.unpack("<H", packet[-2:])[0], crc16_ccitt(packet[2:-2]))
            if packet[2] == 0x62:
                self.assertEqual(packet[10:16], bytes([example["target_model_id"],
                    example["target_rank"], example["seen_count"], *example["slots"]]))

    def test_invalid_target_or_order_rejected_other_domains_not_rewritten(self):
        for selected, order in ((5, []), (True, []), (6, [6, 6]), (6, [0]),
                                (6, [4]), (6, [False]), (6, [6, 7, 8, 6])):
            with self.assertRaises(ValueError):
                build_target_order_packet(0, selected, order)
        self.assertEqual(bind_result(build_ball_order_packet(1, 4, [4]), 7).hex(),
                         "aa55620700090054010004010104ffff8795")
        self.assertEqual(bind_result(build_hostage_order_packet(1, 2, [1, 0, 2]), 7).hex(),
                         "aa5562070009005401000203030100023067")

    def test_only_fully_acked_task2_accepts_target54(self):
        session = ControlSession(modes())
        packet = build_target_order_packet(0, 6, [6])
        self.assertIsNone(session.result(packet))
        commands = ((1, 1, None, None), (2, 2, None, None), (3, 2, 1, 1),
                    (4, 2, 3, 1), (5, 2, 4, 0), (6, 2, 2, 1))
        for request, mode, task, digit in commands:
            ack, _ = session.apply(request, mode, task, digit)
            self.assertIsNone(session.result(packet))
            session.ack_sent(ack[:-1])
            self.assertIsNone(session.result(packet))
            session.ack_sent(ack)
            self.assertEqual(session.result(packet),
                             bind_result(packet, request) if task == 2 else None)
            if mode == 1:
                session.auto_object_after_qr()
                self.assertIsNone(session.result(packet))

    def test_task2_digit_mapping_and_wrong_target_domains_lengths_rejected(self):
        session = ControlSession(modes())
        for request, digit, selected in ((1, 1, 6), (2, 2, 8), (3, 3, 7)):
            ack, _ = session.apply(request, 2, 2, digit)
            packet = build_target_order_packet(0, selected, [selected])
            self.assertIsNone(session.result(packet))
            session.ack_sent(ack)
            self.assertEqual(session.target_class_id, selected)
            self.assertEqual(session.result(packet), bind_result(packet, request))
            wrong = [build_target_order_packet(0, other, [other])
                     for other in (6, 7, 8) if other != selected]
            wrong += [build_ball_order_packet(0, 4, [4]),
                      build_hostage_order_packet(0, 1, [1]),
                      packet[:-1], packet + b"\x00"]
            for item in wrong:
                self.assertIsNone(session.result(item))

    def test_manual_preview_blocks_target54_until_original_mode_resumes(self):
        session = ControlSession(modes())
        ack, _ = session.apply(1, 2, 2, 1)
        session.ack_sent(ack)
        packet = build_target_order_packet(0, 6, [6])
        session.manual_toggle()
        self.assertIsNone(session.result(packet))
        retry_ack, changed = session.apply(1, 2, 2, 1)
        self.assertFalse(changed)
        self.assertNotEqual(retry_ack, ack)
        self.assertIsNone(session.result(packet))
        session.manual_toggle()
        self.assertEqual(session.result(packet), bind_result(packet, 1))


if __name__ == "__main__":
    unittest.main()
