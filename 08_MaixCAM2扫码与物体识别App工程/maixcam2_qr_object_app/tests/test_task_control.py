"""Host-only proof of task requests, ACK gating, and current-frame selection."""
import contextlib
import io
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from control_session import ControlSession
from protocol import (CommandReceiver, bind_result, build_ack_packet,
                      build_control_packet, build_object_packet, build_qr_packet,
                      build_task_packet)
from task_selection import TaskSelection
from utils import crc16_ccitt


def detection(class_id, score=0.8, x=10):
    return SimpleNamespace(class_id=class_id, score=score, x=x, y=20, w=30, h=40)


class FakeModes:
    def __init__(self):
        self.mode, self.calls, self.fail = "IDLE", [], False

    def enter(self, mode):
        self.calls.append(mode)
        if self.fail:
            raise RuntimeError("synthetic model load failure")
        self.mode = mode


class TaskControlTests(unittest.TestCase):
    def setUp(self):
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)
        self.modes = FakeModes()
        self.control = ControlSession(self.modes)
        self.empty = build_object_packet(1, [], 320, 320)

    def test_auto_object_preserves_qr_request_retry_and_needs_new_object_ack(self):
        ack, _ = self.control.apply(1, 1)
        self.assertFalse(self.control.auto_object_after_qr())
        self.control.ack_sent(ack)
        self.assertTrue(self.control.auto_object_after_qr())
        self.assertEqual(self.modes.mode, "OBJECT")
        self.assertEqual(self.control.apply(1, 1), (ack, False))
        self.assertEqual(self.control.result(build_qr_packet(1, "331")),
                         bind_result(build_qr_packet(1, "331"), 1))
        self.assertIsNone(self.control.result(self.empty))
        new_ack, _ = self.control.apply(2, 2, 1, 3)
        self.assertFalse(self.control.qr_handoff)
        self.assertIsNone(self.control.result(self.empty))
        self.control.ack_sent(new_ack)
        self.assertEqual(self.control.result(self.empty), bind_result(self.empty, 2))

    def test_auto_object_failure_preserves_ack_and_qr_business(self):
        ack, _ = self.control.apply(1, 1)
        self.control.ack_sent(ack)
        self.modes.fail = True
        with self.assertRaises(RuntimeError):
            self.control.auto_object_after_qr()
        self.assertFalse(self.control.qr_handoff)
        self.assertEqual(self.modes.mode, "QR")
        self.assertEqual(self.control.result(build_qr_packet(1, "331")),
                         bind_result(build_qr_packet(1, "331"), 1))

    def test_task_wire_body_and_crc(self):
        packet = build_task_packet(0x1234, 4, 0)
        self.assertEqual(packet[:7], b"\xaa\x55\x63\x34\x12\x04\x00")
        self.assertEqual(len(packet), 9)
        self.assertEqual(struct.unpack("<H", packet[-2:])[0], crc16_ccitt(packet[2:-2]))

    def test_task_parser_every_split_and_single_bytes(self):
        packet = build_task_packet(7, 1, 2)
        for split in range(len(packet) + 1):
            with self.subTest(split=split):
                receiver = CommandReceiver()
                self.assertEqual(receiver.feed(packet[:split]) + receiver.feed(packet[split:]), [(7, 2, 1, 2)])
        receiver, commands = CommandReceiver(), []
        for byte in packet:
            commands.extend(receiver.feed(bytes((byte,))))
            self.assertLessEqual(len(receiver.buffer), 8)
        self.assertEqual(commands, [(7, 2, 1, 2)])

    def test_parser_crc_noise_unknown_opcode_and_mixed_commands(self):
        task = build_task_packet(2, 4, 0)
        corrupt = bytearray(task)
        corrupt[-1] ^= 1
        receiver = CommandReceiver()
        stream = (b"noise" * 1000 + b"\xaa\x55\x99" + bytes(corrupt)
                  + build_control_packet(1, 1) + task + build_control_packet(3, 0))
        self.assertEqual(receiver.feed(stream), [(1, 1), (2, 2, 4, 0), (3, 0)])
        self.assertEqual(receiver.buffer, bytearray())

    def test_parser_ignores_reserved_zero_request(self):
        receiver = CommandReceiver()
        self.assertEqual(receiver.feed(build_task_packet(0, 1, 1) + build_control_packet(0, 1)), [])

    def test_all_task_digit_mappings(self):
        expected = {1: (4, 5, 3), 2: (6, 8, 7), 3: (1, 2, 0)}
        for task_id, classes in expected.items():
            for digit, class_id in enumerate(classes, 1):
                with self.subTest(task_id=task_id, digit=digit):
                    self.assertEqual(TaskSelection.class_id_for_request(task_id, digit), class_id)
        self.assertEqual(TaskSelection.class_id_for_request(4, 0), 9)

    def test_invalid_task_digit_pairs(self):
        for task_id, digit in ((0, 1), (5, 1), (1, 0), (2, 0), (3, 0),
                               (4, 1), (4, 2), (4, 3), (1, 4), (None, 1), (1, None)):
            with self.subTest(task_id=task_id, digit=digit):
                with self.assertRaises(ValueError):
                    TaskSelection.class_id_for_request(task_id, digit)

    def test_four_stage_order_binds_the_new_request_only_after_ack(self):
        for request, (task_id, digit, class_id) in enumerate(((1, 1, 4), (4, 0, 9), (2, 2, 8), (3, 3, 0)), 1):
            ack, changed = self.control.apply(request, 2, task_id, digit)
            self.assertTrue(changed)
            self.assertEqual(ack, build_ack_packet(request, 2))
            self.assertEqual(self.control.target_class_id, class_id)
            self.assertIsNone(self.control.result(self.empty))
            self.control.ack_sent(ack)
            self.assertEqual(self.control.result(self.empty), bind_result(self.empty, request))

    def test_exact_retry_reuses_ack_and_does_not_enter_mode_again(self):
        ack, _ = self.control.apply(1, 2, 1, 1)
        self.assertEqual(self.control.apply(1, 2, 1, 1), (ack, False))
        self.assertEqual(self.modes.calls, ["OBJECT"])
        self.assertIsNone(self.control.result(self.empty))
        self.control.ack_sent(ack)
        self.assertEqual(self.control.apply(1, 2, 1, 1), (ack, False))
        self.assertEqual(self.control.result(self.empty), bind_result(self.empty, 1))

    def test_manual_switch_suppresses_business_even_after_return_to_same_mode(self):
        self.modes.toggle = lambda: self.modes.enter("QR" if self.modes.mode == "OBJECT" else "OBJECT")
        ack, _ = self.control.apply(1, 2, 4, 0)
        self.control.ack_sent(ack)
        self.control.manual_toggle()
        self.assertTrue(self.control.ready_for_capture())
        self.assertIsNone(self.control.result(build_qr_packet(1, "123")))
        self.control.manual_toggle()
        self.assertIsNone(self.control.result(self.empty))
        failure, changed = self.control.apply(1, 2, 4, 0)
        self.assertEqual(failure, build_ack_packet(1, 2, 1))
        self.assertFalse(changed)
        self.control.ack_sent(ack)
        self.assertFalse(self.control.acknowledged)
        new_ack, _ = self.control.apply(2, 2, 4, 0)
        self.assertFalse(self.control.manual_override)
        self.assertIsNone(self.control.result(self.empty))
        self.control.ack_sent(new_ack)
        self.assertEqual(self.control.result(self.empty), bind_result(self.empty, 2))

    def test_manual_switch_failure_preserves_live_session(self):
        ack, _ = self.control.apply(1, 2, 4, 0)
        self.control.ack_sent(ack)
        def fail():
            raise RuntimeError("camera failure")
        self.modes.toggle = fail
        with self.assertRaises(RuntimeError):
            self.control.manual_toggle()
        self.assertFalse(self.control.manual_override)
        self.assertEqual(self.control.result(self.empty), bind_result(self.empty, 1))

    def test_result_type_must_match_actual_mode(self):
        ack, _ = self.control.apply(1, 1)
        self.control.ack_sent(ack)
        self.assertIsNone(self.control.result(self.empty))
        self.assertIsNotNone(self.control.result(build_qr_packet(1, "123")))

    def test_same_id_changed_task_or_digit_pauses_business(self):
        for changed_command in ((1, 2, 1, 2), (1, 2, 4, 0), (1, 0), (1, 2)):
            with self.subTest(command=changed_command):
                modes, control = FakeModes(), ControlSession(FakeModes())
                control.modes = modes
                ack, _ = control.apply(1, 2, 1, 1)
                control.ack_sent(ack)
                failure, changed = control.apply(*changed_command)
                self.assertTrue(changed)
                self.assertEqual(failure, build_ack_packet(1, 2, 1))
                self.assertIsNone(control.request_id)
                self.assertIsNone(control.result(self.empty))
                self.assertEqual(control.apply(*changed_command), (failure, False))
                self.assertEqual(modes.calls, ["OBJECT"])

    def test_stale_command_preserves_current_binding_and_ack(self):
        ack, _ = self.control.apply(3, 2, 4, 0)
        self.control.ack_sent(ack)
        current_command = self.control.last_command
        stale, changed = self.control.apply(2, 1)
        self.assertEqual(stale, build_ack_packet(2, 2, 1))
        self.assertFalse(changed)
        self.assertEqual(self.control.request_id, 3)
        self.assertEqual(self.control.last_command, current_command)
        self.assertEqual(self.control.last_ack, ack)
        self.assertEqual(self.control.target_class_id, 9)
        self.assertEqual(self.control.result(self.empty), bind_result(self.empty, 3))
        self.assertEqual(self.modes.calls, ["OBJECT"])

    def test_old_ack_cannot_open_new_request_gate(self):
        old_ack, _ = self.control.apply(1, 2, 1, 1)
        self.control.ack_sent(old_ack)
        new_ack, _ = self.control.apply(2, 2, 4, 0)
        self.control.ack_sent(old_ack)
        self.assertIsNone(self.control.result(self.empty))
        self.control.ack_sent(build_ack_packet(2, 2, 1))
        self.assertIsNone(self.control.result(self.empty))
        self.control.ack_sent(new_ack)
        self.assertEqual(self.control.result(self.empty), bind_result(self.empty, 2))

    def test_invalid_new_task_pauses_until_a_new_valid_request(self):
        ack, _ = self.control.apply(1, 2, 1, 1)
        self.control.ack_sent(ack)
        failure, _ = self.control.apply(2, 2, 4, 1)
        self.assertEqual(failure, build_ack_packet(2, 2, 1))
        self.control.ack_sent(failure)
        self.assertIsNone(self.control.result(self.empty))
        ack, _ = self.control.apply(3, 2, 4, 0)
        self.assertIsNone(self.control.result(self.empty))
        self.control.ack_sent(ack)
        self.assertEqual(self.control.result(self.empty), bind_result(self.empty, 3))

    def test_model_failure_pauses_and_exact_failed_retry_does_not_reload(self):
        self.modes.fail = True
        failure, changed = self.control.apply(1, 2, 1, 1)
        self.assertTrue(changed)
        self.assertEqual(failure, build_ack_packet(1, 0, 1))
        self.control.ack_sent(failure)
        self.assertIsNone(self.control.result(self.empty))
        self.assertEqual(self.control.apply(1, 2, 1, 1), (failure, False))
        self.assertEqual(self.modes.calls, ["OBJECT"])

    def test_generic_object_and_idle_clear_explicit_task_selection(self):
        ack, _ = self.control.apply(1, 2, 4, 0)
        self.control.ack_sent(ack)
        generic, _ = self.control.apply(2, 2)
        self.assertIsNone(self.control.target_class_id)
        self.assertIsNone(self.control.task_id)
        self.control.ack_sent(generic)
        idle, _ = self.control.apply(3, 0)
        self.control.ack_sent(idle)
        self.assertIsNone(self.control.result(self.empty))

    def test_single_class_highest_score_and_no_previous_frame_coordinates(self):
        task = TaskSelection()
        task.observe("123")
        for class_id in (4, 9, 8, 0):
            with self.subTest(class_id=class_id):
                best = detection(class_id, 0.95, 100)
                tied = detection(class_id, 0.95, 200)
                mixed = [detection((class_id + 1) % 10, 0.99), detection(class_id, 0.4), best, tied]
                self.assertEqual(task.select(mixed, class_id), [best])
                fresh = detection(class_id, 0.5, 30)
                self.assertEqual(task.select([fresh], class_id), [fresh])
                self.assertEqual(task.select([detection((class_id + 1) % 10)], class_id), [])
                self.assertEqual(task.select([], class_id), [])

    def test_qr_short_payload_and_empty_heartbeat(self):
        for payload, expected_length in (("123", 11), (None, 8)):
            packet = build_qr_packet(65536, payload)
            self.assertEqual(packet[2:6], b"\x53\x00\x00" + bytes((payload is not None,)))
            self.assertEqual(packet[6:-2], b"123" if payload is not None else b"")
            self.assertEqual(len(packet), expected_length)
            self.assertEqual(len(bind_result(packet, 7)), expected_length + 5)
        for payload in ("", "12", "1234", "120", "1x3", "１２３", True, 123):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                build_qr_packet(1, payload)


if __name__ == "__main__":
    unittest.main(verbosity=2)
