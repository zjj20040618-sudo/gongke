"""V1 state, parser, real main and UART regressions; no device evidence."""
import contextlib
import io
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from boot_session import BootSession, boot_packet, wrap_packet
from control_session import ControlSession
from protocol import _packet, build_control_packet, build_task_packet, build_object_packet
import test_uart_main as fixture

C1, C2 = bytes(range(1, 9)), bytes(range(9, 17))
S1, S2 = bytes(range(17, 25)), bytes(range(25, 33))


def frames(wire):
    out, offset = [], 0
    while offset < len(wire):
        assert wire[offset:offset + 2] == b"\xaa\x55"
        opcode = wire[offset + 2]
        size = {0x65: 22, 0x68: 23}.get(opcode)
        if opcode == 0x66:
            size = 23 + struct.unpack_from("<H", wire, offset + 19)[0]
        assert size is not None and offset + size <= len(wire), "unfinished UART frame"
        out.append(bytes(wire[offset:offset + size]))
        offset += size
    return out


class Modes:
    mode = "QR"
    def enter(self, mode): self.mode = mode
    def toggle(self): self.enter("QR" if self.mode == "OBJECT" else "OBJECT")


class BootSessionTests(unittest.TestCase):
    def setUp(self):
        self.nonces = iter((S1, S2, bytes(range(33, 41))))
        self.boot = BootSession(lambda size: next(self.nonces))
        self.control = ControlSession(Modes(), self.boot)

    def rx(self, packet):
        decoded = self.boot.feed(packet)
        self.assertEqual(len(decoded), 1)
        return self.boot.receive(decoded[0])

    def commit(self, client=C1, server=S1):
        challenge, changed, command = self.rx(boot_packet(0x64, client))
        self.assertEqual(challenge, boot_packet(0x65, client, server))
        self.assertFalse(changed)
        ready, changed, command = self.rx(boot_packet(0x67, client, server))
        self.assertTrue(changed)
        self.control.reset_for_boot()
        self.assertFalse(self.boot.ready)
        self.boot.transmitted(ready)
        return ready

    def apply(self, packet, client=C1, server=S1):
        response, changed, command = self.rx(wrap_packet(packet, client, server))
        self.assertIsNotNone(command)
        ack, changed = self.control.apply(*command)
        self.control.ack_sent(ack)
        return ack

    def test_lengths_offsets_crc_and_no_nested_crc(self):
        self.assertEqual([len(boot_packet(op, C1, S1)) for op in (0x64, 0x65, 0x67, 0x68)],
                         [14, 22, 22, 23])
        packet = wrap_packet(build_task_packet(1, 1, 1), C1, S1)
        self.assertEqual(len(packet), 28)
        self.assertEqual(packet[2:21], b"\x66" + C1 + S1 + b"\x05\x00")
        self.assertEqual(packet[21:-2], b"\x63\x01\x00\x01\x01")

    def test_fragmentation_noise_bad_crc_and_gap_are_bounded(self):
        packet = boot_packet(0x64, C1)
        for cut in range(1, len(packet)):
            receiver = BootSession(lambda n: S1)
            self.assertEqual(receiver.feed(b"noise" + packet[:cut], 0), [])
            self.assertEqual(receiver.feed(packet[cut:], 99), [packet[2:-2]])
        self.assertEqual(self.boot.feed(packet[:7], 0), [])
        self.assertEqual(self.boot.feed(packet[7:], 101), [])
        bad = packet[:-1] + bytes((packet[-1] ^ 1,))
        self.assertEqual(self.boot.feed(bad + packet, 102), [packet[2:-2]])
        self.boot.feed(b"\xaa\x55\x66" + bytes(16) + b"\xff\xff" + bytes(10000))
        self.assertLessEqual(len(self.boot.buffer), 280)

    def test_ready_completion_and_task_ack_are_separate_gates(self):
        self.rx(boot_packet(0x64, C1))
        ready, changed, _ = self.rx(boot_packet(0x67, C1, S1))
        self.control.reset_for_boot()
        task = wrap_packet(build_task_packet(1, 1, 1), C1, S1)
        self.assertIsNone(self.rx(task)[2])
        self.assertFalse(self.control.ready_for_capture())
        self.boot.transmitted(ready[:-1])
        self.assertFalse(self.boot.ready)
        self.boot.transmitted(ready)
        self.assertIsNone(self.control.request_id)
        command = self.rx(task)[2]
        ack, _ = self.control.apply(*command)
        result = build_object_packet(0, [], 640, 480)
        self.assertIsNone(self.control.result(result))
        self.control.ack_sent(ack)
        self.assertEqual(self.control.result(result)[2], 0x66)

    def test_hello_and_duplicate_confirm_preserve_task_and_manual_resume(self):
        ready = self.commit()
        ack = self.apply(build_task_packet(6, 1, 1))
        self.control.manual_toggle()
        self.rx(boot_packet(0x64, C2))
        self.assertEqual(self.boot.active, (C1, S1))
        self.assertEqual(self.control.request_id, 6)
        self.assertTrue(self.control.can_resume_object_after_qr())
        response, changed, _ = self.rx(boot_packet(0x67, C1, S1))
        self.assertEqual(response, ready)
        self.assertFalse(changed)
        self.boot.transmitted(response)
        self.assertEqual(self.control.last_ack, ack)
        self.assertTrue(self.control.manual_override)
        self.assertTrue(self.control.auto_object_after_qr())

    def test_pending_retry_reuses_challenge_active_hello_does_not_replace_pending(self):
        self.commit()
        first = self.rx(boot_packet(0x64, C2))[0]
        self.assertEqual(self.rx(boot_packet(0x64, C2))[0], first)
        self.assertEqual(self.rx(boot_packet(0x64, C1))[0], boot_packet(0x65, C1, S1))
        self.assertEqual(self.boot.pending, (C2, S2))

    def test_new_confirm_resets_old_request_and_rejects_same_number_old_session(self):
        self.commit()
        self.apply(build_task_packet(6, 3, 3))
        self.control.manual_toggle()
        self.commit(C2, S2)
        self.assertIsNone(self.control.last_ack)
        self.assertFalse(self.control.manual_override)
        self.assertFalse(self.control.qr_handoff)
        self.apply(build_task_packet(1, 1, 1), C2, S2)
        self.assertIsNone(self.rx(wrap_packet(build_task_packet(1, 3, 3), C1, S1))[2])
        self.assertEqual(self.control.task_id, 1)
        self.assertIsNone(self.rx(boot_packet(0x67, C1, S1))[0])
        self.rx(boot_packet(0x64, C1))  # Delayed old HELLO gets a FRESH challenge.
        self.assertIsNone(self.rx(boot_packet(0x67, C1, S1))[0])
        self.assertEqual(self.boot.active, (C2, S2))

    def test_same_session_old_number_guard_and_conflicting_reuse_remain(self):
        self.commit()
        self.apply(build_task_packet(6, 1, 1))
        old = self.rx(wrap_packet(build_task_packet(1, 1, 1), C1, S1))[2]
        ack, changed = self.control.apply(*old)
        self.assertFalse(changed)
        self.assertEqual(ack[25], 1)
        self.assertEqual(self.control.request_id, 6)
        same = self.rx(wrap_packet(build_task_packet(6, 3, 3), C1, S1))[2]
        ack, changed = self.control.apply(*same)
        self.assertEqual(ack[25], 1)
        self.assertIsNone(self.control.request_id)

    def test_bad_version_nonce_crc_and_bare_packets_cannot_replace_active(self):
        self.commit()
        self.apply(build_task_packet(6, 1, 1))
        for body in (b"\x64\x02" + C2, b"\x64\x01" + bytes(8),
                     b"\x67\x01" + C2 + S2, b"\x67\x01" + C1 + bytes(8),
                     b"\x66" + C1 + S1 + b"\x04\x00" + b"\x66\x01\x00\x01"):
            self.assertEqual(self.rx(_packet(body)), (None, False, None))
        self.assertEqual(self.rx(build_control_packet(1, 1)), (None, False, None))
        self.assertEqual(self.boot.active, (C1, S1))
        self.assertEqual(self.control.request_id, 6)

    def test_rng_failure_and_reused_or_zero_nonce_never_downgrade(self):
        self.commit()
        for random_source in (lambda n: bytes(8), lambda n: S1,
                              lambda n: (_ for _ in ()).throw(OSError("rng unavailable"))):
            self.boot._random = random_source
            self.assertIsNone(self.rx(boot_packet(0x64, C2))[0])
            self.assertEqual(self.boot.active, (C1, S1))
            self.assertTrue(self.boot.ready)


class BootMainTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.UartMainTests.setUpClass()
        cls.addClassCleanup(fixture.UartMainTests.doClassCleanups)
        cls.harness = fixture.UartMainTests()

    def run_camera(self, commands, detections=(), writes=(), **kwargs):
        nonces = iter((S1, S2, bytes(range(33, 41))))
        serial, captures = self.harness.run_loop(commands, detections, writes,
            boot_enabled=True, boot_random=lambda n: next(nonces), **kwargs)
        return serial, captures, frames(serial.wire)

    def test_main_clears_rank_sequence_and_pipeline_only_on_confirmed_new_boot(self):
        commands = [boot_packet(0x64, C1), boot_packet(0x67, C1, S1),
            wrap_packet(build_task_packet(6, 1, 1), C1, S1), b"",
            boot_packet(0x64, C2), boot_packet(0x67, C2, S2),
            wrap_packet(build_task_packet(1, 1, 1), C2, S2), b"",
            wrap_packet(build_task_packet(7, 3, 3), C1, S1), b""]
        serial, captures, packets = self.run_camera(commands,
            [[fixture.detection(5)], [fixture.detection(4)], [],
             [fixture.detection(5)], [fixture.detection(4)], [], []], dual_buffer=True)
        self.assertEqual([at for at, _, _ in captures], [3, 4, 5, 7, 8, 9, 10])
        results = [p for p in packets if p[2] == 0x66 and p[21] == 0x62]
        new = [p for p in results if p[3:11] == C2]
        self.assertTrue(new)
        self.assertTrue(all(struct.unpack_from("<H", p, 22)[0] == 1 for p in new))
        self.assertEqual(struct.unpack_from("<H", new[0], 27)[0], 0)
        rank = [p for p in new if p[26] == 0x54]
        self.assertEqual(rank[0][30:32], b"\x01\x01")  # Red now first; old green removed.
        self.assertFalse(any(p[26] == 0x53 for p in new))

    def test_partial_ready_blocks_capture_and_wrapped_task_before_completion(self):
        command = wrap_packet(build_task_packet(1, 1, 1), C1, S1)
        serial, captures, packets = self.run_camera(
            [boot_packet(0x64, C1), boot_packet(0x67, C1, S1), command, b"", command],
            [[fixture.detection(4)]], (22, 3, 0, 0, 20))
        self.assertEqual([at for at, _, _ in captures], [5])
        self.assertEqual([p[2] for p in packets[:3]], [0x65, 0x68, 0x66])

    def test_old_partial_tail_finishes_before_new_ready_and_no_old_task_resume(self):
        serial, captures, packets = self.run_camera(
            [boot_packet(0x64, C1), boot_packet(0x67, C1, S1),
             wrap_packet(build_control_packet(6, 1), C1, S1),
             boot_packet(0x64, C2), boot_packet(0x67, C2, S2),
             wrap_packet(build_control_packet(1, 1), C2, S2)], (),
            (22, 23, 28, 3, 0, 29, 22, 23, 28), qr_frames=())
        self.assertEqual([at for at, _, _ in captures], [3, 4, 6])
        old_result = next(i for i, p in enumerate(packets) if p[2] == 0x66 and p[21] == 0x62)
        new_ready = next(i for i, p in enumerate(packets) if p[2] == 0x68 and p[4:12] == C2)
        self.assertLess(old_result, new_ready)
        self.assertTrue(all(p[3:11] == C2 for p in packets[new_ready + 1:] if p[2] == 0x66))

    def test_lost_ready_retries_keep_manual_resume_and_rank_history(self):
        qr = {"payload": "123", "text": "123", "x": 0, "y": 0, "w": 20, "h": 20}
        _, captures, packets = self.run_camera(
            [boot_packet(0x64, C1), boot_packet(0x67, C1, S1),
             wrap_packet(build_task_packet(1, 1, 1), C1, S1), b"",
             boot_packet(0x64, C1), boot_packet(0x67, C1, S1), b""],
            [[fixture.detection(5)], [fixture.detection(4)], [], []],
            manual_toggles=(4,), qr_frames=([qr],))
        self.assertEqual([mode for _, _, mode in captures], ["OBJECT", "QR", "OBJECT", "OBJECT", "OBJECT"])
        ranks = [p for p in packets if p[2] == 0x66 and p[21] == 0x62 and p[26] == 0x54]
        self.assertEqual(ranks[-1][30:32], b"\x02\x02")


if __name__ == "__main__":
    unittest.main()
