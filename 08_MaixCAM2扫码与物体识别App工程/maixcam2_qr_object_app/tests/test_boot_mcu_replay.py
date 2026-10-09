"""Interactive Python V1 sender against immutable 6fe1538 real MCU parser."""
import itertools
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import test_uart_main as fixture
from boot_session import BootSession, wrap_packet
from control_session import ControlSession
from protocol import (build_ack_packet, build_qr_packet, build_object_packet,
                      build_ball_order_packet, build_hostage_order_packet)
from test_boot_session import C1, C2, S1, S2, Modes
from test_uart_main import detection

APP = Path(__file__).resolve().parents[1]
ROOT = APP.parents[1]
MCU_REVISION = "6fe15380268af430b17ff59b30c049a48b7b89e2"


class BootMcuReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="mc2-boot-mcu-")
        cls.addClassCleanup(cls.temporary.cleanup)
        build = Path(cls.temporary.name)
        for name in ("App/proto.c", "App/proto.h", "tests/stubs/main.h",
                     "tests/proto_boot_session_test.c"):
            data = subprocess.run(["git", "show", MCU_REVISION + ":" + name],
                                  cwd=ROOT, capture_output=True, check=True).stdout
            target = build / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
        compiler = os.environ.get("EOD_HOST_CC", "E:/setup/devc++/Dev-Cpp/MinGW64/bin/gcc.exe")
        cls.executable = build / "bridge.exe"
        cls.regressions = build / "mcu_boot.exe"
        for source, executable in ((APP / "tests/boot_session_replay.c", cls.executable),
                                   (build / "tests/proto_boot_session_test.c", cls.regressions)):
            subprocess.run([compiler, "-std=c11", "-fuse-ld=bfd", "-Wall", "-Wextra", "-Werror",
                "-I" + str(build / "App"), "-I" + str(build / "tests/stubs"),
                str(source), str(build / "App/proto.c"), "-o", str(executable)],
                capture_output=True, check=True)

    def setUp(self):
        self.process = subprocess.Popen([str(self.executable)], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.addCleanup(self.close)
        nonces = iter((S1, S2))
        self.boot = BootSession(lambda n: next(nonces))
        self.control = ControlSession(Modes(), self.boot)

    def close(self):
        self.process.stdin.close()
        self.process.wait(timeout=5)
        self.process.stdout.close()
        self.process.stderr.close()

    def exchange(self, command):
        if isinstance(command, bytes): command = command.hex()
        self.process.stdin.write(command + "\n")
        self.process.stdin.flush()
        packets, state = [], None
        while True:
            line = self.process.stdout.readline().strip()
            self.assertTrue(line, "MCU bridge exited unexpectedly")
            if line == "END": break
            if line.startswith("TX,"): packets.append(bytes.fromhex(line[3:]))
            if line.startswith("STATE,"): state = tuple(map(int, line.split(",")[1:]))
        return packets, state

    def reply(self, packet):
        bodies = self.boot.feed(packet)
        self.assertEqual(len(bodies), 1)
        response, committed, command = self.boot.receive(bodies[0])
        if committed: self.control.reset_for_boot()
        if command is not None:
            response, changed = self.control.apply(*command)
        self.assertIsNotNone(response)
        self.boot.transmitted(response)
        self.control.ack_sent(response)
        return response

    def handshake(self, client):
        packets, state = self.exchange("B" + client.hex())
        self.assertEqual((packets[0][2], state[0], state[2]), (0x64, 1, 0))
        challenge = self.reply(packets[0])
        packets, state = self.exchange(challenge)
        self.assertEqual((packets[0][2], state[0]), (0x67, 2))
        ready = self.reply(packets[0])
        packets, state = self.exchange(ready)
        self.assertEqual((packets[0][2], packets[0][21], state[0]), (0x66, 0x60, 3))
        self.assertEqual(state[2:4], (0, 0))  # READY alone is not task ACK/freshness.
        return packets[0], ready

    def test_real_mcu_own_boot_regressions(self):
        result = subprocess.run([str(self.regressions)], capture_output=True, check=True)
        self.assertIn(b"regression: PASS", result.stdout)

    def test_both_directions_qr_and_four_tasks_all_rank_orders(self):
        request, _ = self.handshake(C1)
        self.exchange(self.reply(request))
        _, state = self.exchange(self.control.result(build_qr_packet(0, "123")))
        self.assertEqual(state[8:12], (1, 1, 2, 3))
        for task, digit, target, domain in ((1, 1, 4, (3, 4, 5)), (2, 2, 8, None),
                                           (3, 3, 0, (0, 1, 2)), (4, 0, 9, None)):
            orders = itertools.permutations(domain) if domain else [None]
            for order in orders:
                packets, _ = self.exchange("T{}{}".format(task, digit))
                self.exchange(self.reply(packets[0]))
                seq = 0
                objects = [detection(target)]
                packet = self.control.result(build_object_packet(seq, objects, 640, 480))
                _, state = self.exchange(packet[:7])
                _, state = self.exchange(packet[7:])
                self.assertEqual(state[2:6], (1, 1, task, digit))
                if domain:
                    maker = build_ball_order_packet if task == 1 else build_hostage_order_packet
                    _, state = self.exchange(self.control.result(maker(seq, target, order)))
                    self.assertEqual(state[6:8], (1, order.index(target) + 1))
                callbacks = state[12]
                _, state = self.exchange(self.control.result(build_object_packet(1, [], 640, 480)))
                self.assertEqual(state[3], 1)  # Empty frame is still a fresh link heartbeat.
                self.assertEqual(state[12], callbacks)  # No cached coordinate callback.

    def test_mcu_only_reboot_accepts_req1_and_rejects_old_ack_qr_object_rank_ready(self):
        request, old_ready = self.handshake(C1)
        old_ack = self.reply(request)
        self.exchange(old_ack)
        old_qr = self.control.result(build_qr_packet(0, "123"))
        self.exchange(old_qr)
        request, ready = self.handshake(C2)
        # Same request=1, mode=QR: old ACK/QR must not authorize the new task.
        _, state = self.exchange(old_ready + old_ack + old_qr)
        self.assertEqual(state[2:4], (0, 0))
        self.assertEqual(state[8], 0)
        self.exchange(self.reply(request))
        _, state = self.exchange(self.control.result(build_qr_packet(0, "231")))
        self.assertEqual(state[8:12], (1, 2, 3, 1))
        packets, _ = self.exchange("T11")
        ack = self.reply(packets[0])
        self.exchange(ack)
        # Real stale body uses matching current request=2, then wrong boot identity.
        from protocol import bind_result
        old_object = wrap_packet(bind_result(build_object_packet(0, [detection(4)], 640, 480), 2), C1, S1)
        old_rank = wrap_packet(bind_result(build_ball_order_packet(0, 4, [4]), 2), C1, S1)
        _, state = self.exchange(old_object + old_rank)
        self.assertEqual(state[3], 0)
        self.assertEqual(state[6], 0)
        self.exchange(self.control.result(build_object_packet(0, [detection(4)], 640, 480)))
        _, state = self.exchange(self.control.result(build_ball_order_packet(0, 4, [4])))
        self.assertEqual(state[3], 1)
        self.assertEqual(state[6:8], (1, 1))
        _, state = self.exchange(ready)  # Duplicate READY cannot clear new rank/ACK.
        self.assertEqual(state[2:4], (1, 1))
        self.assertEqual(state[6:8], (1, 1))

    def test_new_mcu_never_accepts_bare_business(self):
        request, _ = self.handshake(C1)
        bare_ack = build_ack_packet(1, 1)
        _, state = self.exchange(bare_ack)
        self.assertEqual(state[2], 0)
        self.exchange(self.reply(request))
        from protocol import bind_result
        _, state = self.exchange(bind_result(build_qr_packet(0, "123"), 1))
        self.assertEqual(state[8], 0)

    def test_real_main_uart_interactive_handshake_reboot_qr_and_rank(self):
        fixture.UartMainTests.setUpClass()
        self.addCleanup(fixture.UartMainTests.doClassCleanups)
        harness = fixture.UartMainTests()
        offset, turn, observed = [0], [0], {}

        def incoming(serial):
            turn[0] += 1
            packets = []
            tail = bytes(serial.wire[offset[0]:])
            if tail:
                offset[0] = len(serial.wire)
                packets, state = self.exchange(tail)
                observed[turn[0]] = state
            if turn[0] == 1:
                packets, _ = self.exchange("B" + C1.hex())
            elif turn[0] == 4:
                self.assertEqual(observed[4][8:12], (1, 1, 2, 3))
                packets, _ = self.exchange("T11")
            elif turn[0] == 6:
                self.assertEqual(observed[6][6:8], (1, 2))  # Green before red.
                packets, _ = self.exchange("B" + C2.hex())
            elif turn[0] == 9:
                self.assertEqual(observed[9][8:12], (1, 2, 3, 1))  # No old QR123 latch.
                packets, _ = self.exchange("T33")
            return b"".join(packets)

        nonces = iter((S1, S2))
        qr = lambda payload: {"payload": payload, "text": payload,
                               "x": 0, "y": 0, "w": 20, "h": 20}
        serial, captures = harness.run_loop([incoming] * 11,
            [[detection(5)], [detection(4)], [], [detection(0)], [], []], (),
            boot_enabled=True, boot_random=lambda n: next(nonces),
            qr_frames=([qr("123")], [qr("231")]))
        _, state = self.exchange(bytes(serial.wire[offset[0]:]))
        self.assertEqual(state[0:3], (3, 2, 1))
        self.assertEqual(state[4:8], (3, 3, 1, 1))
        self.assertEqual([mode for _, _, mode in captures],
                         ["QR", "OBJECT", "OBJECT", "OBJECT", "QR", "OBJECT", "OBJECT", "OBJECT"])


if __name__ == "__main__":
    unittest.main()
