"""串口偶发故障不能退出视觉；失败初始化应在同一App中恢复。"""
import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from test_hardware import hardware, FakeSerial, ACK


class UartRecoveryTests(unittest.TestCase):
    def setUp(self):
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)

    def test_transient_rx_error_keeps_link_and_next_command_readable(self):
        class Serial(FakeSerial):
            def read(self, **kwargs):
                if not hasattr(self, "failed_once"):
                    self.failed_once = True
                    raise OSError("temporary UART read error")
                return ACK
        messages = []
        logger = type("Log", (), {"record": lambda self, line: messages.append(line)})()
        link = hardware.UartLink(Serial(), logger)
        self.assertEqual(link.read(len=256, timeout=0), b"")
        self.assertEqual(link.read(len=256, timeout=0), ACK)
        self.assertTrue(any("[UART RX ERROR]" in line for line in messages))

    def test_initial_open_failure_retries_without_restarting_app(self):
        serial = FakeSerial(incoming=ACK)
        with patch.object(hardware.uart, "UART", side_effect=[OSError("busy"), serial], create=True), \
             patch.object(hardware, "start_log", return_value=None), \
             patch("time.monotonic", side_effect=[0, 0, 2, 2, 2, 2, 2, 2]):
            link = hardware.init_uart()
            self.assertIsInstance(link, hardware.UartLink)
            self.assertEqual(link.read(len=256, timeout=0), ACK)
            self.assertTrue(link.send(ACK))
        self.assertEqual(bytes(serial.accepted), ACK)

    def test_repeated_init_failures_are_rate_limited_and_logged(self):
        now, messages = [0.0], []
        logger = SimpleNamespace(record=lambda line: messages.append(line), close=lambda: None)
        with patch.object(hardware.uart, "UART", side_effect=OSError("busy"), create=True) as driver, \
             patch.object(hardware, "start_log", return_value=logger), \
             patch("time.monotonic", side_effect=lambda: now[0]):
            link = hardware.init_uart()
            for _ in range(10):
                self.assertEqual(link.read(len=256, timeout=0), b"")
                self.assertFalse(link.send(ACK))
            self.assertEqual(driver.call_count, 1)
            now[0] = 1.0
            link.read(len=256, timeout=0)
            self.assertEqual(driver.call_count, 2)
        self.assertEqual(sum("[UART CONNECT ERROR]" in line for line in messages), 2)

    def test_driver_closed_port_reopens_without_discarding_pending_frame(self):
        first, second = FakeSerial((3, 0)), FakeSerial()
        first.is_open = lambda: not first.closed
        now = [0.0]
        with patch("time.monotonic", side_effect=lambda: now[0]):
            link = hardware.UartLink(first, opener=lambda: second)
            self.assertFalse(link.send(ACK))
            first.close()
            now[0] = 2.0
            self.assertTrue(link.send(ACK))
        self.assertEqual(bytes(first.accepted + second.accepted), ACK)

    def test_state_log_explains_manual_pause_ack_wait_and_parser_fragment(self):
        messages, now = [], [0.0]
        link = hardware.UartLink(FakeSerial(), SimpleNamespace(record=lambda line: messages.append(line)))
        control = SimpleNamespace(modes=SimpleNamespace(mode="QR"), request_id=7,
            manual_override=True, acknowledged=False, qr_handoff=False, last_command=(7, 2))
        receiver = SimpleNamespace(buffer=bytearray(b"\xaa\x55"))
        with patch("time.monotonic", side_effect=lambda: now[0]):
            link.record_state(control, ACK, receiver)
            link.record_state(control, ACK, receiver)
            self.assertEqual(len(messages), 1)
            now[0] = 1.0
            link.record_state(control, ACK, receiver)
            control.manual_override = False
            control.acknowledged = True
            link.rx_bytes, link.tx_bytes = 8, 9
            link.record_state(control, None, receiver)
        self.assertIn("manual_paused=True acked=False", messages[0])
        self.assertIn("pending_ack=True port_open=True", messages[0])
        self.assertIn("parser_tail=2", messages[0])
        self.assertIn("rx_bytes=8 rx_errors=0 tx_bytes=9 tx_errors=0", messages[-1])

    def test_disabled_uart_still_creates_and_closes_session_log(self):
        messages = []
        logger = SimpleNamespace(record=lambda line: messages.append(line), close=lambda: messages.append("closed"))
        with patch.object(hardware.config, "UART_ENABLED", False), \
             patch.object(hardware, "start_log", return_value=logger):
            self.assertIsNone(hardware.init_uart())
        self.assertEqual(messages, ["[UART DISABLED] config UART_ENABLED=False", "closed"])


if __name__ == "__main__":
    unittest.main()
