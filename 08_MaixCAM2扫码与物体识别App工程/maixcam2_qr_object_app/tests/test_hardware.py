"""No device required: exercise the actual UART wrapper and reported counts."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

APP_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_DIR))
fake_maix = types.SimpleNamespace(
    err=types.SimpleNamespace(check_raise=lambda result, message: None),
    pinmap=types.SimpleNamespace(set_pin_function=lambda pin, function: 0),
    uart=types.SimpleNamespace(),
)
spec = importlib.util.spec_from_file_location("hardware_under_test", APP_DIR / "hardware.py")
hardware = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {"maix": fake_maix}):
    spec.loader.exec_module(hardware)

ACK = b"\xaa\x55\x61\x01\x00\x01\x00\x99\x88"
RESULT = b"\xaa\x55\x62\x01\x00\x03\x00\x53\x00\x01\x99\x88"


class FakeSerial:
    def __init__(self, returns=(), incoming=b""):
        self.returns = list(returns)
        self.incoming = incoming
        self.calls = []
        self.accepted = bytearray()
        self.read_args = None
        self.closed = False

    def write(self, data):
        self.calls.append(data)
        result = self.returns.pop(0) if self.returns else len(data)
        if isinstance(result, Exception):
            raise result
        if isinstance(result, int) and 0 < result <= len(data):
            self.accepted.extend(data[:result])
        return result

    def read(self, *args, **kwargs):
        self.read_args = (args, kwargs)
        return self.incoming

    def close(self):
        self.closed = True


class UartLinkTests(unittest.TestCase):
    def setUp(self):
        config_patch = patch.multiple(hardware.config, UART_TRACE=True,
            UART_TRACE_EVERY_N_FRAMES=10, UART_WRITE_ATTEMPTS=8, create=True)
        config_patch.start()
        self.addCleanup(config_patch.stop)
        self.output = io.StringIO()
        output_patch = contextlib.redirect_stdout(self.output)
        output_patch.__enter__()
        self.addCleanup(output_patch.__exit__, None, None, None)

    def test_partial_writes_send_only_the_unwritten_suffix(self):
        serial = FakeSerial((3, 2, 4))
        self.assertTrue(hardware.send_packet(hardware.UartLink(serial), ACK))
        self.assertEqual(serial.calls, [ACK, ACK[3:], ACK[5:]])
        self.assertEqual(bytes(serial.accepted), ACK)
        self.assertIn("written=9/9", self.output.getvalue())
        self.assertIn("counts=[3, 2, 4]", self.output.getvalue())

    def test_zero_keeps_the_pending_offset_for_the_next_call(self):
        serial = FakeSerial((3, 0))
        link = hardware.UartLink(serial)
        self.assertFalse(link.send(ACK))
        self.assertTrue(link.send(ACK))
        self.assertEqual(serial.calls, [ACK, ACK[3:], ACK[3:]])
        self.assertEqual(bytes(serial.accepted), ACK)
        self.assertIn("reason=no progress", self.output.getvalue())

    def test_attempt_budget_persists_suffix_across_calls(self):
        serial = FakeSerial([1] * len(ACK))
        link = hardware.UartLink(serial)
        with patch.object(hardware.config, "UART_WRITE_ATTEMPTS", 3):
            self.assertFalse(link.send(ACK))
            self.assertEqual(len(serial.calls), 3)
            self.assertFalse(link.send(ACK))
            self.assertEqual(len(serial.calls), 6)
            self.assertTrue(link.send(ACK))
        self.assertEqual(serial.calls, [ACK[offset:] for offset in range(len(ACK))])
        self.assertEqual(bytes(serial.accepted), ACK)

    def test_new_packet_waits_until_the_previous_packet_is_complete(self):
        serial = FakeSerial((3, 0, 0))
        link = hardware.UartLink(serial)
        self.assertFalse(link.send(ACK))
        self.assertFalse(link.send(RESULT))
        self.assertEqual(serial.calls, [ACK, ACK[3:], ACK[3:]])
        self.assertTrue(link.send(RESULT))
        self.assertEqual(serial.calls[-2:], [ACK[3:], RESULT])
        self.assertEqual(bytes(serial.accepted), ACK + RESULT)

    def test_finishing_old_packet_does_not_report_new_packet_complete(self):
        serial = FakeSerial((3, 6))
        link = hardware.UartLink(serial)
        with patch.object(hardware.config, "UART_WRITE_ATTEMPTS", 1):
            self.assertFalse(link.send(ACK))
            self.assertFalse(link.send(RESULT))
            self.assertEqual(bytes(serial.accepted), ACK)
            self.assertTrue(link.send(RESULT))
        self.assertEqual(serial.calls, [ACK, ACK[3:], RESULT])
        self.assertEqual(bytes(serial.accepted), ACK + RESULT)

    def test_negative_retains_only_the_last_reported_positive_offset(self):
        serial = FakeSerial((2, -5))
        link = hardware.UartLink(serial)
        self.assertFalse(link.send(ACK))
        self.assertIn("written=2/9", self.output.getvalue())
        self.assertIn("counts=[2, -5]", self.output.getvalue())
        self.assertIn("receiver=unconfirmed", self.output.getvalue())
        self.assertTrue(link.send(ACK))
        self.assertEqual(serial.calls[-1], ACK[2:])
        self.assertEqual(bytes(serial.accepted), ACK)

    def test_exception_retains_only_the_last_reported_positive_offset(self):
        serial = FakeSerial((2, RuntimeError("synthetic write failure")))
        link = hardware.UartLink(serial)
        self.assertFalse(link.send(ACK))
        self.assertIn("write exception: synthetic write failure", self.output.getvalue())
        self.assertIn("receiver=unconfirmed", self.output.getvalue())
        self.assertTrue(link.send(ACK))
        self.assertEqual(serial.calls[-1], ACK[2:])
        self.assertEqual(bytes(serial.accepted), ACK)

    def test_invalid_counts_do_not_advance_offset(self):
        for result in (None, True, len(ACK) + 1):
            with self.subTest(result=result):
                serial = FakeSerial((result,))
                link = hardware.UartLink(serial)
                self.assertFalse(link.send(ACK))
                self.assertTrue(link.send(ACK))
                self.assertEqual(serial.calls, [ACK, ACK])

    def test_ack_completion_is_logged_even_when_trace_is_disabled(self):
        serial = FakeSerial()
        with patch.object(hardware.config, "UART_TRACE", False):
            self.assertTrue(hardware.UartLink(serial).send(ACK))
        output = self.output.getvalue()
        self.assertIn("type=61 status=complete", output)
        self.assertIn("hex=" + hardware._hex(ACK), output)

    def test_business_hex_is_limited_to_the_configured_interval(self):
        link = hardware.UartLink(FakeSerial())
        packets = [b"\xaa\x55\x53" + bytes((number,)) for number in range(1, 5)]
        with patch.object(hardware.config, "UART_TRACE_EVERY_N_FRAMES", 3):
            for packet in packets:
                self.assertTrue(link.send(packet))
        output = self.output.getvalue()
        self.assertIn("hex=" + hardware._hex(packets[0]), output)
        self.assertIn("hex=" + hardware._hex(packets[2]), output)
        self.assertNotIn("hex=" + hardware._hex(packets[1]), output)
        self.assertNotIn("hex=" + hardware._hex(packets[3]), output)

    def test_nonempty_rx_logs_hex_and_preserves_read_arguments(self):
        serial = FakeSerial(incoming=b"\xaa\x55\x60")
        link = hardware.UartLink(serial)
        self.assertEqual(link.read(len=256, timeout=0), serial.incoming)
        self.assertEqual(serial.read_args, ((), {"len": 256, "timeout": 0}))
        self.assertIn("[UART RX] bytes=3 hex=AA 55 60", self.output.getvalue())
        self.output.seek(0)
        self.output.truncate(0)
        serial.incoming = b""
        self.assertEqual(link.read(len=256, timeout=0), b"")
        self.assertEqual(self.output.getvalue(), "")

    def test_none_packets_never_write_and_close_delegates(self):
        serial = FakeSerial()
        link = hardware.UartLink(serial)
        self.assertFalse(hardware.send_packet(None, ACK))
        self.assertFalse(hardware.send_packet(link, None))
        self.assertFalse(link.send(b""))
        self.assertEqual(serial.calls, [])
        link.close()
        self.assertTrue(serial.closed)

    def test_init_uart_returns_a_wrapped_serial(self):
        serial = FakeSerial()
        with patch.object(hardware.uart, "UART", return_value=serial, create=True), \
             patch.object(hardware, "start_log", return_value=None), \
             patch.object(hardware.config, "UART_ENABLED", True):
            link = hardware.init_uart()
        self.assertIsInstance(link, hardware.UartLink)
        self.assertIs(link.serial, serial)


if __name__ == "__main__":
    unittest.main(verbosity=2)
