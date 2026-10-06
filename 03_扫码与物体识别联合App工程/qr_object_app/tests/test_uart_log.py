import contextlib
import io
from pathlib import Path
import queue
import tempfile
import unittest
from unittest.mock import patch

from uart_log import DeviceUartLog, start_log, volatile_log_directory, LOG_DIR
from unittest.mock import mock_open
from test_hardware import hardware, FakeSerial, ACK, RESULT


class DeviceLogTests(unittest.TestCase):
    def test_volatile_storage_prefers_shm_and_never_persistent_root(self):
        mounts = "rootfs / rootfs rw 0 0\ntmpfs /dev/shm tmpfs rw 0 0\n/dev/mmcblk0p2 /tmp ext4 rw 0 0\n"
        with patch("builtins.open", mock_open(read_data=mounts)):
            self.assertEqual(volatile_log_directory(), LOG_DIR)
        self.assertEqual(LOG_DIR, "/dev/shm/vision_uart_logs")

    def test_tmp_is_used_only_if_ram_mounted_and_nested_disk_mount_is_rejected(self):
        mounts = "rootfs / rootfs rw 0 0\ntmpfs /tmp tmpfs rw 0 0\n"
        with patch("builtins.open", mock_open(read_data=mounts)):
            self.assertEqual(volatile_log_directory(), "/tmp/vision_uart_logs")
        mounts += "/dev/mmcblk0p2 /tmp/vision_uart_logs ext4 rw 0 0\n"
        with patch("builtins.open", mock_open(read_data=mounts)):
            with self.assertRaises(OSError):
                volatile_log_directory()

    def test_absent_ram_mount_disables_logging_without_writing_to_sd(self):
        with patch("builtins.open", mock_open(read_data="/dev/mmcblk0p2 / ext4 rw 0 0\n")), \
             patch("uart_log.DeviceUartLog") as factory, contextlib.redirect_stdout(io.StringIO()):
            self.assertIsNone(start_log(hardware.config))
        factory.assert_not_called()

    def test_start_log_selects_volatile_directory_and_records_power_off_policy(self):
        with patch("uart_log.volatile_log_directory", return_value="/dev/shm/vision_uart_logs"), \
             patch("uart_log.DeviceUartLog") as factory:
            self.assertIs(start_log(hardware.config), factory.return_value)
        self.assertEqual(factory.call_args.kwargs["directory"], LOG_DIR)
        self.assertIn("storage=RAM power_off=clears", factory.call_args.kwargs["metadata"])

    def test_all_rx_tx_written_even_when_console_sampling_disabled(self):
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            logger = DeviceUartLog(directory, metadata="synthetic-test")
            serial = FakeSerial((3, 6), incoming=ACK)
            link = hardware.UartLink(serial, logger)
            with patch.object(hardware.config, "UART_TRACE", False):
                self.assertEqual(link.read(), ACK)
                self.assertTrue(link.send(ACK))
                for _ in range(12):
                    self.assertTrue(link.send(RESULT))
            link.close()
            text = "".join(Path(p).read_text(encoding="utf-8") for p in logger.paths)
            self.assertIn("[UART RX] bytes=9", text)
            self.assertEqual(text.count("[UART TX] type=62 status=complete"), 12)
            writes = [line for line in text.splitlines() if "[UART WRITE]" in line]
            self.assertIn("bytes=3 receiver=unconfirmed hex=AA 55 61", writes[0])
            self.assertIn("offset=3 total=9 returned=6 bytes=6", writes[1])
            self.assertIn("[SESSION END]", text)
            self.assertTrue(serial.closed)

    def test_rotation_and_unique_session_preserve_old_files(self):
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            first = DeviceUartLog(directory, part_bytes=400)
            for _ in range(10):
                first.record("[UART RX] " + "A" * 100)
            first.close()
            before = {p: Path(p).read_bytes() for p in first.paths}
            second = DeviceUartLog(directory)
            second.close()
            self.assertGreater(len(first.paths), 1)
            self.assertNotEqual(first.session, second.session)
            self.assertTrue(all(Path(p).stat().st_size <= 400 for p in first.paths))
            self.assertTrue(all(Path(p).read_bytes() == data for p, data in before.items()))

    def test_quota_disables_logging_not_uart(self):
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            logger = DeviceUartLog(directory, session_bytes=400)
            logger.record("X" * 1000)
            logger.close()
            self.assertIsNotNone(logger.error)
            serial = FakeSerial()
            self.assertTrue(hardware.UartLink(serial, logger).send(ACK))
            self.assertEqual(bytes(serial.accepted), ACK)
            self.assertLessEqual(sum(Path(p).stat().st_size for p in logger.paths), 400)

    def test_directory_quota_keeps_existing_logs(self):
        with tempfile.TemporaryDirectory() as directory:
            old = Path(directory) / "uart_old_001.txt"
            old.write_bytes(b"old evidence")
            with self.assertRaises(OSError):
                DeviceUartLog(directory, directory_bytes=5)
            self.assertEqual(old.read_bytes(), b"old evidence")

    def test_disk_failure_factory_does_not_fail_uart(self):
        with patch("uart_log.DeviceUartLog", side_effect=OSError("read only")), contextlib.redirect_stdout(io.StringIO()):
            self.assertIsNone(start_log(hardware.config))
        with patch.object(hardware, "start_log", return_value=None), \
             patch.object(hardware.uart, "UART", return_value=FakeSerial(), create=True), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertIsNotNone(hardware.init_uart())

    def test_logger_exception_does_not_change_wire_bytes(self):
        class BrokenLog:
            def record(self, message):
                raise OSError("synthetic failure")
        serial = FakeSerial((3, 6))
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(hardware.UartLink(serial, BrokenLog()).send(ACK))
        self.assertEqual(bytes(serial.accepted), ACK)

    def test_queue_full_is_counted_without_blocking(self):
        logger = object.__new__(DeviceUartLog)
        logger._accepting = True
        logger._queue = queue.Queue(maxsize=1)
        logger.dropped = 0
        self.assertTrue(logger.record("first"))
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(logger.record("second"))
        self.assertEqual(logger.dropped, 1)


if __name__ == "__main__":
    unittest.main()
