"""MaixCAM Pro UART1初始化。失败时视觉程序仍继续。"""
from maix import err, pinmap, uart
import config


def _hex(data):
    return " ".join("{:02X}".format(byte) for byte in data)


class UartLink:
    """Keep each frame contiguous across bounded, main-loop-only write calls.

    Only reported positive lengths advance the offset. A negative return or an
    exception gives no reliable new byte count; retry retains the last known
    offset. Neither a complete write nor an error proves what the MCU received.
    """

    def __init__(self, serial):
        self.serial = serial
        self._pending_packet = None
        self._offset = 0
        self._write_calls = 0
        self._pending_trace = False
        self._business_frames = 0

    def read(self, *args, **kwargs):
        data = self.serial.read(*args, **kwargs)
        if data and getattr(config, "UART_TRACE", True):
            print("[UART RX] bytes={} hex={}".format(len(data), _hex(data)))
        return data

    def close(self):
        return self.serial.close()

    def _begin_packet(self, packet):
        self._pending_packet = packet
        self._offset = self._write_calls = 0
        is_ack = len(packet) >= 3 and packet[:3] == b"\xaa\x55\x61"
        if not is_ack:
            self._business_frames += 1
        every = max(1, int(getattr(config, "UART_TRACE_EVERY_N_FRAMES", 10)))
        self._pending_trace = is_ack or (
            getattr(config, "UART_TRACE", True)
            and (self._business_frames == 1 or self._business_frames % every == 0)
        )

    def _log(self, status, counts, reason=""):
        packet = self._pending_packet
        frame_type = "{:02X}".format(packet[2]) if len(packet) >= 3 else "unknown"
        detail = " hex={}".format(_hex(packet)) if self._pending_trace else ""
        print("[UART TX] type={} status={} written={}/{} calls={} counts={} reason={} receiver=unconfirmed{}".format(
            frame_type, status, self._offset, len(packet), self._write_calls,
            counts, reason, detail))

    def send(self, packet):
        if packet is None:
            return False
        requested = bytes(packet)
        if not requested:
            return False
        if self._pending_packet is None:
            self._begin_packet(requested)
        attempts = max(1, int(getattr(config, "UART_WRITE_ATTEMPTS", 8)))
        counts = []
        for _ in range(attempts):
            suffix = self._pending_packet[self._offset:]
            self._write_calls += 1
            try:
                count = self.serial.write(suffix)
            except Exception as exc:
                counts.append("exception")
                self._log("blocked", counts, "write exception: {}".format(exc))
                return False
            counts.append(count)
            if not isinstance(count, int) or isinstance(count, bool) or count > len(suffix):
                self._log("blocked", counts, "invalid write count")
                return False
            if count <= 0:
                self._log("blocked", counts, "no progress" if count == 0 else "driver error")
                return False
            self._offset += count
            if self._offset < len(self._pending_packet):
                self._log("partial", counts)
                continue
            completed = self._pending_packet
            if self._pending_trace:
                self._log("complete", counts)
            self._pending_packet = None
            if completed == requested:
                return True
            # A previous frame finished. Queue the requested frame even if this
            # call has exhausted its write budget; never append a fresh prefix
            # while the previous frame still has an unwritten suffix.
            self._begin_packet(requested)
            counts = []
        self._log("blocked", counts, "write attempt limit")
        return False


def init_uart():
    if not config.UART_ENABLED:
        return None
    try:
        err.check_raise(pinmap.set_pin_function(config.UART_TX_PIN, "UART1_TX"), "set UART1_TX failed")
        err.check_raise(pinmap.set_pin_function(config.UART_RX_PIN, "UART1_RX"), "set UART1_RX failed")
        serial = uart.UART(config.UART_DEVICE, config.UART_BAUDRATE)
        print("[UART] {} TX={} RX={} baud={}".format(config.UART_DEVICE, config.UART_TX_PIN, config.UART_RX_PIN, config.UART_BAUDRATE))
        return UartLink(serial)
    except Exception as exc:
        print("[UART] init failed; vision continues:", exc)
        return None

def send_packet(serial, packet):
    if serial is None or packet is None:
        return False
    try:
        return serial.send(packet)
    except Exception as exc:
        print("[UART] send failed:", exc)
        return False
