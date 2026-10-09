"""MaixCAM2 UART2初始化；沿用正式工程的分片收发、重连和RAM日志。"""
from maix import err, pinmap, uart
import time
import config
from uart_log import start_log


def _hex(data):
    return " ".join("{:02X}".format(byte) for byte in data)


class UartLink:
    """Keep each frame contiguous across bounded, main-loop-only write calls.

    Only reported positive lengths advance the offset. A negative return or an
    exception gives no reliable new byte count; retry retains the last known
    offset. Neither a complete write nor an error proves what the MCU received.
    """

    def __init__(self, serial, logger=None, opener=None):
        self.serial = serial
        self.logger = logger
        self._pending_packet = None
        self._offset = 0
        self._write_calls = 0
        self._pending_trace = False
        self._business_frames = 0
        self._opener = opener
        self._retry_at = 0.0
        self.rx_bytes = self.rx_errors = self.tx_bytes = self.tx_errors = 0
        self._last_state = None
        self._state_at = 0.0
        self._session_state = None

    def _ensure_open(self):
        # 初始化失败不等于关闭整个App；最多每秒重试一次，避免每帧刷屏/重配引脚。
        if self.serial is not None:
            is_open = getattr(self.serial, "is_open", None)
            if self._opener is None or is_open is None:
                return True
            try:
                if is_open():
                    return True
            except Exception as exc:
                self._record("[UART STATUS ERROR] {}; retry normal read/write".format(exc))
                return True  # 状态查询失败不是已关闭的证据，不丢弃尚未发完的帧。
            self._record("[UART CONNECT] driver reports closed; reopening")
            self.serial = None
        if self._opener is None or time.monotonic() < self._retry_at:
            return False
        self._retry_at = time.monotonic() + 1.0
        try:
            self.serial = self._opener()
            self._record("[UART CONNECT] ready; receiver=unconfirmed")
            print("[UART] connected; waiting MCU request")
            return True
        except Exception as exc:
            self._record("[UART CONNECT ERROR] {} retry_in_ms=1000".format(exc))
            print("[UART] init failed; retry in 1s:", exc)
            return False

    def read(self, *args, **kwargs):
        if not self._ensure_open():
            return b""
        try:
            data = self.serial.read(*args, **kwargs)
        except Exception as exc:
            self.rx_errors += 1
            line = "[UART RX ERROR] {} errors={} retry=next_loop".format(exc, self.rx_errors)
            self._record(line)
            print(line)
            # 一次驱动读异常不能让扫码/识别退出。下一轮继续读，CRC负责重同步。
            return b""
        if data:
            self.rx_bytes += len(data)
            line = "[UART RX] bytes={} hex={}".format(len(data), _hex(data))
            self._record(line)
            if getattr(config, "UART_TRACE", True):
                print(line)
        return data

    def record_event(self, message):
        self._record(message)

    def record_state(self, control, pending_ack=None, receiver=None):
        transport = getattr(control, "transport", None)
        if transport is not None:
            session_state = (transport.active, transport.pending, transport.ready)
            if session_state != self._session_state:
                self._session_state = session_state
                self._record("[SESSION STATE] active={} pending={} ready={}".format(*session_state))
        # 状态改变立即记录；静默时每秒一次，区分没有RX、人工暂停和ACK等待。
        state = (control.modes.mode, control.request_id, control.manual_override,
                 control.acknowledged, control.qr_handoff, control.last_command,
                 pending_ack is not None, self.serial is not None,
                 self.rx_bytes, self.rx_errors, self.tx_errors)
        now = time.monotonic()
        if state == self._last_state and now < self._state_at:
            return
        self._last_state, self._state_at = state, now + 1.0
        self._record("[LINK STATE] mode={} request={} manual_paused={} acked={} qr_handoff={} "
                     "command={} pending_ack={} port_open={} rx_bytes={} rx_errors={} "
                     "tx_bytes={} tx_errors={} pending_written={} pending_total={} parser_tail={}".format(
                         *state[:10], self.tx_bytes, self.tx_errors, self._offset,
                         len(self._pending_packet) if self._pending_packet is not None else 0,
                         len(receiver.buffer) if receiver is not None else 0))

    def _record(self, line):
        if self.logger is not None:
            try:
                self.logger.record(line)
            except Exception as exc:
                print("[UART FILE] record failed; UART continues:", exc)
                self.logger = None

    def close(self):
        try:
            self._record("[UART CLOSE] pending_written={} pending_total={}".format(self._offset,
                len(self._pending_packet) if self._pending_packet is not None else 0))
            return self.serial.close() if self.serial is not None else None
        finally:
            if self.logger is not None:
                try:
                    self.logger.close()
                except Exception as exc:
                    print("[UART FILE] close failed:", exc)

    def _begin_packet(self, packet):
        self._pending_packet = packet
        self._offset = self._write_calls = 0
        is_ack = (len(packet) >= 3 and packet[2] in (0x61, 0x65, 0x68)) or (
            len(packet) >= 26 and packet[2] == 0x66 and packet[21] == 0x61)
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
        line = "[UART TX] type={} status={} written={}/{} calls={} counts={} reason={} receiver=unconfirmed".format(
            frame_type, status, self._offset, len(packet), self._write_calls,
            counts, reason)
        self._record(line + " hex=" + _hex(packet))
        if status != "complete" or self._pending_trace:
            print(line + (" hex=" + _hex(packet) if self._pending_trace else ""))

    def send(self, packet):
        if packet is None:
            return False
        requested = bytes(packet)
        if not requested:
            return False
        if not self._ensure_open():
            self._record("[UART TX BLOCKED] port unavailable type={:02X}".format(requested[2] if len(requested) >= 3 else 0))
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
                self.tx_errors += 1
                counts.append("exception")
                self._record("[UART WRITE] offset={} total={} returned=exception reason={} receiver=unconfirmed".format(
                    self._offset, len(self._pending_packet), exc))
                self._log("blocked", counts, "write exception: {}".format(exc))
                return False
            counts.append(count)
            valid_count = isinstance(count, int) and not isinstance(count, bool) and 0 < count <= len(suffix)
            self._record("[UART WRITE] offset={} total={} returned={} bytes={} receiver=unconfirmed hex={}".format(
                self._offset, len(self._pending_packet), count, count if valid_count else 0,
                _hex(suffix[:count]) if valid_count else ""))
            if not isinstance(count, int) or isinstance(count, bool) or count > len(suffix):
                self.tx_errors += 1
                self._log("blocked", counts, "invalid write count")
                return False
            if count <= 0:
                self.tx_errors += 1
                self._log("blocked", counts, "no progress" if count == 0 else "driver error")
                return False
            self._offset += count
            self.tx_bytes += count
            if self._offset < len(self._pending_packet):
                self._log("partial", counts)
                continue
            completed = self._pending_packet
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


def _open_uart():
    # 引脚名与功能名不同：B0是焊盘名，UART2_TX才是芯片复用功能。
    err.check_raise(pinmap.set_pin_function(config.UART_TX_PIN, config.UART_TX_FUNCTION), "set UART2_TX failed")
    err.check_raise(pinmap.set_pin_function(config.UART_RX_PIN, config.UART_RX_FUNCTION), "set UART2_RX failed")
    serial = uart.UART(config.UART_DEVICE, config.UART_BAUDRATE)
    print("[UART] {} TX={} RX={} baud={}".format(config.UART_DEVICE, config.UART_TX_PIN, config.UART_RX_PIN, config.UART_BAUDRATE))
    return serial


def init_uart():
    logger = start_log(config)  # 先建日志；初始化失败的证据也必须留在本轮会话。
    if not config.UART_ENABLED:
        if logger is not None:
            logger.record("[UART DISABLED] config UART_ENABLED=False")
            logger.close()
        return None
    link = UartLink(None, logger, opener=_open_uart)
    link._ensure_open()
    return link

def send_packet(serial, packet):
    if serial is None or packet is None:
        return False
    try:
        return serial.send(packet)
    except Exception as exc:
        print("[UART] send failed:", exc)
        return False
