"""MC临时UART证据日志。只写RAM文件系统，断电清除，不回退到SD卡。"""
from datetime import datetime, timezone
import os
import queue
import threading
import time
import uuid

LOG_DIR = "/dev/shm/vision_uart_logs"
PART_BYTES = 1024 * 1024
SESSION_BYTES = 4 * 1024 * 1024
DIRECTORY_BYTES = 16 * 1024 * 1024
APP_VERSION = "2.1.16"


def volatile_log_directory(mounts_file="/proc/mounts"):
    """先验证挂载类型；/tmp这个名字本身不能证明文件会随断电消失。

    首选/dev/shm；固件未挂载时尝试/run或/tmp，但必须是tmpfs/ramfs。
    没有RAM目录则明确停止日志，绝不悄悄写入原/root持久目录。
    """
    with open(mounts_file, encoding="utf-8") as mounts:
        entries = []
        for line in mounts:
            fields = line.split()
            if len(fields) >= 3:
                mount = fields[1].replace("\\040", " ").replace("\\134", "\\")
                entries.append((os.path.realpath(mount), fields[2]))
    for directory in (LOG_DIR, "/run/vision_uart_logs", "/tmp/vision_uart_logs"):
        path = os.path.realpath(directory)
        matches = [(mount, kind) for mount, kind in entries
                   if path == mount or path.startswith(mount.rstrip(os.sep) + os.sep)]
        if matches and max(matches, key=lambda entry: len(entry[0]))[1] in ("tmpfs", "ramfs"):
            return directory
    raise OSError("没有已挂载的RAM日志目录；UART继续，但临时日志未开启")


class DeviceUartLog:
    def __init__(self, directory=LOG_DIR, metadata="", part_bytes=PART_BYTES,
                 session_bytes=SESSION_BYTES, directory_bytes=DIRECTORY_BYTES):
        self.directory = directory
        self.part_limit, self.session_limit = part_bytes, session_bytes
        self.directory_limit = directory_bytes
        os.makedirs(directory, exist_ok=True)
        self.directory_size = sum(os.path.getsize(os.path.join(directory, name))
            for name in os.listdir(directory)
            if name.startswith("uart_") and name.endswith(".txt") and os.path.isfile(os.path.join(directory, name)))
        if self.directory_size >= directory_bytes:
            raise OSError("日志目录达到容量上限；下载并人工清理旧日志后重启App")
        self.session = "uart_{}_{}_{}".format(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"), os.getpid(), uuid.uuid4().hex[:8])
        self.paths = []
        self.part_number = self.part_size = self.session_size = 0
        self.file = None
        self.error = None
        self.dropped = 0
        self._reported_drops = 0
        self._accepting = True
        self._stop = threading.Event()
        self._queue = queue.Queue(maxsize=256)
        self._open_part()
        try:
            self._write(self._line("[SESSION] version={} session={} {}".format(APP_VERSION, self.session, metadata)))
            self.file.flush()
            self._thread = threading.Thread(target=self._worker, name="uart-file-log", daemon=True)
            self._thread.start()
        except Exception:
            self.file.close()
            raise
        print("[UART FILE] saving {} (temporary RAM log; download before power off; UTC filenames)".format(self.paths[0]))

    @staticmethod
    def _line(message):
        return "{} mono_ms={} {}\n".format(datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            int(time.monotonic() * 1000), str(message).replace("\r", "\\r").replace("\n", "\\n"))

    def _open_part(self):
        self.part_number += 1
        path = os.path.join(self.directory, "{}_{:03d}.txt".format(self.session, self.part_number))
        self.file = open(path, "xb")  # 不覆盖已有文件；前缀含随机会话ID。
        self.paths.append(path)
        self.part_size = 0

    def _write(self, line):
        data = line.encode("utf-8")
        if self.session_size + len(data) > self.session_limit or self.directory_size + len(data) > self.directory_limit:
            raise OSError("UART日志达到容量上限；保留已有日志，停止本轮记录")
        if self.part_size and self.part_size + len(data) > self.part_limit:
            self.file.close()
            self._open_part()
        if len(data) > self.part_limit:
            raise OSError("单条日志超过分片上限")
        self.file.write(data)
        self.part_size += len(data)
        self.session_size += len(data)
        self.directory_size += len(data)

    def record(self, message):
        if not self._accepting:
            return False
        try:
            self._queue.put_nowait(self._line(message))
            return True
        except queue.Full:
            self.dropped += 1
            if self.dropped == 1:
                print("[UART FILE] queue full; dropped log records, UART continues")
            return False
        except Exception as exc:
            self._fail(exc)
            return False

    def _fail(self, exc):
        self._accepting = False
        if self.error is None:
            self.error = str(exc)
            print("[UART FILE] disabled; UART continues:", exc)

    def _worker(self):
        last_flush = time.monotonic()
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    line = self._queue.get(timeout=0.25)
                except queue.Empty:
                    line = None
                if self.dropped != self._reported_drops:
                    self._write(self._line("[LOG GAP] dropped_records={}".format(self.dropped)))
                    self._reported_drops = self.dropped
                if line is not None:
                    self._write(line)
                if time.monotonic() - last_flush >= 0.5:
                    self.file.flush()
                    last_flush = time.monotonic()
            self._write(self._line("[SESSION END] dropped_records={}".format(self.dropped)))
        except Exception as exc:
            self._fail(exc)
        finally:
            try:
                self.file.close()
            except Exception as exc:
                self._fail(exc)

    def close(self):
        self._accepting = False
        self._stop.set()
        self._thread.join(timeout=2)
        if self._thread.is_alive():
            print("[UART FILE] writer still closing; do not power off yet")


def start_log(config):
    try:
        directory = volatile_log_directory()
        return DeviceUartLog(directory=directory, metadata="storage=RAM power_off=clears model={} UART={} TX={} RX={} baud={} receiver=unconfirmed".format(
            config.MODEL_FILE, config.UART_DEVICE, config.UART_TX_PIN, config.UART_RX_PIN, config.UART_BAUDRATE))
    except Exception as exc:
        print("[UART FILE] cannot start; UART continues:", exc)
        return None
