"""无需设备：模拟 Maix API，验证按键、模式、十类模型和二进制组包。"""
import configparser
import contextlib
import importlib
import io
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest.mock import patch

APP_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_DIR))
clock = [0]
maix = types.ModuleType("maix")
maix.time = types.SimpleNamespace(ticks_ms=lambda: clock[0])
maix.key = types.SimpleNamespace(
    Keys=types.SimpleNamespace(KEY_OK=1),
    State=types.SimpleNamespace(KEY_PRESSED=1, KEY_RELEASED=2, KEY_LONG_PRESSED=3),
    rm_default_listener=lambda: None,
    Key=lambda **kwargs: types.SimpleNamespace(**kwargs),
)
maix.image = types.SimpleNamespace(
    Format=types.SimpleNamespace(FMT_RGB888=1, FMT_GRAYSCALE=2),
    Fit=types.SimpleNamespace(FIT_CONTAIN=1),
    QRCodeDecoderType=types.SimpleNamespace(QRCODE_DECODER_TYPE_ZBAR=1),
    COLOR_GREEN=1, COLOR_YELLOW=2, COLOR_RED=3, COLOR_BLUE=4, COLOR_WHITE=5,
    Color=types.SimpleNamespace(from_rgb=lambda r, g, b: (r, g, b)),
    string_size=lambda text, scale=1, thickness=-1: (len(text) * 8 * scale, 12 * scale),
)
maix.err = types.SimpleNamespace(Err=types.SimpleNamespace(ERR_NONE=0))
maix.nn = types.SimpleNamespace()
maix.pinmap = types.SimpleNamespace()
maix.uart = types.SimpleNamespace()
maix.app = types.SimpleNamespace()
maix.sys = types.SimpleNamespace(device_id=lambda: "maixcam2")
maix.camera = types.SimpleNamespace()
maix.display = types.SimpleNamespace()
sys.modules["maix"] = maix
import config
from object_detector import ObjectDetector
from mode_controller import ModeController
from user_button import UserButton
from protocol import build_object_packet, build_qr_packet
from qr_reader import QrReader, center_roi, task_text_cn
from utils import crc16_ccitt


def raw_object(class_id=9, score=0.82):
    return types.SimpleNamespace(class_id=class_id, score=score, x=10, y=20, w=30, h=40)


class FakeModel:
    labels = config.CLASS_NAMES_CN
    def __init__(self, **kwargs):
        self.kwargs = kwargs
    def input_width(self):
        return 320
    def input_height(self):
        return 320
    def input_format(self):
        return maix.image.Format.FMT_RGB888
    def detect(self, img, **kwargs):
        self.detect_kwargs = kwargs
        return [raw_object(9), raw_object(0, 0.91)]


class FakeCamera:
    def __init__(self, *args, **kwargs):
        self.buffers = [kwargs.get("buff_num")]
        self.width, self.height = args[:2] if len(args) >= 2 else (config.OBJECT_WIDTH, config.OBJECT_HEIGHT)
        self.fps_values = [kwargs.get("fps", -1)]
        self.calls = []
        self.results = []
        self.pixel_format = args[2] if len(args) > 2 else maix.image.Format.FMT_RGB888
        self.formats, self.warmups, self.close_count = [], [], 0
    def format(self):
        return self.pixel_format
    def close(self):
        self.close_count += 1
    def open(self, width, height, format, buff_num, fps=-1):
        self.buffers.append(buff_num)
        self.fps_values.append(fps)
        self.formats.append(format)
        result = self.set_resolution(width, height)
        if result == 0:
            self.pixel_format = format
        return result
    def skip_frames(self, num):
        self.warmups.append(num)
    def set_resolution(self, width, height):
        self.calls.append((width, height))
        result = self.results.pop(0) if self.results else 0
        if isinstance(result, Exception):
            raise result
        if result == 0:
            self.width, self.height = width, height
        return result
    def read(self, **kwargs):
        return FakeImage(self.width, self.height, self.pixel_format)


class FakeCode:
    def payload(self): return "123"
    def x(self): return 420
    def y(self): return 240
    def w(self): return 120
    def h(self): return 120
    def corners(self): return [(420, 240), (540, 240), (540, 360), (420, 360)]


class FakeImage:
    def __init__(self, width=1920, height=1440, pixel_format=None):
        self._width, self._height = width, height
        self.pixel_format = pixel_format or maix.image.Format.FMT_RGB888
    def width(self): return self._width
    def height(self): return self._height
    def format(self): return self.pixel_format
    def resize(self, width, height, **kwargs): return type(self)(width, height, self.pixel_format)
    def to_format(self, pixel_format):
        self.pixel_format = pixel_format
        return self
    def find_qrcodes(self, *args, **kwargs): return [FakeCode()]
    def draw_string(self, *args, **kwargs): pass
    def draw_rect(self, *args, **kwargs): pass
    def draw_cross(self, *args, **kwargs): pass
    def draw_edges(self, *args, **kwargs): pass


maix.image.Image = FakeImage
