"""MC2迁移的直接证明：平台、UART2、真实模型元数据、尺寸和模式切换。

Maix API由maix_fixture模拟；这些结果不代表已经在实体MC2上运行。
"""
import contextlib
import importlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import maix_fixture as fixture
import config
from model_metadata import validate_model
from object_detector import ObjectDetector
from mode_controller import ModeController
from platform_check import check_device
from test_hardware import hardware

APP = Path(__file__).resolve().parents[1]


class MC2Tests(unittest.TestCase):
    def setUp(self):
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)
        model = patch.object(fixture.maix.nn, "YOLO26", fixture.FakeModel, create=True)
        model.start()
        self.addCleanup(model.stop)

    def test_model_has_mc2_binaries_and_matching_class_order(self):
        self.assertEqual(config.MODEL_FILE, "model_9767.mud")
        self.assertEqual(validate_model(str(APP / config.MODEL_FILE), config.CLASS_NAMES_CN),
                         ["model_9767_npu.axmodel", "model_9767_vnpu.axmodel"])
        detector = ObjectDetector()
        self.assertEqual((detector.input_width, detector.input_height), (320, 320))
        self.assertTrue(detector.model.kwargs["dual_buff"])
        self.assertEqual(detector.model.kwargs["model"], str(APP / config.MODEL_FILE))
        detector.close()

    def test_missing_axmodel_or_wrong_labels_are_rejected_before_npu(self):
        with self.assertRaisesRegex(ValueError, "config.py"):
            validate_model(str(APP / config.MODEL_FILE), tuple(reversed(config.CLASS_NAMES_CN)))
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / config.MODEL_FILE
            # 临时目录由标准库自动删除，不把测试垃圾留在工程内。
            target.write_bytes((APP / config.MODEL_FILE).read_bytes())
            with self.assertRaisesRegex(ValueError, "模型文件"):
                validate_model(str(target), config.CLASS_NAMES_CN)

    def test_uart2_pinmux_device_and_baud(self):
        fake = hardware.pinmap
        with patch.object(fake, "set_pin_function", return_value=0) as pin, \
             patch.object(hardware.uart, "UART", return_value=object(), create=True) as serial:
            hardware._open_uart()
        self.assertEqual(pin.call_args_list[0].args, ("B0", "UART2_TX"))
        self.assertEqual(pin.call_args_list[1].args, ("B1", "UART2_RX"))
        serial.assert_called_once_with("/dev/ttyS2", 115200)

    def test_device_guard_prevents_old_mc_execution(self):
        check_device()
        with patch.object(fixture.maix.sys, "device_id", return_value="maixcam"):
            with self.assertRaisesRegex(RuntimeError, "MaixCAM2"):
                check_device()

    def test_reference_parameters_preserved(self):
        self.assertEqual((config.QR_WIDTH, config.QR_HEIGHT), (1920, 1280))
        self.assertEqual((config.QR_ROI_FRACTION, config.QR_ROI_CENTER_X,
                          config.QR_ROI_CENTER_Y), (.20, .44, .35))
        self.assertEqual(config.CONF_THRESHOLD, .35)
        self.assertEqual(config.TASK_CONFIRM_FRAMES, 1)
        self.assertEqual(config.KEY_LONG_PRESS_MS, 1500)

    def test_repeated_camera_reconfiguration_uses_model_size_and_rolls_back(self):
        camera = fixture.FakeCamera()
        modes = ModeController(camera)
        modes.enter("QR")
        for _ in range(4):
            modes.toggle()
            self.assertEqual((modes.width, modes.height, modes.mode), (320, 320, "OBJECT"))
            modes.toggle()
            self.assertEqual((modes.width, modes.height, modes.mode), (1920, 1280, "QR"))
        camera.results = [1, 0]
        with self.assertRaises(RuntimeError):
            modes.enter("OBJECT")
        self.assertEqual(modes.mode, "QR")
        self.assertEqual(camera.format(), fixture.maix.image.Format.FMT_GRAYSCALE)
        modes.close()

    def test_actual_main_short_user_switch_roundtrip_with_dual_buffer(self):
        main = importlib.import_module("main")
        frames = []
        rounds = [0]

        class Frame(fixture.FakeImage):
            def copy(self):
                return Frame(self.width(), self.height(), self.format())

            def find_qrcodes(self, *args, **kwargs):
                return []  # 防止自动扫码切换掩盖USER路径。

        class Camera(fixture.FakeCamera):
            def read(self, **kwargs):
                frames.append((self.width, self.height, self.format()))
                return Frame(self.width, self.height, self.format())

        class Button(fixture.UserButton):
            def take_exit_request(self):
                if rounds[0] in (2, 5):
                    fixture.clock[0] += 300
                    self._on_key(fixture.maix.key.Keys.KEY_OK, fixture.maix.key.State.KEY_PRESSED)
                    fixture.clock[0] += 100
                    self._on_key(fixture.maix.key.Keys.KEY_OK, fixture.maix.key.State.KEY_RELEASED)
                return super().take_exit_request()

        def need_exit():
            rounds[0] += 1
            return rounds[0] > 6

        with patch.object(fixture.maix.camera, "Camera", Camera, create=True), \
             patch.object(fixture.maix.app, "need_exit", side_effect=need_exit, create=True), \
             patch.object(main, "init_uart", return_value=None), \
             patch.object(main, "UserButton", Button), \
             patch.multiple(config, DISPLAY_ENABLED=False, START_MODE="QR"):
            main.main()
        gray = (1920, 1280, fixture.maix.image.Format.FMT_GRAYSCALE)
        rgb = (320, 320, fixture.maix.image.Format.FMT_RGB888)
        self.assertEqual(frames, [gray, rgb, rgb, rgb, gray, gray])


if __name__ == "__main__":
    unittest.main()
