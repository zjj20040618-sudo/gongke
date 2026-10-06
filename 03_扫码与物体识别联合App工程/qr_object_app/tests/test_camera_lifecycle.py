"""复现 MaixCAM 原生 close/open 状态问题，防止模式切换误退出。

普通 FakeCamera 不会释放 MMF，曾让主机测试漏掉实机故障。
这里模拟：close 释放 VI，但旧 Camera 仍认为自己已打开。
"""
import contextlib
import importlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import test_app as fixture
from mode_controller import ModeController
import config
from user_button import UserButton


class NativeStateCamera(fixture.FakeCamera):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.vi_initialized = True

    def close(self):
        super().close()
        self.vi_initialized = False

    def open(self, *args, **kwargs):
        if not self.vi_initialized:
            raise RuntimeError("mmf add vi channel failed: vi not inited")
        return super().open(*args, **kwargs)


class CameraLifecycleTests(unittest.TestCase):
    def setUp(self):
        model = patch.object(fixture.maix.nn, "YOLO26", fixture.FakeModel, create=True)
        model.start()
        self.addCleanup(model.stop)
        output = contextlib.redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)

    def test_old_close_open_sequence_reproduces_device_error(self):
        cam = NativeStateCamera()
        cam.close()
        with self.assertRaisesRegex(RuntimeError, "vi not inited"):
            cam.open(1920, 1440, format=fixture.maix.image.Format.FMT_GRAYSCALE,
                     buff_num=1)

    def test_repeated_mode_switches_keep_vi_alive_until_final_close(self):
        cam = NativeStateCamera()
        modes = ModeController(cam)
        modes.enter("QR")
        for _ in range(8):
            modes.toggle()
            self.assertEqual(modes.mode, "OBJECT")
            self.assertEqual(cam.format(), fixture.maix.image.Format.FMT_RGB888)
            self.assertEqual((cam.width, cam.height), (320, 320))
            self.assertEqual(cam.buffers[-1], 2)
            modes.toggle()
            self.assertEqual(modes.mode, "QR")
            self.assertEqual(cam.format(), fixture.maix.image.Format.FMT_GRAYSCALE)
            self.assertEqual((cam.width, cam.height), (1920, 1440))
            self.assertEqual(cam.buffers[-1], 1)
        self.assertIs(modes.cam, cam)  # main 的采集句柄必须仍指向同一对象。
        self.assertEqual(cam.close_count, 0)
        modes.enter("QR")
        self.assertEqual(len(cam.calls), 17)  # 同模式请求不重复打开。
        modes.close()
        self.assertEqual(cam.close_count, 1)
        self.assertFalse(cam.vi_initialized)

    def test_failed_reconfiguration_can_restore_previous_mode_without_close(self):
        cam = NativeStateCamera()
        modes = ModeController(cam)
        modes.enter("QR")
        cam.results = [1, 0]
        with self.assertRaises(RuntimeError):
            modes.enter("OBJECT")
        self.assertEqual(modes.mode, "QR")
        self.assertTrue(cam.vi_initialized)
        self.assertEqual(cam.close_count, 0)
        self.assertEqual((cam.width, cam.height), (1920, 1440))
        self.assertIsNone(modes.detector)

    def test_real_main_and_user_short_press_roundtrip_with_object_dual_buffer(self):
        main = importlib.import_module("main")
        frames, cameras = [], []
        iterations = [0]

        class Frame(fixture.FakeImage):
            def copy(self):
                return Frame(self.width(), self.height(), self.format())

            def find_qrcodes(self, *args, **kwargs):
                return []  # 单独验证 USER，不让有码自动切换掩盖按键路径。

        class Camera(NativeStateCamera):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                cameras.append(self)

            def read(self, **kwargs):
                frames.append((self.width, self.height, self.format()))
                return Frame(self.width, self.height, self.format())

        class Button(UserButton):
            def take_exit_request(self):
                if iterations[0] > 1:
                    # 用真实 _on_key 模拟按下100ms后松开，而非直接伪造toggle标志。
                    fixture.clock[0] += 300
                    self._on_key(fixture.maix.key.Keys.KEY_OK,
                                 fixture.maix.key.State.KEY_PRESSED)
                    fixture.clock[0] += 100
                    self._on_key(fixture.maix.key.Keys.KEY_OK,
                                 fixture.maix.key.State.KEY_RELEASED)
                return super().take_exit_request()

        def need_exit():
            iterations[0] += 1
            return iterations[0] > 9

        with patch.object(fixture.maix.camera, "Camera", Camera, create=True), \
             patch.object(fixture.maix.app, "need_exit", side_effect=need_exit, create=True), \
             patch.object(main, "init_uart", return_value=None), \
             patch.object(main, "UserButton", Button), \
             patch.multiple(config, START_MODE="QR", DISPLAY_ENABLED=False, DUAL_BUFFER=True):
            main.main()
        gray = (1920, 1440, fixture.maix.image.Format.FMT_GRAYSCALE)
        rgb = (320, 320, fixture.maix.image.Format.FMT_RGB888)
        self.assertEqual(frames, [gray, rgb] * 4 + [gray])
        self.assertEqual(len(cameras), 1)
        self.assertEqual(cameras[0].close_count, 1)
        self.assertEqual(cameras[0].buffers[1:], [1, 2] * 4 + [1])


if __name__ == "__main__":
    unittest.main()
