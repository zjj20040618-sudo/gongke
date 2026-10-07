"""灰度原始采集、测试版预览和单帧确认；所有UART格式保持原契约。"""
import contextlib
import importlib
import io
import itertools
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import test_app as fixture
import config
from mode_controller import ModeController
from protocol import build_control_packet, build_qr_packet, build_ack_packet, bind_result
from task_selection import TaskSelection


class GrayCaptureTests(unittest.TestCase):
    def setUp(self):
        model = patch.object(fixture.maix.nn, "YOLO26", fixture.FakeModel, create=True)
        model.start()
        self.addCleanup(model.stop)
        redirect = contextlib.redirect_stdout(io.StringIO())
        redirect.__enter__()
        self.addCleanup(redirect.__exit__, None, None, None)

    def test_default_single_frame_confirms_all_27_legal_tasks(self):
        self.assertEqual(config.TASK_CONFIRM_FRAMES, 1)
        for digits in itertools.product("123", repeat=3):
            task = TaskSelection()
            payload = "".join(digits)
            self.assertTrue(task.observe_qrs([{"payload": payload}]))
            self.assertEqual(task.payload, payload)
            self.assertFalse(task.observe_qrs([{"payload": "111"}]))
            self.assertEqual(task.payload, payload)

    def test_single_frame_still_rejects_empty_invalid_and_conflicting_codes(self):
        for codes in ([], ["11"], ["1234"], ["120"], [None], ["123", "321"], ["123", "bad"]):
            task = TaskSelection()
            self.assertFalse(task.observe_qrs([{"payload": value} for value in codes]))
            self.assertIsNone(task.payload)
        task = TaskSelection()
        self.assertTrue(task.observe_qrs([{"payload": "123"}] * 3))

    def test_qr_rgb_roundtrip_warms_gray_twice_and_retries_do_not_reopen(self):
        cam = fixture.FakeCamera()
        modes = ModeController(cam)
        modes.enter("QR")
        self.assertEqual(cam.format(), fixture.maix.image.Format.FMT_GRAYSCALE)
        self.assertEqual(cam.calls, [(config.QR_WIDTH, config.QR_HEIGHT)])
        self.assertEqual(cam.warmups, [2])
        modes.enter("QR")
        self.assertEqual(cam.close_count, 0)
        modes.enter("OBJECT")
        self.assertEqual(cam.format(), fixture.maix.image.Format.FMT_RGB888)
        self.assertEqual(cam.calls[-1], (320, 320))
        modes.enter("IDLE")
        modes.enter("QR")
        self.assertEqual(cam.warmups, [2, 2])
        modes.close()
        self.assertEqual(cam.close_count, 1)

    def test_failed_open_restores_previous_gray_format_and_resolution(self):
        cam = fixture.FakeCamera()
        modes = ModeController(cam)
        modes.enter("QR")
        cam.results = [1, 0]
        with self.assertRaises(RuntimeError):
            modes.enter("OBJECT")
        self.assertEqual((modes.mode, cam.format(), cam.width, cam.height),
                         ("QR", fixture.maix.image.Format.FMT_GRAYSCALE, config.QR_WIDTH, config.QR_HEIGHT))
        self.assertIsNone(modes.detector)

    def test_failed_warmup_restores_previous_rgb_mode(self):
        cam = fixture.FakeCamera()
        modes = ModeController(cam)
        modes.enter("OBJECT")
        detector = modes.detector
        with patch.object(cam, "skip_frames", side_effect=RuntimeError("warmup")):
            with self.assertRaises(RuntimeError):
                modes.enter("QR")
        self.assertEqual((modes.mode, cam.format(), cam.width, cam.height),
                         ("OBJECT", fixture.maix.image.Format.FMT_RGB888, 320, 320))
        self.assertIs(modes.detector, detector)

    def test_actual_loop_decodes_raw_gray_then_small_rgb_preview_and_sends_first_frame(self):
        self.run_preview_loop(qr_preview=True, object_preview=True)

    def test_default_black_qr_page_skips_conversion_and_restores_object_camera(self):
        self.assertFalse(config.QR_CAMERA_PREVIEW_ENABLED)
        self.assertTrue(config.OBJECT_CAMERA_PREVIEW_ENABLED)
        self.run_preview_loop(qr_preview=False, object_preview=True)

    def test_both_previews_can_be_hidden_without_changing_uart(self):
        self.run_preview_loop(qr_preview=False, object_preview=False)

    def run_preview_loop(self, qr_preview, object_preview):
        main = importlib.import_module("main")
        events, packets, shown = [], [], []
        test = self
        class Frame(fixture.FakeImage):
            def copy(self):
                return Frame(self.width(), self.height(), self.pixel_format)
            def find_qrcodes(self, roi, **kwargs):
                test.assertEqual(self.format(), fixture.maix.image.Format.FMT_GRAYSCALE)
                events.append("decode")
                return [fixture.FakeCode()]
            def resize(self, width, height, **kwargs):
                test.assertEqual(events[-1], "decode")
                test.assertEqual(kwargs, {"fit": fixture.maix.image.Fit.FIT_CONTAIN})
                events.append("resize")
                return super().resize(width, height, **kwargs)
            def to_format(self, pixel_format):
                test.assertEqual((self.width(), self.height()), (480, 320))
                events.append("preview_rgb")
                return super().to_format(pixel_format)
        class Camera(fixture.FakeCamera):
            def read(self, **kwargs):
                test.assertEqual(kwargs, {"block": True, "block_ms": 2000} if
                                 self.pixel_format == fixture.maix.image.Format.FMT_GRAYSCALE else {})
                if self.pixel_format == fixture.maix.image.Format.FMT_RGB888:
                    events.append("object_read")
                return Frame(self.width, self.height, self.pixel_format)
        incoming = iter([build_control_packet(5, 1), b""])
        exits = iter((False, False, True))
        def show(img):
            test.assertEqual((img.width(), img.height(), img.format()),
                             ((480, 320) if not shown else (320, 320)) +
                             (fixture.maix.image.Format.FMT_RGB888,))
            # 关闭预览时只能显示独立画布；识别预览开启时直接显示配对的真实帧。
            test.assertEqual(isinstance(img, Frame), qr_preview if not shown else object_preview)
            shown.append(img)
            events.append("show")
        with patch.object(fixture.maix.camera, "Camera", Camera, create=True), \
             patch.object(fixture.maix.display, "Display", return_value=SimpleNamespace(width=lambda: 480, height=lambda: 320, show=show), create=True), \
             patch.object(fixture.maix.app, "need_exit", side_effect=lambda: next(exits), create=True), \
             patch.object(main, "init_uart", return_value=SimpleNamespace(read=lambda **kwargs: next(incoming))), \
             patch.object(main, "send_packet", side_effect=lambda serial, packet: packets.append(packet) or True), \
             patch.multiple(config, DISPLAY_ENABLED=True, START_MODE="IDLE",
                            QR_WIDTH=1920, QR_HEIGHT=1440,
                            QR_CAMERA_PREVIEW_ENABLED=qr_preview,
                            OBJECT_CAMERA_PREVIEW_ENABLED=object_preview):
            main.main()
        self.assertEqual(events, ["decode"] + (["resize", "preview_rgb"] if qr_preview else []) +
                         ["show", "object_read", "show"])
        self.assertEqual(packets, [build_ack_packet(5, 1)] +
                         [bind_result(build_qr_packet(seq, "123"), 5) for seq in range(2)])

    def test_preview_remaps_roi_and_boxes_without_mutating_source_results(self):
        ui = importlib.import_module("ui")
        frame = fixture.FakeImage(1920, 1440, fixture.maix.image.Format.FMT_GRAYSCALE)
        code = dict(payload="123", text_ascii="B:R T:G S:Drum", x=480, y=360, w=120, h=120,
                    corners=[(480, 360), (600, 360), (600, 480), (480, 480)])
        canvas, qrs, roi = ui.make_qr_preview(frame, [code], [480, 360, 960, 720], (480, 320))
        self.assertEqual((canvas.width(), canvas.height()), (480, 320))
        self.assertEqual(roi, [133, 80, 213, 160])
        self.assertEqual((qrs[0]["x"], qrs[0]["y"], qrs[0]["w"], qrs[0]["h"]), (133, 80, 26, 26))
        self.assertEqual(code["x"], 480)
        self.assertEqual(frame.format(), fixture.maix.image.Format.FMT_GRAYSCALE)
        self.assertEqual(qrs[0]["corners"][0], (133, 80))

    def test_black_preview_keeps_roi_mapping_and_never_reads_camera_pixels(self):
        ui = importlib.import_module("ui")
        frame = fixture.FakeImage(1920, 1440, fixture.maix.image.Format.FMT_GRAYSCALE)
        code = dict(payload="123", text_ascii="B:R T:G S:Drum", x=480, y=360, w=120, h=120,
                    corners=[(480, 360), (600, 360), (600, 480), (480, 480)])
        with patch.object(frame, "resize", side_effect=AssertionError("camera resized")), \
             patch.object(frame, "to_format", side_effect=AssertionError("camera converted")):
            canvas, qrs, roi = ui.make_qr_preview(
                frame, [code], [480, 360, 960, 720], (480, 320), show_camera=False)
        self.assertIsNot(canvas, frame)
        self.assertEqual((canvas.width(), canvas.height(), canvas.format()), (480, 320, 1))
        self.assertEqual(roi, [133, 80, 213, 160])
        self.assertEqual(qrs[0]["corners"][0], (133, 80))
        self.assertEqual(code["x"], 480)
        self.assertEqual(frame.format(), fixture.maix.image.Format.FMT_GRAYSCALE)

    def test_black_preview_without_downscaling_preserves_original_coordinates(self):
        ui = importlib.import_module("ui")
        frame = fixture.FakeImage(320, 240, fixture.maix.image.Format.FMT_GRAYSCALE)
        roi = [80, 60, 160, 120]
        canvas, qrs, mapped_roi = ui.make_qr_preview(frame, [], roi, (480, 320), show_camera=False)
        self.assertEqual((canvas.width(), canvas.height()), (320, 240))
        self.assertEqual(mapped_roi, roi)
        self.assertIsNot(canvas, frame)


if __name__ == "__main__":
    unittest.main()
