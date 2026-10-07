"""扫码区域细框：与解码ROI一致，空结果也显示，画框在解码后。"""
import importlib
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import test_ui as fixture
import test_app
import config
from qr_reader import QrReader, center_roi
from task_selection import TaskSelection
from touch_inspector import ObjectInspector
from protocol import build_control_packet, bind_result, build_qr_packet


class QrRegionTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixture.UiTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.ui = self.fixture.ui

    def test_empty_qr_still_draws_exact_decode_region_with_thin_outline(self):
        img = fixture.RecordingImage(1920, 1440)
        self.ui.draw_qrs(img, [], (480, 320))
        self.assertEqual(len(img.rectangles), 1)
        args, kwargs = img.rectangles[0]
        self.assertEqual(list(args[:4]), center_roi(img))
        self.assertEqual(args[:4], (480, 360, 960, 720))
        self.assertEqual(args[4], (0, 255, 255))
        self.assertEqual(kwargs["thickness"], 4)
        self.assertEqual(img.strings, [])

    def test_smaller_image_uses_one_pixel_outline_and_current_roi_setting(self):
        img = fixture.RecordingImage(320, 240)
        with patch.object(config, "QR_ROI_FRACTION", .8):
            self.ui.draw_qrs(img, [], (480, 320))
            self.assertEqual(list(img.rectangles[0][0][:4]), center_roi(img))
        self.assertEqual(img.rectangles[0][1]["thickness"], 1)

    def test_full_image_scan_outline_stays_inside_edges(self):
        img = fixture.RecordingImage(640, 480)
        with patch.object(config, "QR_ROI_FRACTION", 1.0):
            self.assertEqual(center_roi(img), [])
            self.ui.draw_qrs(img, [], (640, 480))
        self.assertEqual(img.rectangles[0][0][:4], (0, 0, 639, 479))

    def test_object_mode_does_not_draw_scan_region(self):
        img = fixture.RecordingImage(320, 320)
        self.ui.draw_objects(img, [])
        self.assertEqual(img.rectangles, [])

    def test_fixed_center_moves_region_up_and_clamps_all_edges(self):
        img = fixture.RecordingImage(1920, 1440)
        with patch.multiple(config, QR_ROI_CENTER_X=.5, QR_ROI_CENTER_Y=.35):
            self.assertEqual(center_roi(img), [480, 144, 960, 720])
        self.assertEqual(center_roi(img, (0, 0)), [0, 0, 960, 720])
        self.assertEqual(center_roi(img, (1, 1)), [960, 720, 960, 720])

    def test_touch_accounts_for_letterbox_and_decode_draw_share_moved_roi(self):
        img = fixture.RecordingImage(1920, 1440)
        reader = QrReader()
        self.assertFalse(reader.move_to(img, (0, 96), (480, 320)))
        self.assertTrue(reader.move_to(img, (240, 96), (480, 320)))
        self.assertEqual(reader.roi(img), [480, 72, 960, 720])
        captured = []
        img.find_qrcodes = lambda roi, **kwargs: captured.append(roi) or []
        reader.decode(img)
        self.ui.draw_qrs(img, [], (480, 320), roi=reader.roi(img))
        self.assertEqual(captured[0], list(img.rectangles[0][0][:4]))
        self.assertFalse(reader.move_to(img, (240, 96), (480, 320)))

    def test_touch_clamps_actual_center_and_full_frame_is_immovable(self):
        img = fixture.RecordingImage(1920, 1440)
        reader = QrReader()
        self.assertTrue(reader.move_to(img, (27, 0), (480, 320)))
        self.assertEqual(reader.roi(img), [0, 0, 960, 720])
        self.assertEqual(reader.center, (.25, .25))
        with patch.object(config, "QR_ROI_FRACTION", 1):
            self.assertFalse(reader.move_to(img, (240, 160), (480, 320)))
            self.assertEqual(reader.roi(img), [])

    def test_invalid_region_settings_fail_clearly(self):
        img = fixture.RecordingImage(1920, 1440)
        for value in (0, -1, 1.01, float("nan"), float("inf")):
            with self.subTest(fraction=value), patch.object(config, "QR_ROI_FRACTION", value):
                with self.assertRaisesRegex(ValueError, "QR_ROI_FRACTION"):
                    center_roi(img)
        for center in ((-.1, .5), (.5, 1.1), (.5, float("nan"))):
            with self.subTest(center=center), self.assertRaisesRegex(ValueError, "center"):
                center_roi(img, center)

    def run_touch_loop(self, taps, enabled=True):
        main = importlib.import_module("main")
        events, packets = [], []
        task = TaskSelection()
        tap_events = iter(taps)
        inspector = ObjectInspector((480, 320), enabled=False)
        inspector.poll = lambda: next(tap_events)
        class Frame(test_app.FakeImage):
            def find_qrcodes(self, roi, **kwargs):
                events.append(("decode", tuple(roi)))
                return [test_app.FakeCode()]
            def draw_rect(self, x, y, w, h, color, **kwargs):
                if color == (0, 255, 255):
                    events.append(("outline", (x, y, w, h)))
        class Camera(test_app.FakeCamera):
            def read(self, **kwargs):
                return Frame(self.width, self.height, self.pixel_format)
        incoming = iter([build_control_packet(17, 1)] + [b""] * (len(taps) - 1))
        exits = iter([False] * len(taps) + [True])
        with patch.object(test_app.maix.camera, "Camera", Camera, create=True), \
             patch.object(test_app.maix.image, "Image", Frame), \
             patch.object(test_app.maix.display, "Display", return_value=SimpleNamespace(width=lambda: 480, height=lambda: 320, show=lambda img: None), create=True), \
             patch.object(test_app.maix.app, "need_exit", side_effect=lambda: next(exits), create=True), \
             patch.object(main, "init_uart", return_value=SimpleNamespace(read=lambda **kwargs: next(incoming))), \
             patch.object(main, "send_packet", side_effect=lambda serial, packet: packets.append(packet) or True), \
             patch.object(main, "TaskSelection", return_value=task), \
             patch.object(main, "ObjectInspector", return_value=inspector), \
             patch.multiple(config, QR_ROI_TOUCH_MOVE=enabled, DISPLAY_ENABLED=True, START_MODE="IDLE", TASK_CONFIRM_FRAMES=3):
            main.main()
        return task, events, [p for p in packets if p[2] == 0x62]

    def test_main_move_restarts_only_unconfirmed_candidate_and_preserves_locked_uart(self):
        task, events, packets = self.run_touch_loop([None, None, (240, 96), None, None, (240, 224)])
        self.assertEqual([p for event, p in events if event == "decode"],
                         [(480, 360, 960, 720)] * 2 + [(480, 72, 960, 720)] * 3 + [(480, 648, 960, 720)])
        self.assertEqual([p for event, p in events if event == "outline"],
                         [(133, 80, 213, 160)] * 2 + [(133, 16, 213, 160)] * 3 + [(133, 144, 213, 160)])
        self.assertEqual(packets, [bind_result(build_qr_packet(i, None if i < 4 else "123"), 17) for i in range(6)])
        self.assertEqual(task.payload, "123")

    def test_main_disabled_move_does_not_restart_confirmation(self):
        task, events, packets = self.run_touch_loop([None, None, (240, 96)], enabled=False)
        self.assertEqual([p for event, p in events if event == "decode"], [(480, 360, 960, 720)] * 3)
        self.assertEqual(packets[-1], bind_result(build_qr_packet(2, "123"), 17))
        self.assertEqual(task.payload, "123")

    def test_main_draws_region_each_empty_qr_frame_only_after_decoding(self):
        main = importlib.import_module("main")
        events = []
        class Frame(test_app.FakeImage):
            def find_qrcodes(self, roi, **kwargs):
                events.append(("decode", tuple(roi)))
                return []
            def draw_rect(self, x, y, w, h, color, **kwargs):
                if color == (0, 255, 255):
                    events.append(("outline", (x, y, w, h)))
        class Camera(test_app.FakeCamera):
            def read(self, **kwargs):
                return Frame(self.width, self.height, self.pixel_format)
        exits = iter((False, False, True))
        with patch.object(test_app.maix.camera, "Camera", Camera, create=True), \
             patch.object(test_app.maix.image, "Image", Frame), \
             patch.object(test_app.maix.display, "Display", return_value=SimpleNamespace(width=lambda: 480, height=lambda: 320, show=lambda img: events.append(("show", None))), create=True), \
             patch.object(test_app.maix.app, "need_exit", side_effect=lambda: next(exits), create=True), \
             patch.object(main, "init_uart", return_value=None), \
             patch.object(config, "START_MODE", config.MODE_QR):
            main.main()
        expected = [("decode", (480, 360, 960, 720)), ("outline", (133, 80, 213, 160)), ("show", None)]
        self.assertEqual(events, expected * 2)


if __name__ == "__main__":
    unittest.main()
