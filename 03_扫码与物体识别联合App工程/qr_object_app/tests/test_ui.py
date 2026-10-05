"""显示契约：中心坐标、屏幕字号、文字边界；无需相机或 Maix 固件。"""
import importlib.util
from pathlib import Path
import struct
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch


APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
import config
from protocol import build_object_packet


class RecordingImage:
    def __init__(self, width, height):
        self._width, self._height = width, height
        self.strings, self.crosses, self.rectangles, self.edges = [], [], [], []

    def width(self):
        return self._width

    def height(self):
        return self._height

    def draw_string(self, x, y, text, color, **kwargs):
        self.strings.append(SimpleNamespace(x=x, y=y, text=text, color=color, **kwargs))

    def draw_cross(self, x, y, color, **kwargs):
        self.crosses.append((x, y))

    def draw_rect(self, *args, **kwargs):
        self.rectangles.append((args, kwargs))

    def draw_edges(self, *args, **kwargs):
        self.edges.append(args)


class UiTests(unittest.TestCase):
    def setUp(self):
        self.measured = []

        def string_size(text, scale=1, thickness=1):
            self.measured.append((text, scale, thickness))
            # Maix image.Size has integer dimensions. Rounding exposes clipping
            # that tests using perfectly linear floating measurements can miss.
            return [max(1, round(len(text) * 8 * scale)), max(1, round(12 * scale))]

        self.size = string_size
        fake_maix = ModuleType("maix")
        fake_maix.image = SimpleNamespace(
            string_size=string_size, COLOR_GREEN=1, COLOR_YELLOW=2,
            COLOR_RED=3, COLOR_BLUE=4, COLOR_WHITE=5,
            Color=SimpleNamespace(from_rgb=lambda r, g, b: (r, g, b)),
        )
        spec = importlib.util.spec_from_file_location("ui_display_contract", APP / "ui.py")
        self.ui = importlib.util.module_from_spec(spec)
        # Do not replace the fake Maix API used by the other discovery modules.
        with patch.dict(sys.modules, {"maix": fake_maix}):
            spec.loader.exec_module(self.ui)
        self.scale_patch = patch.object(config, "BOX_TEXT_SCALE", 6)
        self.scale_patch.start()
        self.addCleanup(self.scale_patch.stop)
        self.object_scale_patch = patch.object(config, "OBJECT_TEXT_SCALE", 6)
        self.object_scale_patch.start()
        self.addCleanup(self.object_scale_patch.stop)

    @staticmethod
    def obj(x=10, y=20, w=31, h=45, class_id=9):
        return SimpleNamespace(x=x, y=y, w=w, h=h, class_id=class_id, score=.82)

    @staticmethod
    def qr(x=420, y=240, payload="123", text_ascii="B:R T:G S:Drum"):
        return {"x": x, "y": y, "w": 120, "h": 120, "payload": payload,
                "text_ascii": text_ascii,
                "corners": [(x, y), (x + 120, y), (x + 120, y + 120), (x, y + 120)]}

    def assert_visible(self, img, strings=None, min_y=0):
        for row in img.strings if strings is None else strings:
            width, height = self.size(row.text, row.scale, row.thickness)
            self.assertGreaterEqual(row.x, 0, row.text)
            self.assertGreaterEqual(row.y, min_y, row.text)
            self.assertLessEqual(row.x + width, img.width(), row.text)
            self.assertLessEqual(row.y + height, img.height(), row.text)
            self.assertFalse(row.wrap)
            self.assertEqual(row.thickness, -1)

    def test_center_display_and_cross_equal_real_packet_for_odd_box_sizes(self):
        img = RecordingImage(1600, 900)
        obj = self.obj(x=19, y=23)
        self.ui.draw_objects(img, [obj], display_size=(1600, 900))
        packet = build_object_packet(7, [obj], img.width(), img.height())
        record_start = 2 + struct.calcsize("<BHBHHHHH")
        record = struct.unpack("<BHHHHH", packet[record_start:record_start + 11])
        self.assertEqual(record[2:4], (34, 45))
        self.assertEqual(img.crosses, [(34, 45)])
        self.assertIn("x=34, y=45", [row.text for row in img.strings])
        self.assert_visible(img)

    def test_hidden_information_keeps_all_boxes_and_crosses(self):
        objects = [self.obj(), self.obj(x=400, class_id=4)]
        img = RecordingImage(1600, 900)
        self.ui.draw_objects(img, objects, display_size=(1600, 900), info_objects=[objects[1]])
        self.assertEqual(img.crosses, [(25, 42), (415, 42)])
        self.assertEqual(len([r for r in img.rectangles if r[1]["thickness"] == 3]), 2)
        self.assertNotIn("x=25, y=42", [row.text for row in img.strings])
        self.assertIn("x=415, y=42", [row.text for row in img.strings])
        hidden = RecordingImage(1600, 900)
        self.ui.draw_objects(hidden, objects, info_objects=[])
        self.assertEqual(hidden.strings, [])
        self.assertEqual(hidden.crosses, img.crosses)

    def test_held_detection_info_is_marked_and_live_detection_is_not(self):
        held = self.obj()
        held.held = True
        img = RecordingImage(1600, 900)
        self.ui.draw_objects(img, [held], display_size=(1600, 900))
        self.assertTrue(any(row.text.startswith("HOLD ") for row in img.strings))
        held.held = False
        live = RecordingImage(1600, 900)
        self.ui.draw_objects(live, [held], display_size=(1600, 900))
        self.assertFalse(any(row.text.startswith("HOLD ") for row in live.strings))

    def test_wide_image_keeps_coordinates_on_one_line(self):
        img = RecordingImage(1600, 900)
        self.ui.draw_objects(img, [self.obj()], display_size=(1600, 900))
        coordinate_rows = [row for row in img.strings if row.text.startswith("x=")]
        self.assertEqual([row.text for row in coordinate_rows], ["x=25, y=42"])
        self.assertEqual(coordinate_rows[0].scale, 6)

    def test_object_labels_use_contrast_plate_and_move_outside_box_when_possible(self):
        img = RecordingImage(640, 480)
        obj = self.obj(x=240, y=180, w=100, h=100)
        self.ui.draw_objects(img, [obj], display_size=(640, 480))
        self.assertEqual(img.rectangles[0][1]["thickness"], 3)
        plates = [rect for rect in img.rectangles[1:] if rect[1]["thickness"] == -1]
        self.assertTrue(plates)
        self.assertTrue(all(rect[0][4] == self.ui._object_color(obj.class_id) for rect in plates))
        self.assertEqual(img.rectangles[0][0][4], self.ui._object_color(obj.class_id))
        self.assertTrue(all(row.color == (0, 0, 0) for row in img.strings))
        self.assertTrue(all(row.y + self.size(row.text, row.scale, row.thickness)[1] <= obj.y
                            for row in img.strings))

    def test_narrow_image_splits_x_and_y_without_dropping_values(self):
        img = RecordingImage(320, 320)
        self.ui.draw_objects(img, [self.obj(x=150, y=210)], display_size=(640, 480))
        self.assertEqual([row.text for row in img.strings[1:]], ["x=165", "y=232"])
        self.assertEqual([row.scale for row in img.strings[1:]], [6, 6])
        self.assert_visible(img)

    def test_qr_and_object_coordinates_share_effective_screen_font_size(self):
        screen_size = (640, 480)
        obj_img, qr_img = RecordingImage(640, 480), RecordingImage(1600, 900)
        self.ui.draw_objects(obj_img, [self.obj()], display_size=screen_size)
        self.ui.draw_qrs(qr_img, [self.qr()], display_size=screen_size)
        coordinate_row = next(row for row in obj_img.strings if row.text.startswith("x="))
        qr_row = next(row for row in qr_img.strings if row.text == "QR:123")
        qr_fit = min(screen_size[0] / qr_img.width(), screen_size[1] / qr_img.height())
        self.assertAlmostEqual(coordinate_row.scale, qr_row.scale * qr_fit)
        self.assertEqual(coordinate_row.scale, 6)
        self.assertEqual(qr_row.scale, 15)
        self.assert_visible(obj_img)
        self.assert_visible(qr_img)

    def test_small_image_is_not_compensated_for_upscaling(self):
        img = RecordingImage(320, 320)
        self.assertEqual(self.ui._result_scale(img, (640, 480)), 6)
        self.assertEqual(self.ui._result_scale(img, (320, 320)), 6)

    def test_height_limited_screen_controls_qr_scale(self):
        img = RecordingImage(1600, 900)
        self.assertAlmostEqual(self.ui._result_scale(img, (1920, 300)), 18)

    def test_default_display_dimensions_remain_valid_without_display(self):
        img = RecordingImage(1600, 900)
        expected = 6 * max(1, 1600 / config.OBJECT_WIDTH, 900 / config.OBJECT_HEIGHT)
        self.assertEqual(self.ui._result_scale(img), expected)

    def test_bottom_right_object_block_remains_fully_inside_image(self):
        img = RecordingImage(320, 320)
        self.ui.draw_objects(img, [self.obj(x=300, y=300)], display_size=(640, 480))
        self.assertEqual([row.text for row in img.strings[1:]], ["x=315", "y=322"])
        self.assert_visible(img)

    def test_qr_block_at_bottom_right_preserves_all_three_task_lines(self):
        img = RecordingImage(1600, 900)
        self.ui.draw_qrs(img, [self.qr(x=1550, y=880)], display_size=(640, 480))
        self.assertEqual([row.text for row in img.strings], ["QR:123", "B:R T:G", "S:Drum"])
        self.assert_visible(img)

    def test_long_class_name_does_not_remove_center_coordinates(self):
        img = RecordingImage(320, 320)
        self.ui.draw_objects(img, [self.obj(class_id=2, x=295, y=295)], display_size=(640, 480))
        self.assertEqual(img.strings[0].text, "truncated_cone 0.82")
        self.assertEqual([row.text for row in img.strings[1:]], ["x=310", "y=317"])
        self.assert_visible(img)

    def test_long_raw_qr_text_is_fitted_without_dropping_payload(self):
        img = RecordingImage(1600, 900)
        payload = "unrecognized-data-" * 30
        self.ui.draw_qrs(img, [self.qr(x=1550, y=880, payload=payload, text_ascii="RAW")],
                         display_size=(640, 480))
        self.assertEqual([row.text for row in img.strings], ["QR:" + payload, "RAW"])
        self.assert_visible(img)

    def test_header_boundary_keeps_object_text_below_header(self):
        img = RecordingImage(320, 320)
        header_bottom = self.ui.draw_header(img, config.MODE_OBJECT, 12.3, 70, 2, True)
        header_rows = list(img.strings)
        self.ui.draw_objects(img, [self.obj(x=0, y=0)], display_size=(640, 480), min_y=header_bottom)
        labels = img.strings[len(header_rows):]
        self.assert_visible(img, header_rows)
        self.assert_visible(img, labels, min_y=header_bottom)
        for first, second in zip(header_rows, header_rows[1:]):
            _, height = self.size(first.text, first.scale, first.thickness)
            self.assertLessEqual(first.y + height, second.y)

    def test_header_boundary_keeps_qr_text_below_header(self):
        img = RecordingImage(1600, 900)
        header_bottom = self.ui.draw_header(img, config.MODE_QR, 2.0, 300, 3)
        header_count = len(img.strings)
        self.ui.draw_qrs(img, [self.qr(x=0, y=0)], display_size=(640, 480), min_y=header_bottom)
        self.assert_visible(img, img.strings[header_count:], min_y=header_bottom)

    def test_measurement_and_drawing_use_matching_filled_font_thickness(self):
        img = RecordingImage(320, 320)
        self.ui.draw_objects(img, [self.obj(class_id=2)], display_size=(640, 480))
        measured_before_assert = list(self.measured)
        self.assertTrue(measured_before_assert)
        self.assertTrue(all(item[2] == -1 for item in measured_before_assert))
        for row in img.strings:
            self.assertIn((row.text, row.scale, row.thickness), measured_before_assert)
            self.assertFalse(row.wrap)

    def test_empty_detection_lists_draw_no_stale_labels(self):
        img = RecordingImage(320, 320)
        self.ui.draw_objects(img, [], display_size=(640, 480))
        self.ui.draw_qrs(img, [], display_size=(640, 480))
        self.assertEqual(img.strings, [])
        self.assertEqual(img.crosses, [])

    def test_object_size_three_does_not_change_qr_size(self):
        img, qr_img = RecordingImage(640, 480), RecordingImage(1600, 900)
        with patch.object(config, "OBJECT_TEXT_SCALE", 3):
            self.ui.draw_objects(img, [self.obj()], display_size=(640, 480))
            self.ui.draw_qrs(qr_img, [self.qr()], display_size=(640, 480))
        self.assertTrue(all(row.scale == 3 and row.thickness == -1 for row in img.strings))
        self.assertEqual(qr_img.strings[0].scale, 15)  # 原来的QR缩小补偿仍生效。

    def test_all_ten_class_labels_use_their_own_matching_color_plate(self):
        colors = []
        for class_id in range(10):
            img = RecordingImage(1600, 900)
            self.ui.draw_objects(img, [self.obj(class_id=class_id)], display_size=(1600, 900))
            color = img.rectangles[0][0][4]
            colors.append(color)
            plates = [rect for rect in img.rectangles[1:] if rect[1]["thickness"] == -1]
            self.assertTrue(plates)
            self.assertTrue(all(rect[0][4] == color for rect in plates))
            self.assertTrue(all(row.color == (0, 0, 0) for row in img.strings))
            self.assert_visible(img)
        self.assertEqual(len(set(colors)), 10)


if __name__ == "__main__":
    unittest.main()
