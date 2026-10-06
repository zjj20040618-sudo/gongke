"""多物体标签避让与单选详情；复用现有图像记录器。"""
import unittest
import test_ui as fixture


class LayoutTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixture.UiTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.ui = self.fixture.ui

    def assert_no_overlap(self, img):
        plates = [args[:4] for args, kwargs in img.rectangles if kwargs["thickness"] == -1]
        for i, (x, y, w, h) in enumerate(plates):
            for ox, oy, ow, oh in plates[i + 1:]:
                self.assertTrue(x + w <= ox or ox + ow <= x or y + h <= oy or oy + oh <= y,
                                ((x, y, w, h), (ox, oy, ow, oh)))
        self.fixture.assert_visible(img)

    def test_nearby_object_text_is_repositioned_without_losing_coordinates(self):
        img = fixture.RecordingImage(640, 480)
        objects = [self.fixture.obj(x=240, y=190), self.fixture.obj(x=245, y=195, class_id=4)]
        self.ui.draw_objects(img, objects, (640, 480), 40)
        self.assertEqual(len(img.strings), 4)
        self.assertIn("x=255, y=212", [r.text for r in img.strings])
        self.assertIn("x=260, y=217", [r.text for r in img.strings])
        self.assert_no_overlap(img)

    def test_many_overlapping_detections_keep_boxes_without_overlapping_text(self):
        img = fixture.RecordingImage(320, 320)
        objects = [self.fixture.obj(x=140, y=180, class_id=i) for i in range(10)]
        self.ui.draw_objects(img, objects, (480, 320), 80)
        self.assertEqual(len(img.crosses), 10)
        self.assert_no_overlap(img)
        self.assertTrue(all(row.y >= 80 for row in img.strings))

    def test_selected_details_include_size_and_current_center(self):
        img = fixture.RecordingImage(320, 320)
        self.ui.draw_objects(img, [self.fixture.obj()], (480, 320), 60, True)
        self.assertIn("w=31, h=45", [r.text for r in img.strings])
        self.assertIn("x=25", [r.text for r in img.strings])
        self.assertIn("y=42", [r.text for r in img.strings])
        self.assert_no_overlap(img)

    def test_lost_status_is_shown_without_old_object_coordinates(self):
        img = fixture.RecordingImage(320, 320)
        bottom = self.ui.draw_header(img, "OBJECT", 10, 70, 2, True, "ID:9 LOST / TAP EMPTY: ALL")
        self.ui.draw_objects(img, [], (480, 320), bottom, True)
        self.assertIn("LOST", img.strings[-1].text)
        self.assertFalse(any(row.text.startswith("x=") for row in img.strings))
        self.fixture.assert_visible(img)


if __name__ == "__main__":
    unittest.main()
