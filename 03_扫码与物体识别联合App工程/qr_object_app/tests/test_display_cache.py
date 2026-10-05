"""显示防闪烁：按真实毫秒到期，不复用原检测对象，不改变实时任务结果。"""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from display_cache import DisplayCache
from touch_inspector import ObjectInspector


def obj(cid=9, x=10, score=.9):
    return SimpleNamespace(class_id=cid, score=score, x=x, y=20, w=30, h=40)


class DisplayCacheTests(unittest.TestCase):
    def test_missing_frame_holds_until_exact_expiry_without_extending_timer(self):
        cache = DisplayCache(200)
        source = obj()
        before = vars(source).copy()
        boxes, defaults = cache.update([source], [source], (320, 320), 1000)
        self.assertIsNot(boxes[0], source)
        self.assertFalse(boxes[0].held)
        self.assertEqual(defaults, boxes)
        for now in (1050, 1150, 1199):
            boxes, defaults = cache.update([], [], (320, 320), now)
            self.assertEqual(len(boxes), 1)
            self.assertTrue(boxes[0].held)
            self.assertEqual(defaults, boxes)
        self.assertEqual(cache.update([], [], (320, 320), 1200), ([], []))
        self.assertEqual(vars(source), before)

    def test_detected_box_updates_immediately_and_renews_its_own_deadline(self):
        cache = DisplayCache(200)
        cache.update([obj()], [], (320, 320), 0)
        moved = obj(x=12, score=.7)
        boxes, defaults = cache.update([moved], [moved], (320, 320), 150)
        self.assertEqual(len(boxes), 1)
        self.assertEqual((boxes[0].x, boxes[0].score), (12, .7))
        self.assertFalse(boxes[0].held)
        self.assertEqual(defaults, boxes)
        self.assertEqual(len(cache.update([], [], (320, 320), 349)[0]), 1)
        self.assertEqual(cache.update([], [], (320, 320), 350), ([], []))

    def test_one_missing_object_does_not_expire_another_or_duplicate_matching_box(self):
        cache = DisplayCache(200)
        first, second = obj(), obj(x=150)
        cache.update([first, second], [first], (320, 320), 0)
        moved = obj(x=152)
        boxes, defaults = cache.update([moved], [], (320, 320), 100)
        self.assertEqual([(box.x, box.held) for box in boxes], [(152, False), (10, True)])
        self.assertEqual([box.x for box in defaults], [10])
        boxes, defaults = cache.update([obj(x=154)], [], (320, 320), 200)
        self.assertEqual([(box.x, box.held) for box in boxes], [(154, False)])
        self.assertEqual(defaults, [])

    def test_other_class_at_same_position_does_not_replace_old_box(self):
        cache = DisplayCache(200)
        cache.update([obj(9)], [], (320, 320), 0)
        boxes, _ = cache.update([obj(4)], [], (320, 320), 50)
        self.assertEqual([(box.class_id, box.held) for box in boxes], [(4, False), (9, True)])

    def test_zero_hold_disables_retention(self):
        cache = DisplayCache(0)
        self.assertEqual(len(cache.update([obj()], [], (320, 320), 0)[0]), 1)
        self.assertEqual(cache.update([], [], (320, 320), 0), ([], []))

    def test_resolution_change_clock_rollback_and_explicit_reset_discard_cache(self):
        cache = DisplayCache(200)
        for action in ("resize", "clock", "reset"):
            cache.update([obj()], [], (320, 320), 1000)
            if action == "reset":
                cache.reset()
            size = (640, 480) if action == "resize" else (320, 320)
            now = 999 if action == "clock" else 1050
            self.assertEqual(cache.update([], [], size, now), ([], []))

    def test_touch_toggle_survives_brief_dropout_then_clears_after_expiry(self):
        cache = DisplayCache(200)
        inspector = ObjectInspector((320, 320), enabled=False)
        boxes, defaults = cache.update([obj()], [], (320, 320), 0)
        self.assertEqual(inspector.choose(boxes, (320, 320), (25, 40), defaults), boxes)
        boxes, defaults = cache.update([], [], (320, 320), 100)
        self.assertEqual(inspector.choose(boxes, (320, 320), default_visible=defaults), boxes)
        boxes, defaults = cache.update([obj(x=12)], [], (320, 320), 150)
        self.assertEqual(inspector.choose(boxes, (320, 320), default_visible=defaults), boxes)
        boxes, defaults = cache.update([], [], (320, 320), 350)
        self.assertEqual(inspector.choose(boxes, (320, 320), default_visible=defaults), [])
        self.assertFalse(inspector.selected)
        boxes, defaults = cache.update([obj()], [], (320, 320), 400)
        self.assertEqual(inspector.choose(boxes, (320, 320), default_visible=defaults), [])


if __name__ == "__main__":
    unittest.main()
