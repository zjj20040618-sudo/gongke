"""触摸显示回归：映射、按下沿、资源释放和UART隔离。"""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from touch_inspector import ObjectInspector, screen_to_image


def obj(cid=9, x=10, y=20, w=30, h=40, score=.9):
    return SimpleNamespace(class_id=cid, x=x, y=y, w=w, h=h, score=score)


class FakeTouch:
    def __init__(self, events=()):
        self.events = list(events)
        self.closed = False
        self.reads = 0

    def is_opened(self):
        return True

    def available(self, timeout):
        assert timeout == 0
        return bool(self.events)

    def read(self):
        self.reads += 1
        return self.events.pop(0)

    def close(self):
        self.closed = True


class InspectorTests(unittest.TestCase):
    def make(self, events=()):
        inspector = ObjectInspector((480, 320), enabled=False)
        inspector.device = FakeTouch(events)
        self.addCleanup(inspector.close)
        return inspector

    def test_centered_small_image_and_black_bars(self):
        self.assertEqual(screen_to_image((105, 40), (480, 320), (320, 320)), (25, 40))
        self.assertIsNone(screen_to_image((20, 40), (480, 320), (320, 320)))
        self.assertIsNone(screen_to_image((400, 40), (480, 320), (320, 320)))

    def test_large_image_letterbox_maps_both_axes(self):
        self.assertEqual(screen_to_image((240, 160), (480, 320), (1920, 1440)), (960, 720))
        self.assertIsNone(screen_to_image((1, 160), (480, 320), (1920, 1440)))
        self.assertIsNone(screen_to_image((1, 1), (0, 320), (320, 320)))

    def test_fast_tap_hold_release_and_event_budget(self):
        inspector = self.make([(105, 40, 1), (106, 40, 1), (106, 40, 0)])
        self.assertEqual(inspector.poll(), (105, 40))
        self.assertIsNone(inspector.poll())
        inspector.device.events = [(105, 40, 1)] * 30
        self.assertEqual(inspector.poll(), (105, 40))
        self.assertEqual(len(inspector.device.events), 22)
        self.assertIsNone(inspector.poll())

    def test_select_one_blank_restores_all_and_does_not_mutate(self):
        inspector = self.make()
        objects = [obj(), obj(4, x=150)]
        before = [vars(o).copy() for o in objects]
        self.assertEqual(inspector.choose(objects, (320, 320), (105, 40)), [objects[0]])
        self.assertIn("SELECTED", inspector.status)
        self.assertEqual(inspector.choose(objects, (320, 320), (20, 40)), [objects[0]])
        self.assertEqual(inspector.choose(objects, (320, 320), (350, 280)), objects)
        self.assertIsNone(inspector.status)
        self.assertEqual([vars(o) for o in objects], before)

    def test_can_switch_directly_to_another_object(self):
        inspector = self.make()
        objects = [obj(), obj(4, x=150)]
        inspector.choose(objects, (320, 320), (105, 40))
        self.assertEqual(inspector.choose(objects, (320, 320), (245, 40)), [objects[1]])

    def test_overlapping_boxes_choose_smallest_then_score(self):
        inspector = self.make()
        objects = [obj(w=100, h=100), obj(4), obj(5, score=.95)]
        self.assertEqual(inspector.choose(objects, (320, 320), (105, 40)), [objects[2]])

    def test_spatial_association_ignores_other_class_and_high_score_far_box(self):
        inspector = self.make()
        inspector.choose([obj()], (320, 320), (105, 40))
        near, far = obj(x=12, score=.4), obj(x=150, score=.99)
        self.assertEqual(inspector.choose([far, obj(4, x=11), near], (320, 320)), [near])

    def test_lost_selection_never_shows_old_coordinates_or_reacquires_automatically(self):
        inspector = self.make()
        inspector.choose([obj()], (320, 320), (105, 40))
        self.assertEqual(inspector.choose([], (320, 320)), [])
        self.assertIn("LOST", inspector.status)
        self.assertEqual(inspector.choose([obj()], (320, 320)), [])
        self.assertEqual(inspector.choose([obj()], (320, 320), (105, 40)), [obj()])

    def test_reset_discards_selection_but_does_not_retrigger_held_finger(self):
        inspector = self.make([(105, 40, 1)])
        inspector.choose([obj()], (320, 320), inspector.poll())
        inspector.reset()
        inspector.device.events = [(105, 40, 1)]
        self.assertIsNone(inspector.poll())
        self.assertFalse(inspector.selected)

    def test_read_error_disables_touch_without_failure_and_closes_resource(self):
        inspector = self.make()
        device = inspector.device
        inspector.choose([obj()], (320, 320), (105, 40))
        with patch.object(device, "available", side_effect=RuntimeError("driver")):
            self.assertIsNone(inspector.poll())
        self.assertTrue(device.closed)
        self.assertIsNone(inspector.device)
        self.assertFalse(inspector.selected)


class TouchMainTests(unittest.TestCase):
    def test_actual_touch_filters_display_not_black_barrel_uart_or_request_ack(self):
        import test_app
        import main
        from protocol import build_task_packet, build_ack_packet, bind_result, build_object_packet
        barrel, red = obj(), obj(4, x=150)
        sent, shown = [], []
        touch = FakeTouch()
        loops = [0]
        # 同号请求重试不得清掉触摸选择；黑桶每帧仍持续发送。
        commands = iter((build_task_packet(3, 4, 0), b"", build_task_packet(3, 4, 0)))
        serial = SimpleNamespace(read=lambda **kw: next(commands), close=lambda: None)
        def need_exit():
            loops[0] += 1
            touch.events = [(245, 40, 1), (245, 40, 0)] if loops[0] == 1 else []
            return loops[0] > 3
        def send(serial, packet):
            sent.append(packet)
            return True
        with patch.object(test_app.maix, "touchscreen", SimpleNamespace(TouchScreen=lambda: touch), create=True), \
             patch.object(test_app.maix.nn, "YOLO26", test_app.FakeModel, create=True), \
             patch.object(test_app.FakeModel, "detect", return_value=[barrel, red]), \
             patch.object(test_app.maix.camera, "Camera", test_app.FakeCamera, create=True), \
             patch.object(test_app.maix.display, "Display", return_value=SimpleNamespace(width=lambda: 480, height=lambda: 320, show=lambda im: None), create=True), \
             patch.object(test_app.maix.app, "need_exit", side_effect=need_exit, create=True), \
             patch.object(main, "init_uart", return_value=serial), \
             patch.object(main, "send_packet", side_effect=send), \
             patch.object(main, "draw_objects", side_effect=lambda im, objects, *args: shown.append(list(objects))):
            main.main()
        self.assertEqual([[o.class_id for o in frame] for frame in shown], [[4]] * 3)
        self.assertTrue(all((frame[0].x, frame[0].y) == (150, 20) for frame in shown))
        packets = [p for p in sent if p[2] == 0x62]
        self.assertEqual(packets, [bind_result(build_object_packet(seq, [barrel], 320, 320), 3) for seq in range(3)])
        self.assertEqual([p for p in sent if p[2] == 0x61], [build_ack_packet(3, 2)] * 2)
        self.assertTrue(touch.closed)

    def test_initialization_failure_also_closes_touch(self):
        import test_app
        import main
        touch = FakeTouch()
        with patch.object(test_app.maix, "touchscreen", SimpleNamespace(TouchScreen=lambda: touch), create=True), \
             patch.object(test_app.maix.camera, "Camera", test_app.FakeCamera, create=True), \
             patch.object(test_app.maix.display, "Display", return_value=SimpleNamespace(width=lambda: 480, height=lambda: 320), create=True), \
             patch.object(main, "init_uart", side_effect=RuntimeError("UART init")):
            with self.assertRaisesRegex(RuntimeError, "UART init"):
                main.main()
        self.assertTrue(touch.closed)


if __name__ == "__main__":
    unittest.main()
