"""05筛选行为在03中的回归：三码确认、当前目标和诊断/任务分工。"""
import contextlib
import io
import itertools
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import config

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from task_selection import TaskSelection


def qr(payload):
    return {"payload": payload}


def obj(class_id, score=.9, x=10, **fields):
    values = dict(class_id=class_id, score=score, x=x, y=20, w=30, h=40)
    values.update(fields)
    return SimpleNamespace(**values)


class TaskFilterTests(unittest.TestCase):
    def setUp(self):
        confirmation = patch.object(config, "TASK_CONFIRM_FRAMES", 3)
        confirmation.start()  # 保留可配置多帧确认的旧回归；默认单帧另有测试。
        self.addCleanup(confirmation.stop)
        redirect = contextlib.redirect_stdout(io.StringIO())
        redirect.__enter__()
        self.addCleanup(redirect.__exit__, None, None, None)

    def confirm(self, task, payload):
        self.assertFalse(task.observe_qrs([qr(payload)]))
        self.assertFalse(task.observe_qrs([qr(payload)]))
        self.assertIsNone(task.payload)
        self.assertTrue(task.observe_qrs([qr(payload)]))

    def test_all_27_tasks_select_three_required_classes_without_bucket(self):
        expected = ({"1": 4, "2": 5, "3": 3}, {"1": 6, "2": 8, "3": 7}, {"1": 1, "2": 2, "3": 0})
        objects = [obj(cid) for cid in range(10)]
        for digits in itertools.product("123", repeat=3):
            with self.subTest(digits=digits):
                task = TaskSelection()
                self.confirm(task, "".join(digits))
                selected = task.select(objects, include_barrel=False, img_w=640, img_h=480)
                self.assertEqual([item.class_id for item in selected], [table[digit] for table, digit in zip(expected, digits)])

    def test_no_task_does_not_select_standalone_objects(self):
        task = TaskSelection()
        self.assertEqual(task.select([obj(i) for i in range(10)], include_barrel=False), [])
        self.assertEqual([o.class_id for o in task.select([obj(9)])], [9])
        self.assertEqual([o.class_id for o in task.select([obj(9)], 9)], [9])

    def test_empty_invalid_conflicting_and_changed_code_restart_confirmation(self):
        for interruption in ([], [qr("120")], [qr(None)], [qr("331"), qr("123")], [qr("222")]):
            with self.subTest(interruption=interruption):
                task = TaskSelection()
                task.observe_qrs([qr("331")])
                task.observe_qrs([qr("331")])
                self.assertFalse(task.observe_qrs(interruption))
                self.confirm(task, "331")

    def test_duplicate_same_codes_count_once_per_frame_and_late_conflict_is_seen(self):
        task = TaskSelection()
        self.assertFalse(task.observe_qrs([qr("331")] * 10))
        self.assertEqual(task.candidate_count, 1)
        self.assertFalse(task.observe_qrs([qr("331")] * 4 + [qr("123")]))
        self.assertEqual(task.candidate_count, 0)
        self.confirm(task, "331")

    def test_locked_task_survives_disappearance_other_codes_and_reset_clears_count(self):
        task = TaskSelection()
        self.confirm(task, "331")
        for frame in ([], [qr("123")], [qr("bad")]):
            self.assertFalse(task.observe_qrs(frame))
            self.assertEqual(task.payload, "331")
        task.reset()
        self.confirm(task, "123")

    def test_best_current_target_missing_frame_and_invalid_candidates(self):
        task = TaskSelection()
        self.confirm(task, "331")
        best = obj(3, .95, x=100)
        bad = [obj(3, float("nan")), obj(3, float("inf")), obj(3, 1.1), obj(3, .1),
               obj(3, x=-1), obj(3, w=0), obj(3, x=630), obj(3, h=500), obj(True)]
        selected = task.select(bad + [obj(3, .4), best, obj(3, .95, x=200), obj(4)],
                               include_barrel=False, img_w=640, img_h=480)
        self.assertEqual(selected, [best])
        self.assertEqual(task.select([], include_barrel=False, img_w=640, img_h=480), [])


if __name__ == "__main__":
    unittest.main()
