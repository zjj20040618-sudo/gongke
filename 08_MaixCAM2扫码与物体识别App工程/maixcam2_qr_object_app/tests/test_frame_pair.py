"""图像副本、首帧、切任务隔离：不依赖Maix硬件。"""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import config
from frame_pair import FramePair


class FramePairTests(unittest.TestCase):
    def test_defaults_qr_single_buffer_object_dual_buffer_and_one_qr_frame(self):
        self.assertTrue(config.DUAL_BUFFER)
        self.assertEqual(config.QR_CAMERA_BUFFERS, 1)
        self.assertEqual(config.OBJECT_CAMERA_BUFFERS, 2)
        self.assertEqual(config.TASK_CONFIRM_FRAMES, 1)
        self.assertEqual(config.START_MODE, config.MODE_QR)

    def test_snapshot_is_independent_and_metadata_follows_source_frame(self):
        class Image:
            def __init__(self, pixels): self.pixels = pixels
            def copy(self): return deepcopy(self)
        pair = FramePair(True)
        original = Image([1, 2])
        self.assertIsNone(pair.align(original, 3, 100))
        original.pixels[0] = 99  # 模拟驱动复用或UI绘制。
        snapshot, capture, started = pair.align(Image([3, 4]), 5, 200)
        self.assertEqual((snapshot.pixels, capture, started), ([1, 2], 3, 100))
        self.assertIsNot(snapshot, original)
        self.assertEqual(pair.pending[0].pixels, [3, 4])

    def test_reset_primes_again_without_old_task_image(self):
        pair = FramePair(True)
        self.assertIsNone(pair.align([1], 1, 1))
        pair.reset()
        self.assertIsNone(pair.align([2], 2, 2))
        self.assertEqual(pair.align([3], 3, 3), ([2], 2, 2))

    def test_synchronous_mode_does_not_copy(self):
        image = object()
        self.assertEqual(FramePair(False).align(image, 1, 2), (image, 1, 2))

    def test_ambiguous_setting_rejected(self):
        for value in (None, 0, "True"):
            with self.assertRaises(ValueError): FramePair(value)


if __name__ == "__main__":
    unittest.main()
