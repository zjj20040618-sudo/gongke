"""模式状态机：集中管理分辨率和YOLO模型生命周期。"""
import gc
from maix import err, time
import config
from object_detector import ObjectDetector

class ModeController:
    def __init__(self, cam):
        self.cam, self.mode, self.detector = cam, None, None
        self.width, self.height = 0, 0

    def _resolution_for(self, mode):
        return (config.QR_WIDTH, config.QR_HEIGHT) if mode == config.MODE_QR else (config.OBJECT_WIDTH, config.OBJECT_HEIGHT)

    def enter(self, new_mode):
        if new_mode not in (config.MODE_QR, config.MODE_OBJECT):
            raise ValueError("unknown mode: " + str(new_mode))
        old_size = (self.width, self.height)
        width, height = self._resolution_for(new_mode)
        started = time.ticks_ms()
        result = self.cam.set_resolution(width, height)
        if result != err.Err.ERR_NONE:
            raise RuntimeError("set_resolution {}x{} failed: {}".format(width, height, result))
        try:
            if new_mode == config.MODE_OBJECT and self.detector is None:
                self.detector = ObjectDetector()
            elif new_mode == config.MODE_QR and self.detector is not None:
                self.detector.close(); self.detector = None; gc.collect()
        except Exception:
            if old_size[0] > 0:
                self.cam.set_resolution(old_size[0], old_size[1])
            raise
        self.mode, self.width, self.height = new_mode, width, height
        print("[MODE] {} {}x{} switch={}ms".format(new_mode, width, height, time.ticks_ms() - started))

    def toggle(self):
        self.enter(config.MODE_OBJECT if self.mode == config.MODE_QR else config.MODE_QR)

    def close(self):
        if self.detector is not None:
            self.detector.close(); self.detector = None

