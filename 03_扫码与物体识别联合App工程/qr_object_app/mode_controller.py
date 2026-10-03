"""集中管理模式、摄像头分辨率和模型；切换失败尽量恢复原模式。"""
import gc
from maix import err, time
import config
from object_detector import ObjectDetector


class ModeController:
    def __init__(self, cam):
        self.cam, self.mode, self.detector = cam, None, None
        self.width, self.height = config.OBJECT_WIDTH, config.OBJECT_HEIGHT

    def enter(self, new_mode):
        if new_mode == "IDLE":
            self.mode = new_mode  # 电控停识别；保留模型，下次请求可复用。
            return
        if new_mode not in (config.MODE_QR, config.MODE_OBJECT):
            raise ValueError("unknown mode: " + str(new_mode))
        if new_mode == self.mode:
            return
        old_size = (self.width, self.height)
        candidate = self.detector
        resolution_attempted = False
        started = time.ticks_ms()
        try:
            if new_mode == config.MODE_OBJECT:
                if candidate is None:
                    candidate = ObjectDetector()
                # 不猜测转换后的模型尺寸，读取实际输入宽高。
                width, height = candidate.input_width, candidate.input_height
            else:
                width, height = config.QR_WIDTH, config.QR_HEIGHT
            resolution_attempted = True
            result = self.cam.set_resolution(width, height)
            if result != err.Err.ERR_NONE:
                raise RuntimeError("set_resolution {}x{} failed: {}".format(width, height, result))
        except Exception:
            if candidate is not None and candidate is not self.detector:
                candidate.close()
            if resolution_attempted:
                try:
                    result = self.cam.set_resolution(*old_size)
                    if result != err.Err.ERR_NONE:
                        raise RuntimeError("camera rollback error: " + str(result))
                except Exception as exc:
                    self.mode = None
                    raise RuntimeError("camera rollback failed; restart app") from exc
            gc.collect()
            raise
        # 只有模型、分辨率均准备好，才更新模式状态。
        if new_mode == config.MODE_OBJECT:
            self.detector = candidate
        elif self.detector is not None:
            self.detector.close()
            self.detector = None
            gc.collect()
        self.mode, self.width, self.height = new_mode, width, height
        print("[MODE] {} {}x{} switch={}ms".format(new_mode, width, height, time.ticks_ms() - started))

    def toggle(self):
        self.enter(config.MODE_OBJECT if self.mode == config.MODE_QR else config.MODE_QR)

    def close(self):
        if self.detector is not None:
            self.detector.close()
            self.detector = None
        self.cam = None

