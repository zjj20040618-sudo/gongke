"""集中管理模式、摄像头分辨率和模型；切换失败尽量恢复原模式。"""
import gc
from maix import err, image, time
import config
from object_detector import ObjectDetector


class ModeController:
    def __init__(self, cam):
        self.cam, self.mode, self.detector = cam, None, None
        self.width, self.height = config.QR_WIDTH, config.QR_HEIGHT
        self.format = cam.format()

    def _configure_camera(self, width, height, pixel_format):
        # MC2使用公共Camera.open：参数变化时，AX驱动内部关闭并重新打开通道。
        # 不继承原MC的“保留MMF初始化”假设，也不手动操作GC4653寄存器。
        # 保留同一Python对象，让main里的cam引用始终有效；set_resolution不能改格式。
        # 切换失败时enter会重新open原参数，若回滚也失败则明确退出，不发送假坐标。
        buffers = (config.QR_CAMERA_BUFFERS if pixel_format == image.Format.FMT_GRAYSCALE
                   else config.OBJECT_CAMERA_BUFFERS)
        result = self.cam.open(width, height, format=pixel_format,
                               fps=config.CAMERA_FPS, buff_num=buffers)
        if result != err.Err.ERR_NONE:
            raise RuntimeError("camera open {}x{} format={} failed: {}".format(width, height, pixel_format, result))
        if pixel_format == image.Format.FMT_GRAYSCALE:
            self.cam.skip_frames(config.QR_WARMUP_FRAMES)

    def enter(self, new_mode):
        if new_mode == "IDLE":
            self.mode = new_mode  # 电控停识别；保留模型，下次请求可复用。
            return
        if new_mode not in (config.MODE_QR, config.MODE_OBJECT):
            raise ValueError("unknown mode: " + str(new_mode))
        if new_mode == self.mode:
            return
        old_size = (self.width, self.height)
        old_format = self.format
        candidate = self.detector
        resolution_attempted = False
        started = time.ticks_ms()
        try:
            if new_mode == config.MODE_OBJECT:
                if candidate is None:
                    candidate = ObjectDetector()
                # 不猜测转换后的模型尺寸，读取实际输入宽高。
                width, height = candidate.input_width, candidate.input_height
                pixel_format = image.Format.FMT_RGB888
            else:
                width, height = config.QR_WIDTH, config.QR_HEIGHT
                pixel_format = image.Format.FMT_GRAYSCALE
            resolution_attempted = True
            self._configure_camera(width, height, pixel_format)
        except Exception:
            if candidate is not None and candidate is not self.detector:
                candidate.close()
            if resolution_attempted:
                try:
                    self._configure_camera(*old_size, old_format)
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
        self.format = pixel_format
        print("[MODE] {} {}x{} format={} switch={}ms".format(new_mode, width, height, pixel_format, time.ticks_ms() - started))

    def toggle(self):
        self.enter(config.MODE_OBJECT if self.mode == config.MODE_QR else config.MODE_QR)

    def close(self):
        if self.detector is not None:
            self.detector.close()
            self.detector = None
        if self.cam is not None:
            self.cam.close()
        self.cam = None
