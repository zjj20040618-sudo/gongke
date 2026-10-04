# 模式切换器：管理摄像头分辨率与模型寿命，主循环调用enter来切换。
# try执行可能失败的操作；except捕获异常；raise重新抛出，让上层决定如何处理。
# 先准备新资源，全部成功后才发布新状态；失败尽量回到原分辨率，不能假报已切换。

"""集中管理模式、摄像头分辨率和模型；切换失败尽量恢复原模式。"""
import gc
from maix import err, time
import config
from object_detector import ObjectDetector


class ModeController:
    # 功能：保存摄像头引用，建立初始状态。
    # 参数：cam：摄像头对象。
    # 返回：None（没有显式return时默认返回None）。
    def __init__(self, cam):
        self.cam, self.mode, self.detector = cam, None, None
        self.width, self.height = config.OBJECT_WIDTH, config.OBJECT_HEIGHT

    # 功能：切换到IDLE/QR/OBJECT，必要时加载模型和调整画幅。
    # 参数：new_mode：模式字符串。
    # 返回：None（没有显式return时默认返回None）。
    # 理解：若准备失败会抛异常；IDLE保留已有模型以便复用，QR成功后会释放检测模型。
    def enter(self, new_mode):
        if new_mode == "IDLE":
            self.mode = new_mode  # 电控停识别；保留模型，下次请求可复用。
            return
        if new_mode not in (config.MODE_QR, config.MODE_OBJECT):
            raise ValueError("unknown mode: " + str(new_mode))
        if new_mode == self.mode:
            return
        old_size = (self.width, self.height)
        # candidate只是另一份对象引用，不是复制整套模型；后面可先准备一个候选模型。
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
                    # *元组会拆成位置参数；old_size=(宽,高)，相当于set_resolution(宽,高)。
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
        # 这里才把成功的新模式和画幅写入对象，供下一帧主循环读取。
        self.mode, self.width, self.height = new_mode, width, height
        print("[MODE] {} {}x{} switch={}ms".format(new_mode, width, height, time.ticks_ms() - started))

    # 功能：在QR和OBJECT之间切换，用于接管前按键试验。
    # 返回：None（没有显式return时默认返回None）。
    # 理解：当前是QR就选OBJECT，否则选QR；电控接管后主循环不允许按键调用它。
    def toggle(self):
        self.enter(config.MODE_OBJECT if self.mode == config.MODE_QR else config.MODE_QR)

    # 功能：清除模型和摄像头引用，配合退出时释放资源。
    # 返回：None（没有显式return时默认返回None）。
    def close(self):
        if self.detector is not None:
            self.detector.close()
            self.detector = None
        self.cam = None

