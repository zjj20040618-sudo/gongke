"""双缓冲图像配对，独立于Maix硬件，便于电脑测试。

dual_buff=True时detect返回上次输入的结果。保存一张未画框的独立图像副本，
结果只画在这张对应图上。新任务、模式切换清空配对；首轮结果丢弃，
第二轮才可返回本轮新取图的结果。不用显示缓存或旧任务图像生成坐标。
"""


class FramePair:
    def __init__(self, dual_buffer):
        if not isinstance(dual_buffer, bool):
            raise ValueError("DUAL_BUFFER must be True or False")
        self.dual_buffer = dual_buffer
        self.reset()

    def reset(self):
        self.pending = None

    def align(self, current, capture_ms, started_ms):
        """返回与detect输出对应的(图像,采集耗时,采集循环起点)，首轮为None。

        图像copy在任何绘制之前执行，不让下一次输入包含上次标签。
        元组类似C结构体；pending只保留一个输入，不累积图像队列。
        """
        if not self.dual_buffer:
            return current, capture_ms, started_ms
        previous = self.pending
        self.pending = (current.copy(), capture_ms, started_ms)
        return previous
