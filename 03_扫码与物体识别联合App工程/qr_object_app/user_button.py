# 按键模块：回调只记请求标志，主循环负责真正切换模型或退出。
# self._xxx前置下划线是“内部属性”的命名约定；与C中static/私有访问限制不同。
# True/False是布尔值；标志先置True，主循环take后清False，避免一次按键重复执行。

"""USER键驱动：回调只置位，主循环执行切换，类似C的中断标志。"""
from maix import key, time
import config

class UserButton:
    # 功能：初始化按键监听与消抖状态。
    # 返回：None（没有显式return时默认返回None）。
    def __init__(self):
        self._pressed_at = None
        self._last_release = -100000
        self._toggle_requested = False
        self._exit_requested = False
        try:
            # MaixPy默认把USER/OK键当作退出键，必须移除默认监听器。
            key.rm_default_listener()
            # callback=self._on_key传的是方法本身；加括号会变成现在就调用，不是注册回调。
            self._key = key.Key(callback=self._on_key, long_press_time=config.KEY_LONG_PRESS_MS)
            print("[KEY] standalone USER: short switches, hold 1.5s exits; ignored after UART control")
        except Exception as exc:
            self._key = None
            print("[KEY] init failed; mode switch disabled:", exc)

    # 功能：处理USER键按下、长按和释放事件。
    # 参数：key_id：哪个键；state：当前按键事件。
    # 返回：None（没有显式return时默认返回None）。
    # 理解：只置请求标志；电控接管后是否忽略标志由main.py决定。
    def _on_key(self, key_id, state):
        try:
            if int(key_id) != int(key.Keys.KEY_OK):
                return
        except Exception:
            pass
        now = time.ticks_ms()
        if int(state) == int(key.State.KEY_PRESSED):
            self._pressed_at = now
        elif int(state) == int(getattr(key.State, "KEY_LONG_PRESSED", -1)):
            self._pressed_at = None
            self._exit_requested = True
        elif int(state) == int(key.State.KEY_RELEASED):
            if self._pressed_at is None:
                return
            # 按释放时刻减按下时刻得到按住多久；达到长按门槛请求退出，否则检查消抖间隔。
            held_ms = now - self._pressed_at
            self._pressed_at = None
            if held_ms >= config.KEY_LONG_PRESS_MS:
                self._exit_requested = True
            elif now - self._last_release >= config.KEY_DEBOUNCE_MS:
                self._last_release = now
                self._toggle_requested = True

    # 功能：读出并清除一次切换请求。
    # 返回：True=刚取到一次请求；False=没有请求。
    def take_toggle_request(self):
        if not self._toggle_requested:
            return False
        self._toggle_requested = False
        return True

    # 功能：读出退出请求并立即清标志。
    # 返回：读取前的退出请求布尔值。
    def take_exit_request(self):
        requested = self._exit_requested
        self._exit_requested = False
        return requested

    # 功能：清除按键监听对象引用。
    # 返回：None（没有显式return时默认返回None）。
    def close(self):
        self._key = None
