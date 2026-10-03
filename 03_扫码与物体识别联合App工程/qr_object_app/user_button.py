"""USER键驱动：回调只置位，主循环执行切换，类似C的中断标志。"""
from maix import key, time
import config

class UserButton:
    def __init__(self):
        self._pressed_at = None
        self._last_release = -100000
        self._toggle_requested = False
        self._exit_requested = False
        try:
            # MaixPy默认把USER/OK键当作退出键，必须移除默认监听器。
            key.rm_default_listener()
            self._key = key.Key(callback=self._on_key, long_press_time=config.KEY_LONG_PRESS_MS)
            print("[KEY] standalone USER: short switches, hold 1.5s exits; ignored after UART control")
        except Exception as exc:
            self._key = None
            print("[KEY] init failed; mode switch disabled:", exc)

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
            held_ms = now - self._pressed_at
            self._pressed_at = None
            if held_ms >= config.KEY_LONG_PRESS_MS:
                self._exit_requested = True
            elif now - self._last_release >= config.KEY_DEBOUNCE_MS:
                self._last_release = now
                self._toggle_requested = True

    def take_toggle_request(self):
        if not self._toggle_requested:
            return False
        self._toggle_requested = False
        return True

    def take_exit_request(self):
        requested = self._exit_requested
        self._exit_requested = False
        return requested

    def close(self):
        self._key = None
