"""扫码+九类识别联合App主程序。

给C语言读者：main()就是main函数；while not app.need_exit()就是主循环；
try/finally保证退出时释放资源；cam.read()可理解为Camera_Read(&cam)。
"""
import gc
from maix import app, camera, display, image, time
import config
from hardware import init_uart, send_packet
from mode_controller import ModeController
from protocol import build_object_packet, build_qr_packet, CommandReceiver
from control_session import ControlSession
from qr_reader import QrReader
from ui import draw_header, draw_objects, draw_qrs
from user_button import UserButton

def main():
    # 摄像头只创建一次；两模式均使用RGB888，只切分辨率，减少驱动重启风险。
    cam = camera.Camera(config.OBJECT_WIDTH, config.OBJECT_HEIGHT, image.Format.FMT_RGB888)
    screen = display.Display() if config.DISPLAY_ENABLED else None
    serial, button = init_uart(), UserButton()
    modes, qr_reader = ModeController(cam), QrReader()
    receiver, control = CommandReceiver(), ControlSession(modes)
    sequence = frame_count = fps_count = 0
    fps_value, fps_started = 0.0, time.ticks_ms()
    cached_qrs, cached_qr_left = [], 0
    modes.enter(config.START_MODE)
    print("[APP] ready IDLE; UART controls recognition; USER for standalone only")
    try:
        while not app.need_exit():
            loop_started = time.ticks_ms()
            if serial is not None:
                # Official MaixPy UART: timeout=0 returns immediately, max 256 bytes this loop.
                data = serial.read(len=256, timeout=0)
                for request_id, mode in receiver.feed(data):
                    ack, changed = control.apply(request_id, mode)
                    if changed:
                        cached_qrs, cached_qr_left = [], 0
                        fps_count, fps_value, fps_started = 0, 0.0, time.ticks_ms()
                    send_packet(serial, ack)  # applied mode ACK precedes any new-result frame
            # 回调线程只置位；主线程在没有旧图像被占用时安全切换。
            if button.take_toggle_request() and not control.remote_owned:
                try:
                    modes.toggle()
                    cached_qrs, cached_qr_left = [], 0
                    fps_count, fps_value, fps_started = 0, 0.0, time.ticks_ms()
                    gc.collect()
                except Exception as exc:
                    print("[MODE] switch failed; old mode continues:", exc)

            if modes.mode == "IDLE" or (control.remote_owned and control.request_id is None):
                time.sleep_ms(10)
                continue

            capture_started = time.ticks_ms()
            img = cam.read()
            capture_ms = time.ticks_ms() - capture_started
            work_ms = uart_ms = 0

            if modes.mode == config.MODE_QR:
                work_started = time.ticks_ms()
                qrs = qr_reader.decode(img)
                work_ms = time.ticks_ms() - work_started
                if qrs:
                    cached_qrs, cached_qr_left = qrs, config.QR_KEEP_FRAMES
                    for qr in qrs:
                        print("[QR] payload={} {} center=({}, {})".format(qr["payload"], qr["text"], qr["x"] + qr["w"] // 2, qr["y"] + qr["h"] // 2))
                # Empty QR is a fresh heartbeat, not QR success. Cached drawing is never sent.
                uart_started = time.ticks_ms()
                if send_packet(serial, control.result(build_qr_packet(sequence, qrs))):
                    sequence = (sequence + 1) & 0xFFFF
                uart_ms = time.ticks_ms() - uart_started
                if cached_qr_left > 0:
                    draw_qrs(img, cached_qrs); cached_qr_left -= 1
            else:
                objects, work_ms = modes.detector.detect(img)
                vision_ms = time.ticks_ms() - loop_started
                uart_started = time.ticks_ms()
                packet = build_object_packet(sequence, objects, img.width(), img.height(), capture_ms, work_ms, vision_ms)
                if send_packet(serial, control.result(packet)):
                    sequence = (sequence + 1) & 0xFFFF
                uart_ms = time.ticks_ms() - uart_started
                draw_objects(img, objects)

            fps_count += 1; frame_count += 1
            now, elapsed = time.ticks_ms(), time.ticks_ms() - fps_started
            if elapsed >= 1000:
                fps_value = fps_count * 1000.0 / elapsed
                fps_count, fps_started = 0, now
            draw_header(img, modes.mode, fps_value, work_ms, uart_ms)
            if screen is not None:
                screen.show(img)
            if frame_count % config.PRINT_EVERY_N_FRAMES == 0:
                print("[PERF] mode={} frame={} size={}x{} fps={:.2f} cap={}ms work={}ms uart={}ms loop={}ms".format(modes.mode, frame_count, img.width(), img.height(), fps_value, capture_ms, work_ms, uart_ms, time.ticks_ms() - loop_started))
    finally:
        modes.close(); button.close()
        serial = screen = cam = None
        gc.collect()
        print("[APP] stopped")

if __name__ == "__main__":
    main()

