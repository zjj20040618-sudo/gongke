"""扫码 + 十类识别正式 App；保留电控请求、ACK 和本轮结果绑定。

给 C 语言读者：def 定义函数；None 类似空指针；while 是主循环；
try/finally 保证初始化失败或退出时也会释放资源。
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
from utils import class_name
from task_selection import TaskSelection


def main():
    cam = screen = serial = button = modes = img = None
    try:
        cam = camera.Camera(config.OBJECT_WIDTH, config.OBJECT_HEIGHT, image.Format.FMT_RGB888)
        screen = display.Display() if config.DISPLAY_ENABLED else None
        serial, button = init_uart(), UserButton()
        modes, qr_reader = ModeController(cam), QrReader()
        receiver, control = CommandReceiver(), ControlSession(modes)
        task = TaskSelection()
        sequence = frame_count = fps_count = 0
        fps_value, fps_started = 0.0, time.ticks_ms()
        cached_qrs, cached_qr_left = [], 0
        modes.enter(config.START_MODE)
        print("[APP] ready IDLE; model=9541 classes=10; UART controls recognition")
        print("[UART] protocol=v2 QR status only; OBJECT class+center, target X only; MCU parser update required")
        for class_id, name in enumerate(config.CLASS_NAMES_CN):
            print("[CLASS] {} {} ({})".format(class_id, name, class_name(class_id)))

        while not app.need_exit():
            loop_started = time.ticks_ms()
            # 前一帧已释放；只有主循环会修改摄像头和模型。
            if serial is not None:
                data = serial.read(len=256, timeout=0)
                for request_id, mode in receiver.feed(data or b""):
                    ack, changed = control.apply(request_id, mode)
                    if changed:
                        cached_qrs, cached_qr_left = [], 0
                        fps_count, fps_value, fps_started = 0, 0.0, time.ticks_ms()
                        if mode == 1 and control.request_id == request_id:
                            task.reset()  # 只有成功的新扫码请求清任务；同号重试不清。
                    send_packet(serial, ack)  # 必须先确认实际模式，再取新图和发新结果。
            # 接管后 USER 短按/长按均不覆盖电控，避免识别中途被人为退出。
            take_exit = getattr(button, "take_exit_request", None)
            if take_exit is not None and take_exit() and not control.remote_owned:
                break
            if button.take_toggle_request() and not control.remote_owned:
                gc.collect()
                try:
                    modes.toggle()
                    if modes.mode == config.MODE_QR:
                        task.reset()
                    cached_qrs, cached_qr_left = [], 0
                    fps_count, fps_value, fps_started = 0, 0.0, time.ticks_ms()
                except Exception as exc:
                    if modes.mode is None:
                        raise
                    print("[MODE] switch failed; old mode continues:", exc)

            if modes.mode == "IDLE" or (control.remote_owned and control.request_id is None):
                time.sleep_ms(10)
                continue
            loop_started = time.ticks_ms()  # 切换/命令处理耗时不计入单帧处理。
            capture_started = time.ticks_ms()
            img = cam.read()
            width, height = img.width(), img.height()
            capture_ms = time.ticks_ms() - capture_started
            objects, qrs, selected_objects = [], [], []
            work_ms = uart_ms = 0

            if modes.mode == config.MODE_QR:
                work_started = time.ticks_ms()
                qrs = qr_reader.decode(img)
                for qr in qrs:
                    task.observe(qr["payload"])
                if task.payload is not None:
                    qrs = [qr for qr in qrs if qr["payload"] == task.payload]
                work_ms = time.ticks_ms() - work_started
                if qrs:
                    cached_qrs, cached_qr_left = qrs, config.QR_KEEP_FRAMES
                    for qr in qrs:
                        print("[QR] payload={} {} center=({}, {})".format(qr["payload"], qr["text"], qr["x"] + qr["w"] // 2, qr["y"] + qr["h"] // 2))
                # 每帧只回有效任务码状态；任务码文字和二维码框仅供视觉端内部使用。
                if serial is not None:
                    uart_started = time.ticks_ms()
                    if send_packet(serial, control.result(build_qr_packet(sequence, task.payload is not None))):
                        sequence = (sequence + 1) & 0xFFFF
                    uart_ms = time.ticks_ms() - uart_started
            else:
                objects, work_ms = modes.detector.detect(img)
                selected_objects = task.select(objects)
                vision_ms = time.ticks_ms() - loop_started
                # 每帧发送任务指定类别与中心坐标；靶子只带X，黑桶持续发送。
                if serial is not None:
                    uart_started = time.ticks_ms()
                    packet = build_object_packet(sequence, selected_objects, width, height)
                    if send_packet(serial, control.result(packet)):
                        sequence = (sequence + 1) & 0xFFFF
                    uart_ms = time.ticks_ms() - uart_started

            draw_started = time.ticks_ms()
            if modes.mode == config.MODE_QR:
                if cached_qr_left > 0:
                    draw_qrs(img, cached_qrs)
                    cached_qr_left -= 1
            else:
                draw_objects(img, objects[:config.MAX_OBJECTS])
            draw_header(img, modes.mode, fps_value, work_ms, uart_ms, control.remote_owned)
            if screen is not None:
                screen.show(img)
            draw_ms = time.ticks_ms() - draw_started
            img = None  # 不让旧图像持有摄像头缓冲，防止下一轮切分辨率出错。

            fps_count += 1
            frame_count += 1
            now = time.ticks_ms()
            elapsed = now - fps_started
            if elapsed >= 1000:
                fps_value = fps_count * 1000.0 / elapsed
                fps_count, fps_started = 0, now
            loop_ms = time.ticks_ms() - loop_started
            if frame_count % config.PRINT_EVERY_N_FRAMES == 0:
                details = ", ".join("{}:{:.3f}@({},{})".format(class_name(obj.class_id), obj.score, obj.x + obj.w // 2, obj.y + obj.h // 2) for obj in objects)
                selected_ids = ",".join(str(obj.class_id) for obj in selected_objects)
                print("[PERF] mode={} request={} task={} frame={} size={}x{} fps={:.2f} obj={} sent_ids=[{}] qr={} cap={}ms work={}ms draw/display={}ms uart={}ms loop={}ms {}".format(modes.mode, control.request_id, task.payload, frame_count, width, height, fps_value, len(objects), selected_ids, len(qrs), capture_ms, work_ms, draw_ms, uart_ms, loop_ms, details))
    finally:
        img = None
        if modes is not None:
            modes.close()
        if button is not None:
            button.close()
        serial = screen = cam = None
        gc.collect()
        print("[APP] stopped")


if __name__ == "__main__":
    main()

