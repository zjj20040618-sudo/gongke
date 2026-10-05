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
        display_size = (screen.width(), screen.height()) if screen is not None else (config.OBJECT_WIDTH, config.OBJECT_HEIGHT)
        serial, button = init_uart(), UserButton()
        modes, qr_reader = ModeController(cam), QrReader()
        receiver, control = CommandReceiver(), ControlSession(modes)
        task = TaskSelection()
        sequence = frame_count = fps_count = 0
        fps_value, fps_started = 0.0, time.ticks_ms()
        cached_qrs, cached_qr_left = [], 0
        pending_ack = None
        modes.enter(config.START_MODE)
        print("[APP] ready {}; model={} classes=10; UART controls recognition".format(modes.mode, config.MODEL_FILE))
        print("[UART] QR=0x53 three ASCII digits; OBJECT=0x01; task request=0x63; MCU QR/task update required")
        for class_id, name in enumerate(config.CLASS_NAMES_CN):
            print("[CLASS] {} {} ({})".format(class_id, name, class_name(class_id)))

        while not app.need_exit():
            loop_started = time.ticks_ms()
            ack_attempted = False
            # 前一帧已释放；只有主循环会修改摄像头和模型。
            if serial is not None:
                data = serial.read(len=256, timeout=0)
                for command in receiver.feed(data or b""):
                    ack, changed = control.apply(*command)
                    if changed:
                        cached_qrs, cached_qr_left = [], 0
                        fps_count, fps_value, fps_started = 0, 0.0, time.ticks_ms()
                        if modes.mode == config.MODE_QR and control.request_id == command[0]:
                            task.reset()  # 只有成功的新扫码请求清任务；同号重试不清。
                    ack_attempted = True
                    pending_ack = ack
                    if send_packet(serial, ack):
                        control.ack_sent(ack)
                        pending_ack = None
                    else:
                        print("[CONTROL] ACK write incomplete; business paused request={}".format(command[0]))
                if pending_ack is None and control.request_id is not None and not control.acknowledged:
                    pending_ack = control.last_ack
                if pending_ack is not None and not ack_attempted:
                    if send_packet(serial, pending_ack):
                        control.ack_sent(pending_ack)
                        pending_ack = None
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

            if pending_ack is not None or modes.mode == "IDLE" or (control.remote_owned and (control.request_id is None or not control.acknowledged)):
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
                # 重发本轮锁存任务码至换模式，目标坐标绝不沿用旧帧。
                if serial is not None:
                    uart_started = time.ticks_ms()
                    packet = build_qr_packet(sequence, task.payload)
                    sequence = (sequence + 1) & 0xFFFF  # 分配给新帧；尾包完成与新帧不能共用seq。
                    send_packet(serial, control.result(packet))
                    uart_ms = time.ticks_ms() - uart_started
            else:
                objects, work_ms = modes.detector.detect(img)
                selected_objects = task.select(objects, control.target_class_id)
                vision_ms = time.ticks_ms() - loop_started
                # 0x63只发当前任务；通用0x60 OBJECT保留三任务加桶诊断。
                if serial is not None:
                    uart_started = time.ticks_ms()
                    packet = build_object_packet(sequence, selected_objects, width, height, capture_ms, work_ms, vision_ms)
                    sequence = (sequence + 1) & 0xFFFF
                    send_packet(serial, control.result(packet))
                    uart_ms = time.ticks_ms() - uart_started

            draw_started = time.ticks_ms()
            header_bottom = draw_header(img, modes.mode, fps_value, work_ms, uart_ms, control.remote_owned) or 0
            if modes.mode == config.MODE_QR:
                if cached_qr_left > 0:
                    draw_qrs(img, cached_qrs, display_size, header_bottom)
                    cached_qr_left -= 1
            else:
                draw_objects(img, objects[:config.MAX_OBJECTS], display_size, header_bottom)
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

