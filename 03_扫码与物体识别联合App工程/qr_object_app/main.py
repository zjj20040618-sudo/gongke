"""扫码 + 十类识别正式 App；保留电控请求、ACK 和本轮结果绑定。

给 C 语言读者：def 定义函数；None 类似空指针；while 是主循环；
try/finally 保证初始化失败或退出时也会释放资源。
"""
import gc
from maix import app, camera, display, image, time
import config
from hardware import init_uart, send_packet
from mode_controller import ModeController
from protocol import build_object_packet, build_qr_packet, build_hostage_order_packet, CommandReceiver
from control_session import ControlSession
from qr_reader import QrReader
from ui import draw_header, draw_objects, draw_qrs, make_qr_preview
from user_button import UserButton
from utils import class_name
from task_selection import TaskSelection
from touch_inspector import ObjectInspector
from display_cache import DisplayCache
from frame_pair import FramePair
from hostage_order import HostageOrder


def main():
    cam = screen = serial = button = modes = img = canvas = inspector = None
    try:
        cam = camera.Camera(config.OBJECT_WIDTH, config.OBJECT_HEIGHT, image.Format.FMT_RGB888,
                            buff_num=config.OBJECT_CAMERA_BUFFERS)
        screen = display.Display() if config.DISPLAY_ENABLED else None
        display_size = (screen.width(), screen.height()) if screen is not None else (config.OBJECT_WIDTH, config.OBJECT_HEIGHT)
        inspector = ObjectInspector(display_size, enabled=screen is not None)
        object_display = DisplayCache(config.OBJECT_BOX_HOLD_MS)
        frame_pair = FramePair(config.DUAL_BUFFER)
        serial, button = init_uart(), UserButton()
        modes, qr_reader = ModeController(cam), QrReader()
        receiver, control = CommandReceiver(), ControlSession(modes)
        task = TaskSelection()
        hostage_order = HostageOrder()
        sequence = frame_count = fps_count = 0
        fps_value, fps_started = 0.0, time.ticks_ms()
        cached_qrs, cached_qr_left = [], 0
        pending_ack = None
        last_task_message = None
        pending_auto_object = auto_object_failed = False
        modes.enter(config.START_MODE)
        manual_object_view = modes.mode == config.MODE_OBJECT
        print("[APP] ready {}; model={} classes=10; UART controls recognition".format(modes.mode, config.MODEL_FILE))
        print("[UART] QR=0x53; OBJECT=0x01; HOSTAGE ORDER=0x54; task request=0x63; MCU order parser update required")
        for class_id, name in enumerate(config.CLASS_NAMES_CN):
            print("[CLASS] {} {} ({})".format(class_id, name, class_name(class_id)))

        while not app.need_exit():
            loop_started = time.ticks_ms()
            ack_attempted = False
            # 前一帧已释放；只有主循环会修改摄像头和模型。
            if serial is not None:
                data = serial.read(len=256, timeout=0)
                for command in receiver.feed(data or b""):
                    record_event = getattr(serial, "record_event", None)
                    if record_event is not None:
                        record_event("[CONTROL RX] parsed={} previous_mode={} previous_request={}".format(
                            command, modes.mode, control.request_id))
                    ack, changed = control.apply(*command)
                    if changed:
                        hostage_order.reset(control.target_class_id if control.request_id == command[0]
                                            and control.task_id == 3 else None)
                        pending_auto_object = auto_object_failed = False
                        frame_pair.reset()  # 新请求，即使仍是OBJECT，也拒绝前任务的流水线结果。
                        manual_object_view = False
                        inspector.reset()
                        object_display.reset()
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
                if pending_ack is None and not control.manual_override and control.request_id is not None and not control.acknowledged:
                    pending_ack = control.last_ack
                if pending_ack is not None and not ack_attempted:
                    if send_packet(serial, pending_ack):
                        control.ack_sent(pending_ack)
                        pending_ack = None
            # 短按始终切换QR/OBJECT；长按优先退出，绝不同时触发短按。
            # 只退出视觉 App，不是电机急停；停车/断流保护由电控独立处理。
            take_exit = getattr(button, "take_exit_request", None)
            if take_exit is not None and take_exit():
                print("[KEY] manual exit; remote_owned={}; MCU stop unconfirmed".format(control.remote_owned))
                break
            if button.take_toggle_request():
                pending_auto_object = False  # USER的明确操作优先于上一帧自动切换。
                gc.collect()
                try:
                    control.manual_toggle()
                    hostage_order.reset()  # 手动预览不记录场外物体，也不冒充本轮任务结果。
                    pending_ack = None  # 不继续确认已被人工暂停的业务请求。
                    frame_pair.reset()
                    manual_object_view = modes.mode == config.MODE_OBJECT
                    inspector.reset()
                    object_display.reset()
                    if modes.mode == config.MODE_QR:
                        task.reset()
                    auto_object_failed = False
                    cached_qrs, cached_qr_left = [], 0
                    fps_count, fps_value, fps_started = 0, 0.0, time.ticks_ms()
                except Exception as exc:
                    auto_object_failed = True  # 不在本轮再次自动重试同一个加载失败。
                    if modes.mode is None:
                        raise
                    print("[MODE] switch failed; old mode continues:", exc)

            if pending_auto_object and pending_ack is None:
                pending_auto_object = False
                try:
                    if control.auto_object_after_qr():
                        frame_pair.reset()
                        manual_object_view = False
                        inspector.reset()
                        object_display.reset()
                        cached_qrs, cached_qr_left = [], 0
                        fps_count, fps_value, fps_started = 0, 0.0, time.ticks_ms()
                        print("[MODE] QR task={} auto OBJECT; waiting_object_request={}".format(
                            task.payload, control.qr_handoff))
                except Exception as exc:
                    auto_object_failed = True
                    if modes.mode is None:
                        raise
                    print("[MODE] QR auto OBJECT failed; old QR continues, USER/new request retries:", exc)

            # 即使IDLE/QR/ACK等待也排空触摸事件，不让旧点击跨模式生效。
            tap = inspector.poll()
            record_state = getattr(serial, "record_state", None)
            if record_state is not None:
                record_state(control, pending_ack, receiver)
            if pending_ack is not None or modes.mode == "IDLE" or not control.ready_for_capture():
                time.sleep_ms(10)
                continue
            loop_started = time.ticks_ms()  # 切换/命令处理耗时不计入单帧处理。
            capture_started = time.ticks_ms()
            img = cam.read(block=True, block_ms=config.QR_READ_TIMEOUT_MS) if modes.mode == config.MODE_QR else cam.read()
            if img is None:
                raise RuntimeError("Camera read timed out; restart App")
            width, height = img.width(), img.height()
            capture_ms = time.ticks_ms() - capture_started
            objects, qrs, selected_objects = [], [], []
            work_ms = uart_ms = 0
            result_ready = True

            if modes.mode == config.MODE_QR:
                work_started = time.ticks_ms()
                if config.QR_ROI_TOUCH_MOVE and tap is not None and qr_reader.move_to(img, tap, display_size):
                    cached_qrs, cached_qr_left = [], 0
                    task.observe_qrs([])  # 只清未确认的连续计数；已锁存任务不改变。
                qrs = qr_reader.decode(img)
                task.observe_qrs(qrs)
                if task.message != last_task_message:
                    print("[TASK]", task.message)
                    last_task_message = task.message
                if task.payload is not None:
                    qrs = [qr for qr in qrs if qr["payload"] == task.payload]
                work_ms = time.ticks_ms() - work_started
                if qrs:
                    cached_qrs, cached_qr_left = qrs[:config.QR_MAX], config.QR_KEEP_FRAMES
                    for qr in qrs:
                        print("[QR] payload={} {} center=({}, {})".format(qr["payload"], qr["text"], qr["x"] + qr["w"] // 2, qr["y"] + qr["h"] // 2))
                # 重发本轮锁存任务码至换模式，目标坐标绝不沿用旧帧。
                qr_sent = serial is None
                if serial is not None:
                    uart_started = time.ticks_ms()
                    packet = build_qr_packet(sequence, task.payload)
                    sequence = (sequence + 1) & 0xFFFF  # 分配给新帧；尾包完成与新帧不能共用seq。
                    qr_sent = send_packet(serial, control.result(packet))
                    uart_ms = time.ticks_ms() - uart_started
                if task.payload is not None and qr_sent and not auto_object_failed and not control.manual_override:
                    pending_auto_object = True  # 完整报码并释放原图后，下一轮才换格式。
            else:
                objects, work_ms = modes.detector.detect(img)
                paired = frame_pair.align(img, capture_ms, loop_started)
                if paired is None:
                    # 第一轮只提交本任务图像；返回值可能属于旧任务，既不画也不发。
                    objects, result_ready = [], False
                else:
                    img, capture_ms, source_started = paired
                    width, height = img.width(), img.height()
                selected_objects = task.select(objects, control.target_class_id,
                    include_barrel=control.remote_owned or task.payload is not None, img_w=width, img_h=height)
                if result_ready:
                    # 使用全部新检测结果；若先筛成抓取目标，就无法知道另外两种谁先出现。
                    hostage_order.observe(objects, width, height)
                vision_ms = time.ticks_ms() - source_started if result_ready else 0
                # 0x63只发当前任务；通用0x60 OBJECT保留三任务加桶诊断。
                if serial is not None and (result_ready or control.qr_handoff):
                    uart_started = time.ticks_ms()
                    # QR请求下只能回53，重复报码覆盖扫码结果丢包；不冒用QR号发01。
                    frame_sequence = sequence
                    packet = (build_qr_packet(frame_sequence, task.payload) if control.qr_handoff else
                              build_object_packet(sequence, selected_objects, width, height, capture_ms, work_ms, vision_ms))
                    sequence = (sequence + 1) & 0xFFFF
                    send_packet(serial, control.result(packet))
                    if hostage_order.active and result_ready:
                        # 每个新检测帧重报锁定序号，丢包后可恢复；01仍只含本帧目标坐标。
                        # 两种包共用本帧seq，不能让旧01的帧序号因54而跳号。
                        order_packet = build_hostage_order_packet(frame_sequence,
                            hostage_order.target_class_id, hostage_order.order)
                        send_packet(serial, control.result(order_packet))
                    uart_ms = time.ticks_ms() - uart_started

            draw_started = time.ticks_ms()
            # 所有检测框都能点击；默认文字与UART筛选是两套列表，互不影响。
            # USER手动识别默认显示全部；自动/电控任务只默认显示任务目标。
            default_info = objects if manual_object_view else selected_objects
            display_objects, visible_objects = [], []
            if modes.mode == config.MODE_OBJECT:
                # 先完成本帧推理和UART，再更新显示缓存；旧框不会进入模型或串口。
                display_objects, display_defaults = object_display.update(
                    objects, default_info, (width, height), time.ticks_ms())
                visible_objects = inspector.choose(display_objects, (width, height), tap,
                    default_visible=display_defaults)
            status = inspector.status if modes.mode == config.MODE_OBJECT else None
            task_status = "TASK:{} DIGIT:{}".format(control.task_id, control.qr_digit) if control.target_class_id is not None else task.message
            if manual_object_view:
                task_status = "MANUAL: ALL CLASSES"
            if control.manual_override:
                task_status += " MCU PAUSED: NEW REQUEST REQUIRED"
            if not result_ready:
                task_status += " PIPELINE WARMUP"
            if control.qr_handoff:
                task_status += " WAIT MCU OBJECT REQUEST"
            if hostage_order.active:
                task_status += " ORDER:{} TARGET#{}".format(
                    ",".join(str(cid) for cid in hostage_order.order) or "NONE", hostage_order.target_rank)
            status = "{} {}".format(task_status, status or "").strip()
            if modes.mode == config.MODE_QR:
                # 原始灰度图仍解码；预览关闭只改显示背景，触屏范围映射保持不变。
                canvas = img
                display_qrs = cached_qrs if cached_qr_left > 0 else []
                display_roi = qr_reader.roi(img)
                if screen is not None:
                    canvas, display_qrs, display_roi = make_qr_preview(
                        img, display_qrs, display_roi, display_size,
                        show_camera=config.QR_CAMERA_PREVIEW_ENABLED)
                header_bottom = draw_header(canvas, modes.mode, fps_value, work_ms, uart_ms,
                    control.remote_owned, status, source_size=(width, height), text_scale=3) or 0
                draw_qrs(canvas, display_qrs, display_size, header_bottom, roi=display_roi)
                if cached_qr_left > 0:
                    cached_qr_left -= 1
            else:
                canvas = img
                if screen is not None and not config.OBJECT_CAMERA_PREVIEW_ENABLED:
                    # 保持模型图幅，确保框、点触和串口使用同一组原始坐标。
                    canvas = image.Image(width, height, image.Format.FMT_RGB888)
                    canvas.draw_rect(0, 0, width, height, image.Color.from_rgb(0, 0, 0), thickness=-1)
                header_bottom = draw_header(canvas, modes.mode, fps_value, work_ms, uart_ms, control.remote_owned, status) or 0
                # 单独的显示列表；上面的任务筛选和UART已完成，不受触摸影响。
                draw_objects(canvas, display_objects, display_size, header_bottom, inspector.selected,
                             info_objects=visible_objects)
            if screen is not None:
                screen.show(canvas)
            draw_ms = time.ticks_ms() - draw_started
            img = canvas = None  # 切换前释放原始图和预览，不占用旧格式缓冲。

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
        img = canvas = None
        if 'frame_pair' in locals():
            frame_pair.reset()
        if inspector is not None:
            inspector.close()
        if serial is not None:
            close_uart = getattr(serial, "close", None)
            if close_uart is not None:
                try:
                    close_uart()  # 先收尾本机日志，再释放视觉资源。
                except Exception as exc:
                    print("[UART] close failed:", exc)
        if modes is not None:
            modes.close()
        if button is not None:
            button.close()
        serial = screen = cam = None
        gc.collect()
        print("[APP] stopped")


if __name__ == "__main__":
    main()

