# 阅读入口：这是视觉程序的主循环，先处理电控请求，再取新图、识别、发送和画框。
# 建议先读main()理解先后顺序，再按import进入各模块；不用一开始钻进模型或CRC算法。
# import config类似C中引用配置模块，但Python导入也会执行该模块顶层语句。
# None表示“当前没有对象/有效值”，不能像C指针那样解引用；用is None判断。

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


# 功能：初始化硬件和业务对象，并一直处理画面，退出时清理资源。
# 返回：正常结束返回None；不能处理的异常会向上传递。
def main():
    # 1.先把资源变量都置为None。若初始化中途失败，finally仍能判断哪些资源需要清理。
    cam = screen = serial = button = modes = img = None
    try:
        # 2.创建对象：camera.Camera(...)类似你笔记中的Car(...)；cam保存这个摄像头对象。
        cam = camera.Camera(config.OBJECT_WIDTH, config.OBJECT_HEIGHT, image.Format.FMT_RGB888)
        screen = display.Display() if config.DISPLAY_ENABLED else None
        serial, button = init_uart(), UserButton()
        # 逗号赋值把右边两项分别给左边两变量；不是C中“最后一个表达式作为结果”的逗号运算符。
        modes, qr_reader = ModeController(cam), QrReader()
        receiver, control = CommandReceiver(), ControlSession(modes)
        # task负责记住本轮扫码选了哪三类；detector负责找框，两者职责不同。
        task = TaskSelection()
        sequence = frame_count = fps_count = 0
        fps_value, fps_started = 0.0, time.ticks_ms()
        cached_qrs, cached_qr_left = [], 0
        modes.enter(config.START_MODE)
        print("[APP] ready IDLE; model=9541 classes=10; UART controls recognition")
        print("[UART] sends QR-selected objects plus black_barrel=9 every OBJECT frame; paired MCU FW=20261004-VISION-DIAG33")
        for class_id, name in enumerate(config.CLASS_NAMES_CN):
            print("[CLASS] {} {} ({})".format(class_id, name, class_name(class_id)))

        # 3.主循环：not相当于C的逻辑非!；只要SDK没有要求退出，就继续下一帧。
        while not app.need_exit():
            loop_started = time.ticks_ms()
            # 前一帧已释放；只有主循环会修改摄像头和模型。
            if serial is not None:
                # 每轮最多取256字节，timeout=0不等待完整命令；分次收到的内容由receiver缓冲拼接。
                # len=、timeout=是按参数名传值，不是给全局变量赋值。
                data = serial.read(len=256, timeout=0)
                # feed返回命令列表；for把每个(request_id,mode)元组拆成两个变量。
                # b""是空字节串；data or b""在没有读到数据时给解析器一个可遍历的空值。
                for request_id, mode in receiver.feed(data or b""):
                    # apply的第二项表示是否处理了一条新的请求，不等于切换一定成功；成功/失败还在ACK中。
                    ack, changed = control.apply(request_id, mode)
                    if changed:
                        cached_qrs, cached_qr_left = [], 0
                        fps_count, fps_value, fps_started = 0, 0.0, time.ticks_ms()
                        if mode == 1 and control.request_id == request_id:
                            task.reset()  # 只有成功的新扫码请求清任务；同号重试不清。
                    send_packet(serial, ack)  # 必须先确认实际模式，再取新图和发新结果。
            # 接管后 USER 短按/长按均不覆盖电控，避免识别中途被人为退出。
            # getattr按名字取成员，第三个参数是在成员不存在时使用的默认值；便于兼容替代按键对象。
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

            # 4.未运行识别或电控切换失败时只短暂等待，不取旧模式画面冒充本轮新结果。
            # continue直接进入下一次while循环，后面的识别/组包/显示都跳过。
            if modes.mode == "IDLE" or (control.remote_owned and control.request_id is None):
                time.sleep_ms(10)
                continue
            loop_started = time.ticks_ms()  # 切换/命令处理耗时不计入单帧处理。
            capture_started = time.ticks_ms()
            # 5.取一张新图。img.width()/height()是调用图像对象的方法，括号表示实际执行。
            img = cam.read()
            width, height = img.width(), img.height()
            capture_ms = time.ticks_ms() - capture_started
            objects, qrs, selected_objects = [], [], []
            work_ms = uart_ms = 0

            if modes.mode == config.MODE_QR:
                work_started = time.ticks_ms()
                qrs = qr_reader.decode(img)
                # qrs是列表，每个qr是字典；qr["payload"]用字符串键取二维码内容。
                for qr in qrs:
                    task.observe(qr["payload"])
                if task.payload is not None:
                    # 列表推导式相当于“创建空列表、遍历、满足条件就append”，这里只留下本轮锁存码。
                    qrs = [qr for qr in qrs if qr["payload"] == task.payload]
                work_ms = time.ticks_ms() - work_started
                if qrs:
                    cached_qrs, cached_qr_left = qrs, config.QR_KEEP_FRAMES
                    for qr in qrs:
                        print("[QR] payload={} {} center=({}, {})".format(qr["payload"], qr["text"], qr["x"] + qr["w"] // 2, qr["y"] + qr["h"] // 2))
                # 空二维码仍发本轮心跳；画框缓存绝不作为新扫码结果发送。
                if serial is not None:
                    uart_started = time.ticks_ms()
                    if send_packet(serial, control.result(build_qr_packet(sequence, qrs))):
                        sequence = (sequence + 1) & 0xFFFF
                    uart_ms = time.ticks_ms() - uart_started
            else:
                objects, work_ms = modes.detector.detect(img)
                # 检测框objects用于显示；selected_objects才是按QR筛选后要发送的目标，不能混用。
                selected_objects = task.select(objects)
                vision_ms = time.ticks_ms() - loop_started
                # 每帧发送任务指定的球/靶/人质，并持续附带黑桶；电控决定当前抓哪个。
                if serial is not None:
                    uart_started = time.ticks_ms()
                    # 6.组包时使用本帧真实宽高和当前目标，不把屏幕残留框当作新识别。
                    packet = build_object_packet(sequence, selected_objects, width, height, capture_ms, work_ms, vision_ms)
                    if send_packet(serial, control.result(packet)):
                        sequence = (sequence + 1) & 0xFFFF
                    uart_ms = time.ticks_ms() - uart_started

            # 7.显示是最后一段；QR可暂时保留旧框帮助看清，但发送仍只用本帧qrs。
            draw_started = time.ticks_ms()
            if modes.mode == config.MODE_QR:
                if cached_qr_left > 0:
                    draw_qrs(img, cached_qrs)
                    cached_qr_left -= 1
            else:
                # [:上限]是列表切片，只影响画框数量；任务筛选已经在前面从完整检测列表完成。
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
                # join把多段文字拼成一串；format中的{:.3f}显示3位小数，和C的格式化输出用途相似。
                details = ", ".join("{}:{:.3f}@({},{})".format(class_name(obj.class_id), obj.score, obj.x + obj.w // 2, obj.y + obj.h // 2) for obj in objects)
                selected_ids = ",".join(str(obj.class_id) for obj in selected_objects)
                print("[PERF] mode={} request={} task={} frame={} size={}x{} fps={:.2f} obj={} sent_ids=[{}] qr={} cap={}ms work={}ms draw/display={}ms uart={}ms loop={}ms {}".format(modes.mode, control.request_id, task.payload, frame_count, width, height, fps_value, len(objects), selected_ids, len(qrs), capture_ms, work_ms, draw_ms, uart_ms, loop_ms, details))
    # 8.finally不论正常退出、break还是异常都会运行；清资源不是另一条识别流程。
    finally:
        img = None
        if modes is not None:
            modes.close()
        if button is not None:
            button.close()
        serial = screen = cam = None
        gc.collect()
        print("[APP] stopped")


# 直接运行本文件时调用main；被测试代码import时只定义函数，不自动启动摄像头。
if __name__ == "__main__":
    main()

