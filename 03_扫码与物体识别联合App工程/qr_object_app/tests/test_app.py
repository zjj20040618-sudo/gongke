"""无需设备：模拟 Maix API，验证按键、模式、十类模型和二进制组包。"""
import configparser
import contextlib
import importlib
import io
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest.mock import patch

APP_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_DIR))
clock = [0]
maix = types.ModuleType("maix")
maix.time = types.SimpleNamespace(ticks_ms=lambda: clock[0])
maix.key = types.SimpleNamespace(
    Keys=types.SimpleNamespace(KEY_OK=1),
    State=types.SimpleNamespace(KEY_PRESSED=1, KEY_RELEASED=2, KEY_LONG_PRESSED=3),
    rm_default_listener=lambda: None,
    Key=lambda **kwargs: types.SimpleNamespace(**kwargs),
)
maix.image = types.SimpleNamespace(
    Format=types.SimpleNamespace(FMT_RGB888=1),
    QRCodeDecoderType=types.SimpleNamespace(QRCODE_DECODER_TYPE_ZBAR=1),
    COLOR_GREEN=1, COLOR_YELLOW=2, COLOR_RED=3, COLOR_BLUE=4, COLOR_WHITE=5,
    Color=types.SimpleNamespace(from_rgb=lambda r, g, b: (r, g, b)),
    string_size=lambda text, scale=1, thickness=-1: (len(text) * 8 * scale, 12 * scale),
)
maix.err = types.SimpleNamespace(Err=types.SimpleNamespace(ERR_NONE=0))
maix.nn = types.SimpleNamespace()
maix.pinmap = types.SimpleNamespace()
maix.uart = types.SimpleNamespace()
maix.app = types.SimpleNamespace()
maix.camera = types.SimpleNamespace()
maix.display = types.SimpleNamespace()
sys.modules["maix"] = maix
import config
from object_detector import ObjectDetector
from mode_controller import ModeController
from user_button import UserButton
from protocol import build_object_packet, build_qr_packet
from qr_reader import QrReader, center_roi, task_text_cn
from utils import crc16_ccitt


def raw_object(class_id=9, score=0.82):
    return types.SimpleNamespace(class_id=class_id, score=score, x=10, y=20, w=30, h=40)


class FakeModel:
    labels = config.CLASS_NAMES_CN
    def __init__(self, **kwargs):
        self.kwargs = kwargs
    def input_width(self):
        return 320
    def input_height(self):
        return 320
    def input_format(self):
        return maix.image.Format.FMT_RGB888
    def detect(self, img, **kwargs):
        self.detect_kwargs = kwargs
        return [raw_object(9), raw_object(0, 0.91)]


class FakeCamera:
    def __init__(self, *args):
        self.width, self.height = config.OBJECT_WIDTH, config.OBJECT_HEIGHT
        self.calls = []
        self.results = []
    def set_resolution(self, width, height):
        self.calls.append((width, height))
        result = self.results.pop(0) if self.results else 0
        if isinstance(result, Exception):
            raise result
        if result == 0:
            self.width, self.height = width, height
        return result
    def read(self):
        return FakeImage(self.width, self.height)


class FakeCode:
    def payload(self): return "123"
    def x(self): return 420
    def y(self): return 240
    def w(self): return 120
    def h(self): return 120
    def corners(self): return [(420, 240), (540, 240), (540, 360), (420, 360)]


class FakeImage:
    def __init__(self, width=1920, height=1440):
        self._width, self._height = width, height
    def width(self): return self._width
    def height(self): return self._height
    def find_qrcodes(self, *args, **kwargs): return [FakeCode()]
    def draw_string(self, *args, **kwargs): pass
    def draw_rect(self, *args, **kwargs): pass
    def draw_cross(self, *args, **kwargs): pass
    def draw_edges(self, *args, **kwargs): pass


class AppTests(unittest.TestCase):
    def setUp(self):
        clock[0] = 0
        self.log = contextlib.redirect_stdout(io.StringIO())
        self.log.__enter__()
        self.model_patch = patch.object(maix.nn, "YOLO26", FakeModel, create=True)
        self.model_patch.start()
    def tearDown(self):
        self.model_patch.stop()
        self.log.__exit__(None, None, None)

    def test_model_metadata_and_ten_class_order(self):
        mud = configparser.ConfigParser()
        mud.read(APP_DIR / config.MODEL_FILE, encoding="utf-8")
        self.assertEqual(tuple(s.strip() for s in mud["extra"]["labels"].split(",")), config.CLASS_NAMES_CN)
        self.assertEqual(mud["extra"]["model_type"], "yolo26")
        self.assertEqual(config.MODEL_FILE, "model_9564.mud")
        self.assertEqual(len(config.CLASS_NAMES_CN), 10)
        self.assertEqual(config.CLASS_NAMES_CN[9], "黑桶")
        self.assertTrue((APP_DIR / mud["basic"]["model"]).is_file())

    def test_detector_dimensions_and_sort(self):
        detector = ObjectDetector()
        objects, _ = detector.detect(FakeImage(320, 320))
        self.assertEqual((detector.input_width, detector.input_height), (320, 320))
        self.assertEqual([o.class_id for o in objects], [0, 9])
        self.assertEqual(detector.model.detect_kwargs, {"conf_th": config.CONF_THRESHOLD})
        self.assertFalse(detector.model.kwargs["dual_buff"])
        detector.close()
        self.assertIsNone(detector.model)

    def test_detector_rejects_wrong_label_order(self):
        with patch.object(FakeModel, "labels", tuple(reversed(config.CLASS_NAMES_CN))):
            with self.assertRaisesRegex(RuntimeError, "config.py"):
                ObjectDetector()

    def test_detector_rejects_out_of_range_class(self):
        detector = ObjectDetector()
        with patch.object(FakeModel, "detect", return_value=[raw_object(10)]):
            with self.assertRaises(RuntimeError):
                detector.detect(FakeImage())

    def test_detector_requires_yolo26_firmware(self):
        with patch("object_detector.nn", types.SimpleNamespace()):
            with self.assertRaisesRegex(RuntimeError, "YOLO26"):
                ObjectDetector()

    def test_mode_roundtrip_reads_model_size(self):
        cam = FakeCamera()
        modes = ModeController(cam)
        modes.enter(config.MODE_QR)
        modes.toggle()
        detector = modes.detector
        self.assertEqual(modes.mode, config.MODE_OBJECT)
        self.assertEqual(cam.calls[-1], (320, 320))
        modes.toggle()
        self.assertEqual(cam.calls[-1], (1920, 1440))
        self.assertIsNone(detector.model)
        self.assertIsNone(modes.detector)

    def test_model_load_failure_keeps_qr(self):
        cam = FakeCamera()
        modes = ModeController(cam)
        modes.enter(config.MODE_QR)
        with patch("mode_controller.ObjectDetector", side_effect=RuntimeError("load failed")):
            with self.assertRaises(RuntimeError):
                modes.toggle()
        self.assertEqual(modes.mode, config.MODE_QR)
        self.assertEqual(cam.calls, [(1920, 1440)])

    def test_resolution_failure_rolls_back(self):
        cam = FakeCamera()
        modes = ModeController(cam)
        modes.enter(config.MODE_QR)
        cam.results = [1, 0]
        with self.assertRaises(RuntimeError):
            modes.toggle()
        self.assertEqual(modes.mode, config.MODE_QR)
        self.assertEqual(cam.calls[-2:], [(320, 320), (1920, 1440)])
        self.assertIsNone(modes.detector)

    def test_failed_rollback_marks_mode_invalid(self):
        cam = FakeCamera()
        modes = ModeController(cam)
        modes.enter(config.MODE_QR)
        cam.results = [1, 1]
        with self.assertRaisesRegex(RuntimeError, "rollback"):
            modes.toggle()
        self.assertIsNone(modes.mode)

    def test_unknown_mode_rejected(self):
        with self.assertRaises(ValueError):
            ModeController(FakeCamera()).enter("BAD")

    def test_rollback_exception_marks_mode_invalid(self):
        cam = FakeCamera()
        modes = ModeController(cam)
        modes.enter(config.MODE_QR)
        cam.results = [RuntimeError("resize"), RuntimeError("restore")]
        with self.assertRaisesRegex(RuntimeError, "rollback"):
            modes.toggle()
        self.assertIsNone(modes.mode)

    def test_button_short_press_once(self):
        button = UserButton()
        button._on_key(1, 1)
        clock[0] = 100
        button._on_key(1, 2)
        self.assertTrue(button.take_toggle_request())
        self.assertFalse(button.take_toggle_request())
        self.assertFalse(button.take_exit_request())

    def test_button_debounce(self):
        button = UserButton()
        button._on_key(1, 1)
        clock[0] = 50
        button._on_key(1, 2)
        self.assertTrue(button.take_toggle_request())
        clock[0] = 70
        button._on_key(1, 1)
        clock[0] = 90
        button._on_key(1, 2)
        self.assertFalse(button.take_toggle_request())

    def test_button_long_event_exits_without_toggle(self):
        button = UserButton()
        button._on_key(1, 1)
        clock[0] = 1600
        button._on_key(1, 3)
        button._on_key(1, 2)
        self.assertTrue(button.take_exit_request())
        self.assertFalse(button.take_exit_request())
        self.assertFalse(button.take_toggle_request())

    def test_button_long_release_fallback(self):
        button = UserButton()
        button._on_key(1, 1)
        clock[0] = 1600
        button._on_key(1, 2)
        self.assertTrue(button.take_exit_request())
        self.assertFalse(button.take_toggle_request())

    def test_other_key_ignored(self):
        button = UserButton()
        button._on_key(2, 1)
        clock[0] = 100
        button._on_key(2, 2)
        self.assertFalse(button.take_toggle_request())

    def test_black_barrel_packet_and_crc(self):
        packet = build_object_packet(65536, [raw_object()], 320, 320)
        self.assertEqual(packet[:2], b"\xaa\x55")
        self.assertEqual(struct.unpack("<BHBHHHHH", packet[2:16]), (1, 0, 1, 320, 320, 0, 0, 0))
        self.assertEqual(struct.unpack("<BHHHHH", packet[16:27]), (9, 820, 25, 40, 30, 40))
        self.assertEqual(struct.unpack("<H", packet[-2:])[0], crc16_ccitt(packet[2:-2]))

    def test_target_packet_preserves_full_geometry(self):
        packet = build_object_packet(1, [raw_object(8)], 320, 320)
        self.assertEqual(packet[2], 0x01)
        self.assertEqual(struct.unpack("<BHBHHHHH", packet[2:16]), (1, 1, 1, 320, 320, 0, 0, 0))
        self.assertEqual(struct.unpack("<BHHHHH", packet[16:27]), (8, 820, 25, 40, 30, 40))
        self.assertEqual(len(packet), 29)

    def test_empty_detection_packet(self):
        packet = build_object_packet(1, [], 320, 320)
        self.assertEqual(len(packet), 18)
        self.assertEqual(packet[5], 0)

    def test_crc_reference_vector(self):
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_qr_packet_contains_three_digits_without_geometry(self):
        packet = build_qr_packet(4, "123")
        self.assertEqual(packet[:9], b"\xaa\x55\x53\x04\x00\x01" + b"123")
        self.assertEqual(len(packet), 11)
        self.assertEqual(struct.unpack("<H", packet[-2:])[0], crc16_ccitt(packet[2:-2]))
        self.assertEqual(build_qr_packet(5, None)[5], 0)

    def test_qr_roi_and_task_text(self):
        self.assertEqual(center_roi(FakeImage()), [480, 360, 960, 720])
        self.assertIn("红", task_text_cn("123"))
        self.assertEqual(task_text_cn("hello"), "非赛题任务码")

    def test_main_qr_object_qr_and_long_exit(self):
        main = importlib.import_module("main")
        buttons = []
        cameras = []
        shown = []
        def make_button():
            button = UserButton()
            buttons.append(button)
            return button
        def make_camera(*args):
            cam = FakeCamera(*args)
            cameras.append(cam)
            return cam
        turns = [0]
        def need_exit():
            turns[0] += 1
            clock[0] += 1000
            if turns[0] in (2, 3):
                buttons[0]._toggle_requested = True
            if turns[0] == 4:
                buttons[0]._exit_requested = True
            return turns[0] > 5
        output = io.StringIO()
        with patch.object(main, "UserButton", side_effect=make_button), \
             patch.object(maix.camera, "Camera", side_effect=make_camera, create=True), \
             patch.object(maix.display, "Display", return_value=types.SimpleNamespace(width=lambda: 480, height=lambda: 320, show=lambda img: shown.append((img.width(), img.height()))), create=True), \
             patch.object(maix.app, "need_exit", side_effect=need_exit, create=True), \
             patch.object(config, "PRINT_EVERY_N_FRAMES", 1), \
             patch.object(config, "START_MODE", config.MODE_QR), \
             patch.object(main, "init_uart", return_value=None), \
             patch.object(main, "send_packet") as send, \
             contextlib.redirect_stdout(output):
            main.main()
        self.assertEqual(shown, [(1920, 1440), (320, 320), (1920, 1440)])
        self.assertIn("black_barrel:0.820@(25,40)", output.getvalue())
        self.assertIn("[QR] payload=123", output.getvalue())
        self.assertIn("UART controls recognition", output.getvalue())
        self.assertIn("[APP] stopped", output.getvalue())
        send.assert_not_called()
        self.assertIsNone(buttons[0]._key)

    def test_start_failure_releases_button(self):
        main = importlib.import_module("main")
        button = UserButton()
        with patch.object(main, "UserButton", return_value=button), \
             patch.object(main.ModeController, "enter", side_effect=RuntimeError("camera")), \
             patch.object(maix.camera, "Camera", FakeCamera, create=True), \
             patch.object(maix.display, "Display", return_value=types.SimpleNamespace(width=lambda: 480, height=lambda: 320), create=True):
            with self.assertRaises(RuntimeError):
                main.main()
        self.assertIsNone(button._key)


    def test_production_defaults_preserved(self):
        self.assertEqual(config.START_MODE, "IDLE")
        self.assertTrue(config.UART_ENABLED)
        self.assertEqual(config.CLASS_NAMES[9], "black_barrel")

    def test_qr_uses_field_verified_stage_36_resolution(self):
        self.assertEqual((config.QR_WIDTH, config.QR_HEIGHT), (1920, 1440))
        cam = FakeCamera()
        modes = ModeController(cam)
        modes.enter(config.MODE_QR)
        self.assertEqual(cam.calls, [(1920, 1440)])
        self.assertEqual(center_roi(cam.read()), [480, 360, 960, 720])
        self.assertEqual(len(build_qr_packet(4, "123")), 11)

    def test_stage_36_package_and_device_log_share_version(self):
        import uart_log

        self.assertEqual(uart_log.APP_VERSION, "2.1.5")
        self.assertIn("version: 2.1.5", (APP_DIR / "app.yaml").read_text(encoding="utf-8"))

    def test_idle_retains_model_and_reuses_it(self):
        cam = FakeCamera()
        modes = ModeController(cam)
        modes.enter(config.MODE_OBJECT)
        detector = modes.detector
        modes.enter("IDLE")
        self.assertIs(modes.detector, detector)
        self.assertEqual(modes.mode, "IDLE")
        modes.enter(config.MODE_OBJECT)
        self.assertIs(modes.detector, detector)

    def test_remote_controls_black_transmit_ack_retry_and_idle(self):
        main = importlib.import_module("main")
        from protocol import build_control_packet, build_ack_packet, bind_result
        sent, shown, buttons = [], [], []
        commands = iter(build_control_packet(r, m) for r, m in ((1, 1), (2, 2), (2, 2), (3, 0)))
        turns = [0]
        def make_button():
            button = UserButton()
            buttons.append(button)
            return button
        def need_exit():
            turns[0] += 1
            clock[0] += 1000
            buttons[0]._toggle_requested = True
            # 接管后短按不得切模式；长按退出另行验证。
            return turns[0] > 4
        class Serial:
            def read(self, **kwargs):
                if kwargs != {"len": 256, "timeout": 0}:
                    raise AssertionError("unbounded UART read")
                return next(commands)
        def send(serial, packet):
            if packet is None:
                return False
            sent.append(packet)
            return True
        with patch.object(main, "UserButton", side_effect=make_button), \
             patch.object(main, "init_uart", return_value=Serial()), \
             patch.object(main, "send_packet", side_effect=send), \
             patch.object(maix.camera, "Camera", FakeCamera, create=True), \
             patch.object(maix.display, "Display", return_value=types.SimpleNamespace(width=lambda: 480, height=lambda: 320, show=lambda img: shown.append((img.width(), img.height()))), create=True), \
             patch.object(maix.app, "need_exit", side_effect=need_exit, create=True), \
             patch.object(maix.time, "sleep_ms", lambda ms: None, create=True), \
             patch.object(config, "PRINT_EVERY_N_FRAMES", 1), \
             patch.object(FakeModel, "detect", return_value=[raw_object(9), raw_object(4, 0.91)]):
            main.main()
        self.assertEqual(shown, [(1920, 1440), (320, 320), (320, 320)])
        self.assertEqual(sent[0], build_ack_packet(1, 1))
        self.assertEqual(sent[2], build_ack_packet(2, 2))
        self.assertEqual(sent[4], sent[2])  # 同请求重发ACK，不变请求号。
        self.assertEqual(sent[-1], build_ack_packet(3, 0))
        # 62 外壳：两帧都保留黑桶ID9及红球ID4，没有只发一次或过滤。
        for packet in (sent[3], sent[5]):
            self.assertEqual(packet[2], 0x62)
            self.assertEqual(struct.unpack("<H", packet[3:5])[0], 2)
            self.assertEqual(packet[7], 0x01)
            self.assertEqual(packet[10], 2)
            self.assertEqual({packet[21], packet[32]}, {9, 4})
        self.assertEqual(len(sent), 7)

    def test_remote_long_press_exits_even_while_ack_pending(self):
        main = importlib.import_module("main")
        from protocol import build_control_packet
        for ack_complete in (True, False):
            with self.subTest(ack_complete=ack_complete):
                button = UserButton()
                turns, reads, sent = [0], [], []
                serial = types.SimpleNamespace(
                    read=lambda **kwargs: build_control_packet(7, 2) if turns[0] == 1 else b"",
                )
                modes = ModeController(FakeCamera())

                class TrackingCamera(FakeCamera):
                    def read(self):
                        reads.append(turns[0])
                        return super().read()

                def need_exit():
                    turns[0] += 1
                    if turns[0] == 2:
                        button._on_key(maix.key.Keys.KEY_OK, maix.key.State.KEY_LONG_PRESSED)
                        button._toggle_requested = True  # 长按优先，不执行同轮短按。
                    return turns[0] > 4  # 防止旧实现无限循环；不能靠此条件通过。

                def send(serial, packet):
                    sent.append((turns[0], packet))
                    return ack_complete

                with patch.object(main, "UserButton", return_value=button), \
                     patch.object(main, "ModeController", return_value=modes), \
                     patch.object(main, "init_uart", return_value=serial), \
                     patch.object(main, "send_packet", side_effect=send), \
                     patch.object(maix.camera, "Camera", TrackingCamera, create=True), \
                     patch.object(maix.app, "need_exit", side_effect=need_exit, create=True), \
                     patch.object(maix.time, "sleep_ms", lambda ms: None, create=True), \
                     patch.object(config, "DISPLAY_ENABLED", False), \
                     patch.object(modes, "toggle") as toggle:
                    main.main()
                self.assertEqual(turns[0], 2)
                self.assertEqual(reads, [1] if ack_complete else [])
                self.assertFalse(any(turn >= 2 and packet[2] == 0x62 for turn, packet in sent))
                self.assertIsNone(button._key)
                self.assertIsNone(modes.detector)
                toggle.assert_not_called()

    def test_only_black_barrel_sends_class9_frame(self):
        main = importlib.import_module("main")
        from protocol import build_control_packet, build_ack_packet, bind_result
        sent = []
        turns = [0]
        def need_exit():
            turns[0] += 1
            return turns[0] > 1
        serial = types.SimpleNamespace(read=lambda **kwargs: build_control_packet(7, 2))
        def send(serial, packet):
            sent.append(packet)
            return True
        with patch.object(main, "init_uart", return_value=serial), \
             patch.object(main, "send_packet", side_effect=send), \
             patch.object(maix.camera, "Camera", FakeCamera, create=True), \
             patch.object(maix.app, "need_exit", side_effect=need_exit, create=True), \
             patch.object(config, "DISPLAY_ENABLED", False), \
             patch.object(FakeModel, "detect", return_value=[raw_object(9)]):
            main.main()
        self.assertEqual(sent, [build_ack_packet(7, 2), bind_result(build_object_packet(0, [raw_object(9)], 320, 320), 7)])

    def test_pack_manifest_includes_session_and_correct_model(self):
        build_spec = importlib.util.spec_from_file_location("build_packages", APP_DIR.parent / "build_packages.py")
        builder = importlib.util.module_from_spec(build_spec)
        build_spec.loader.exec_module(builder)
        files = builder.manifest()
        self.assertIn("control_session.py", files)
        self.assertIn("model_9564.mud", files)
        self.assertIn("model_9564.cvimodel", files)
        self.assertNotIn("model_9541.cvimodel", files)
        self.assertNotIn("model_9302.cvimodel", files)



    def test_task_all_27_combinations(self):
        from task_selection import TaskSelection
        import itertools
        for digits in itertools.product("123", repeat=3):
            code = "".join(digits)
            task = TaskSelection()
            self.assertTrue(task.observe(code))
            selected = task.select([raw_object(i) for i in range(10)])
            expected = ({"1": 4, "2": 5, "3": 3}[digits[0]],
                        {"1": 6, "2": 8, "3": 7}[digits[1]],
                        {"1": 1, "2": 2, "3": 0}[digits[2]], 9)
            self.assertEqual(tuple(obj.class_id for obj in selected), expected)

    def test_task_invalid_code_never_selects_other_objects(self):
        from task_selection import TaskSelection
        for code in ("", "12", "1234", "000", "ABC", None):
            task = TaskSelection()
            self.assertFalse(task.observe(code))
            self.assertEqual([obj.class_id for obj in task.select([raw_object(4), raw_object(9)])], [9])

    def test_task_lock_and_reset(self):
        from task_selection import TaskSelection
        task = TaskSelection()
        task.observe("123")
        self.assertFalse(task.observe("321"))
        self.assertEqual(task.payload, "123")
        self.assertTrue(task.observe("123"))
        task.reset()
        self.assertTrue(task.observe("321"))
        self.assertEqual(task.class_ids, (3, 8, 1))

    def test_task_current_frame_best_and_lost_barrel(self):
        from task_selection import TaskSelection
        task = TaskSelection()
        task.observe("123")
        best = raw_object(9, 0.93)
        selected = task.select([raw_object(3, 0.99), raw_object(9, 0.5), best, raw_object(4)])
        self.assertEqual([obj.class_id for obj in selected], [4, 9])
        self.assertIs(selected[-1], best)
        # 下一帧黑桶丢失，只发送当前红球；再下一帧全丢失，不造零坐标。
        self.assertEqual([obj.class_id for obj in task.select([raw_object(4)])], [4])
        self.assertEqual(task.select([]), [])

    def test_many_distractors_do_not_truncate_barrel_before_selection(self):
        from task_selection import TaskSelection
        task = TaskSelection()
        task.observe("123")
        detector = ObjectDetector()
        raw = [raw_object(3, 0.99) for _ in range(20)] + [raw_object(9, 0.51), raw_object(4, 0.55)]
        with patch.object(FakeModel, "detect", return_value=raw):
            objects, _ = detector.detect(FakeImage())
        self.assertEqual([obj.class_id for obj in task.select(objects)], [4, 9])

    def test_main_task_persists_idle_retries_and_resets_on_new_qr(self):
        main = importlib.import_module("main")
        from protocol import build_control_packet
        sent = []
        commands = iter(build_control_packet(r, m) for r, m in ((1, 1), (1, 1), (2, 2), (3, 0), (4, 2), (5, 1), (6, 2)))
        qr_codes = iter(("123", "111", "321"))
        turns = [0]
        def need_exit():
            turns[0] += 1
            return turns[0] > 7
        def decode(img):
            qrs = QrReader().decode(img)
            qrs[0]["payload"] = next(qr_codes)
            return qrs
        def send(serial, packet):
            sent.append(packet)
            return True
        serial = types.SimpleNamespace(read=lambda **kwargs: next(commands))
        with patch.object(main, "init_uart", return_value=serial), \
             patch.object(main, "send_packet", side_effect=send), \
             patch.object(main, "QrReader", return_value=types.SimpleNamespace(decode=decode)), \
             patch.object(maix.camera, "Camera", FakeCamera, create=True), \
             patch.object(maix.app, "need_exit", side_effect=need_exit, create=True), \
             patch.object(maix.time, "sleep_ms", lambda ms: None, create=True), \
             patch.object(config, "DISPLAY_ENABLED", False), \
             patch.object(FakeModel, "detect", return_value=[raw_object(i) for i in range(10)]):
            main.main()
        def object_ids(packet):
            ids = []
            offset = 21
            for _ in range(packet[10]):
                class_id = packet[offset]
                ids.append(class_id)
                offset += 11
            return tuple(ids)

        rows = []
        for packet in sent:
            if packet[2] == 0x62 and packet[7] == 0x01:
                request = struct.unpack("<H", packet[3:5])[0]
                rows.append((request, object_ids(packet)))
        self.assertEqual(rows, [(2, (4, 8, 0, 9)), (4, (4, 8, 0, 9)), (6, (3, 8, 1, 9))])
        qr_packets = [packet for packet in sent if packet[2] == 0x62 and packet[7] == 0x53]
        self.assertEqual([packet[10] for packet in qr_packets], [1, 1, 1])
        self.assertEqual([packet[11:14] for packet in qr_packets], [b"123", b"123", b"321"])

    def test_main_task_requests_and_ack_retry_gate(self):
        main = importlib.import_module("main")
        from protocol import build_control_packet, build_task_packet
        commands = iter((build_task_packet(1, 1, 1), b"", build_task_packet(2, 4, 0),
                         build_task_packet(2, 4, 0), build_task_packet(3, 2, 2),
                         build_task_packet(4, 3, 3), build_control_packet(5, 0)))
        turns, reads, attempts, delivered = [0], [], [], []
        class TrackingCamera(FakeCamera):
            def read(self):
                reads.append(turns[0])
                return super().read()
        def need_exit():
            turns[0] += 1
            return turns[0] > 7
        def send(serial, packet):
            attempts.append((turns[0], packet))
            if turns[0] == 1:  # First ACK write incomplete; no MCU retry follows.
                return False
            delivered.append((turns[0], packet))
            return True
        serial = types.SimpleNamespace(read=lambda **kwargs: next(commands))
        with patch.object(main, "init_uart", return_value=serial), \
             patch.object(main, "send_packet", side_effect=send), \
             patch.object(maix.camera, "Camera", TrackingCamera, create=True), \
             patch.object(maix.app, "need_exit", side_effect=need_exit, create=True), \
             patch.object(maix.time, "sleep_ms", lambda ms: None, create=True), \
             patch.object(config, "DISPLAY_ENABLED", False), \
             patch.object(FakeModel, "detect", return_value=[raw_object(i) for i in range(10)]):
            main.main()
        self.assertEqual(reads, [2, 3, 4, 5, 6])
        self.assertEqual(attempts[0][1], attempts[1][1])
        rows = []
        for turn, packet in delivered:
            if packet[2] == 0x62:
                self.assertEqual(packet[7], 0x01)
                self.assertEqual(packet[10], 1)
                rows.append((turn, struct.unpack("<H", packet[3:5])[0], packet[21]))
        self.assertEqual(rows, [(2, 1, 4), (3, 2, 9), (4, 2, 9), (5, 3, 8), (6, 4, 0)])

if __name__ == "__main__":
    unittest.main()
