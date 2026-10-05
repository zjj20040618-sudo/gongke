import hashlib
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

import uart_log_collector as tool


def packet(body):
    return tool.HEADER + body + struct.pack("<H", tool.crc16(body))


class LogTests(unittest.TestCase):
    def test_known_crc(self):
        self.assertEqual(tool.crc16(b"123456789"), 0x29B1)
        self.assertEqual(packet(bytes.fromhex("60 01 00 01")).hex(), "aa556001000103fa")
        self.assertEqual(packet(bytes.fromhex("61 01 00 01 00")).hex(), "aa55610100010004e7")

    def test_request_and_task(self):
        for body, expected in ((bytes.fromhex("60 01 00 01"), {"mode": 1}), (bytes.fromhex("63 02 00 04 00"), {"task": 4, "digit": 0})):
            frames, warnings = tool.FrameStream().feed(packet(body))
            self.assertFalse(warnings)
            for key, value in expected.items():
                self.assertEqual(frames[0][key], value)

    def test_qr_wrapper(self):
        wire = bytes.fromhex("AA 55 62 01 00 07 00 53 01 00 01 31 32 33 42 CE")
        frames, warnings = tool.FrameStream().feed(wire)
        self.assertFalse(warnings)
        self.assertEqual(frames[0]["request"], 1)
        self.assertEqual(frames[0]["inner"]["qr"], "123")
        self.assertTrue(frames[0]["inner"]["qr_valid"])

    def test_bucket_and_empty(self):
        body = struct.pack("<BHBHHHHH", 1, 4, 1, 320, 240, 2, 50, 80)
        body += struct.pack("<BHHHHH", 9, 900, 160, 120, 40, 50)
        frames, warnings = tool.FrameStream().feed(packet(body))
        self.assertFalse(warnings)
        self.assertEqual(frames[0]["objects"][0]["name"], "黑桶")
        self.assertEqual(frames[0]["objects"][0]["cx"], 160)
        frames, _ = tool.FrameStream().feed(packet(bytes.fromhex("53 01 00 00")))
        self.assertEqual(frames[0]["qr"], "")
        self.assertFalse(frames[0]["qr_valid"])

    def test_rx_fragments_and_multiple(self):
        wire = packet(bytes.fromhex("60 01 00 01"))
        text = "[UART RX] bytes=1 hex=AA\n[UART RX] bytes=15 hex=" + (wire[1:] + wire).hex(" ")
        events = tool.analyze(text, "camera")
        self.assertEqual(len(events[0]["frames"]), 0)
        self.assertEqual(len(events[1]["frames"]), 2)

    def test_bad_crc_resync(self):
        good = packet(bytes.fromhex("60 01 00 01"))
        frames, warnings = tool.FrameStream().feed(good[:-1] + b"\x00" + good)
        self.assertTrue(warnings)
        self.assertEqual(len(frames), 1)

    def test_partial_not_complete_evidence(self):
        ack = packet(bytes.fromhex("61 01 00 01 00")).hex(" ")
        events = tool.analyze("[UART TX] status=partial written=4/9 hex=" + ack, "camera")
        self.assertTrue(events[0]["warnings"])
        report = tool.make_report(events, "x", "", "test")
        self.assertIn("TX complete日志可解码帧0个", report)

    def test_mcu_prefix_and_counters(self):
        text = "VW req=1 mode=1 txtry=97 rxB=776 ackN=0\nVR type=0x60 len=8 hex=AA556001000103FA failed=0"
        events = tool.analyze(text, "mcu")
        self.assertEqual(events[0]["counters"]["rxB"], "776")
        self.assertEqual(events[1]["direction"], "RX前缀")
        self.assertFalse(events[1]["frames"])

    def test_no_hex_and_truncated(self):
        events = tool.analyze("[UART TX] status=blocked written=0/9\n[UART RX] hex=AA 55 60", "camera")
        self.assertTrue(events[0]["warnings"])
        self.assertIn("未成帧", events[-1]["warnings"][0])

    def test_invalid_inner_and_unknown(self):
        frames, warnings = tool.FrameStream().feed(packet(bytes.fromhex("62 01 00 01 00 99")))
        self.assertFalse(frames)
        self.assertTrue(warnings)

    def test_missing_hex_does_not_bridge_fragments(self):
        events = tool.analyze("[UART RX] hex=AA\n[UART RX] bytes=1\n[UART RX] hex=55 60 01 00 01 03 FA", "camera")
        self.assertFalse(any(e["frames"] for e in events))

    def test_device_file_write_fragments_and_gap(self):
        events = tool.analyze("2026-10-05T12:00:00Z [UART WRITE] bytes=3 hex=AA 55 60\n"
            "[UART WRITE] returned=0 bytes=0 receiver=unconfirmed hex=\n"
            "[UART WRITE] bytes=5 hex=01 00 01 03 FA", "camera")
        self.assertEqual(events[2]["frames"][0]["request"], 1)
        self.assertEqual(events[2]["direction"], "TX写入片段")
        self.assertIn("不证明对端收到", events[2]["evidence"])
        events = tool.analyze("[UART RX] hex=AA\n[LOG GAP] dropped_records=1\n[UART RX] hex=55 60 01 00 01 03 FA", "camera")
        self.assertFalse(any(e["frames"] for e in events))
        _, warnings = tool.FrameStream().feed(b"\xaa\x55\xff\x00")
        self.assertTrue(warnings)

    def test_empty_save_refused(self):
        with self.assertRaises(ValueError):
            tool.save_session("", " ", "unused")

    def test_unique_sessions_raw_hashes(self):
        with tempfile.TemporaryDirectory() as directory:
            raw = "测试\r\n[UART RX] hex=AA 55 60 01 00 01 03 FA\r\n"
            first = tool.save_session(raw, "VW req=1 ackN=0", directory, "合成测试，不是实机")
            second = tool.save_session(raw, "", directory)
            self.assertNotEqual(first, second)
            self.assertEqual((first / "camera_raw.txt").read_bytes(), raw.encode())
            metadata = json.loads((first / "metadata.json").read_text(encoding="utf-8"))
            self.assertFalse(metadata["uploaded"])
            for filename, sha in metadata["sha256"].items():
                self.assertEqual(hashlib.sha256((first / filename).read_bytes()).hexdigest(), sha)
            self.assertTrue((first / "rx_tx.csv").read_bytes().startswith(b"\xef\xbb\xbf"))

    def test_log_encodings(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ble.txt"
            for encoding in ("utf-8-sig", "utf-16", "gb18030"):
                path.write_bytes("黑桶日志".encode(encoding))
                self.assertEqual(tool.read_log(path), "黑桶日志")

    def test_gui_paste_import_save(self):
        import tkinter as tk
        real_tk, real_save = tk.Tk, tool.save_session
        failures, saved = [], []
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "ble.txt"
            log.write_text("VW req=1 mode=1 ackN=0", encoding="utf-8")

            def create_root():
                root = real_tk()
                root.withdraw()
                root.clipboard_get = lambda: "[UART RX] hex=AA 55 60 01 00 01 03 FA"

                def exercise():
                    try:
                        def descendants(widget):
                            children = widget.winfo_children()
                            return children + [child for w in children for child in descendants(w)]
                        buttons = [w for w in descendants(root) if isinstance(w, tk.Button)]
                        next(w for w in buttons if w.cget("text") == "粘贴整段日志").invoke()
                        imports = [w for w in buttons if w.cget("text") == "导入TXT文件"]
                        imports[1].invoke()
                        next(w for w in buttons if w.cget("text").startswith("保存本轮")).invoke()
                    except Exception as exc:
                        failures.append(exc)
                    finally:
                        root.destroy()
                root.after(0, exercise)
                return root

            def redirected_save(camera, mcu, output, note):
                folder = real_save(camera, mcu, directory, note)
                saved.append(folder)
                return folder

            with patch("tkinter.Tk", side_effect=create_root), patch("tkinter.filedialog.askopenfilename", return_value=str(log)), \
                    patch("tkinter.messagebox.showinfo"), patch("tkinter.messagebox.showerror") as error, \
                    patch.object(tool, "save_session", side_effect=redirected_save):
                tool.show_gui()
                error.assert_not_called()
            self.assertFalse(failures)
            self.assertEqual(len(saved), 1)
            self.assertIn("AA 55", (saved[0] / "camera_raw.txt").read_text(encoding="utf-8"))
            self.assertIn("ackN=0", (saved[0] / "mcu_ble_raw.txt").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
