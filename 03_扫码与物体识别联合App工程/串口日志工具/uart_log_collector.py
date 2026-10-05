"""离线收集 MaixVision 和手机 BLE 日志；不打开串口，不发送硬件命令。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import struct
from datetime import datetime
import uuid

HEADER = b"\xaa\x55"
CLASS_NAMES = ["腰鼓", "圆柱", "圆锥", "蓝球", "红球", "绿球", "红靶", "蓝靶", "绿靶", "黑桶"]


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def frame_length(data):
    """None表示需要更多字节；-1表示未知或不合理结构。"""
    if len(data) < 3:
        return None
    kind = data[2]
    if kind in (0x60, 0x61, 0x63):
        return 8 if kind == 0x60 else 9
    if kind == 0x62:
        if len(data) < 7:
            return None
        size = int.from_bytes(data[5:7], "little")
        return 9 + size if 1 <= size <= 16384 else -1
    if kind in (0x01, 0x53):
        if len(data) < 6:
            return None
        count = data[5]
        return (8 + 3 * count if count <= 1 else -1) if kind == 0x53 else 18 + 11 * count
    return -1


def decode_body(body):
    kind = body[0]
    result = {"type": f"0x{kind:02X}"}
    if kind in (0x60, 0x61, 0x63):
        if len(body) != (4 if kind == 0x60 else 5):
            raise ValueError("命令/ACK长度不符")
        result["request"] = int.from_bytes(body[1:3], "little")
        if kind == 0x63:
            result.update(task=body[3], digit=body[4])
        else:
            result["mode"] = body[3]
            if kind == 0x61:
                result["status"] = body[4]
    elif kind == 0x62:
        if len(body) < 6:
            raise ValueError("结果包装头不完整")
        result["request"] = int.from_bytes(body[1:3], "little")
        size = int.from_bytes(body[3:5], "little")
        if len(body) != 5 + size or body[5] not in (0x01, 0x53):
            raise ValueError("结果内层类型或长度不符")
        result["inner"] = decode_body(body[5:])
    elif kind in (0x01, 0x53):
        if len(body) < 4:
            raise ValueError("业务头不完整")
        result.update(sequence=int.from_bytes(body[1:3], "little"), count=body[3])
        if kind == 0x53:
            if body[3] > 1 or len(body) != 4 + 3 * body[3]:
                raise ValueError("QR53长度不符")
            result["qr"] = body[4:].decode("ascii")
            result["qr_valid"] = body[3] == 1 and all(b in b"123" for b in body[4:])
        else:
            if len(body) != 14 + 11 * body[3]:
                raise ValueError("01目标长度不符")
            result["image_size"] = list(struct.unpack_from("<HH", body, 4))
            result["objects"] = []
            for offset in range(14, len(body), 11):
                model_id, score, cx, cy, width, height = struct.unpack_from("<BHHHHH", body, offset)
                result["objects"].append({"id": model_id, "name": CLASS_NAMES[model_id] if model_id < 10 else "未知类别",
                    "score": score / 1000, "cx": cx, "cy": cy, "width": width, "height": height})
    else:
        raise ValueError("未支持的帧类型")
    return result


class FrameStream:
    """按日志顺序拼接RX碎片；缺日志时不能还原丢失字节。"""
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        decoded, warnings = [], []
        while self.buffer:
            start = self.buffer.find(HEADER)
            if start < 0:
                keep = 1 if self.buffer[-1] == 0xAA else 0
                if len(self.buffer) > keep:
                    warnings.append("存在非AA55字节或日志不连续")
                self.buffer[:] = self.buffer[-1:] if keep else b""
                break
            if start:
                warnings.append(f"跳过{start}个非帧头字节")
                del self.buffer[:start]
            size = frame_length(self.buffer)
            if size is None:
                break
            if size == -1:
                warnings.append("未知帧类型或非法长度；仅保留原文")
                del self.buffer[0]
                continue
            if len(self.buffer) < size:
                break
            packet = bytes(self.buffer[:size])
            if crc16(packet[2:-2]) != int.from_bytes(packet[-2:], "little"):
                warnings.append("CRC不符：可能损坏、日志截断或缺片；不能据此独断实体链路故障")
                del self.buffer[0]
                continue
            del self.buffer[:size]
            try:
                fields = decode_body(packet[2:-2])
                fields.update(crc_ok=True, hex=packet.hex(" ").upper())
                decoded.append(fields)
            except (ValueError, UnicodeError, struct.error) as exc:
                warnings.append(f"CRC正确但结构解码失败：{exc}")
        return decoded, warnings


def analyze(text, source):
    events = []
    rx_stream = FrameStream()
    for number, line in enumerate(text.splitlines(), 1):
        direction, evidence, stream = "", "", None
        if "[UART RX]" in line:
            direction, evidence, stream = "RX", "相机read读到的字节", rx_stream
        elif "[UART TX]" in line:
            direction, evidence, stream = "TX", "相机发送日志；HEX是整帧意图，不是逐次write字节", FrameStream()
        elif re.search(r"\bVW req=", line):
            direction, evidence = "RX/TX统计", "MCU累计快照，不是逐包原文；重启可能清零"
        elif re.search(r"\bVR type=", line):
            direction, evidence = "RX前缀", "MCU接收缓存前缀；可能截断，不作完整帧CRC检验"
        elif any(tag in line for tag in ("VD33", "[CONTROL", "READY", "DIAG FW", "OK QR=")):
            direction, evidence = "状态", "设备状态原文"
        if not direction:
            continue
        event = {"source": source, "line": number, "direction": direction, "evidence": evidence,
                 "raw": line, "frames": [], "warnings": []}
        if "[UART TX]" in line:
            match = re.search(r"\bstatus=(\w+)", line)
            event["write_status"] = match.group(1) if match else "unknown"
            if event["write_status"] != "complete":
                event["warnings"].append("未确认完整写出；不能把HEX当作已发送的整包")
        if stream is not None:
            match = re.search(r"\bhex=([0-9A-Fa-f ]+)", line)
            if match:
                try:
                    event["frames"], warnings = stream.feed(bytes.fromhex(match.group(1)))
                    event["warnings"].extend(warnings)
                    if direction == "TX" and stream.buffer:
                        event["warnings"].append("TX日志中的HEX不完整")
                except ValueError:
                    event["warnings"].append("HEX格式不完整")
                    if direction == "RX":
                        rx_stream.buffer.clear()
            else:
                event["warnings"].append("没有HEX，仅保留写入状态/原文")
                if direction == "RX":
                    rx_stream.buffer.clear()
        if evidence.startswith("MCU累计"):
            event["counters"] = dict(re.findall(r"\b(\w+)=(-?\d+)", line))
        events.append(event)
    if rx_stream.buffer:
        events.append({"source": source, "line": len(text.splitlines()), "direction": "RX",
            "evidence": "末尾剩余碎片", "raw": "", "frames": [],
            "warnings": [f"末尾{len(rx_stream.buffer)}字节未成帧；原始日志可能未复制完整"]})
    return events


def make_report(events, camera, mcu, note):
    camera_rx, camera_tx = [], []
    for event in events:
        if event["source"] == "camera":
            if event["direction"] == "RX":
                camera_rx.extend(event["frames"])
            elif event["direction"] == "TX" and event.get("write_status") == "complete":
                camera_tx.extend(event["frames"])
    requests = {f["request"] for f in camera_rx if f["type"] in ("0x60", "0x63")}
    ack = {f["request"] for f in camera_tx if f["type"] == "0x61" and f["status"] == 0}
    missing = sorted(requests - ack)
    warnings = sum(len(e["warnings"]) for e in events)
    lines = ["# 双端联调日志（本地收集，尚未实机验收）", "", "## 本轮信息", "",
        note or "未填写设备版本/测试条件，请补充。", "", "## 日志观察", "",
        f"- 相机原文：{'已提供' if camera.strip() else '缺失'}；电控BLE原文：{'已提供' if mcu.strip() else '缺失'}。",
        f"- 提取{len(events)}条观察记录，提示{warnings}条。数量不是实体UART包总数。",
        f"- 相机RX可解码帧{len(camera_rx)}个，TX complete日志可解码帧{len(camera_tx)}个。",
        f"- 在已复制的相机日志内，没有成功complete ACK记录的请求ID：{missing or '无/未观察到请求'}。",
        "- 请求ID统计不校验时序/模式，不证明MCU收到ACK；同一份日志不要混入多次重启。", "",
        "## 电控累计快照（保留原值，不相加）", ""]
    snapshots = [e for e in events if e["source"] == "mcu" and "counters" in e]
    if snapshots:
        lines.append("```text")
        lines.extend(e["raw"] for e in snapshots[-10:])
        lines.append("```")
    else:
        lines.append("没有识别到VW快照。查看mcu_ble_raw.txt的VD33、READY/DIAG及其他原文。")
    lines += ["", "## 解释边界", "",
        "- 工具不连接COM、不发送指令、不监听实体导线。相机日志需复制MaixVision DEVICE；MCU日志需手机BLE导出。",
        "- RX指该设备读到的字节；相机TX complete只代表驱动报告整帧写入，不等于MCU收到/解析/执行。",
        "- 相机TX partial/blocked的HEX是整帧发送意图，不是实际完整上线路径；见events.jsonl警告。",
        "- 相机业务TX默认首帧及每10帧打印；没日志不等于没发送。VR仅是缓存前缀，不作为完整帧。",
        "- CRC通过只证明日志内该帧校验正确；不证明几何、任务、阶段门控、时效或硬件动作验收。",
        "- 两端时间没有自动同步，保存时间是电脑收集时间；保持原始顺序，结合请求ID/序号人工核对。",
        "- 上传前审查设备标识、电话号码、个人路径及其他隐私；本工具不会自动上传或联网。", ""]
    return "\n".join(lines)


def save_session(camera, mcu, output, note=""):
    if not camera.strip() and not mcu.strip():
        raise ValueError("请先粘贴或导入至少一端真实日志。")
    now = datetime.now().astimezone()
    folder = Path(output) / (now.strftime("test_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8])
    folder.mkdir(parents=True, exist_ok=False)
    events = analyze(camera, "camera") + analyze(mcu, "mcu")
    for filename, content in (("camera_raw.txt", camera), ("mcu_ble_raw.txt", mcu)):
        (folder / filename).write_bytes(content.encode("utf-8"))
    with (folder / "events.jsonl").open("w", encoding="utf-8", newline="\n") as handle:
        for event in events:
            handle.write(json.dumps(event, ensure_ascii=False) + "\n")
    with (folder / "rx_tx.csv").open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["source", "source_line", "direction", "write_status", "decoded_frames", "warnings", "raw"])
        for event in events:
            # Excel公式危险前缀加引号；完整原文保留于TXT/JSONL。
            values = [event["source"], event["line"], event["direction"], event.get("write_status", ""),
                json.dumps(event["frames"], ensure_ascii=False), "; ".join(event["warnings"]), event["raw"]]
            writer.writerow(["'" + v if isinstance(v, str) and v.startswith(("=", "+", "-", "@", "\t", "\r")) else v for v in values])
    (folder / "核对报告.md").write_text(make_report(events, camera, mcu, note), encoding="utf-8")
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir() if p.is_file()}
    (folder / "metadata.json").write_text(json.dumps({"collected_at": now.isoformat(),
        "capture_method": "manual_console_copy_and_ble_import", "note": note, "uploaded": False,
        "sha256": hashes}, ensure_ascii=False, indent=2), encoding="utf-8")
    return folder


def read_log(path):
    data = Path(path).read_bytes()
    if data.startswith((b"\xff\xfe", b"\xfe\xff")):
        return data.decode("utf-16")
    for encoding in ("utf-8-sig", "gb18030"):
        try:
            return data.decode(encoding)
        except UnicodeError:
            pass
    raise ValueError("日志编码不支持；请另存为UTF-8后导入，避免丢失原文。")


def show_gui():
    import tkinter as tk
    from tkinter import filedialog, messagebox
    from tkinter.scrolledtext import ScrolledText
    root = tk.Tk()
    root.title("双端RX/TX日志收集器（离线，不发送命令）")
    root.geometry("1060x740")
    root.minsize(780, 540)
    tk.Label(root, text="1. 相机：复制MaixVision DEVICE日志。  2. 电控：导入手机BLE导出的txt。  3. 保存。\n不占用串口；不是实时抓线工具。先用33静止诊断，勿误启动运动模式。", anchor="w", justify="left").pack(fill="x", padx=12, pady=8)
    tk.Label(root, text="本轮说明：相机App/MCU固件版本、测试模式、开始结束时间、现象（可选）", anchor="w").pack(fill="x", padx=12)
    note = tk.Entry(root)
    note.pack(fill="x", padx=12, pady=4)
    panes = tk.PanedWindow(root, orient="horizontal", sashwidth=8)
    panes.pack(fill="both", expand=True, padx=12, pady=8)
    boxes = []
    for title in ("相机 MaixVision DEVICE", "电控 手机BLE导出日志"):
        panel = tk.Frame(panes)
        panes.add(panel, stretch="always")
        tk.Label(panel, text=title).pack(anchor="w")
        buttons = tk.Frame(panel)
        buttons.pack(fill="x")
        box = ScrolledText(panel, wrap="none", undo=True, width=48)
        box.pack(fill="both", expand=True)
        boxes.append(box)

        def paste(target=box):
            try:
                content = root.clipboard_get()
            except tk.TclError:
                messagebox.showwarning("未读取到文本", "先复制设备日志，再点击粘贴。")
                return
            if target.get("1.0", "end-1c").strip() and not messagebox.askyesno("替换？", "用剪贴板替换这栏日志？重复追加会重复计数。"):
                return
            target.delete("1.0", "end")
            target.insert("1.0", content)

        def load(target=box):
            path = filedialog.askopenfilename(filetypes=[("日志", "*.txt *.log"), ("所有文件", "*.*")])
            if not path:
                return
            if target.get("1.0", "end-1c").strip() and not messagebox.askyesno("替换？", "导入将替换这栏内容，继续？"):
                return
            try:
                content = read_log(path)
            except (OSError, ValueError) as exc:
                messagebox.showerror("导入失败", str(exc))
                return
            target.delete("1.0", "end")
            target.insert("1.0", content)

        tk.Button(buttons, text="粘贴整段日志", command=paste).pack(side="left", pady=4)
        tk.Button(buttons, text="导入TXT文件", command=load).pack(side="left", padx=6)
    last = tk.StringVar(value="尚未保存。先开始两端日志记录，再进行同一轮测试；保留启动版本信息。")
    tk.Label(root, textvariable=last, anchor="w", wraplength=1000, justify="left").pack(fill="x", padx=12, pady=8)

    def save():
        try:
            folder = save_session(boxes[0].get("1.0", "end-1c"), boxes[1].get("1.0", "end-1c"), Path(__file__).parent / "sessions", note.get())
        except (OSError, ValueError) as exc:
            messagebox.showerror("保存失败", str(exc))
            return
        last.set("已保存：" + str(folder) + "\n尚未上传。测试结束后将这整个目录交给Codex核对。")
        messagebox.showinfo("本地保存成功", f"{folder}\n\n保存原文、RX/TX表、结构化记录和核对报告。没有联网或上传。")

    tk.Button(root, text="保存本轮双端日志（不上传）", command=save, height=2).pack(fill="x", padx=12, pady=(0, 12))
    root.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=Path, help="MaixVision DEVICE导出的日志")
    parser.add_argument("--mcu", type=Path, help="手机BLE导出的日志")
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "sessions")
    parser.add_argument("--note", default="")
    args = parser.parse_args()
    if args.camera or args.mcu:
        try:
            print(save_session(read_log(args.camera) if args.camera else "", read_log(args.mcu) if args.mcu else "", args.output, args.note))
        except (OSError, ValueError) as exc:
            parser.exit(1, str(exc) + "\n")
    else:
        show_gui()


if __name__ == "__main__":
    main()
