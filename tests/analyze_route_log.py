#!/usr/bin/env python3
"""Read-only BLE/ASCII record analysis. Never infer physical travel from encoders.

python tests/analyze_route_log.py BLE_Log.txt --out C:/temp/route-review
python tests/analyze_route_log.py BLE_Log.txt --measurements route_measurements_template.csv
python tests/analyze_route_log.py --self-test

The output directory receives summaries.csv, records.jsonl and decoded RX text.
Input files and measurement sheets are never rewritten. No third-party packages.
"""

import argparse
import csv
import io
import json
import math
import re
import sys
import unittest
from collections import defaultdict
from pathlib import Path

TYPES = {"DIST", "ROUTE", "TURN90", "TURN180", "TURNL90", "TURN180_FORMAL", "CROSS", "IMU"}
PAIR = re.compile(r"([A-Za-z_][A-Za-z_0-9]*)=([^\s;]+)")
EVENT = re.compile(r"\b(REC|TRC)\s+type=([A-Za-z_0-9]+)\b")
RX = re.compile(r"(?:接收数据|Received data|\[RX\])", re.I)
TX = re.compile(r"(?:\[写入\]|\[发送\]|\[TX\]|写入成功)", re.I)
HEX = re.compile(r"(?:[0-9a-fA-F]{2}\s+)*[0-9a-fA-F]{2}")
KNOWN_STATUS = {"DONE", "OK", "STOP", "CANCELLED", "CANCELED", "ABORT"}


def read_text(path):
    data = Path(path).read_bytes()
    if data.startswith((b"\xff\xfe", b"\xfe\xff")):
        return data.decode("utf-16")
    for encoding in ("utf-8-sig", "gb18030"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            pass
    raise ValueError(f"Cannot decode text: {path}; export ASCII/UTF-8 text, not a binary capture")


def decode_lines(text):
    """Reassemble only RX notification bytes; never execute or parse sent commands."""
    pending = ""
    start = 0
    logical = []
    warnings = []
    for number, raw in enumerate(text.splitlines(), 1):
        if TX.search(raw):
            continue
        if RX.search(raw) and ("数据:" in raw or "Data:" in raw):
            payload = re.split(r"数据:|Data:", raw, maxsplit=1)[1].strip()
            if HEX.fullmatch(payload):
                payload = bytes.fromhex(payload).decode("ascii", errors="replace")
                if "\ufffd" in payload:
                    warnings.append(f"line {number}: non-ASCII RX bytes; retained replacement characters")
                if not pending:
                    start = number
                pending += payload
                while "\n" in pending:
                    line, pending = pending.split("\n", 1)
                    logical.append((start, number, line.rstrip("\r"), True))
                    start = number
                continue
            # A decoded mobile RX payload is treated as a complete logical line.
            if pending:
                logical.append((start, number - 1, pending, False))
                warnings.append(f"line {start}: interrupted RX fragment")
                pending = ""
            logical.append((number, number, payload, True))
        else:
            if EVENT.search(raw) or "READY FW=" in raw or "DIAG FW=" in raw or raw.startswith(("PARAM ", "ERR ")):
                if pending:
                    logical.append((start, number - 1, pending, False))
                    warnings.append(f"line {start}: interrupted RX fragment")
                    pending = ""
                logical.append((number, number, raw, True))
    if pending:
        logical.append((start, len(text.splitlines()), pending, False))
        warnings.append(f"line {start}: unterminated RX tail; record may be truncated")
    return logical, warnings


def parse_capture(text, source):
    logical, warnings = decode_lines(text)
    boot = 0  # 0 means the capture contains no observed startup banner yet.
    epoch = 0
    previous_test = None
    fw = ""
    events = []
    for first, last, raw, complete in logical:
        if "READY FW=" in raw:
            boot += 1
            epoch = 0
            previous_test = None
        if "FW=" in raw:
            fw = dict(PAIR.findall(raw)).get("FW", fw)
        if raw.startswith("ERR "):
            warnings.append(f"line {first}: {raw}")
        match = EVENT.search(raw)
        if not match or match.group(2) not in TYPES:
            continue
        fields = dict(PAIR.findall(raw[match.start():]))
        test = fields.get("test", "")
        if not test.isdigit():
            warnings.append(f"line {first}: missing/integer-invalid test id; not merged with numbered tests")
            test = f"missing-line-{first}"
        elif previous_test is not None and int(test) < previous_test:
            epoch += 1
            warnings.append(f"line {first}: test counter decreased without READY; new fragment, not a proven power cycle")
        if test.isdigit():
            previous_test = int(test)
        phase = fields.get("phase", "OUT" if fields["type"] in {"DIST", "ROUTE"} else "TEST")
        events.append({"source_file": source, "boot_batch": str(boot), "counter_epoch": str(epoch),
                       "fw_id": fw, "kind": match.group(1), "record_type": fields["type"],
                       "test_id": test, "phase": phase, "line_start": first, "line_end": last,
                       "complete_line": complete, "fields": fields, "raw": raw})
    # Firmware route legs reuse DIST traces. Associate only when a unique ROUTE REC proves it.
    route_keys = defaultdict(set)
    for event in events:
        if event["record_type"] == "ROUTE" and event["kind"] == "REC":
            identity = tuple(event[k] for k in ("boot_batch", "counter_epoch", "test_id", "phase"))
            route_keys[identity].add(event["fields"].get("leg", ""))
    for event in events:
        identity = tuple(event[k] for k in ("boot_batch", "counter_epoch", "test_id", "phase"))
        if event["record_type"] == "DIST" and event["kind"] == "TRC" and len(route_keys[identity]) == 1:
            event["record_type"] = "ROUTE"
            event["fields"]["leg"] = next(iter(route_keys[identity]))
        leg = event["fields"].get("leg", "") if event["record_type"] == "ROUTE" else ""
        event["record_key"] = "|".join((source, event["boot_batch"], event["counter_epoch"],
                                         event["record_type"], event["test_id"], event["phase"], leg))
    if events and not boot:
        warnings.append("No READY banner: boot_batch=0 is unknown; files are never merged by test id alone")
    if not events:
        warnings.append("No supported REC/TRC records found")
    return events, logical, warnings


def number(fields, *names):
    for name in names:
        try:
            value = float(fields[name])
            if math.isfinite(value):
                return value
        except (KeyError, TypeError, ValueError):
            pass
    return None


def summarize(events):
    groups = defaultdict(list)
    for event in events:
        groups[event["record_key"]].append(event)
    rows = []
    for key, group in groups.items():
        recs = [e for e in group if e["kind"] == "REC"]
        traces = [e for e in group if e["kind"] == "TRC"]
        last = recs[-1] if recs else group[-1]
        fields = last["fields"]
        warnings = []
        if not recs:
            warnings.append("missing_final_REC")
        if len(recs) > 1:
            warnings.append("multiple_REC_review_pause_resume_or_duplicate")
        if any(not e["complete_line"] for e in group):
            warnings.append("truncated_or_unterminated_line")
        status = fields.get("status", "MISSING_REC" if not recs else "MISSING_STATUS")
        if status not in {"DONE", "OK"}:
            warnings.append("not_completed")
        if status not in KNOWN_STATUS:
            warnings.append(f"review_status_{status}")
        yaws = [number(e["fields"], "yaw_deg", "yaw", "yend_deg") for e in group]
        if any(y is not None and abs(y) >= 9999 for y in yaws):
            warnings.append("invalid_yaw_sentinel")
        yaws = [y for y in yaws if y is not None and abs(y) < 9999]
        yaw_errors = [number(e["fields"], "yaw_err") for e in group]
        yaw_errors = [y for y in yaw_errors if y is not None and abs(y) < 9999]
        row = {k: last[k] for k in ("source_file", "boot_batch", "counter_epoch", "fw_id", "record_type", "test_id", "phase")}
        row.update({"record_key": key, "line_start": group[0]["line_start"], "line_end": group[-1]["line_end"],
                    "mode": fields.get("mode", ""), "leg": fields.get("leg", ""), "status": status,
                    "completion": "firmware_done" if status == "DONE" else "measurement_reported" if status == "OK" else "not_completed",
                    "rec_count": len(recs), "trace_count": len(traces),
                    "sampled_yaw_abs_max_deg": max(map(abs, yaws)) if yaws else "",
                    "sampled_yaw_error_abs_max_deg": max(map(abs, yaw_errors)) if yaw_errors else "",
                    "yaw_imu_end_deg": number(fields, "yaw_deg", "yend_deg", "yaw"),
                    "cross_pitch_deg": number(fields, "pitch"), "cross_pitch_pp_deg": number(fields, "pp"),
                    "cross_rise_seen": number(fields, "rise"), "cross_full_seen": number(fields, "full"),
                    "cross_yaw_error_deg": number(fields, "yaw_err"),
                    "cmd_deg": number(fields, "cmd_deg"), "err_deg": number(fields, "err_deg"),
                    "cmd_mm": number(fields, "cmd_mm"), "enc_mm": number(fields, "enc_mm"),
                    "brake_mm": number(fields, "brake_mm"), "run_ms": number(fields, "run_ms", "ms"),
                    "settle_ms": number(fields, "settle_ms"), "ff_ratio": number(fields, "ff_ratio"),
                    "imu_bad_samples": number(fields, "bad"), "imu_software_samples": number(fields, "n"),
                    "imu_ypp_deg": number(fields, "ypp_deg"), "encoder_brake_delta_mm": "",
                    "imu_net_yaw_rate_dps": "", "measurement_items": "", "physical_mm": "",
                    "orth_max_mm": "", "yaw_physical_end_deg": ""})
        if row["enc_mm"] is not None and row["brake_mm"] is not None:
            row["encoder_brake_delta_mm"] = row["enc_mm"] - row["brake_mm"]
        if row["record_type"] == "IMU":
            elapsed = row["run_ms"]
            if (elapsed and elapsed > 0 and status == "OK" and not row["imu_bad_samples"]
                    and row["yaw_imu_end_deg"] is not None and abs(row["yaw_imu_end_deg"]) < 9999):
                row["imu_net_yaw_rate_dps"] = row["yaw_imu_end_deg"] * 1000 / elapsed
            if row["imu_bad_samples"]:
                warnings.append("IMU_invalid_samples_present")
            if any(e["fields"].get("ok") == "0" for e in traces):
                warnings.append("IMU_invalid_trace_present")
            warnings.append("stationary_required_for_drift_interpretation")
        if row["boot_batch"] == "0":
            warnings.append("power_cycle_unknown")
        row["warnings"] = ";".join(warnings)
        rows.append(row)
    return rows


def associate_measurements(rows, sheet):
    warnings = []
    for measure in csv.DictReader(io.StringIO(read_text(sheet))):
        exact = measure.get("record_key", "").strip()
        source = measure.get("log_file", "").strip()
        test = measure.get("test_id", "").strip()
        if not exact and not (source and test):
            continue  # Blank planning rows contain no measured association yet.
        matches = []
        for row in rows:
            if exact:
                match = row["record_key"] == exact
            else:
                match = (row["source_file"] == source or Path(row["source_file"]).name == Path(source).name) and row["test_id"] == test
                for sheet_key, key in (("boot_batch", "boot_batch"), ("record_type", "record_type"), ("phase", "phase")):
                    value = measure.get(sheet_key, "").strip()
                    if value and row[key] != value:
                        match = False
            if match:
                matches.append(row)
        if len(matches) != 1:
            warnings.append(f"measurement {measure.get('item_id', '?')}: {len(matches)} matches; not attached")
            continue
        row = matches[0]
        row["measurement_items"] = ";".join(filter(None, (row["measurement_items"], measure.get("item_id", ""))))
        for field in ("physical_mm", "orth_max_mm", "yaw_physical_end_deg"):
            value = measure.get(field, "").strip()
            if value:
                row[field] = ";".join(filter(None, (row[field], value)))
    return warnings


class AnalyzerTests(unittest.TestCase):
    def test_hex_fragments_and_tx(self):
        payload = "REC type=DIST test=1 phase=OUT status=DONE enc_mm=100 brake_mm=96\r\n"
        chunks = [payload[:19], payload[19:41], payload[41:]]
        text = "\n".join("[通知] 接收数据 | 数据: " + chunk.encode().hex(" ") for chunk in chunks)
        text += "\n[写入] 数据: 67"
        events, _, _ = parse_capture(text, "one")
        self.assertEqual(len(events), 1)
        self.assertEqual(summarize(events)[0]["encoder_brake_delta_mm"], 4)

    def test_boot_phase_and_route(self):
        text = ("READY FW=v1\nTRC type=DIST test=1 yaw_deg=0.2\n"
                "REC type=ROUTE test=1 leg=2 phase=OUT status=DONE\n"
                "REC type=ROUTE test=1 leg=2 phase=RETURN status=STOP\n"
                "READY FW=v1\nREC type=DIST test=1 phase=OUT status=DONE\n")
        rows = summarize(parse_capture(text, "one")[0])
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[0]["trace_count"], 1)
        self.assertEqual(rows[1]["phase"], "RETURN")
        self.assertEqual(rows[2]["boot_batch"], "2")

    def test_reset_and_truncated(self):
        text = "REC type=TURN90 test=7 status=DONE yaw_deg=90\nTRC type=TURNL90 test=1 yaw=-10\n"
        events, _, warnings = parse_capture(text, "one")
        self.assertEqual(events[-1]["counter_epoch"], "1")
        self.assertTrue(any("counter decreased" in w for w in warnings))
        self.assertIn("missing_final_REC", summarize(events)[-1]["warnings"])
        tail = "REC type=IMU test=1 status=OK ms=1000 yend_deg=0.2"
        events, _, _ = parse_capture("[通知] 接收数据 | 数据: " + tail.encode().hex(" "), "one")
        self.assertIn("truncated", summarize(events)[0]["warnings"])

    def test_ambiguous_measurement_is_not_attached(self):
        from unittest.mock import patch
        text = ("READY FW=v1\nREC type=DIST test=1 phase=OUT status=DONE\n"
                "REC type=DIST test=1 phase=RETURN status=DONE\n")
        rows = summarize(parse_capture(text, "one.txt")[0])
        sheet = "item_id,log_file,test_id,phase,physical_mm\nA,one.txt,1,,500\n"
        with patch(__name__ + ".read_text", return_value=sheet):
            self.assertEqual(len(associate_measurements(rows, "unused")), 1)
        self.assertTrue(all(row["physical_mm"] == "" for row in rows))
        sheet = "item_id,log_file,test_id,phase,physical_mm\nA,one.txt,1,OUT,500\n"
        with patch(__name__ + ".read_text", return_value=sheet):
            self.assertEqual(associate_measurements(rows, "unused"), [])
        self.assertEqual(rows[0]["physical_mm"], "500")
        self.assertEqual(rows[1]["physical_mm"], "")

    def test_stop_is_not_completed_and_cross_error_is_separate(self):
        text = ("READY FW=v1\nTRC type=CROSS test=1 pitch=2.1 pp=2.6 rise=1 full=0 yaw_err=-1.2\n"
                "REC type=CROSS test=1 status=STOP pitch=0.1 pp=0.4 rise=1 full=1 yaw_err=0.5\n")
        row = summarize(parse_capture(text, "cross.txt")[0])[0]
        self.assertEqual(row["completion"], "not_completed")
        self.assertIn("not_completed", row["warnings"])
        self.assertEqual(row["sampled_yaw_error_abs_max_deg"], 1.2)
        self.assertEqual(row["sampled_yaw_abs_max_deg"], "")
        self.assertEqual(row["cross_pitch_pp_deg"], 0.4)
        self.assertEqual(row["cross_full_seen"], 1)

    def test_imu_trace_invalid_sentinel_is_excluded(self):
        text = ("READY FW=v1\nTRC type=IMU test=1 ms=1000 yaw_deg=0.1 pitch=0 roll=0 age_ms=10 ok=1\n"
                "TRC type=IMU test=1 ms=2000 yaw_deg=9999 pitch=0 roll=0 age_ms=220 ok=0\n"
                "REC type=IMU test=1 status=OK ms=3000 n=150 bad=1 yend_deg=0.2 ypp_deg=0.2\n")
        row = summarize(parse_capture(text, "imu.txt")[0])[0]
        self.assertEqual(row["sampled_yaw_abs_max_deg"], 0.2)
        self.assertEqual(row["imu_net_yaw_rate_dps"], "")
        self.assertIn("IMU_invalid_trace_present", row["warnings"])
        self.assertIn("stationary_required", row["warnings"])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logs", nargs="*", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--measurements", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)
    if args.self_test:
        return 0 if unittest.TextTestRunner().run(unittest.defaultTestLoader.loadTestsFromTestCase(AnalyzerTests)).wasSuccessful() else 1
    if not args.logs:
        parser.error("provide at least one log or --self-test")
    events, decoded, warnings = [], [], []
    for path in args.logs:
        source = str(path.resolve())
        current, lines, notes = parse_capture(read_text(path), source)
        events.extend(current)
        decoded.append((path.name, lines))
        warnings.extend(f"{path.name}: {note}" for note in notes)
    rows = summarize(events)
    if args.measurements:
        warnings.extend(associate_measurements(rows, args.measurements))
    for row in rows:
        print(f"{Path(row['source_file']).name} boot={row['boot_batch']} epoch={row['counter_epoch']} "
              f"{row['record_type']} test={row['test_id']} {row['phase']} {row['status']} "
              f"yaw={row['yaw_imu_end_deg']} enc={row['enc_mm']} brake_delta={row['encoder_brake_delta_mm']} "
              f"[{row['warnings']}]")
    for warning in warnings:
        print("NOTE " + warning)
    print(f"{len(rows)} groups; {len(events)} decoded records. Physical displacement requires independent measurement.")
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        with (args.out / "records.jsonl").open("w", encoding="utf-8") as stream:
            for event in events:
                stream.write(json.dumps(event, ensure_ascii=False) + "\n")
        if rows:
            with (args.out / "summaries.csv").open("w", newline="", encoding="utf-8-sig") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        for index, (name, lines) in enumerate(decoded, 1):
            output = args.out / f"{index:02d}_{Path(name).stem}_decoded.txt"
            with output.open("w", encoding="utf-8") as stream:
                for first, last, raw, complete in lines:
                    stream.write(f"source_lines={first}-{last} complete={int(complete)} {raw}\n")
        (args.out / "notes.txt").write_text("\n".join(warnings), encoding="utf-8")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, UnicodeError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
