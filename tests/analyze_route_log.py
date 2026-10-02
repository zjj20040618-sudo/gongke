#!/usr/bin/env python3
"""Read-only BLE/ASCII record analysis. Never infer physical travel from encoders.

python tests/analyze_route_log.py BLE_Log.txt --out C:/temp/route-review
python tests/analyze_route_log.py BLE_Log.txt --measurements route_measurements_template.csv
python tests/analyze_route_log.py --self-test

The output directory receives summaries.csv, records.jsonl and decoded RX text.
records.jsonl retains PARAM_BUNDLE objects plus the parameter snapshot seen by
each REC/TRC. RETURN_RESUME never changes an earlier pause record's parameters.
Input files and measurement sheets are never rewritten. No third-party packages.
"""

import argparse
import copy
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
PARAM_EVENT = re.compile(r"\b(PARAM(?:_START|_END|_MOVE|_TURN|_CROSS|_JOG|_SERVO|_WHEEL)?)\s+")
PARAM_REQUIRED = {
    "PARAM_START": {"FW", "test", "mode", "leg", "phase"},
    "PARAM": {"test", "phase", "kp", "ki", "lp", "dead", "ykp", "okp", "acc", "dec", "lff", "rff", "fff"},
    "PARAM_END": {"test", "phase", "tx_drop"},
    "PARAM_MOVE": {"test", "phase", "v_mms", "cmd_mm", "ff_ratio"},
    "PARAM_TURN": {"test", "phase", "target_deg", "kp", "wmax", "wmin", "tol_deg", "max_ms"},
    "PARAM_CROSS": {"test", "phase", "v_mms", "xrise", "xflat", "max_ms"},
    "PARAM_JOG": {"test", "phase", "axis", "dir", "n"},
    "PARAM_SERVO": {"test", "phase", "us", "no_feedback"},
    "PARAM_WHEEL": {"test", "phase", "wheel", "duty"},
}
PARAM_DETAIL_BY_MODE = {
    **dict.fromkeys([*range(1, 7), *range(15, 19)], "PARAM_MOVE"),
    **dict.fromkeys([20, 22, 30, 32], "PARAM_TURN"),
    **dict.fromkeys(range(24, 28), "PARAM_JOG"),
    **dict.fromkeys([28, 29], "PARAM_SERVO"),
    **dict.fromkeys(range(7, 11), "PARAM_WHEEL"),
    31: "PARAM_CROSS",
}
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
            if EVENT.search(raw) or PARAM_EVENT.search(raw) or "READY FW=" in raw or "DIAG FW=" in raw or raw.startswith("ERR "):
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
    bundles = []
    latest_bundle = {}
    active_bundle = None

    def normalized_phase(phase):
        return "RETURN" if phase == "RETURN_RESUME" else phase

    def update_counter(test, first):
        nonlocal previous_test, epoch, active_bundle
        if previous_test is not None and int(test) < previous_test:
            if active_bundle is not None:
                active_bundle["issues"].append("counter_changed_before_PARAM_END")
                active_bundle = None
            epoch += 1
            warnings.append(f"line {first}: test counter decreased without READY; new fragment, not a proven power cycle")
        previous_test = int(test)

    def append_parameter_line(bundle, kind, fields, first, last, raw, complete):
        bundle["lines"].append({"kind": kind, "fields": fields, "line_start": first,
                                "line_end": last, "complete_line": complete, "raw": raw})
        bundle["line_end"] = last
        if not complete:
            bundle["issues"].append("truncated_or_unterminated_parameter_line")
        missing = PARAM_REQUIRED[kind] - fields.keys()
        if missing:
            bundle["issues"].append(kind + "_missing_" + ",".join(sorted(missing)))
        if fields.get("test") != bundle["test_id"] or fields.get("phase") != bundle["phase"]:
            bundle["issues"].append("parameter_identity_mismatch")
        if kind in bundle["parameters"]:
            bundle["issues"].append("duplicate_" + kind)
        bundle["parameters"][kind] = fields

    def new_bundle(fields, first, last):
        bundle = {"bundle_id": "|".join((source, str(boot), str(epoch), "PARAM", str(len(bundles) + 1))),
                  "source_file": source, "boot_batch": str(boot), "counter_epoch": str(epoch),
                  "fw_id": fw, "test_id": fields.get("test", ""), "phase": fields.get("phase", ""),
                  "line_start": first, "line_end": last, "complete": False,
                  "parameters": {}, "lines": [], "issues": []}
        bundles.append(bundle)
        return bundle

    for first, last, raw, complete in logical:
        if "READY FW=" in raw:
            if active_bundle is not None:
                active_bundle["issues"].append("READY_before_PARAM_END")
                active_bundle = None
            boot += 1
            epoch = 0
            previous_test = None
        if "FW=" in raw:
            fw = dict(PAIR.findall(raw)).get("FW", fw)
        if raw.startswith("ERR "):
            warnings.append(f"line {first}: {raw}")
        param_match = PARAM_EVENT.search(raw)
        if param_match:
            kind = param_match.group(1)
            fields = dict(PAIR.findall(raw[param_match.start():]))
            test = fields.get("test", "")
            # Historical manual PARAM dumps have no identity; preserve them in decoded text only.
            if kind == "PARAM" and (not test.isdigit() or not fields.get("phase")):
                continue
            if kind == "PARAM_START":
                if test.isdigit():
                    update_counter(test, first)
                if active_bundle is not None:
                    active_bundle["issues"].append("new_PARAM_START_before_PARAM_END")
                active_bundle = new_bundle(fields, first, last)
                if not test.isdigit():
                    active_bundle["issues"].append("invalid_parameter_test")
                if fields.get("phase") not in {"OUT", "TEST", "RETURN", "RETURN_RESUME"}:
                    active_bundle["issues"].append("invalid_parameter_phase")
                latest_bundle[(str(boot), str(epoch), test, normalized_phase(fields.get("phase", "")))] = active_bundle
                append_parameter_line(active_bundle, kind, fields, first, last, raw, complete)
            else:
                if active_bundle is None:
                    active_bundle = new_bundle(fields, first, last)
                    active_bundle["issues"].append("missing_PARAM_START")
                    latest_bundle[(str(boot), str(epoch), test, normalized_phase(fields.get("phase", "")))] = active_bundle
                append_parameter_line(active_bundle, kind, fields, first, last, raw, complete)
                if kind == "PARAM_END":
                    if "PARAM" not in active_bundle["parameters"]:
                        active_bundle["issues"].append("missing_common_PARAM")
                    mode_text = active_bundle["parameters"].get("PARAM_START", {}).get("mode", "")
                    required_detail = PARAM_DETAIL_BY_MODE.get(int(mode_text)) if mode_text.isdigit() else None
                    if required_detail and required_detail not in active_bundle["parameters"]:
                        active_bundle["issues"].append("missing_required_" + required_detail)
                    active_bundle["complete"] = not active_bundle["issues"]
                    active_bundle = None
            continue
        match = EVENT.search(raw)
        if not match or match.group(2) not in TYPES:
            continue
        fields = dict(PAIR.findall(raw[match.start():]))
        test = fields.get("test", "")
        if not test.isdigit():
            warnings.append(f"line {first}: missing/integer-invalid test id; not merged with numbered tests")
            test = f"missing-line-{first}"
        if test.isdigit():
            update_counter(test, first)
        phase = fields.get("phase", "OUT" if fields["type"] in {"DIST", "ROUTE"} else "TEST")
        # Snapshot NOW. A later RETURN_RESUME packet cannot alter a previous pause record.
        snapshot = copy.deepcopy(latest_bundle.get((str(boot), str(epoch), test, normalized_phase(phase))))
        events.append({"source_file": source, "boot_batch": str(boot), "counter_epoch": str(epoch),
                       "fw_id": fw, "kind": match.group(1), "record_type": fields["type"],
                       "test_id": test, "phase": phase, "line_start": first, "line_end": last,
                       "complete_line": complete, "fields": fields, "raw": raw,
                       "parameter_bundle": snapshot})
    if active_bundle is not None:
        active_bundle["issues"].append("missing_PARAM_END")
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
    # Keep even unassociated/partial packages in records.jsonl; summaries ignore this kind.
    for bundle in bundles:
        if not bundle["complete"]:
            warnings.append(f"line {bundle['line_start']}: partial parameter bundle: " + ";".join(bundle["issues"]))
        events.append({"kind": "PARAM_BUNDLE", **bundle})
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
        if event["kind"] != "PARAM_BUNDLE":
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
        snapshot = last.get("parameter_bundle")
        if snapshot is None:
            parameter_status = "missing"
            warnings.append("missing_parameter_bundle_legacy_or_incomplete_capture")
        elif not snapshot["complete"]:
            parameter_status = "partial"
            warnings.append("partial_parameter_bundle")
        else:
            parameter_status = "complete"
        if snapshot and number(snapshot["parameters"].get("PARAM_END", {}), "tx_drop"):
            warnings.append("tx_drop_counter_nonzero_review_capture")
        snapshot_ids = list(dict.fromkeys(e["parameter_bundle"]["bundle_id"] for e in group if e.get("parameter_bundle")))
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
                    "parameter_status": parameter_status,
                    "parameter_phase": snapshot["phase"] if snapshot else "",
                    "parameter_bundle_ids": ";".join(snapshot_ids),
                    "parameters_json": json.dumps(snapshot["parameters"], ensure_ascii=False) if snapshot else "",
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
    @staticmethod
    def parameter_packet(test=1, phase="OUT", speed=300, distance=500, end=True):
        text = (f"PARAM_START FW=20261002-AUTO-PARAM test={test} mode=15 leg=0 phase={phase}\n"
                f"PARAM test={test} phase={phase} kp=0.05 ki=0.004 lp=0.2 dead=25 ykp=2 okp=0 acc=0 dec=0 lff=0.02 rff=0.02 fff=0\n"
                f"PARAM_MOVE test={test} phase={phase} v_mms={speed} cmd_mm={distance} ff_ratio=0\n")
        if end:
            text += f"PARAM_END test={test} phase={phase} tx_drop=0\n"
        return text

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

    def test_parameter_firmware_without_ready_and_hex_reassembly(self):
        payload = self.parameter_packet().replace("\n", "\r\n")
        payload += "REC type=DIST test=1 phase=OUT status=DONE enc_mm=501\r\n"
        text = "\n".join("[通知] 接收数据 | 数据: " + payload[i:i + 17].encode().hex(" ")
                         for i in range(0, len(payload), 17))
        events, _, _ = parse_capture(text, "new.txt")
        row = summarize(events)[0]
        self.assertEqual(row["fw_id"], "20261002-AUTO-PARAM")
        self.assertEqual(row["boot_batch"], "0")
        self.assertEqual(row["parameter_status"], "complete")
        self.assertEqual(json.loads(row["parameters_json"])["PARAM_MOVE"]["cmd_mm"], "500")
        self.assertEqual(len([e for e in events if e["kind"] == "PARAM_BUNDLE"]), 1)

    def test_parameter_out_return_and_resume_do_not_rewrite_pause(self):
        text = self.parameter_packet(speed=300, distance=500)
        text += "REC type=DIST test=1 phase=OUT status=STOP\n"
        text += self.parameter_packet(phase="RETURN", speed=180, distance=-140)
        text += "REC type=DIST test=1 phase=RETURN status=STOP\n"
        text += self.parameter_packet(phase="RETURN_RESUME", speed=200, distance=-70)
        text += "REC type=DIST test=1 phase=RETURN status=DONE\n"
        events, _, _ = parse_capture(text, "return.txt")
        recs = [e for e in events if e["kind"] == "REC"]
        self.assertEqual([e["parameter_bundle"]["parameters"]["PARAM_MOVE"]["v_mms"] for e in recs],
                         ["300", "180", "200"])
        self.assertEqual([e["parameter_bundle"]["parameters"]["PARAM_MOVE"]["cmd_mm"] for e in recs],
                         ["500", "-140", "-70"])
        rows = summarize(events)
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[1]["parameter_phase"], "RETURN_RESUME")
        self.assertEqual(len(rows[1]["parameter_bundle_ids"].split(";")), 2)
        self.assertTrue(all(e["counter_epoch"] == "0" for e in recs))

    def test_parameter_missing_end_stays_partial(self):
        text = self.parameter_packet(end=False) + "REC type=DIST test=1 phase=OUT status=DONE\n"
        events, _, _ = parse_capture(text, "partial.txt")
        self.assertEqual(summarize(events)[0]["parameter_status"], "partial")
        bundle = next(e for e in events if e["kind"] == "PARAM_BUNDLE")
        self.assertIn("missing_PARAM_END", bundle["issues"])
        self.assertFalse(bundle["complete"])

    def test_parameter_counter_reset_and_ready_boundaries(self):
        text = self.parameter_packet(test=7) + "REC type=DIST test=7 phase=OUT status=DONE\n"
        text += self.parameter_packet(test=1, speed=200) + "REC type=DIST test=1 phase=OUT status=DONE\n"
        text += "READY FW=20261002-AUTO-PARAM\nREC type=DIST test=1 phase=OUT status=DONE\n"
        events, _, warnings = parse_capture(text, "reset.txt")
        recs = [e for e in events if e["kind"] == "REC"]
        self.assertEqual([e["counter_epoch"] for e in recs], ["0", "1", "0"])
        self.assertEqual([e["boot_batch"] for e in recs], ["0", "0", "1"])
        self.assertEqual(recs[1]["parameter_bundle"]["counter_epoch"], "1")
        self.assertIsNone(recs[2]["parameter_bundle"])
        self.assertEqual(sum("counter decreased" in w for w in warnings), 1)

    def test_parameter_mode_requires_its_action_detail(self):
        for mode, detail in PARAM_DETAIL_BY_MODE.items():
            with self.subTest(mode=mode, required=detail):
                packet = self.parameter_packet().replace("mode=15 ", f"mode={mode} ")
                packet = "\n".join(line for line in packet.splitlines() if not line.startswith("PARAM_MOVE ")) + "\n"
                packet += "REC type=DIST test=1 phase=OUT status=DONE\n"
                events, _, _ = parse_capture(packet, "missing-detail.txt")
                self.assertEqual(summarize(events)[0]["parameter_status"], "partial")
                bundle = next(e for e in events if e["kind"] == "PARAM_BUNDLE")
                self.assertIn("missing_required_" + detail, bundle["issues"])

    def test_parameter_wheel_and_modes_without_detail(self):
        packet = self.parameter_packet().replace("mode=15 ", "mode=7 ")
        packet = re.sub(r"PARAM_MOVE[^\n]+", "PARAM_WHEEL test=1 phase=OUT wheel=0 duty=120", packet)
        events, _, _ = parse_capture(packet, "wheel.txt")
        bundle = next(e for e in events if e["kind"] == "PARAM_BUNDLE")
        self.assertTrue(bundle["complete"])
        self.assertEqual(bundle["parameters"]["PARAM_WHEEL"]["duty"], "120")
        for mode in (13, 14, 19, 21, 23):
            with self.subTest(mode=mode):
                packet = self.parameter_packet().replace("mode=15 ", f"mode={mode} ")
                packet = "\n".join(line for line in packet.splitlines() if not line.startswith("PARAM_MOVE ")) + "\n"
                events, _, _ = parse_capture(packet, "no-detail-required.txt")
                self.assertTrue(next(e for e in events if e["kind"] == "PARAM_BUNDLE")["complete"])


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
              f"params={row['parameter_status']} "
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
