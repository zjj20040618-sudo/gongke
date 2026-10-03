"""Build a self-contained, non-actuating vision teammate handoff from this checkout.

Build dependencies: qrcode, Pillow, opencv-python, Markdown. This script never
connects to a board, installs an App, flashes firmware, or publishes a repository.
"""
import argparse
import configparser
from datetime import datetime, timedelta, timezone
import hashlib
import html
import importlib.util
import json
from pathlib import Path
import re
import shutil
import subprocess
from urllib.parse import quote, unquote, urlsplit, urlunsplit
import zipfile

ROOT = Path(__file__).resolve().parents[1]
VISION = ROOT / "03_扫码与物体识别联合App工程"
DOCS = ("VISION_TEAM_HANDOFF.md", "VISION_CONTROL_PROTOCOL.md", "COLLABORATION.md",
        "VISION_INTEGRATION.md", "VISION_CONTROL_TODO.md", "PROJECT_GUIDE.md",
        "CONTROL_TUNING_TODO.md", "ACTUATOR_TEAMMATE_HANDOFF.md")
CSS = """body{font:16px/1.65 system-ui,sans-serif;max-width:950px;margin:30px auto;padding:0 22px;color:#18222b}table{border-collapse:collapse;width:100%}td,th{border:1px solid #ccd4da;padding:8px;text-align:left}pre{white-space:pre-wrap;background:#eef2f5;padding:12px}a{color:#135dad}.qr{display:inline-block;margin:15px;text-align:center}.qr img{width:220px;height:220px}@media print{.qr{break-inside:avoid}.qr img{width:80mm;height:80mm}body{margin:0}}"""
SHANGHAI = timezone(timedelta(hours=8), "Asia/Shanghai")


def git(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True, encoding="utf-8").strip()


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def page(title, body):
    return '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>' + html.escape(title) + '</title><style>' + CSS + '</style><body>' + body + '</body></html>'


def rewrite_links(rendered, source_commit):
    """Keep external links; bind local docs and repository files to this package."""
    def replace(match):
        original = html.unescape(match.group(1))
        parts = urlsplit(original)
        if parts.scheme or parts.netloc or not parts.path:
            return match.group(0)
        path = Path(unquote(parts.path)).as_posix().removeprefix("./")
        if path in DOCS:
            target = urlunsplit(("", "", Path(path).with_suffix(".html").as_posix(), parts.query, parts.fragment))
        else:
            source = (ROOT / path).resolve()
            if not source.is_relative_to(ROOT) or not source.is_file():
                raise ValueError("unresolved repository link in handoff: " + original)
            target = "https://github.com/zjj20040618-sudo/gongke/blob/" + source_commit + "/" + quote(source.relative_to(ROOT).as_posix(), safe="/")
            if parts.query:
                target += "?" + parts.query
            if parts.fragment:
                target += "#" + parts.fragment
        return 'href="' + html.escape(target, quote=True) + '"'
    return re.sub(r'href="([^"]*)"', replace, rendered)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware-hex", required=True, type=Path)
    parser.add_argument("--build-log", required=True, type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("C:/Users/15119/gongkesai/exports"))
    args = parser.parse_args()
    import cv2
    import markdown
    import numpy as np
    import qrcode

    for path in (args.firmware_hex, args.build_log):
        if not path.is_file():
            raise ValueError("missing required build artifact: " + path.name)
    build_log = args.build_log.read_text(encoding="utf-8", errors="replace")
    summaries = re.findall(r"(?<!\d)(\d+)\s+Error\(s\),\s*(\d+)\s+Warning\(s\)", build_log)
    if not summaries or summaries[-1] != ("0", "0"):
        raise ValueError("build log must confirm 0 Error(s), 0 Warning(s)")
    # Metadata must name committed package sources, not an unreported dirty snapshot.
    tracked_paths = [str(VISION.relative_to(ROOT)), "App", "Inc", "Src", "Drivers", "Middlewares", "Startup", "tests", "MDK-ARM/jiejie.uvprojx", "tools/build_vision_handoff.py", *DOCS]
    if git("status", "--porcelain", "--", *tracked_paths):
        raise ValueError("commit handoff sources before packaging; do not discard user changes")
    source_commit = git("rev-parse", "HEAD")
    firmware_id = re.search(r'#define FW_BUILD_ID "([^"]+)"', (ROOT / "App/robot.c").read_text(encoding="utf-8")).group(1)
    hex_data = b"".join(bytes.fromhex(line[9:-2]) for line in args.firmware_hex.read_text(encoding="ascii").splitlines() if line.startswith(":") and line[7:9] == "00")
    if firmware_id.encode("ascii") not in hex_data:
        raise ValueError("HEX does not contain the current firmware ID")

    spec = importlib.util.spec_from_file_location("vision_packager", VISION / "build_packages.py")
    packager = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(packager)
    names = packager.manifest()
    for name in names:
        if name.endswith(".py"):
            compile((packager.APP / name).read_text(encoding="utf-8"), name, "exec")
    mud = configparser.ConfigParser()
    mud.read(packager.APP / "model_9541.mud", encoding="utf-8")
    if mud["basic"]["model"] != "model_9541.cvimodel" or "model_9302.cvimodel" in names:
        raise ValueError("deployment manifest must select the current 9541 model only")
    guide = (ROOT / DOCS[0]).read_text(encoding="utf-8")
    if "__VISION_DIAG_COMMANDS__" in guide:
        raise ValueError("diagnostic commands are not finalized")

    output = args.output_dir.resolve()
    if any(part.lower() == "onedrive" for part in output.parts):
        raise ValueError("handoff exports must be outside OneDrive")
    output.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(SHANGHAI).strftime("%Y%m%d_%H%M%S_%f")
    bundle = output / ("gongke_视觉电控静止联调包_" + stamp)
    bundle.mkdir()
    for folder in ("vision", "firmware", "qrcodes", "evidence"):
        (bundle / folder).mkdir()
    app_zip = bundle / "vision/maix-qr_object_switch-v1.1.0.zip"
    source_zip = bundle / "vision/qr_object_switch_source_v1.1.0.zip"
    packager.write_zip(app_zip, [(packager.APP / name, name) for name in names])
    source_members = [(VISION / name, name) for name in (".gitignore", "build_packages.py")]
    source_members += [(packager.APP / name, "qr_object_app/" + name) for name in names + ["README.md"]]
    source_members += [(p, "qr_object_app/tests/" + p.name) for p in sorted((packager.APP / "tests").glob("test_*.py"))]
    packager.write_zip(source_zip, source_members)
    shutil.copyfile(args.firmware_hex, bundle / "firmware/jiejie.hex")
    shutil.copyfile(args.build_log, bundle / "firmware/keil_rebuild.txt")
    for name in DOCS:
        shutil.copyfile(ROOT / name, bundle / name)
        text = (ROOT / name).read_text(encoding="utf-8")
        rendered = rewrite_links(markdown.markdown(text, extensions=["tables", "fenced_code"]), source_commit)
        (bundle / Path(name).with_suffix(".html")).write_text(page(Path(name).stem, rendered), encoding="utf-8")
    guide_html = rewrite_links(markdown.markdown(guide, extensions=["tables", "fenced_code"]), source_commit)
    (bundle / "先读我.html").write_text(page("视觉电控静止联调", '<p><a href="qrcodes/打印二维码.html">打印测试二维码</a> · <a href="evidence/回包填写模板.html">回包填写模板</a> · <a href="VERSION.json">版本与验证范围</a></p>' + guide_html), encoding="utf-8")

    payloads = {"111": "红球 红靶 圆柱", "222": "绿球 绿靶 圆锥", "333": "蓝球 蓝靶 腰鼓", "123": "红球 绿靶 腰鼓"}
    cards = []
    for text, meaning in payloads.items():
        path = bundle / "qrcodes" / (text + ".png")
        code = qrcode.QRCode(box_size=10, border=4, error_correction=qrcode.constants.ERROR_CORRECT_M)
        code.add_data(text, optimize=0)
        code.make(fit=True)
        code.make_image(fill_color="black", back_color="white").save(path)
        pixels = cv2.imdecode(np.frombuffer(path.read_bytes(), dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        decoded, _, _ = cv2.QRCodeDetector().detectAndDecode(pixels)
        if decoded != text:
            raise ValueError("generated QR round-trip failed: " + text)
        cards.append('<section class="qr"><h2>' + text + '</h2><img src="' + text + '.png" alt="QR ' + text + '"><p>' + meaning + '</p></section>')
    (bundle / "qrcodes/打印二维码.html").write_text(page("联调测试二维码", "<h1>联调测试二维码</h1><p>内容只有三个 ASCII 数字，无逗号、空格或换行。打印时选择实际大小或100%，每张含白边的二维码为80mm正方形；PNG已生成后解码核对，打印后的扫描效果仍待实测。这是扫码联调纸，不用于站距或机械工作点标定。</p>" + "".join(cards)), encoding="utf-8")
    template = """# 视觉电控联调回包

请保留下面条目，不确定就填不知道；本轮禁止用统计计数代替原始日志。

- 操作者与测试日期：
- 包中 VERSION.json 的 source_commit 与 firmware_id：
- 摄像头实际板型、MaixPy 固件版本、部署方式及启动记录：
- 使用 App ID/version，model_9541 文件 SHA256 是否相同：
- 单片机是否烧录本包 HEX，手机 READY 实际固件号：
- OBJECT 实际 img_w/img_h（不可只抄 config 默认值）：
- UART 接线及电平、供电与共地检查：
- 扫码 111/222/333/123 各自返回值，ACK/请求号：
- 三种人质实物对应 ID1/ID2/ID0 是否正确，附实物与识别画面同框照片：
- 红绿蓝球、红绿蓝靶、黑桶的类别和坐标是否正确：
- 目标移出画面后是否不再重复旧坐标；连续遮挡、重新出现的情况：
- 连续完整 [PERF] 和启动/UART 日志文件名：
- 完整手机蓝牙原始日志文件名（不删开头，不只截单行）：
- 识别断续时的屏幕视频文件名；目标距离与照明、有没有晃动：
- 本轮是否全程无运动、不发激光，有没有异常动作：

把模板、上述原始文件和 VERSION.json 一起回传，不要只发总结。没有机械臂时不填写最终爪工作点；像素坐标不等于毫米。
"""
    (bundle / "evidence/回包填写模板.md").write_text(template, encoding="utf-8")
    (bundle / "evidence/回包填写模板.html").write_text(page("视觉电控联调回包", '<p><a href="回包填写模板.md" download>下载可填写的 Markdown 原稿</a></p>' + markdown.markdown(template)), encoding="utf-8")
    metadata = {
        "created_shanghai": datetime.now(SHANGHAI).isoformat(timespec="seconds"),
        "repository": "https://github.com/zjj20040618-sudo/gongke",
        "branch": git("branch", "--show-current"), "source_commit": source_commit,
        "vision_source_commit": git("log", "-1", "--format=%H", "--", str(VISION.relative_to(ROOT))),
        "firmware_id": firmware_id, "firmware_sha256": digest(args.firmware_hex),
        "model_sha256": digest(packager.APP / "model_9541.cvimodel"),
        "app_manifest_files": names, "qr_decode_verified": list(payloads),
        "verification_scope": "Archive byte checks and generated QR decoding only. Use keil_rebuild.txt and source tests for software evidence; no device install, flashing, UART, recognition, motion or whole-mission acceptance is claimed.",
        "hostage_alias_caution": "ID0 oblate means waist drum and ID2 truncated_cone means cone in this training convention; confirm against competition physical objects."
    }
    (bundle / "VERSION.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    members = sorted(p for p in bundle.rglob("*") if p.is_file())
    (bundle / "SHA256SUMS.txt").write_text("".join(digest(p) + "  " + p.relative_to(bundle).as_posix() + "\n" for p in members), encoding="utf-8")
    zip_path = bundle.with_suffix(".zip")
    packager.write_zip(zip_path, [(p, bundle.name + "/" + p.relative_to(bundle).as_posix()) for p in sorted(bundle.rglob("*")) if p.is_file()])
    zip_hash = digest(zip_path)
    zip_path.with_suffix(".zip.sha256").write_text(zip_hash + "  " + zip_path.name + "\n", encoding="utf-8")
    with zipfile.ZipFile(app_zip) as archive:
        if set(archive.namelist()) != set(names):
            raise ValueError("App package does not match its deployment manifest")
    print(json.dumps({"bundle": str(bundle), "zip": str(zip_path), "sha256": zip_hash, "source_commit": source_commit, "firmware_id": firmware_id}, ensure_ascii=False))


if __name__ == "__main__":
    main()
