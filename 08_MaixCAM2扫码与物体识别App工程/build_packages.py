"""生成MC2安装包与源码包；校验MUD、双axmodel及包内文件，输出到本工程dist。"""
import hashlib
from pathlib import Path
import re
import runpy
import zipfile
import sys

ROOT = Path(__file__).resolve().parent
APP = ROOT / "maixcam2_qr_object_app"
sys.path.insert(0, str(APP))
from model_metadata import validate_model as check_model


def app_metadata():
    # This controlled template uses plain top-level id/version values.
    values = {}
    for line in (APP / "app.yaml").read_text(encoding="utf-8").splitlines():
        if not line or line[0].isspace():
            continue
        key, separator, value = line.partition(":")
        if separator and key in ("id", "version"):
            value = value.strip().strip("\"'")
            if key in values or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", value):
                raise ValueError("invalid or duplicate app " + key)
            values[key] = value
    if set(values) != {"id", "version"}:
        raise ValueError("app id and version are required")
    return values


def manifest():
    text = (APP / "app.yaml").read_text(encoding="utf-8")
    # app.yaml 是本工程受控模板；这里只读取 files 清单，不充当通用 YAML 解析器。
    items = text.split("\nfiles:\n", 1)[1].splitlines()
    names = [line.strip()[2:] for line in items if line.strip().startswith("- ")]
    if len(names) != len(set(names)):
        raise ValueError("duplicate app files")
    for name in names:
        if Path(name).name != name or not (APP / name).is_file():
            raise ValueError("invalid or missing app file: " + name)
    return names


def validate_model(names):
    if "app.yaml" not in names or "config.py" not in names:
        raise ValueError("app manifest must include app.yaml and config.py")
    settings = runpy.run_path(str(APP / "config.py"))
    model_file = settings.get("MODEL_FILE")
    if not isinstance(model_file, str) or model_file not in names or not model_file.endswith(".mud"):
        raise ValueError("config MODEL_FILE must name a MUD in the app manifest")
    binaries = check_model(str(APP / model_file), settings["CLASS_NAMES_CN"])
    if any(name not in names for name in binaries):
        raise ValueError("NPU and VNPU binaries must both be in manifest")
    return model_file, tuple(binaries)


def write_zip(path, members):
    targets = [target for source, target in members]
    if len(targets) != len(set(targets)):
        raise ValueError("duplicate zip members")
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for source, target in members:
            archive.write(source, target)
    # 再读取包内每个文件并与源文件比对，防漏打包/损坏。
    with zipfile.ZipFile(path) as archive:
        if archive.testzip() is not None:
            raise ValueError("corrupt zip: " + str(path))
        if archive.namelist() != targets:
            raise ValueError("zip member mismatch: " + str(path))
        for source, target in members:
            if archive.read(target) != source.read_bytes():
                raise ValueError("zip mismatch: " + target)
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    metadata = app_metadata()
    names = manifest()
    for name in names:
        if name.endswith(".py"):
            compile((APP / name).read_text(encoding="utf-8"), name, "exec")
    model_file, binary_file = validate_model(names)
    output = ROOT / "dist"
    output.mkdir(exist_ok=True)
    app_zip = output / "maix-{}-v{}.zip".format(metadata["id"], metadata["version"])
    source_zip = output / "{}_source_v{}.zip".format(metadata["id"], metadata["version"])
    app_members = [(APP / name, name) for name in names]
    source_members = [(ROOT / name, name) for name in (".gitignore", ".gitattributes", "README.md", "build_packages.py")]
    source_names = names + (["README.md"] if "README.md" not in names else [])
    source_members += [(APP / name, "maixcam2_qr_object_app/" + name) for name in source_names]
    source_members += [(path, "maixcam2_qr_object_app/tests/" + path.name) for path in sorted((APP / "tests").glob("*.py"))]
    hashes = [(app_zip.name, write_zip(app_zip, app_members)), (source_zip.name, write_zip(source_zip, source_members))]
    (output / "SHA256SUMS.txt").write_text("".join(digest + "  " + name + "\n" for name, digest in hashes), encoding="utf-8")
    for name, digest in hashes:
        print(name, digest)
    print("App files:", len(names), "Source files:", len(source_members))
    print("App:", metadata["id"], "Version:", metadata["version"], "Model:", model_file, binary_file)


if __name__ == "__main__":
    main()
