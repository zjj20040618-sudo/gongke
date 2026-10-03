"""生成 MaixCAM App 包与源码包；仅使用 Python 标准库。"""
import configparser
import hashlib
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parent
APP = ROOT / "qr_object_app"


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


def write_zip(path, members):
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for source, target in members:
            archive.write(source, target)
    # 再读取包内每个文件并与源文件比对，防漏打包/损坏。
    with zipfile.ZipFile(path) as archive:
        if archive.testzip() is not None:
            raise ValueError("corrupt zip: " + str(path))
        for source, target in members:
            if archive.read(target) != source.read_bytes():
                raise ValueError("zip mismatch: " + target)
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    names = manifest()
    for name in names:
        if name.endswith(".py"):
            compile((APP / name).read_text(encoding="utf-8"), name, "exec")
    mud = configparser.ConfigParser()
    mud.read(APP / "model_9541.mud", encoding="utf-8")
    if mud["basic"]["model"] != "model_9541.cvimodel":
        raise ValueError("unexpected model file")
    output = ROOT / "dist"
    output.mkdir(exist_ok=True)
    app_zip = output / "maix-qr_object_switch-v1.1.0.zip"
    source_zip = output / "qr_object_switch_source_v1.1.0.zip"
    app_members = [(APP / name, name) for name in names]
    source_members = [(ROOT / name, name) for name in (".gitignore", "build_packages.py")]
    source_members += [(APP / name, "qr_object_app/" + name) for name in names + ["README.md"]]
    source_members += [(path, "qr_object_app/tests/" + path.name) for path in sorted((APP / "tests").glob("test_*.py"))]
    hashes = [(app_zip.name, write_zip(app_zip, app_members)), (source_zip.name, write_zip(source_zip, source_members))]
    (output / "SHA256SUMS.txt").write_text("".join(digest + "  " + name + "\n" for name, digest in hashes), encoding="utf-8")
    for name, digest in hashes:
        print(name, digest)
    print("App files:", len(names), "Source files:", len(source_members))


if __name__ == "__main__":
    main()
