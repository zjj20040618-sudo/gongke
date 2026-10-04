# 电脑端打包工具，不在摄像头主循环中运行；执行它会重建dist中的安装包和源码包。
# Path是路径对象，/在Path之间表示拼子路径，不是数值除法。
# 生成App安装包和源码包后逐文件校验；SHA256记录包内容，注释改动也会改变新包哈希。

"""生成 MaixCAM App 包与源码包；仅使用 Python 标准库。"""
import configparser
import hashlib
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parent
APP = ROOT / "qr_object_app"


# 功能：读取受控app.yaml的文件清单并检查重复/缺失。
# 返回：文件名字符串列表。
# 理解：这里手动拆模板文本，不是通用YAML解析器；文件清单不等于整个目录自动打包。
def manifest():
    text = (APP / "app.yaml").read_text(encoding="utf-8")
    # app.yaml 是本工程受控模板；这里只读取 files 清单，不充当通用 YAML 解析器。
    items = text.split("\nfiles:\n", 1)[1].splitlines()
    names = [line.strip()[2:] for line in items if line.strip().startswith("- ")]
    # set会去重；原列表长度与去重后长度不同，就存在重复文件名。
    if len(names) != len(set(names)):
        raise ValueError("duplicate app files")
    for name in names:
        if Path(name).name != name or not (APP / name).is_file():
            raise ValueError("invalid or missing app file: " + name)
    return names


# 功能：写ZIP并逐文件核对解压内容与原文件一致。
# 参数：path：输出ZIP路径；members：(来源Path, 包内名称)元组列表。
# 返回：ZIP文件SHA256十六进制字符串。
# 理解：with退出时自动关闭文件，类似C中保证最终调用fclose的资源管理。
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


# 功能：检查Python语法和模型描述，生成两类包与校验值文件。
# 返回：None（没有显式return时默认返回None）。
# 理解：compile(...,"exec")只编译检查源码，不执行摄像头初始化或识别主循环。
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
    # 列表推导式收集“源路径、包内路径”；两个ZIP的目录布局不同，不能把源码包当安装包。
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
