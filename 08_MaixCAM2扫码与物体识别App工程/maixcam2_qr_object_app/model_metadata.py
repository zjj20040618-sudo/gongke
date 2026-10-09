"""MUD模型检查；仅Python标准库，电脑打包与MC2运行共用。

MUD不是权重本体，类似C工程的配置索引。两个axmodel才包含转换后的模型。
只检查格式、引用和标签；真实输入尺寸必须由nn.YOLO26读取。
"""
import configparser
import os


def validate_model(model_path, expected_labels):
    metadata = configparser.ConfigParser()
    with open(model_path, encoding="utf-8") as stream:
        metadata.read_file(stream)
    if metadata.get("basic", "type") != "axmodel":
        raise ValueError("MC2需要axmodel格式MUD，不能使用旧MC的cvimodel")
    if (metadata.get("extra", "model_type") != "yolo26"
            or metadata.get("extra", "type") != "detector"
            or metadata.get("extra", "input_type") != "rgb"):
        raise ValueError("本测试工程只接受YOLO26 RGB物体检测模型")
    labels = tuple(name.strip() for name in metadata.get("extra", "labels").split(","))
    if labels != tuple(expected_labels):
        raise ValueError("MUD标签与config.py顺序不一致: " + repr(labels))
    binaries = []
    for key in ("model_npu", "model_vnpu"):
        name = metadata.get("basic", key)
        # 禁止意外引用外部路径，使整个App文件夹能独立上传和删除。
        if (not name.endswith(".axmodel") or os.path.basename(name) != name
                or "/" in name or "\\" in name):
            raise ValueError("模型引用必须是本目录axmodel文件: " + key)
        full_path = os.path.join(os.path.dirname(model_path), name)
        if not os.path.isfile(full_path) or os.path.getsize(full_path) == 0:
            raise ValueError("模型文件缺失或为空: " + full_path)
        binaries.append(name)
    if len(set(binaries)) != 2:
        raise ValueError("NPU和VNPU模型引用不能相同")
    return binaries
