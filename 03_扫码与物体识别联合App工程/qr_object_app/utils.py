# 基础函数：不依赖摄像头或串口，电脑测试也能运行。
# 位运算沿用你学过的C含义：<<左移、&按位与、^按位异或；Python的^也不是乘方。

"""无硬件依赖的基础函数，可在电脑上测试。"""

# 功能：把数值转成整数并限制到16位无符号范围。
# 参数：value：要写入协议的数值。
# 返回：0..65535整数。
# 理解：int截去小数，min限制上界，max限制下界；不是自动证明输入测量正确。
def clamp_u16(value):
    return max(0, min(65535, int(value)))

# 功能：按CRC-16/CCITT-FALSE计算校验值。
# 参数：data：参与校验的字节序列。
# 返回：0..65535的CRC整数。
# 理解：初值FFFF、多项式1021；每字节处理8次bit，&FFFF保留低16位。
def crc16_ccitt(data):
    """CRC-16/CCITT-FALSE：初值0xFFFF，多项式0x1021。"""
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        # range(8)循环8次；_表示这里不用循环编号，不是特别的运算符。
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

# 功能：根据模型类别ID取英文名，越界时给出兜底文字。
# 参数：class_id：类别整数。
# 返回：配置中的类别名或class_编号字符串。
def class_name(class_id):
    import config
    return config.CLASS_NAMES[class_id] if 0 <= class_id < len(config.CLASS_NAMES) else "class_{}".format(class_id)

