"""无硬件依赖的基础函数，可在电脑上测试。"""

def clamp_u16(value):
    return max(0, min(65535, int(value)))

def crc16_ccitt(data):
    """CRC-16/CCITT-FALSE：初值0xFFFF，多项式0x1021。"""
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def class_name(class_id):
    import config
    return config.CLASS_NAMES[class_id] if 0 <= class_id < len(config.CLASS_NAMES) else "class_{}".format(class_id)

