"""UART组包：0x01坐标、0x53任务码、0x54人质顺序、0x60/0x63请求。"""
import struct
from utils import clamp_u16, crc16_ccitt

def _packet(body):
    return b"\xaa\x55" + bytes(body) + struct.pack("<H", crc16_ccitt(body))

def build_control_packet(request_id, mode):
    return _packet(struct.pack("<BHB", 0x60, request_id, mode))

def build_task_packet(request_id, task_id, qr_digit):
    return _packet(struct.pack("<BHBB", 0x63, request_id, task_id, qr_digit))

def build_ack_packet(request_id, mode, status=0):
    return _packet(struct.pack("<BHBB", 0x61, request_id, mode, status))

def bind_result(packet, request_id):
    """外层保护请求号及内层body，不嵌套内层帧头或CRC。"""
    body = packet[2:-2]
    return _packet(struct.pack("<BHH", 0x62, request_id, len(body)) + body)

class CommandReceiver:
    """有界分片接收，0x60为8字节，0x63为9字节。"""
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        commands = []
        for byte in data:
            self.buffer.append(byte)
            while self.buffer:
                if self.buffer[0] != 0xAA:
                    del self.buffer[0]; continue
                if len(self.buffer) < 2:
                    break
                if self.buffer[1] != 0x55:
                    del self.buffer[0]; continue
                if len(self.buffer) < 3:
                    break
                opcode = self.buffer[2]
                if opcode not in (0x60, 0x63):
                    del self.buffer[0]; continue
                size = 8 if opcode == 0x60 else 9
                if len(self.buffer) < size:
                    break
                body = self.buffer[2:size - 2]
                if crc16_ccitt(body) != struct.unpack("<H", self.buffer[size - 2:size])[0]:
                    del self.buffer[0]; continue
                request_id = struct.unpack("<H", body[1:3])[0]
                if request_id:
                    commands.append((request_id, body[3]) if opcode == 0x60 else (request_id, 2, body[3], body[4]))
                del self.buffer[:size]
        return commands

def build_object_packet(sequence, objects, img_w, img_h, capture_ms=0, inference_ms=0, vision_ms=0):
    """保留电控现有0x01目标格式，坐标必须来自当前图像。"""
    payload = bytearray(struct.pack(
        "<BHBHHHHH", 0x01, sequence & 0xFFFF, len(objects),
        clamp_u16(img_w), clamp_u16(img_h),
        clamp_u16(capture_ms), clamp_u16(inference_ms), clamp_u16(vision_ms),
    ))
    for obj in objects:
        payload.extend(struct.pack("<BHHHHH", int(obj.class_id) & 0xFF,
            clamp_u16(round(obj.score * 1000.0)), clamp_u16(obj.x + obj.w // 2),
            clamp_u16(obj.y + obj.h // 2), clamp_u16(obj.w), clamp_u16(obj.h)))
    return _packet(payload)

def build_qr_packet(sequence, payload):
    """0x53仅含三位ASCII码，空码count=0，不带二维码几何。"""
    if payload is not None and (not isinstance(payload, str) or len(payload) != 3 or any(ch not in "123" for ch in payload)):
        raise ValueError("QR task must be exactly three ASCII digits 1..3")
    body = struct.pack("<BHB", 0x53, sequence & 0xFFFF, 0 if payload is None else 1)
    if payload is not None:
        body += payload.encode("ascii")
    return _packet(body)


def build_hostage_order_packet(sequence, target_class_id, order):
    """0x54 为本轮历史顺序，不是当前可见目标或物体坐标。

    0 表示目标尚未出现；空站位填 FF。编号一经分配，本轮不改变。
    与同一检测帧的 01 使用相同 seq，由 62 绑定人质请求号。
    """
    if (type(target_class_id) is not int or target_class_id not in (0, 1, 2)
            or len(order) > 3 or any(type(cid) is not int or cid not in (0, 1, 2) for cid in order)
            or len(set(order)) != len(order)):
        raise ValueError("invalid hostage order")
    rank = order.index(target_class_id) + 1 if target_class_id in order else 0
    slots = list(order) + [0xFF] * (3 - len(order))
    return _packet(struct.pack("<BHBBB3B", 0x54, sequence & 0xFFFF,
                              target_class_id, rank, len(order), *slots))

