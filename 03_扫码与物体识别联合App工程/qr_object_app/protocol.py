"""UART纯组包模块：v2使用0x02目标帧和0x52扫码状态帧。"""
import struct
from utils import clamp_u16, crc16_ccitt

def _packet(body):
    return b"\xaa\x55" + bytes(body) + struct.pack("<H", crc16_ccitt(body))

def build_control_packet(request_id, mode):
    return _packet(struct.pack("<BHB", 0x60, request_id, mode))

def build_ack_packet(request_id, mode, status=0):
    return _packet(struct.pack("<BHBB", 0x61, request_id, mode, status))

def bind_result(packet, request_id):
    """Wrap a v2 inner body; outer CRC protects request and body together."""
    body = packet[2:-2]
    return _packet(struct.pack("<BHH", 0x62, request_id, len(body)) + body)

class CommandReceiver:
    """Bounded byte-wise parser; commands are always eight bytes."""
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
                if self.buffer[2] != 0x60:
                    del self.buffer[0]; continue
                if len(self.buffer) < 8:
                    break
                body = self.buffer[2:6]
                if crc16_ccitt(body) != struct.unpack("<H", self.buffer[6:8])[0]:
                    del self.buffer[0]; continue
                request_id, mode = struct.unpack("<HB", body[1:])
                if request_id:
                    commands.append((request_id, mode))
                del self.buffer[:8]
        return commands

def build_object_packet(sequence, objects, img_w, img_h):
    """Encode selected class IDs and centers; targets carry X only."""
    payload = bytearray(struct.pack(
        "<BHBHH", 0x02, sequence & 0xFFFF, len(objects),
        clamp_u16(img_w), clamp_u16(img_h),
    ))
    for obj in objects:
        class_id = int(obj.class_id) & 0xFF
        center_x = clamp_u16(obj.x + obj.w // 2)
        if class_id in (6, 7, 8):
            payload.extend(struct.pack("<BH", class_id, center_x))
        else:
            center_y = clamp_u16(obj.y + obj.h // 2)
            payload.extend(struct.pack("<BHH", class_id, center_x, center_y))
    return _packet(payload)

def build_qr_packet(sequence, valid_task_scanned):
    """Send scan status only; never include QR payload or geometry."""
    body = struct.pack(
        "<BHB", 0x52, sequence & 0xFFFF,
        1 if valid_task_scanned else 0,
    )
    return _packet(body)

