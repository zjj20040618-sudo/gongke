"""UART纯组包模块：0x01为目标帧，0x51为二维码帧。"""
import struct
from utils import clamp_u16, crc16_ccitt

def _packet(body):
    return b"\xaa\x55" + bytes(body) + struct.pack("<H", crc16_ccitt(body))

def build_control_packet(request_id, mode):
    return _packet(struct.pack("<BHB", 0x60, request_id, mode))

def build_ack_packet(request_id, mode, status=0):
    return _packet(struct.pack("<BHBB", 0x61, request_id, mode, status))

def bind_result(packet, request_id):
    """Keep legacy body unchanged; outer CRC protects request and body together."""
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

def build_object_packet(sequence, objects, img_w, img_h, capture_ms, inference_ms, vision_ms):
    payload = bytearray()
    payload.extend(struct.pack("<BHBHHHHH", 1, sequence & 0xFFFF, len(objects), clamp_u16(img_w), clamp_u16(img_h), clamp_u16(capture_ms), clamp_u16(inference_ms), clamp_u16(vision_ms)))
    for obj in objects:
        payload.extend(struct.pack("<BHHHHH", int(obj.class_id) & 0xFF, clamp_u16(round(obj.score * 1000.0)), clamp_u16(obj.x + obj.w // 2), clamp_u16(obj.y + obj.h // 2), clamp_u16(obj.w), clamp_u16(obj.h)))
    packet = bytearray((0xAA, 0x55)); packet.extend(payload); packet.extend(struct.pack("<H", crc16_ccitt(payload)))
    return bytes(packet)

def build_qr_packet(sequence, qrs):
    body = bytearray((0x51,)); body.extend(struct.pack("<H", sequence & 0xFFFF)); body.append(len(qrs) & 0xFF)
    for qr in qrs:
        raw = qr["payload"].encode("utf-8")[:255]
        body.append(len(raw)); body.extend(raw)
        body.extend(struct.pack("<HHHH", clamp_u16(qr["x"] + qr["w"] // 2), clamp_u16(qr["y"] + qr["h"] // 2), clamp_u16(qr["w"]), clamp_u16(qr["h"])))
    packet = bytearray((0xAA, 0x55)); packet.extend(body); packet.extend(struct.pack("<H", crc16_ccitt(body)))
    return bytes(packet)

