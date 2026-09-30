"""UART纯组包模块：0x01为目标帧，0x51为二维码帧。"""
import struct
from utils import clamp_u16, crc16_ccitt

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

