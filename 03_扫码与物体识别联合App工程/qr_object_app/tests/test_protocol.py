import os, struct, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from protocol import build_object_packet, build_qr_packet
from utils import crc16_ccitt

class Obj:
    class_id, score, x, y, w, h = 8, 0.875, 10, 20, 30, 40

def check(packet, expected_type):
    assert packet[:2] == b"\xAA\x55" and packet[2] == expected_type
    assert struct.unpack("<H", packet[-2:])[0] == crc16_ccitt(packet[2:-2])

check(build_object_packet(7, [Obj()], 480, 320), 0x02)
check(build_qr_packet(8, True), 0x52)
print("PASS: object/QR headers and CRC")
