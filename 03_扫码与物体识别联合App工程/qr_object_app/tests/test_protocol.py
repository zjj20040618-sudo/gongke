import os, struct, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from protocol import build_object_packet, build_qr_packet
from utils import crc16_ccitt

class Obj:
    class_id, score, x, y, w, h = 8, 0.875, 10, 20, 30, 40

def check(packet, expected_type):
    assert packet[:2] == b"\xAA\x55" and packet[2] == expected_type
    assert struct.unpack("<H", packet[-2:])[0] == crc16_ccitt(packet[2:-2])

object_packet = build_object_packet(7, [Obj()], 480, 320, 8, 14, 25)
check(object_packet, 0x01)
assert struct.unpack("<BHBHHHHH", object_packet[2:16]) == (1, 7, 1, 480, 320, 8, 14, 25)
assert struct.unpack("<BHHHHH", object_packet[16:27]) == (8, 875, 25, 40, 30, 40)
qr_packet = build_qr_packet(8, "123")
check(qr_packet, 0x53)
assert qr_packet[2:-2] == b"\x53\x08\x00\x01" + b"123"
assert len(qr_packet) == 11
empty_qr = build_qr_packet(9, None)
check(empty_qr, 0x53)
assert len(empty_qr) == 8 and empty_qr[5] == 0
print("PASS: OBJECT01 full geometry and QR53 three digits, headers and CRC")
