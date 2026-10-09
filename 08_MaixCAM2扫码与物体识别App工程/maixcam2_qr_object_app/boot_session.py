"""Opt-in V1 boot transport matching MCU 6fe1538; no legacy fallback.

HELLO only prepares a challenge. CONFIRM commits identity; complete READY
write opens business reception. UART retains old partial tails for framing.
"""
import os
import struct
from protocol import _packet
from utils import crc16_ccitt


def nonce_valid(value):
    return isinstance(value, bytes) and len(value) == 8 and any(value)


def boot_packet(opcode, client, server=None, status=0):
    if opcode not in (0x64, 0x65, 0x67, 0x68) or not nonce_valid(client):
        raise ValueError("invalid boot command/client nonce")
    body = bytes((opcode, 1)) + client
    if opcode != 0x64:
        if not nonce_valid(server):
            raise ValueError("invalid server nonce")
        body += server
    if opcode == 0x68:
        if status not in (0, 1):
            raise ValueError("invalid READY status")
        body += bytes((status,))
    return _packet(body)


def wrap_packet(packet, client, server):
    if not nonce_valid(client) or not nonce_valid(server):
        raise ValueError("invalid session identity")
    body = packet[2:-2]
    if not 4 <= len(body) <= 257 or body[0] not in (0x60, 0x63, 0x61, 0x62):
        raise ValueError("invalid session business body")
    return _packet(b"\x66" + client + server + struct.pack("<H", len(body)) + body)


class BootSession:
    def __init__(self, random_bytes=None):
        self.buffer = bytearray()
        self.active = self.pending = None
        self.ready = False
        self.ready_packet = None
        self._random = random_bytes or os.urandom
        self._last_nonce = None
        self._rx_tick = None

    def feed(self, data, now_ms=None):
        """Bounded decoder; authorization happens sequentially in receive()."""
        if data and now_ms is not None:
            if self._rx_tick is not None and ((now_ms - self._rx_tick) & 0xFFFFFFFF) > 100:
                self.buffer.clear()
            self._rx_tick = now_ms
        frames = []
        for byte in data:
            self.buffer.append(byte)
            while self.buffer:
                if self.buffer[0] != 0xAA:
                    del self.buffer[0]
                    continue
                if len(self.buffer) < 2:
                    break
                if self.buffer[1] != 0x55:
                    del self.buffer[0]
                    continue
                if len(self.buffer) < 3:
                    break
                opcode = self.buffer[2]
                sizes = {0x64: 14, 0x65: 22, 0x67: 22, 0x68: 23,
                         0x60: 8, 0x63: 9, 0x61: 9}
                if opcode == 0x66:
                    if len(self.buffer) < 21:
                        break
                    length = struct.unpack_from("<H", self.buffer, 19)[0]
                    if not 4 <= length <= 257:
                        del self.buffer[0]
                        continue
                    size = 23 + length
                elif opcode == 0x62:
                    if len(self.buffer) < 7:
                        break
                    size = 9 + struct.unpack_from("<H", self.buffer, 5)[0]
                    if size > 280:
                        del self.buffer[0]
                        continue
                elif opcode in sizes:
                    size = sizes[opcode]
                else:
                    del self.buffer[0]
                    continue
                if len(self.buffer) < size:
                    break
                body = bytes(self.buffer[2:size - 2])
                if crc16_ccitt(body) != struct.unpack_from("<H", self.buffer, size - 2)[0]:
                    del self.buffer[0]
                    continue
                frames.append(body)
                del self.buffer[:size]
        return frames

    def _challenge(self):
        used = [self._last_nonce]
        if self.active:
            used.append(self.active[1])
        if self.pending:
            used.append(self.pending[1])
        for _ in range(4):
            value = self._random(8)
            if nonce_valid(value) and value not in used:
                self._last_nonce = value
                return value
        raise RuntimeError("secure random nonce unavailable; no legacy fallback")

    def receive(self, body):
        """Return (response, committed_new_session, command). Never clear on HELLO."""
        if not body:
            return None, False, None
        opcode = body[0]
        if opcode in (0x64, 0x67):
            size = 10 if opcode == 0x64 else 18
            if len(body) != size or body[1] != 1 or not nonce_valid(body[2:10]):
                return None, False, None
            client = body[2:10]
            if opcode == 0x64:
                if self.active and self.active[0] == client:
                    identity = self.active
                elif self.pending and self.pending[0] == client:
                    identity = self.pending
                else:
                    try:
                        identity = (client, self._challenge())
                    except Exception as exc:
                        print("[SESSION] challenge failed:", exc)
                        return None, False, None
                    self.pending = identity
                return boot_packet(0x65, *identity), False, None
            identity = (client, body[10:18])
            if identity == self.active:
                return self.ready_packet, False, None
            if identity != self.pending or not nonce_valid(identity[1]):
                return None, False, None
            self.active, self.pending = identity, None
            self.ready = False
            self.ready_packet = boot_packet(0x68, *identity)
            print("[SESSION] committed C={} S={}; waiting READY write".format(
                client.hex(), identity[1].hex()))
            return self.ready_packet, True, None
        if opcode != 0x66 or not self.ready or len(body) < 23:
            return None, False, None
        if (body[1:9], body[9:17]) != self.active:
            return None, False, None
        length = struct.unpack_from("<H", body, 17)[0]
        inner = body[19:]
        if not 4 <= length <= 257 or length != len(inner):
            return None, False, None
        if (inner[0] == 0x60 and length == 4) or (inner[0] == 0x63 and length == 5):
            request = struct.unpack_from("<H", inner, 1)[0]
            if request:
                command = ((request, inner[3]) if inner[0] == 0x60 else
                           (request, 2, inner[3], inner[4]))
                return None, False, command
        return None, False, None

    def transmitted(self, packet):
        if packet == self.ready_packet and self.active is not None:
            changed = not self.ready
            self.ready = True
            if changed:
                print("[SESSION] READY written; business enabled")

    def wrap(self, packet):
        if not self.ready or packet is None:
            return None
        return wrap_packet(packet, *self.active)
