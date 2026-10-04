# 串口组包与收命令：业务数据要先编码成bytes，再由hardware.send_packet发送。
# bytes是不可变字节串，bytearray可append/extend修改；与普通str文字字符串不同。
# struct.pack把数值按固定字节宽度编码；unpack反向读取，不是在拼可读的十进制文字。
# 格式<表示小端：B是1字节无符号数，H是2字节无符号数；例如< B H B依次占1+2+1字节。
# 帧头AA55用于找开始位置，CRC用于检查内容；请求编号与结果序号不是同一个概念。

"""UART纯组包模块：0x01为目标帧，0x51为二维码帧。"""
import struct
from utils import clamp_u16, crc16_ccitt

# 功能：给消息体加AA55帧头和小端CRC16。
# 参数：body：消息体的字节数据，不含帧头/CRC。
# 返回：完整bytes包。
# 理解：下划线开头约定为模块内部辅助函数；不是Python强制访问权限。
def _packet(body):
    return b"\xaa\x55" + bytes(body) + struct.pack("<H", crc16_ccitt(body))

# 功能：构建固定8字节的电控请求包，用于协议测试。
# 参数：request_id：16位请求号；mode：0/1/2。
# 返回：完整0x60请求bytes包。
def build_control_packet(request_id, mode):
    return _packet(struct.pack("<BHB", 0x60, request_id, mode))

# 功能：构建视觉对电控的确认包。
# 参数：request_id：对应请求号；mode：实际模式；status：0成功/1失败，默认0。
# 返回：完整0x61 ACK字节包。
def build_ack_packet(request_id, mode, status=0):
    return _packet(struct.pack("<BHBB", 0x61, request_id, mode, status))

# 功能：保留原业务消息体，加本轮请求编号和外层CRC。
# 参数：packet：已组好的裸业务帧；request_id：本轮电控请求号。
# 返回：完整0x62结果外壳包。
# 理解：不是给原帧再包一层完整AA55，而是先取裸帧中间的body。
def bind_result(packet, request_id):
    """Keep legacy body unchanged; outer CRC protects request and body together."""
    # 切片[2:-2]从第3字节取到倒数第2字节之前，去掉2字节帧头与2字节旧CRC。
    body = packet[2:-2]
    return _packet(struct.pack("<BHH", 0x62, request_id, len(body)) + body)

class CommandReceiver:
    """Bounded byte-wise parser; commands are always eight bytes."""
    def __init__(self):
        self.buffer = bytearray()

    # 功能：把分批到来的字节拼成有效电控命令。
    # 参数：data：本次收到的bytes，可为空。
    # 返回：(请求号,模式号)元组组成的列表，本次可能得到0条或多条命令。
    # 理解：buffer在对象内持续保存；收到不足8字节时break等待下次，不清半包。
    def feed(self, data):
        commands = []
        for byte in data:
            self.buffer.append(byte)
            while self.buffer:
                # 不匹配帧头就只丢一个字节再找；del修改缓冲，continue开始下一次while检查。
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
                # unpack即使只有一个字段也返回元组，所以末尾[0]取CRC整数再比较。
                if crc16_ccitt(body) != struct.unpack("<H", self.buffer[6:8])[0]:
                    del self.buffer[0]; continue
                request_id, mode = struct.unpack("<HB", body[1:])
                if request_id:
                    commands.append((request_id, mode))
                del self.buffer[:8]
        return commands

# 功能：编码本帧选中目标及画幅/耗时信息。
# 参数：sequence：帧序号；objects：已筛选列表；img_w/img_h：真实画幅；后三项单位ms。
# 返回：0x01裸目标帧bytes；正式会话随后用bind_result绑定请求。
def build_object_packet(sequence, objects, img_w, img_h, capture_ms, inference_ms, vision_ms):
    payload = bytearray()
    payload.extend(struct.pack("<BHBHHHHH", 1, sequence & 0xFFFF, len(objects), clamp_u16(img_w), clamp_u16(img_h), clamp_u16(capture_ms), clamp_u16(inference_ms), clamp_u16(vision_ms)))
    for obj in objects:
        # 每目标11字节：类别1字节，其余分数、cx、cy、宽、高各2字节。
        # 分数乘1000再round取整；中心x+w//2、y+h//2仍是像素，不是毫米或电机指令。
        payload.extend(struct.pack("<BHHHHH", int(obj.class_id) & 0xFF, clamp_u16(round(obj.score * 1000.0)), clamp_u16(obj.x + obj.w // 2), clamp_u16(obj.y + obj.h // 2), clamp_u16(obj.w), clamp_u16(obj.h)))
    packet = bytearray((0xAA, 0x55)); packet.extend(payload); packet.extend(struct.pack("<H", crc16_ccitt(payload)))
    return bytes(packet)

# 功能：编码本帧二维码内容与框坐标。
# 参数：sequence：16位帧序号；qrs：二维码字典列表，可为空。
# 返回：0x51裸QR帧bytes；空列表生成空心跳，不伪造二维码。
def build_qr_packet(sequence, qrs):
    body = bytearray((0x51,)); body.extend(struct.pack("<H", sequence & 0xFFFF)); body.append(len(qrs) & 0xFF)
    for qr in qrs:
        # encode把文字转成UTF-8字节；[:255]按字节截断，长度字段也是字节数，不是字符数。
        raw = qr["payload"].encode("utf-8")[:255]
        body.append(len(raw)); body.extend(raw)
        body.extend(struct.pack("<HHHH", clamp_u16(qr["x"] + qr["w"] // 2), clamp_u16(qr["y"] + qr["h"] // 2), clamp_u16(qr["w"]), clamp_u16(qr["h"])))
    packet = bytearray((0xAA, 0x55)); packet.extend(body); packet.extend(struct.pack("<H", crc16_ccitt(body)))
    return bytes(packet)

