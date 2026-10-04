# 硬件接口：把引脚设为UART功能，创建串口对象，再负责写bytes包。
# 视觉初始化失败时仍可继续画面处理，但serial=None意味着串口没有建立，不能称联调成功。

"""MaixCAM Pro UART1初始化。失败时视觉程序仍继续。"""
from maix import err, pinmap, uart
import config

# 功能：按config配置映射引脚并打开UART1。
# 返回：UART对象，或未启用/初始化失败时的None。
def init_uart():
    if not config.UART_ENABLED:
        return None
    try:
        err.check_raise(pinmap.set_pin_function(config.UART_TX_PIN, "UART1_TX"), "set UART1_TX failed")
        err.check_raise(pinmap.set_pin_function(config.UART_RX_PIN, "UART1_RX"), "set UART1_RX failed")
        serial = uart.UART(config.UART_DEVICE, config.UART_BAUDRATE)
        print("[UART] {} TX={} RX={} baud={}".format(config.UART_DEVICE, config.UART_TX_PIN, config.UART_RX_PIN, config.UART_BAUDRATE))
        return serial
    except Exception as exc:
        print("[UART] init failed; vision continues:", exc)
        return None

# 功能：向串口写一个完整包，并检查写入字节数。
# 参数：serial：UART对象；packet：bytes包；任何一项为None都不写。
# 返回：True=write返回的长度等于包长；False=未写完整或发生异常。
# 理解：这只证明本端写调用返回，不证明电控已收到、CRC通过或任务已经完成。
def send_packet(serial, packet):
    if serial is None or packet is None:
        return False
    try:
        return serial.write(packet) == len(packet)
    except Exception as exc:
        print("[UART] send failed:", exc)
        return False
