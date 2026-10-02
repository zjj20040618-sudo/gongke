"""MaixCAM Pro UART1初始化。失败时视觉程序仍继续。"""
from maix import err, pinmap, uart
import config

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

def send_packet(serial, packet):
    if serial is None or packet is None:
        return False
    try:
        return serial.write(packet) == len(packet)
    except Exception as exc:
        print("[UART] send failed:", exc)
        return False
