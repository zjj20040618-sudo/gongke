"""集中管理画框和状态文字。"""
from maix import image
import config
from utils import class_name

def draw_header(img, mode, fps, work_ms, uart_ms, remote_owned=False):
    title = "QR SCAN" if mode == config.MODE_QR else "OBJ10 YOLO26"
    img.draw_string(8, 8, "{} {}x{} FPS:{:.1f}".format(title, img.width(), img.height(), fps), image.COLOR_GREEN, scale=config.STATUS_TEXT_SCALE)
    owner = "UART:CTRL" if remote_owned else "USER:SWITCH"
    img.draw_string(8, 40, "WORK:{}ms UART:{}ms {}".format(work_ms, uart_ms, owner), image.COLOR_YELLOW, scale=config.STATUS_TEXT_SCALE)

def draw_objects(img, objects):
    for obj in objects:
        color = image.COLOR_RED if obj.class_id >= 6 else image.COLOR_BLUE
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color, thickness=3)
        img.draw_cross(obj.x + obj.w // 2, obj.y + obj.h // 2, color, size=9, thickness=2)
        img.draw_string(obj.x, max(76, obj.y - 26), "{} {:.2f}".format(class_name(obj.class_id), obj.score), color, scale=config.BOX_TEXT_SCALE)

def draw_qrs(img, qrs):
    for qr in qrs:
        try:
            img.draw_edges(qr["corners"], image.COLOR_GREEN, thickness=4)
        except Exception:
            img.draw_rect(qr["x"], qr["y"], qr["w"], qr["h"], image.COLOR_GREEN, thickness=4)
        img.draw_string(qr["x"], max(76, qr["y"] - 32), "QR:{} {}".format(qr["payload"], qr["text_ascii"]), image.COLOR_YELLOW, scale=config.BOX_TEXT_SCALE)
