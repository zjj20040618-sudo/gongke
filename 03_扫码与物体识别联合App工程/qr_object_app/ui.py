"""集中管理画框和状态文字。"""
from maix import image
import config
from utils import class_name


def _text_size(text, scale):
    return image.string_size(text, scale=scale, thickness=-1)


def _result_scale(img, display_size=None):
    width, height = display_size or (config.OBJECT_WIDTH, config.OBJECT_HEIGHT)
    # Display.show默认FIT_CONTAIN；小图居中不放大，大图等比例缩小。
    return config.BOX_TEXT_SCALE * max(1.0, img.width() / max(1, width), img.height() / max(1, height))


def _draw_lines(img, x, y, lines, color, scale, min_y=0, background=None, avoid_rect=None):
    """测量后夹紧位置；只对超宽行/超高块缩字，不裁掉中心坐标。"""
    margin = max(2, round(scale))
    available_width = max(1, img.width() - 2 * margin)
    rows = []
    for text in lines:
        text_width, _ = _text_size(text, scale)
        row_scale = scale * min(1.0, available_width / max(1, text_width))
        width, height = _text_size(text, row_scale)
        rows.append((text, row_scale, width, height))
    gap = max(2, round(scale / 2))
    top = max(margin, int(min_y))
    total_height = sum(row[3] for row in rows) + gap * max(0, len(rows) - 1)
    available_height = max(1, img.height() - margin - top)
    if total_height > available_height:
        factor = available_height / total_height
        rows = [(text, row_scale * factor, *_text_size(text, row_scale * factor))
                for text, row_scale, _, _ in rows]
        gap *= factor
        total_height = sum(row[3] for row in rows) + gap * max(0, len(rows) - 1)
    if avoid_rect is not None:
        _, rect_y, _, rect_h = avoid_rect
        above_y = rect_y - total_height - gap
        below_y = rect_y + rect_h + gap
        if above_y >= top:
            y = above_y
        elif below_y + total_height <= img.height() - margin:
            y = below_y
    y = max(top, min(y, img.height() - margin - total_height))
    for text, row_scale, width, height in rows:
        left = max(margin, min(x, img.width() - margin - width))
        if background is not None:
            pad = max(2, round(row_scale / 3))
            panel_x = max(0, round(left) - pad)
            panel_y = max(0, round(y) - pad)
            panel_right = min(img.width(), round(left + width) + pad)
            panel_bottom = min(img.height(), round(y + height) + pad)
            img.draw_rect(panel_x, panel_y, panel_right - panel_x, panel_bottom - panel_y,
                          background, thickness=-1)
        img.draw_string(round(left), round(y), text, color, scale=row_scale, thickness=-1, wrap=False)
        y += height + gap
    return round(y)


def draw_header(img, mode, fps, work_ms, uart_ms, remote_owned=False):
    title = "QR SCAN" if mode == config.MODE_QR else "OBJ10 YOLO26"
    bottom = _draw_lines(img, 8, 8, ["{} {}x{} FPS:{:.1f}".format(title, img.width(), img.height(), fps)], image.COLOR_GREEN, config.STATUS_TEXT_SCALE)
    owner = "UART:CTRL" if remote_owned else "USER:SWITCH"
    return _draw_lines(img, 8, bottom, ["WORK:{}ms UART:{}ms {}".format(work_ms, uart_ms, owner)], image.COLOR_YELLOW, config.STATUS_TEXT_SCALE)

def draw_objects(img, objects, display_size=None, min_y=0):
    scale = _result_scale(img, display_size)
    for obj in objects:
        color = image.COLOR_RED if obj.class_id >= 6 else image.COLOR_BLUE
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color, thickness=3)
        cx, cy = obj.x + obj.w // 2, obj.y + obj.h // 2
        img.draw_cross(cx, cy, color, size=9, thickness=2)
        coordinates = "x={}, y={}".format(cx, cy)
        lines = ["{} {:.2f}".format(class_name(obj.class_id), obj.score)]
        if _text_size(coordinates, scale)[0] <= img.width() - 2 * max(2, round(scale)):
            lines.append(coordinates)
        else:
            lines.extend(["x={}".format(cx), "y={}".format(cy)])
        _draw_lines(img, obj.x, obj.y, lines, image.COLOR_WHITE, scale, min_y,
                    background=image.Color.from_rgb(0, 0, 0),
                    avoid_rect=(obj.x, obj.y, obj.w, obj.h))

def draw_qrs(img, qrs, display_size=None, min_y=0):
    scale = _result_scale(img, display_size)
    for qr in qrs:
        try:
            img.draw_edges(qr["corners"], image.COLOR_GREEN, thickness=4)
        except Exception:
            img.draw_rect(qr["x"], qr["y"], qr["w"], qr["h"], image.COLOR_GREEN, thickness=4)
        parts = qr["text_ascii"].split()
        lines = ["QR:" + qr["payload"]]
        lines += [" ".join(parts[:2]), parts[2]] if len(parts) == 3 else [qr["text_ascii"]]
        _draw_lines(img, qr["x"], qr["y"], lines, image.COLOR_YELLOW, scale, min_y)
