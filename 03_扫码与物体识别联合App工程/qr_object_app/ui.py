"""集中管理画框和状态文字。"""
from maix import image
import config
from utils import class_name


def _text_size(text, scale):
    return image.string_size(text, scale=scale, thickness=-1)


def _result_scale(img, display_size=None, text_scale=None):
    width, height = display_size or (config.OBJECT_WIDTH, config.OBJECT_HEIGHT)
    # Display.show默认FIT_CONTAIN；小图居中不放大，大图等比例缩小。
    base = config.BOX_TEXT_SCALE if text_scale is None else text_scale
    return base * max(1.0, img.width() / max(1, width), img.height() / max(1, height))


def _object_color(class_id):
    """十类固定配色；仅显示样式，不修改模型ID、排序或协议。"""
    colors = ((255, 180, 60), (60, 215, 240), (190, 140, 255),
              (90, 165, 255), (255, 105, 105), (95, 230, 120),
              (255, 120, 195), (135, 160, 255), (175, 235, 85),
              (195, 195, 195))
    rgb = colors[class_id] if 0 <= class_id < len(colors) else (220, 220, 220)
    return image.Color.from_rgb(*rgb)


def _label_position(x, y, width, height, pad, margin, img, top, occupied):
    """尝试原位置及已放标签的边界；使用含底色的矩形做碰撞判断。"""
    left, right = max(pad, margin), img.width() - max(pad, margin) - width
    upper, lower = top + pad, img.height() - pad - height
    if right < left or lower < upper:
        return None
    x, y = max(left, min(x, right)), max(upper, min(y, lower))
    xs, ys = {x, left, right}, {y, upper, lower}
    for ox, oy, ow, oh in occupied:
        xs.update((ox - width - pad - 1, ox + ow + pad + 1))
        ys.update((oy - height - pad - 1, oy + oh + pad + 1))
    positions = [(px, py) for px in xs for py in ys if left <= px <= right and upper <= py <= lower]
    positions.sort(key=lambda point: (abs(point[0] - x) + abs(point[1] - y), point[1], point[0]))
    for px, py in positions:
        rect = (round(px) - pad, round(py) - pad,
                round(width) + 2 * pad + 1, round(height) + 2 * pad + 1)
        rx, ry, rw, rh = rect
        if all(rx + rw <= ox or ox + ow <= rx or ry + rh <= oy or oy + oh <= ry
               for ox, oy, ow, oh in occupied):
            occupied.append(rect)
            return px, py
    return None


def _draw_lines(img, x, y, lines, color, scale, min_y=0, background=None, avoid_rect=None,
                occupied=None, fit_attempt=0):
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
    if background is not None:
        gap = max(gap, 2 * max(max(2, round(row[1] / 3)) for row in rows) + 1)
    top = max(margin, int(min_y))
    total_height = sum(row[3] for row in rows) + gap * max(0, len(rows) - 1)
    available_height = max(1, img.height() - margin - top)
    if total_height > available_height:
        factor = available_height / total_height
        rows = [(text, row_scale * factor, *_text_size(text, row_scale * factor))
                for text, row_scale, _, _ in rows]
        gap *= factor
        if background is not None:
            gap = max(gap, 2 * max(max(2, round(row[1] / 3)) for row in rows) + 1)
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
    if occupied is not None:
        pad = max(2, round(max(row[1] for row in rows) / 3))
        position = _label_position(x, y, max(row[2] for row in rows), total_height,
                                   pad, margin, img, top, occupied)
        if position is None:
            if fit_attempt < 3:
                return _draw_lines(img, x, y, lines, color, scale * 0.8, min_y,
                                   background, avoid_rect, occupied, fit_attempt + 1)
            return None  # 极拥挤时保留框；可点击其他框隐藏信息，腾出位置。
        x, y = position
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


def draw_header(img, mode, fps, work_ms, uart_ms, remote_owned=False, selection_status=None):
    title = "QR SCAN" if mode == config.MODE_QR else "OBJ10 YOLO26"
    bottom = _draw_lines(img, 8, 8, ["{} {}x{} FPS:{:.1f}".format(title, img.width(), img.height(), fps)], image.COLOR_GREEN, config.STATUS_TEXT_SCALE)
    owner = "UART:CTRL" if remote_owned else "USER:SWITCH"
    bottom = _draw_lines(img, 8, bottom, ["WORK:{}ms UART:{}ms {}".format(work_ms, uart_ms, owner)], image.COLOR_YELLOW, config.STATUS_TEXT_SCALE)
    if selection_status:
        bottom = _draw_lines(img, 8, bottom, [selection_status], image.COLOR_YELLOW,
                             config.BOX_TEXT_SCALE, bottom)
    return bottom

def draw_objects(img, objects, display_size=None, min_y=0, details=False, info_objects=None):
    """全部检测框都画；info_objects只控制哪些框附带文字信息。"""
    scale = _result_scale(img, display_size, config.OBJECT_TEXT_SCALE)
    occupied = []
    labels = []
    info_ids = {id(obj) for obj in (objects if info_objects is None else info_objects)}
    # 先画全部框，再画文字，避免后画的框划穿已经放好的文字。
    for obj in objects:
        color = _object_color(obj.class_id)
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color, thickness=3)
        cx, cy = obj.x + obj.w // 2, obj.y + obj.h // 2
        img.draw_cross(cx, cy, color, size=9, thickness=2)
        if id(obj) in info_ids:
            labels.append((obj, color, cx, cy))
    for obj, color, cx, cy in labels:
        coordinates = "x={}, y={}".format(cx, cy)
        lines = ["{} {:.2f}".format(class_name(obj.class_id), obj.score)]
        if _text_size(coordinates, scale)[0] <= img.width() - 2 * max(2, round(scale)):
            lines.append(coordinates)
        else:
            lines.extend(["x={}".format(cx), "y={}".format(cy)])
        if details:
            lines.append("w={}, h={}".format(obj.w, obj.h))
        _draw_lines(img, obj.x, obj.y, lines, image.Color.from_rgb(0, 0, 0), scale, min_y,
                    background=color,
                    avoid_rect=(obj.x, obj.y, obj.w, obj.h), occupied=occupied)

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
