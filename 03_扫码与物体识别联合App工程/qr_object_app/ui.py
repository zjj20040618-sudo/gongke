# 显示模块只在图像上画文字和框，不做任务选择，不发串口，不控制底盘。
# 图像左上角是(0,0)，x向右增大，y向下增大，坐标单位都是像素。

"""集中管理画框和状态文字。"""
from maix import image
import config
from utils import class_name

# 功能：画模式、画幅、帧率、耗时与控制来源。
# 参数：img：当前图像；mode：模式；fps：帧率；work_ms/uart_ms：耗时；remote_owned：是否电控接管。
# 返回：None（没有显式return时默认返回None）。
def draw_header(img, mode, fps, work_ms, uart_ms, remote_owned=False):
    title = "QR SCAN" if mode == config.MODE_QR else "OBJ10 YOLO26"
    img.draw_string(8, 8, "{} {}x{} FPS:{:.1f}".format(title, img.width(), img.height(), fps), image.COLOR_GREEN, scale=config.STATUS_TEXT_SCALE)
    owner = "UART:CTRL" if remote_owned else "USER:SWITCH"
    img.draw_string(8, 40, "WORK:{}ms UART:{}ms {}".format(work_ms, uart_ms, owner), image.COLOR_YELLOW, scale=config.STATUS_TEXT_SCALE)

# 功能：给检测框画矩形、中心十字和类别分数。
# 参数：img：当前图像；objects：用于展示的Detection列表。
# 返回：None（没有显式return时默认返回None）。
# 理解：颜色仅是显示规则；框颜色不能替代模型类别或二维码指定目标。
def draw_objects(img, objects):
    for obj in objects:
        color = image.COLOR_RED if obj.class_id >= 6 else image.COLOR_BLUE
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color, thickness=3)
        img.draw_cross(obj.x + obj.w // 2, obj.y + obj.h // 2, color, size=9, thickness=2)
        # max(76,...)让标签纵坐标至少为76，减少顶部状态栏重叠；不会改发送的目标坐标。
        img.draw_string(obj.x, max(76, obj.y - 26), "{} {:.2f}".format(class_name(obj.class_id), obj.score), color, scale=config.BOX_TEXT_SCALE)

# 功能：优先按四角画二维码轮廓，失败时画矩形框。
# 参数：img：当前图像；qrs：用于显示的二维码字典列表。
# 返回：None（没有显式return时默认返回None）。
def draw_qrs(img, qrs):
    for qr in qrs:
        try:
            img.draw_edges(qr["corners"], image.COLOR_GREEN, thickness=4)
        except Exception:
            img.draw_rect(qr["x"], qr["y"], qr["w"], qr["h"], image.COLOR_GREEN, thickness=4)
        img.draw_string(qr["x"], max(76, qr["y"] - 32), "QR:{} {}".format(qr["payload"], qr["text_ascii"]), image.COLOR_YELLOW, scale=config.BOX_TEXT_SCALE)
