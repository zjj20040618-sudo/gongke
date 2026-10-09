"""MaixCAM2测试配置，作用类似C工程的config.h。

保留MC正式工程的算法和协议参数；本文件只负责硬件/模型等可调常量。
改阈值前先观察PERF及实物框；不要修改类别顺序来“修复”识别。
"""
MODE_QR = "QR"
MODE_OBJECT = "OBJECT"
START_MODE = MODE_QR  # 开机扫码；USER短按识别/扫码，MCU新请求可接管。
KEY_DEBOUNCE_MS = 180
KEY_LONG_PRESS_MS = 1500
QR_WIDTH, QR_HEIGHT = 1920, 1280  # 07工程第43档；扫描区域位置由下面配置/触摸调整。
OBJECT_WIDTH, OBJECT_HEIGHT = 480, 320
CAMERA_FPS = 30  # 沿用参考值；MC2驱动决定传感器实际帧率，不套用GC4653私有设置。
QR_ROI_FRACTION = 0.25
QR_ROI_CENTER_X = 0.44  # 扫描框中心：0最左、1最右；越界自动贴边。
QR_ROI_CENTER_Y = 0.34  # 0最上、1最下；例如0.35将框向上移动。
QR_ROI_TOUCH_MOVE = True  # QR模式点击MC实体屏移动扫描框；重启恢复上面配置。
QR_MAX = 1
QR_KEEP_FRAMES = 15
QR_WARMUP_FRAMES = 2  # 原07测试版：切换扫码采集后丢弃2个过渡帧。
QR_READ_TIMEOUT_MS = 2000  # 原07测试版读取超时；不缩小扫码输入。
OBJECT_BOX_HOLD_MS = 200  # 仅屏幕：漏检后保留旧框200毫秒；0立即消失，不延迟UART。
TASK_CONFIRM_FRAMES = 1  # 用户确认合法三码一帧即可；空帧、冲突、非法内容不确认。
MODEL_FILE = "model_9767.mud"  # MC2的MUD入口；同目录必须同时存在NPU/VNPU两个axmodel。
CONF_THRESHOLD = 0.50
MAX_OBJECTS = 10
DUAL_BUFFER = True  # 物体模型CPU/NPU流水线；frame_pair.py配回上一张输入图。
QR_CAMERA_BUFFERS = 1  # 保留单缓冲请求值；MC2驱动可能提升其内部缓冲数。扫码不使用NPU双缓冲。
OBJECT_CAMERA_BUFFERS = 2  # 相机请求值；模型双缓冲由DUAL_BUFFER控制，与采集缓冲不是同一件事。
CLASS_NAMES = ("oblate", "cylinder", "truncated_cone", "blue_ball", "red_ball", "green_ball", "red_target", "blue_target", "green_target", "black_barrel")
CLASS_NAMES_CN = ("扁圆物体", "圆柱体", "圆台体", "蓝球", "红球", "绿球", "红靶子", "蓝靶子", "绿靶子", "黑桶")
QR_COLOR_NAMES_CN = {"1": "红", "2": "绿", "3": "蓝"}
QR_SHAPE_NAMES_CN = {"1": "圆柱", "2": "圆锥", "3": "腰鼓"}
QR_COLOR_CODES = {"1": "R", "2": "G", "3": "B"}
QR_SHAPE_CODES = {"1": "Cyl", "2": "Cone", "3": "Drum"}
UART_ENABLED = True
BOOT_SESSION_ENABLED = False  # 配对电控PROTO_BOOT_SESSION_ENABLE=1时设True；不自动降级。
UART_DEVICE = "/dev/ttyS2"  # MC2的UART2；不是原MC的UART1。
UART_BAUDRATE = 115200
UART_TX_PIN = "B0"  # MC2 TX 接 MCU RX（如原电控PD6），3.3V TTL。
UART_RX_PIN = "B1"  # MC2 RX 接 MCU TX（如原电控PD5）；必须共地。
UART_TX_FUNCTION = "UART2_TX"
UART_RX_FUNCTION = "UART2_RX"
UART_TRACE = True
UART_TRACE_EVERY_N_FRAMES = 10
UART_WRITE_ATTEMPTS = 8
DISPLAY_ENABLED = True
QR_CAMERA_PREVIEW_ENABLED = True  # 扫码：False黑底显示状态和范围框，True显示相机画面；不停止采集/解码。
OBJECT_CAMERA_PREVIEW_ENABLED = True  # 识别：True显示相机画面；False仅隐藏背景，仍推理、画框和发串口。
PRINT_EVERY_N_FRAMES = 10
STATUS_TEXT_SCALE = 5
BOX_TEXT_SCALE = 2  # QR结果基础字号；保留当前扫码显示设置。
OBJECT_TEXT_SCALE = 2  # 物体标签和中心X/Y；按实际屏幕缩小比例补偿。
