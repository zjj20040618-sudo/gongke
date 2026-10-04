# 配置阅读：大写名字是约定的配置常量；Python并不会自动禁止修改，实际默认值在此集中定义。
# MODE_*是字符串模式名，UART命令中的0/1/2在control_session.py转换成这些名字。
# 下面的参数保持原值；看清用途和单位后再区分“配置值”与“本帧测量值”。

"""联合 App 配置，作用类似 C 工程的 config.h。"""
MODE_QR = "QR"
MODE_OBJECT = "OBJECT"
# 上电待机，不自动扫码/推理；电控请求或接管前的USER按键负责选择运行模式。
START_MODE = "IDLE"  # wait for MCU; USER can start standalone QR before UART ownership
# 按键消抖窗口和长按门槛，单位毫秒；1000ms=1s。
KEY_DEBOUNCE_MS = 180
KEY_LONG_PRESS_MS = 1500
# 二维码模式画幅。下一行物体画幅是初始默认，进入OBJECT后改用模型实际输入宽高。
QR_WIDTH, QR_HEIGHT = 1600, 900
OBJECT_WIDTH, OBJECT_HEIGHT = 480, 320
# ROI是扫描区域：宽和高各取中央50%，所以面积约占整张图25%。
QR_ROI_FRACTION = 0.50
QR_MAX = 1
# 仅屏幕二维码框保留这么多帧；不保留用于串口发送的旧识别结果。
QR_KEEP_FRAMES = 15
# 模型描述文件的名称；object_detector会以当前源码所在目录为基准寻找它。
MODEL_FILE = "model_9564.mud"
# 置信度下限0.35；score是模型输出的分值，不等于实物识别正确率已达到35%。
CONF_THRESHOLD = 0.35
MAX_OBJECTS = 10
# False表示关闭模型双缓冲；不要仅为提高速度就改成True，时序行为需另测。
DUAL_BUFFER = False
# tuple（元组）按模型类别ID顺序保存标签，索引0..9；中文表还会与模型真实标签核对。
CLASS_NAMES = ("oblate", "cylinder", "truncated_cone", "blue_ball", "red_ball", "green_ball", "red_target", "blue_target", "green_target", "black_barrel")
CLASS_NAMES_CN = ("扁圆物体", "圆柱体", "圆台体", "蓝球", "红球", "绿球", "红靶子", "蓝靶子", "绿靶子", "黑桶")
# dict（字典）按键"1"/"2"/"3"查显示名；这里的字符串数字不是模型类别ID。
QR_COLOR_NAMES_CN = {"1": "红", "2": "绿", "3": "蓝"}
QR_SHAPE_NAMES_CN = {"1": "圆柱", "2": "圆锥", "3": "腰鼓"}
QR_COLOR_CODES = {"1": "R", "2": "G", "3": "B"}
QR_SHAPE_CODES = {"1": "Cyl", "2": "Cone", "3": "Drum"}
# UART串口使能、设备名、波特率和引脚；TX是本板发送，RX是本板接收。
UART_ENABLED = True
UART_DEVICE = "/dev/ttyS1"
UART_BAUDRATE = 115200
UART_TX_PIN = "A19"
UART_RX_PIN = "A18"
DISPLAY_ENABLED = True
# 日志每多少个处理帧打印一次；两项SCALE控制文字大小，不改变串口包内容。
PRINT_EVERY_N_FRAMES = 10
STATUS_TEXT_SCALE = 3
BOX_TEXT_SCALE = 3
