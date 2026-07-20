import threading

from maix import app, camera, display, image
from maix import time, uart

stuts = ""  # 串口接收状态标志。
device = "/dev/ttyS0"
serial = uart.UART(device, 115200)


def re_uart(serial):
    global stuts
    while 1:
        data = serial.read()
        data = data.decode("utf-8", errors="ignore")
        if data != "":
            print(f"uart0:{data}")
            stuts = data
            data = ""


uart0_thread = threading.Thread(target=re_uart, args=(serial,))
uart0_thread.daemon = True
uart0_thread.start()


xunhuan_num = 0
VALID_TASK_COLORS = ("red", "pink", "blue", "purple", "yellow")
task_loaded = False
task_raw = ""
task_colors = []
task_counts = []
grab_task = []
grab_queue = []
current_task_index = 0
target_reported = False

# 屏幕中心捕获框：30x30，真正以 (320, 240) 为中心。
TARGET_LEFT = 305
TARGET_TOP = 225
TARGET_WIDTH = 30
TARGET_HEIGHT = 30
TARGET_RIGHT = TARGET_LEFT + TARGET_WIDTH
TARGET_BOTTOM = TARGET_TOP + TARGET_HEIGHT
TARGET_CENTER_X = TARGET_LEFT + TARGET_WIDTH / 2
TARGET_AXIS_TOLERANCE_PX = 1
# 第一次发生真实重叠后锁存完成状态，本轮任务不再重新启动。
target_reached = False


def rects_overlap(
    left1,
    top1,
    right1,
    bottom1,
    left2,
    top2,
    right2,
    bottom2,
):
    """X 方向要求竖直对称轴重合，Y 方向只要求投影有重叠。"""
    center1_x = (left1 + right1) / 2
    center2_x = (left2 + right2) / 2
    x_axis_aligned = abs(center1_x - center2_x) <= TARGET_AXIS_TOLERANCE_PX
    y_overlapped = not (bottom1 < top2 or top1 > bottom2)
    return x_axis_aligned and y_overlapped


def axis_gap(blob_low, blob_high, target_low, target_high):
    """返回目标框到识别框的有符号边缘距离；重叠时返回 0。"""
    if blob_high < target_low:
        return target_low - blob_high
    if blob_low > target_high:
        return target_high - blob_low
    return 0


def center_axis_gap(blob_left, blob_right, target_center):
    """返回目标竖直对称轴到识别框竖直对称轴的有符号距离。"""
    blob_center = (blob_left + blob_right) / 2
    gap = target_center - blob_center
    if abs(gap) <= TARGET_AXIS_TOLERANCE_PX:
        return 0
    return int(gap)


def parse_qrcode_task(payload):
    """解析二维码任务，格式为：颜色1 颜色2 颜色3 数量1 数量2 数量3。"""
    parts = str(payload).strip().split()
    if len(parts) != 6:
        return None

    colors = [item.lower() for item in parts[:3]]
    counts_text = parts[3:]
    for color in colors:
        if color not in VALID_TASK_COLORS:
            return None

    try:
        counts = [int(item) for item in counts_text]
    except ValueError:
        return None

    task = []
    queue = []
    for color, count in zip(colors, counts):
        if count < 0:
            return None
        task.append(
            {
                "color": color,
                "count": count,
            }
        )
        for _ in range(count):
            queue.append(color)
    return colors, counts, task, queue


thresholds_red = [[14, 51, 33, 70, 13, 43]]
thresholds_pink = [[36, 88, 10, 43, -16, 10]]
thresholds_blue = [[10, 68, -11, 16, -71, -13]]
thresholds_purple = [[-5, 39, 5, 50, -47, -3]]
thresholds_yellow = [[37, 92, -25, 1, 40, 92]]
thresholds_by_color = {
    "red": thresholds_red,
    "pink": thresholds_pink,
    "blue": thresholds_blue,
    "purple": thresholds_purple,
    "yellow": thresholds_yellow,
}
draw_color_by_task_color = {
    "red": image.COLOR_RED,
    "pink": image.COLOR_RED,
    "blue": image.COLOR_GREEN,
    "purple": image.COLOR_BLACK,
    "yellow": image.COLOR_GREEN,
}

# 摄像头初始化。
cam = camera.Camera(640, 480)
disp = display.Display()

while not app.need_exit():
    img = cam.read()

    if stuts != "":
        command = stuts.strip().lower()
        stuts = ""
        if command == "next" and task_loaded:
            current_task_index += 1
            target_reached = False
            target_reported = False
            if current_task_index < len(grab_queue):
                print("切换到下一个抓取目标:", grab_queue[current_task_index])
            else:
                print("全部视觉抓取目标已完成。")
        elif command == "ok":
            print("小车端已确认二维码任务。")

    if not task_loaded:
        qrcodes = img.find_qrcodes()
        for qr in qrcodes:
            corners = qr.corners()
            for i in range(4):
                img.draw_line(
                    corners[i][0],
                    corners[i][1],
                    corners[(i + 1) % 4][0],
                    corners[(i + 1) % 4][1],
                    image.COLOR_RED,
                )
            img.draw_string(qr.x(), qr.y() - 15, qr.payload(), image.COLOR_RED)

            task_info = parse_qrcode_task(qr.payload())
            if task_info is None:
                print("二维码任务格式错误:", qr.payload())
                continue

            task_raw = qr.payload()
            task_colors, task_counts, grab_task, grab_queue = task_info
            task_loaded = True
            current_task_index = 0
            target_reached = False
            target_reported = False
            print("二维码任务:", task_raw)
            print("抓取任务:", grab_task)
            print("抓取队列:", grab_queue)
            serial.write_str(task_raw + "\n")
            break

    if (
        task_loaded
        and current_task_index < len(grab_queue)
        and not target_reported
        and xunhuan_num >= 1
    ):
        current_color = grab_queue[current_task_index]
        thresholds = thresholds_by_color[current_color]
        blobs = img.find_blobs(thresholds, pixels_threshold=1500)
        if blobs != []:
            # 多个同色区域同时出现时只跟踪面积最大的一个，避免指令来回切换。
            blob = max(blobs, key=lambda item: item[2] * item[3])
            blob_left = blob[0]
            blob_top = blob[1]
            blob_right = blob_left + blob[2]
            blob_bottom = blob_top + blob[3]

            print(current_color)
            draw_color = draw_color_by_task_color[current_color]
            img.draw_rect(blob[0], blob[1], blob[2], blob[3], draw_color, 5)

            overlaps_target = rects_overlap(
                blob_left,
                blob_top,
                blob_right,
                blob_bottom,
                TARGET_LEFT,
                TARGET_TOP,
                TARGET_RIGHT,
                TARGET_BOTTOM,
            )
            if overlaps_target:
                target_reached = True

            if target_reached:
                # 首次真实重叠后持续发送零偏差，彻底终止本轮移动。
                delta_x = 0
                delta_y = 0
            else:
                # X 方向按竖直对称轴对齐，Y 方向仍按两个矩形之间的边缘距离移动。
                delta_x = center_axis_gap(
                    blob_left,
                    blob_right,
                    TARGET_CENTER_X,
                )
                delta_y = axis_gap(
                    blob_top,
                    blob_bottom,
                    TARGET_TOP,
                    TARGET_BOTTOM,
                )

            print(
                "gap:",
                delta_x,
                delta_y,
                "overlap:",
                overlaps_target,
                "reached:",
                target_reached,
            )
            u_data = f"sx{delta_x:04d}{delta_y:04d}"
            serial.write_str(u_data + "\n")
            if target_reached:
                target_reported = True
            
        xunhuan_num = 0

    img.draw_rect(
        TARGET_LEFT,
        TARGET_TOP,
        TARGET_WIDTH,
        TARGET_HEIGHT,
        image.COLOR_RED,
        5,
    )
    xunhuan_num = xunhuan_num + 1
    disp.show(img)
