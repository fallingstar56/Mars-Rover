import os
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
        time.sleep(0.01)

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
multi_detect_requested = False
multi_anchor_requested = False
multi_debug_image_index = 0

DEBUG_SAVE_DIR = "./debug"

# 屏幕中心捕获框：30x30，真正以 (320, 240) 为中心。
TARGET_LEFT = 305
TARGET_TOP = 225
TARGET_WIDTH = 30
TARGET_HEIGHT = 30
TARGET_RIGHT = TARGET_LEFT + TARGET_WIDTH
TARGET_BOTTOM = TARGET_TOP + TARGET_HEIGHT
TARGET_CENTER_X = TARGET_LEFT + TARGET_WIDTH / 2
TARGET_CENTER_Y = TARGET_TOP + TARGET_HEIGHT / 2
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
    """X 方向要求竖直对称轴重合，Y 方向要求目标框被识别框覆盖。"""
    center1_x = (left1 + right1) / 2
    center2_x = (left2 + right2) / 2
    x_axis_aligned = abs(center1_x - center2_x) <= TARGET_AXIS_TOLERANCE_PX
    y_target_covered = top1 <= top2 and bottom1 >= bottom2
    return x_axis_aligned and y_target_covered


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


thresholds_red = [[30, 43, 30, 40, 9, 18]]
thresholds_pink = [[56, 64, 18, 25, -1, -7]]
thresholds_blue = [[36, 46, -2, 5, -45, -36]]
thresholds_purple = [[24, 37, 13, 19, -35, -44]]
thresholds_yellow = [[54, 64, -6, 1, 39, 57]]
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


def image_color(name, fallback):
    return getattr(image, name, fallback)


debug_draw_color_by_task_color = {
    "red": image.COLOR_RED,
    "pink": image_color("COLOR_WHITE", image.COLOR_RED),
    "blue": image_color("COLOR_BLUE", image.COLOR_GREEN),
    "purple": image.COLOR_BLACK,
    "yellow": image_color("COLOR_YELLOW", image.COLOR_GREEN),
}


def collect_multi_blocks(img):
    candidates = []
    for color, thresholds in thresholds_by_color.items():
        blobs = img.find_blobs(thresholds, pixels_threshold=1500)
        for blob in blobs:
            candidates.append((color, blob))
    return candidates


def ensure_debug_save_dir():
    try:
        os.mkdir(DEBUG_SAVE_DIR)
    except OSError:
        pass


def draw_multi_debug_blob(img, color, blob):
    draw_color = debug_draw_color_by_task_color[color]
    x = blob[0]
    y = blob[1]
    w = blob[2]
    h = blob[3]
    center_x = int(blob[5])
    center_y = int(blob[6])
    label_y = y - 16
    if label_y < 0:
        label_y = y + h + 2

    img.draw_rect(x, y, w, h, draw_color, 3)
    img.draw_string(x, label_y, "%s %d,%d" % (color, center_x, center_y), draw_color)
    img.draw_line(center_x - 8, center_y, center_x + 8, center_y, draw_color)
    img.draw_line(center_x, center_y - 8, center_x, center_y + 8, draw_color)


def save_multi_debug_image(img, candidates, prefix):
    global multi_debug_image_index
    multi_debug_image_index += 1

    for color, blob in candidates:
        draw_multi_debug_blob(img, color, blob)

    img.draw_rect(
        TARGET_LEFT,
        TARGET_TOP,
        TARGET_WIDTH,
        TARGET_HEIGHT,
        image.COLOR_RED,
        5,
    )
    img.draw_string(4, 4, "%s blocks=%d" % (prefix, len(candidates)), image.COLOR_RED)

    ensure_debug_save_dir()
    path = "%s/%s_%04d_%02d.jpg" % (
        DEBUG_SAVE_DIR,
        prefix,
        multi_debug_image_index,
        len(candidates),
    )
    try:
        img.save(path)
        print("multi debug image saved:", path)
    except Exception as error:
        print("multi debug image save failed:", path, error)


def find_multi_left_bottom_block_from_candidates(candidates):
    if not candidates:
        return None, 0

    # 3x3 阵列中左下角：先按中心 y 取最下面一行，再取这一行最靠左的色块。
    bottom_row = sorted(candidates, key=lambda item: item[1][6], reverse=True)[:3]
    color, blob = min(bottom_row, key=lambda item: item[1][5])
    return (color, blob), len(candidates)


def find_multi_first_row_block(img):
    candidates = collect_multi_blocks(img)
    if not candidates:
        return None

    def center_distance_sq(item):
        blob = item[1]
        dx = blob[5] - TARGET_CENTER_X
        dy = blob[6] - TARGET_CENTER_Y
        return dx * dx + dy * dy

    color, blob = min(candidates, key=center_distance_sq)
    return color, blob


def find_multi_left_bottom_block(img):
    candidates = collect_multi_blocks(img)
    return find_multi_left_bottom_block_from_candidates(candidates)


def report_multi_detection(img, detection, prefix):
    if detection is None:
        serial.write_str("md none\n")
        print(prefix, "none")
        return

    detected_color, detected_blob = detection
    center_x = int(detected_blob[5])
    center_y = int(detected_blob[6])
    dx = center_x - int(TARGET_CENTER_X)
    dy = center_y - int(TARGET_CENTER_Y)
    serial.write_str("md %s %d %d\n" % (detected_color, dx, dy))
    img.draw_rect(
        detected_blob[0],
        detected_blob[1],
        detected_blob[2],
        detected_blob[3],
        draw_color_by_task_color[detected_color],
        5,
    )
    print(prefix, detected_color, dx, dy)


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
        elif command == "scan":
            if task_loaded and task_raw != "":
                serial.write_str(task_raw + "\n")
                print("按请求重发二维码任务:", task_raw)
            else:
                print("收到扫码请求，等待识别二维码。")
        elif command == "multi_detect":
            multi_detect_requested = True
        elif command == "multi_anchor":
            multi_anchor_requested = True

    if multi_detect_requested:
        multi_detect_requested = False
        report_multi_detection(img, find_multi_first_row_block(img), "multi detect:")

    if multi_anchor_requested:
        multi_anchor_requested = False
        candidates = collect_multi_blocks(img)
        detection, block_count = find_multi_left_bottom_block_from_candidates(candidates)
        save_multi_debug_image(img, candidates, "multi_anchor")
        print("multi anchor block count:", block_count)
        report_multi_detection(img, detection, "multi anchor:")

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
