import threading

from maix import app, camera, display, image
from maix import time, uart


# 线颜色阈值，全局变量，后续按实际绿色线标定填写。
# 格式为 Maix LAB 阈值：[L_min, L_max, A_min, A_max, B_min, B_max]。
LINE_COLOR_THRESHOLDS = [[46, 75, -27, -10, -39, 5]]
CALIBRATION_MARKER_THRESHOLDS = [[57, 83, -34, -12, 18, 60]]

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER_X = FRAME_WIDTH // 2

TARGET_LEFT = 305
TARGET_TOP = 225
TARGET_WIDTH = 30
TARGET_HEIGHT = 30
TARGET_RIGHT = TARGET_LEFT + TARGET_WIDTH
TARGET_BOTTOM = TARGET_TOP + TARGET_HEIGHT
TARGET_CENTER_X = TARGET_LEFT + TARGET_WIDTH / 2
TARGET_CENTER_Y = TARGET_TOP + TARGET_HEIGHT / 2
TARGET_AXIS_TOLERANCE_PX = 1

# 只使用画面下方区域巡线，减少远处杂色干扰。
LINE_SEARCH_TOP = 260
LINE_SEARCH_BOTTOM = 470
LINE_MIN_PIXELS = 1200
LINE_MIN_AREA = 1500
CALIBRATION_MARKER_MIN_PIXELS = 1200
CALIBRATION_MARKER_MIN_AREA = 1500
SEND_EVERY_FRAMES = 2

device = "/dev/ttyS0"
serial = uart.UART(device, 115200)
stuts = ""


def re_uart(serial):
    global stuts
    while True:
        data = serial.read()
        data = data.decode("utf-8", errors="ignore")
        if data != "":
            print("uart0:", data)
            stuts = data
        time.sleep(0.01)


uart0_thread = threading.Thread(target=re_uart, args=(serial,))
uart0_thread.daemon = True
uart0_thread.start()

cam = camera.Camera(FRAME_WIDTH, FRAME_HEIGHT)
disp = display.Display()

VALID_TASK_COLORS = ("red", "pink", "blue", "purple", "yellow")
thresholds_red = [[0, 50, 35, 80, 10, 80]]
thresholds_pink = [[29, 70, 11, 39, -8, 15]]
thresholds_blue = [[0, 70, -10, 20, -80, -20]]
thresholds_purple = [[0, 80, 7, 37, -25, -5]]
thresholds_yellow = [[33, 75, -9, 10, 50, 73]]
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
multi_detect_requested = False
multi_detect_target_color = None
multi_column_requested = False
multi_column_target_color = None
multi_column_expected_count = 0
multi_anchor_requested = False


def intersects_search_band(blob):
    blob_top = blob[1]
    blob_bottom = blob[1] + blob[3]
    return not (blob_bottom < LINE_SEARCH_TOP or blob_top > LINE_SEARCH_BOTTOM)


def blob_area(blob):
    return int(blob[2] * blob[3])


def rects_overlap(left1, top1, right1, bottom1, left2, top2, right2, bottom2):
    center1_x = (left1 + right1) / 2
    center2_x = (left2 + right2) / 2
    x_axis_aligned = abs(center1_x - center2_x) <= TARGET_AXIS_TOLERANCE_PX
    y_target_covered = top1 <= top2 and bottom1 >= bottom2
    return x_axis_aligned and y_target_covered


def axis_gap(blob_low, blob_high, target_low, target_high):
    if blob_high < target_low:
        return target_low - blob_high
    if blob_low > target_high:
        return target_high - blob_low
    return 0


def center_axis_gap(blob_left, blob_right, target_center):
    blob_center = (blob_left + blob_right) / 2
    gap = target_center - blob_center
    if abs(gap) <= TARGET_AXIS_TOLERANCE_PX:
        return 0
    return int(gap)


def collect_multi_blocks(img):
    candidates = []
    for color, thresholds in thresholds_by_color.items():
        blobs = img.find_blobs(thresholds, pixels_threshold=1500)
        for blob in blobs:
            candidates.append((color, blob))
    return candidates


def find_multi_first_row_blocks_from_candidates(candidates):
    if not candidates:
        return [], 0

    first_row = sorted(candidates, key=lambda item: item[1][6], reverse=True)[:3]
    first_row = sorted(first_row, key=lambda item: item[1][5])
    return first_row, len(candidates)


def find_multi_left_bottom_block_from_candidates(candidates):
    first_row, block_count = find_multi_first_row_blocks_from_candidates(candidates)
    if not first_row:
        return None, block_count
    return first_row[0], block_count


def find_multi_first_row_block(img):
    candidates = collect_multi_blocks(img)
    if not candidates:
        return None

    def center_distance_sq(item):
        blob = item[1]
        dx = blob[5] - TARGET_CENTER_X
        dy = blob[6] - TARGET_CENTER_Y
        return dx * dx + dy * dy

    return min(candidates, key=center_distance_sq)


def find_multi_target_color_block(img, target_color):
    candidates = [
        (color, blob)
        for color, blob in collect_multi_blocks(img)
        if color == target_color
    ]
    if not candidates:
        return None

    def center_distance_sq(item):
        blob = item[1]
        dx = blob[5] - TARGET_CENTER_X
        dy = blob[6] - TARGET_CENTER_Y
        return dx * dx + dy * dy

    return min(candidates, key=center_distance_sq)


def find_multi_column_first_row_block(img, target_color):
    candidates = [
        (color, blob)
        for color, blob in collect_multi_blocks(img)
        if color == target_color
    ]
    if not candidates:
        return None, 0

    color, blob = max(candidates, key=lambda item: item[1][6])
    return (color, blob), len(candidates)


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


def report_multi_column_detection(img, target_color, expected_count, detection, count):
    if count != expected_count or detection is None:
        serial.write_str(
            "mc %s %d %d mismatch\n" % (target_color, count, expected_count)
        )
        print("multi column:", target_color, count, expected_count, "mismatch")
        return

    detected_color, detected_blob = detection
    center_x = int(detected_blob[5])
    center_y = int(detected_blob[6])
    dx = center_x - int(TARGET_CENTER_X)
    dy = center_y - int(TARGET_CENTER_Y)
    serial.write_str(
        "mc %s %d %d ok %d %d\n"
        % (detected_color, count, expected_count, dx, dy)
    )
    img.draw_rect(
        detected_blob[0],
        detected_blob[1],
        detected_blob[2],
        detected_blob[3],
        draw_color_by_task_color[detected_color],
        5,
    )
    print("multi column:", detected_color, count, expected_count, "ok", dx, dy)


def handle_multi_command_frame(img):
    global stuts
    global multi_detect_requested, multi_detect_target_color
    global multi_column_requested, multi_column_target_color, multi_column_expected_count
    global multi_anchor_requested

    if stuts != "":
        command = stuts.strip().lower()
        stuts = ""
        if command.startswith("multi_detect"):
            parts = command.split()
            multi_detect_requested = True
            if len(parts) >= 2 and parts[1] in VALID_TASK_COLORS:
                multi_detect_target_color = parts[1]
            else:
                multi_detect_target_color = None
        elif command.startswith("multi_column"):
            parts = command.split()
            if len(parts) >= 3 and parts[1] in VALID_TASK_COLORS:
                try:
                    multi_column_target_color = parts[1]
                    multi_column_expected_count = int(parts[2])
                    multi_column_requested = True
                except ValueError:
                    multi_column_requested = False
                    multi_column_target_color = None
                    multi_column_expected_count = 0
        elif command == "multi_anchor":
            multi_anchor_requested = True
        elif command == "ok":
            print("小车端已确认二维码任务。")

    if multi_detect_requested:
        multi_detect_requested = False
        if multi_detect_target_color is None:
            detection = find_multi_first_row_block(img)
            prefix = "multi detect:"
        else:
            detection = find_multi_target_color_block(img, multi_detect_target_color)
            prefix = "multi detect %s:" % multi_detect_target_color
        multi_detect_target_color = None
        report_multi_detection(img, detection, prefix)

    if multi_column_requested:
        multi_column_requested = False
        detection, count = find_multi_column_first_row_block(
            img,
            multi_column_target_color,
        )
        report_multi_column_detection(
            img,
            multi_column_target_color,
            multi_column_expected_count,
            detection,
            count,
        )
        multi_column_target_color = None
        multi_column_expected_count = 0

    if multi_anchor_requested:
        multi_anchor_requested = False
        candidates = collect_multi_blocks(img)
        detection, block_count = find_multi_left_bottom_block_from_candidates(candidates)
        print("multi anchor block count:", block_count)
        report_multi_detection(img, detection, "multi anchor:")


def main():
    frame_count = 0

    while not app.need_exit():
        img = cam.read()
        handle_multi_command_frame(img)
        img.draw_rect(
            0,
            LINE_SEARCH_TOP,
            FRAME_WIDTH,
            LINE_SEARCH_BOTTOM - LINE_SEARCH_TOP,
            image.COLOR_GREEN,
            2,
        )
        img.draw_line(
            FRAME_CENTER_X,
            LINE_SEARCH_TOP,
            FRAME_CENTER_X,
            LINE_SEARCH_BOTTOM,
            image.COLOR_RED,
        )

        frame_count += 1
        if frame_count >= SEND_EVERY_FRAMES:
            frame_count = 0
            blobs = img.find_blobs(
                LINE_COLOR_THRESHOLDS,
                pixels_threshold=LINE_MIN_PIXELS,
            )
            line_blobs = [
                blob
                for blob in blobs
                if intersects_search_band(blob) and blob_area(blob) >= LINE_MIN_AREA
            ]
            calibration_blobs = img.find_blobs(
                CALIBRATION_MARKER_THRESHOLDS,
                pixels_threshold=CALIBRATION_MARKER_MIN_PIXELS,
            )
            calibration_blobs = [
                blob
                for blob in calibration_blobs
                if blob_area(blob) >= CALIBRATION_MARKER_MIN_AREA
            ]

            if calibration_blobs:
                marker_blob = max(calibration_blobs, key=blob_area)
                marker_left = marker_blob[0]
                marker_top = marker_blob[1]
                marker_right = marker_left + marker_blob[2]
                marker_bottom = marker_top + marker_blob[3]
                marker_area = blob_area(marker_blob)

                img.draw_rect(
                    marker_blob[0],
                    marker_blob[1],
                    marker_blob[2],
                    marker_blob[3],
                    image.COLOR_YELLOW,
                    5,
                )
                if rects_overlap(
                    marker_left,
                    marker_top,
                    marker_right,
                    marker_bottom,
                    TARGET_LEFT,
                    TARGET_TOP,
                    TARGET_RIGHT,
                    TARGET_BOTTOM,
                ):
                    marker_dx = 0
                    marker_dy = 0
                else:
                    marker_dx = center_axis_gap(
                        marker_left,
                        marker_right,
                        TARGET_CENTER_X,
                    )
                    marker_dy = 0

                serial.write_str("lc %d %d %d\n" % (marker_dx, marker_dy, marker_area))
                print(
                    "绿色标定色块识别成功: x=%d y=%d w=%d h=%d cx=%d cy=%d dx=%d dy=%d area=%d"
                    % (
                        marker_blob[0],
                        marker_blob[1],
                        marker_blob[2],
                        marker_blob[3],
                        int(marker_blob[5]),
                        int(marker_blob[6]),
                        marker_dx,
                        marker_dy,
                        marker_area,
                    )
                )
            elif line_blobs:
                line_blob = max(line_blobs, key=blob_area)
                center_x = int(line_blob[5])
                dx = center_x - FRAME_CENTER_X
                area = blob_area(line_blob)
                img.draw_rect(
                    line_blob[0],
                    line_blob[1],
                    line_blob[2],
                    line_blob[3],
                    image.COLOR_GREEN,
                    5,
                )
                center_y = int(line_blob[6])
                img.draw_line(
                    center_x,
                    line_blob[1],
                    center_x,
                    line_blob[1] + line_blob[3],
                    image.COLOR_BLUE,
                )
                img.draw_line(center_x - 12, center_y, center_x + 12, center_y, image.COLOR_RED)
                img.draw_line(center_x, center_y - 12, center_x, center_y + 12, image.COLOR_RED)
                serial.write_str("ln %d %d\n" % (dx, area))
                print("ln", dx, area)
            else:
                serial.write_str("ln lost\n")
                print("ln lost")

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
                payload = qr.payload()
                img.draw_string(qr.x(), qr.y() - 15, payload, image.COLOR_RED)
                serial.write_str(payload + "\n")
                print("二维码任务已发送:", payload)
                break

        img.draw_rect(
            TARGET_LEFT,
            TARGET_TOP,
            TARGET_WIDTH,
            TARGET_HEIGHT,
            image.COLOR_RED,
            3,
        )
        disp.show(img)


if __name__ == "__main__":
    main()
