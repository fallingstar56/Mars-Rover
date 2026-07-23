from maix import app, camera, display, image
from maix import uart


# 线颜色阈值，全局变量，后续按实际绿色线标定填写。
# 格式为 Maix LAB 阈值：[L_min, L_max, A_min, A_max, B_min, B_max]。
LINE_COLOR_THRESHOLDS = [[46, 65, -27, -8, -39, -22]]
CALIBRATION_MARKER_THRESHOLDS = [[60, 80, -30, -15, 40, 60]]

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

cam = camera.Camera(FRAME_WIDTH, FRAME_HEIGHT)
disp = display.Display()


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


def main():
    frame_count = 0

    while not app.need_exit():
        img = cam.read()
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
                    marker_dy = axis_gap(
                        marker_top,
                        marker_bottom,
                        TARGET_TOP,
                        TARGET_BOTTOM,
                    )

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
