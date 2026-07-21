from maix import app, camera, display, image
from maix import uart


# 线颜色阈值，全局变量，后续按实际绿色线标定填写。
# 格式为 Maix LAB 阈值：[L_min, L_max, A_min, A_max, B_min, B_max]。
LINE_COLOR_THRESHOLDS = [[48, 60, -19, -10, -23, -30]]

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER_X = FRAME_WIDTH // 2

# 只使用画面下方区域巡线，减少远处杂色干扰。
LINE_SEARCH_TOP = 260
LINE_SEARCH_BOTTOM = 470
LINE_MIN_PIXELS = 1200
LINE_MIN_AREA = 1500
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
            blobs = [
                blob
                for blob in blobs
                if intersects_search_band(blob) and blob_area(blob) >= LINE_MIN_AREA
            ]

            if blobs:
                line_blob = max(blobs, key=blob_area)
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
                img.draw_line(center_x - 12, center_y, center_x + 12, center_y, image.COLOR_RED)
                img.draw_line(center_x, center_y - 12, center_x, center_y + 12, image.COLOR_RED)
                serial.write_str("ln %d %d\n" % (dx, area))
                print("ln", dx, area)
            else:
                serial.write_str("ln lost\n")
                print("ln lost")

        disp.show(img)


if __name__ == "__main__":
    main()
