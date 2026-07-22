from maix import app, camera, display, image
from maix import time, uart

xunhuan_num = 0
thresholds_red = [[10, 31, 18, 40, -11, 9]]
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
draw_color_by_color = {
    "red": image.COLOR_RED,
    "pink": image.COLOR_RED,
    "blue": image.COLOR_GREEN,
    "purple": image.COLOR_BLACK,
    "yellow": image.COLOR_GREEN,
}

# 摄像头初始化。
cam = camera.Camera(640, 480, fps=60)
disp = display.Display()

while not app.need_exit():
    img = cam.read()

    if xunhuan_num >= 1:  # 60 帧判断一次。
        for color, thresholds in thresholds_by_color.items():
            blobs = img.find_blobs(thresholds, pixels_threshold=1500)
            for blob in blobs:
                print(color)
                # 左上角为 0 点，矩形框选参数为 x、y、宽、高、中心点 X、中心点 Y。
                print(blob[0], blob[1], blob[2], blob[3], blob[5], blob[6])
                img.draw_rect(
                    blob[0],
                    blob[1],
                    blob[2],
                    blob[3],
                    draw_color_by_color[color],
                    5,
                )
        xunhuan_num = 0

    img.draw_rect(320, 240, 30, 30, image.COLOR_RED, 5)  # 在中心绘制中心点。
    xunhuan_num = xunhuan_num + 1
    disp.show(img)

