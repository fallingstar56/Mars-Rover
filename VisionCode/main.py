from maix import app, camera, display, image
from maix import time, uart

xunhuan_num = 0
thresholds_red = [[14, 51, 33, 70, 13, 43]]
thresholds_green = [[0, 60, 0, 127, -128, -35]]  # Green.
thresholds_black = [[9, 29, -10, 10, -11, 9]]  # Black.

# 摄像头初始化。
cam = camera.Camera(640, 480, fps=60)
disp = display.Display()

while not app.need_exit():
    img = cam.read()

    if xunhuan_num >= 1:  # 60 帧判断一次。
        blobs = img.find_blobs(thresholds_red, pixels_threshold=10)
        if blobs != []:
            for blob in blobs:
                print("red")
                # 左上角为 0 点，矩形框选参数为 x、y、宽、高、中心点 X、中心点 Y。
                print(blob[0], blob[1], blob[2], blob[3], blob[5], blob[6])
                img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_RED, 5)

        blobs = img.find_blobs(thresholds_green, pixels_threshold=10)
        if blobs != []:
            for blob in blobs:
                print("blue")
                # 左上角为 0 点，矩形框选参数为 x、y、宽、高、中心点 X、中心点 Y。
                print(blob[0], blob[1], blob[2], blob[3], blob[5], blob[6])
                img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_GREEN, 5)

        blobs = img.find_blobs(thresholds_black, pixels_threshold=1000)
        if blobs != []:
            for blob in blobs:
                print("black")
                # 左上角为 0 点，矩形框选参数为 x、y、宽、高、中心点 X、中心点 Y。
                print(blob[0], blob[1], blob[2], blob[3], blob[5], blob[6])
                img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_BLACK, 5)
        xunhuan_num = 0

    img.draw_rect(320, 240, 30, 30, image.COLOR_RED, 5)  # 在中心绘制中心点。
    xunhuan_num = xunhuan_num + 1
    disp.show(img)

