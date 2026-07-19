from maix import camera, display, image, app
from maix import uart, time
import threading

stuts = ""            # 串口接收状态标志
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
# 屏幕中心捕获框：30x30，真正以 (320, 240) 为中心。
TARGET_LEFT = 305
TARGET_TOP = 225
TARGET_WIDTH = 30
TARGET_HEIGHT = 30
TARGET_RIGHT = TARGET_LEFT + TARGET_WIDTH
TARGET_BOTTOM = TARGET_TOP + TARGET_HEIGHT
# 第一次发生真实重叠后锁存完成状态，本轮任务不再重新启动。
target_reached = False

def rects_overlap(left1, top1, right1, bottom1,
                  left2, top2, right2, bottom2):
    return not (
        right1 < left2 or left1 > right2 or
        bottom1 < top2 or top1 > bottom2
    )


def axis_gap(blob_low, blob_high, target_low, target_high):
    """返回目标框到识别框的有符号边缘距离；重叠时返回 0。"""
    if blob_high < target_low:
        return target_low - blob_high
    if blob_low > target_high:
        return target_high - blob_low
    return 0


thresholds_red   = [[14, 51,  33,  70,   13,  43]]    # red
thresholds_pink = [[36, 88,   10, 43, -16, 10]]    # green
thresholds_blue = [[10, 68, -11,  16,  -71,   -13]]    # black
thresholds_purple=[[-5, 39, 5, 50, -47, -3]]
thresholds_yellow=[[37, 92, -25, 1, 40, 92]]
#摄像头初始化
cam = camera.Camera(640, 480, ) 
disp = display.Display()

while not app.need_exit():
    img = cam.read()
    
    if xunhuan_num >= 1 :  #60帧判断一次
        blobs = img.find_blobs(thresholds_red, pixels_threshold=1500)  #pixels_threshold 色块阈值
        if blobs != [] :    #判断是否识别成功
            # 多个红色区域同时出现时只跟踪面积最大的一个，避免指令来回切换。
            blob = max(blobs, key=lambda item: item[2] * item[3])
            blob_left = blob[0]
            blob_top = blob[1]
            blob_right = blob_left + blob[2]
            blob_bottom = blob_top + blob[3]

            print("red")
            img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_RED, 5)

            overlaps_target = rects_overlap(
                blob_left, blob_top, blob_right, blob_bottom,
                TARGET_LEFT, TARGET_TOP, TARGET_RIGHT, TARGET_BOTTOM,
            )
            if overlaps_target:
                target_reached = True

            if target_reached:
                # 首次真实重叠后持续发送零偏差，彻底终止本轮移动。
                delta_x = 0
                delta_y = 0
            else:
                # 按两个矩形之间的边缘距离移动，不再强求中心点精确重合。
                delta_x = axis_gap(
                    blob_left, blob_right, TARGET_LEFT, TARGET_RIGHT
                )
                delta_y = axis_gap(
                    blob_top, blob_bottom, TARGET_TOP, TARGET_BOTTOM
                )

            print("gap:", delta_x, delta_y,
                  "overlap:", overlaps_target, "reached:", target_reached)
            u_data = f"sx{delta_x:04d}{delta_y:04d}".encode("utf-8")
            serial.write_str(u_data)


    #    blobs = img.find_blobs(thresholds_green, pixels_threshold=10)  #pixels_threshold 色块阈值
    #    if blobs != [] :    #判断是否识别成功
    #        for blob in blobs:
    #           print("blue")
    #           print(blob[0], blob[1], blob[2], blob[3], blob[5], blob[6])                 # 左上角为0点 矩形框选 （x, y, 宽，高, 中心点X, 中心点Y ）
    #           img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_GREEN, 5)     # 左上角为0点 矩形框选 （x, y, 宽，高, 线色, 线宽）


    #    blobs = img.find_blobs(thresholds_black, pixels_threshold=1000)  #pixels_threshold 色块阈值
    #    if blobs != [] :    #判断是否识别成功
    #        for blob in blobs:
    #            print("black")
    #            print(blob[0], blob[1], blob[2], blob[3], blob[5], blob[6])                 # 左上角为0点 矩形框选 （x, y, 宽，高, 中心点X, 中心点Y ）
    #            img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_BLACK, 5)     # 左上角为0点 矩形框选 （x, y, 宽，高, 线色, 线宽）
        xunhuan_num = 0


    img.draw_rect(
        TARGET_LEFT, TARGET_TOP, TARGET_WIDTH, TARGET_HEIGHT,
        image.COLOR_RED, 5
    )
    xunhuan_num = xunhuan_num + 1  
    disp.show(img)

  
