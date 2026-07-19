from maix import camera, display, image, uart

cam = camera.Camera(640, 480)  # 屏幕大小。
disp = display.Display()

ewm = []

device = "/dev/ttyS0"
serial = uart.UART(device, 115200)

while 1:
    img = cam.read()
    # 寻找二维码，找不到二维码时 qrcodes 为空。
    qrcodes = img.find_qrcodes()
    for qr in qrcodes:
        corners = qr.corners()  # 获取已扫描到的二维码四个顶点坐标。
        for i in range(4):
            img.draw_line(
                corners[i][0],
                corners[i][1],
                corners[(i + 1) % 4][0],
                corners[(i + 1) % 4][1],
                image.COLOR_RED,
            )
        img.draw_string(qr.x(), qr.y() - 15, qr.payload(), image.COLOR_RED)
        print(qr.payload())
        ewm = qr.payload()  # 获取二维码内容。
        serial.write_str(ewm)
    disp.show(img)
