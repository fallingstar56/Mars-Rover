from maix import app, camera, display, image


# 色块阈值沿用 sekuai.py，便于直接对照测试结果。
THRESHOLDS_BY_COLOR = {
    "red": [[0, 80, 0, 80, 10, 80]],
    "pink": [[36, 88, 10, 43, -16, 10]],
    "blue": [[10, 68, -11, 16, -71, -13]],
    "purple": [[-5, 39, 5, 50, -47, -3]],
    "yellow": [[37, 92, -25, 1, 40, 92]],
}

DRAW_COLORS = {
    "red": image.COLOR_RED,
    "pink": image.COLOR_RED,
    "blue": image.COLOR_GREEN,
    "purple": image.COLOR_BLACK,
    "yellow": image.COLOR_GREEN,
}


def colors_from_qrcode(payload):
    """直接提取二维码中出现的颜色，不做任务格式和数量验证。"""
    words = str(payload).lower().replace(",", " ").split()
    colors = []
    for word in words:
        if word in THRESHOLDS_BY_COLOR and word not in colors:
            colors.append(word)
    return colors


cam = camera.Camera(640, 480)
disp = display.Display()

last_qrcode = ""
# 尚未读到二维码时也显示所有色块，方便调整镜头和阈值。
active_colors = list(THRESHOLDS_BY_COLOR.keys())

while not app.need_exit():
    img = cam.read()

    # 二维码读取不设置前置验证，也不在首次读取后关闭扫码。
    for qr in img.find_qrcodes():
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
        img.draw_string(qr.x(), max(0, qr.y() - 15), payload, image.COLOR_RED)

        if payload != last_qrcode:
            last_qrcode = payload
            selected_colors = colors_from_qrcode(payload)
            # 二维码未包含已知颜色时仍识别全部颜色，避免测试画面空白。
            active_colors = selected_colors or list(THRESHOLDS_BY_COLOR.keys())
            print("qrcode:", payload)
            print("detect colors:", active_colors)

    # 每一帧持续识别和绘制，不因二维码已读取或色块已出现而停止。
    for color in active_colors:
        blobs = img.find_blobs(
            THRESHOLDS_BY_COLOR[color],
            pixels_threshold=1500,
        )
        for blob in blobs:
            img.draw_rect(
                blob[0],
                blob[1],
                blob[2],
                blob[3],
                DRAW_COLORS[color],
                5,
            )
            img.draw_string(
                blob[0],
                max(0, blob[1] - 15),
                color,
                DRAW_COLORS[color],
            )

    if last_qrcode:
        img.draw_string(5, 5, "QR: " + last_qrcode, image.COLOR_RED)

    disp.show(img)
