from maix import app, camera, display, image


thresholds_red = [[0, 50, 35, 80, 10, 80]]
thresholds_pink = [[29, 70, 11, 39, -8, 15]]
thresholds_blue = [[0, 70, -10, 20, -80, -20]]
thresholds_purple = [[0, 80, 7, 37, -25, -5]]
thresholds_yellow = [[33, 75, -9, 10, 50, 73]]

COLOR_CONFIG = (
    ("red", thresholds_red, image.COLOR_RED),
    ("pink", thresholds_pink, image.COLOR_WHITE),
    ("blue", thresholds_blue, image.COLOR_BLUE),
    ("purple", thresholds_purple, image.COLOR_BLACK),
    ("yellow", thresholds_yellow, image.COLOR_YELLOW),
)

PIXELS_THRESHOLD = 1500
TARGET_LEFT = 305
TARGET_TOP = 225
TARGET_WIDTH = 30
TARGET_HEIGHT = 30


cam = camera.Camera(640, 480)
disp = display.Display()
cam.awb_mode(camera.AwbMode.Manual)
cam.set_wb_gain([0.1254,0.0625,0.0625,0.1113])

while not app.need_exit():
    img = cam.read()

    for color_name, thresholds, draw_color in COLOR_CONFIG:
        blobs = img.find_blobs(thresholds, pixels_threshold=PIXELS_THRESHOLD)
        for blob in blobs:
            x = blob[0]
            y = blob[1]
            w = blob[2]
            h = blob[3]
            center_x = int(blob[5])
            center_y = int(blob[6])

            img.draw_rect(x, y, w, h, draw_color, 4)
            img.draw_string(x, max(0, y - 16), color_name, draw_color)
            img.draw_line(center_x - 8, center_y, center_x + 8, center_y, draw_color)
            img.draw_line(center_x, center_y - 8, center_x, center_y + 8, draw_color)
            print(
                "%s blob: x=%d y=%d w=%d h=%d cx=%d cy=%d"
                % (color_name, x, y, w, h, center_x, center_y)
            )

    img.draw_rect(
        TARGET_LEFT,
        TARGET_TOP,
        TARGET_WIDTH,
        TARGET_HEIGHT,
        image.COLOR_RED,
        3,
    )
    disp.show(img)
