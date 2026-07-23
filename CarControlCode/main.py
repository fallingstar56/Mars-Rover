"""
月球小车示例主程序。

底层协议已经拆成库：
- motor_lib.py：电机 CAN 速度模式
- servo_lib.py：Fashion Star 舵机二进制协议
- servo_control.py：按功能划分的学生舵机控制接口
- chassis_control.py：底盘组合控制（依赖 motor_lib + servo_control）
- arm_control.py：机械臂关节点动与相机控制（依赖 servo_control）
- robot_config.py：ID、引脚、速度、尺寸等配置

作者 王笑
日期 20260528
"""

import _thread
import machine
import math
import time

time.sleep(3)

from machine import UART
from esp32 import CAN

from chassis_control import LunarRover
from arm_control import ArmKinematicsError, RobotArm
from motor_lib import MotorBus
from ps2_control import button_pressed, map_joystick, ps2_loop
from ps2_lib import PS2Controller, PS2Receiver
from robot_config import (
    ARM_AUTO_ACTION_DELAY_MS,
    ARM_GRAB_PITCH1_DEG,
    
    ARM_GRAB_PITCH2_DEG,
    ARM_GRAB_PITCH3_DEG,
    ARM_INIT_PITCH1_DEG,
    ARM_INIT_PITCH2_DEG,
    ARM_INIT_PITCH3_DEG,
    ARM_INIT_ROLL_DEG,
    ARM_PLACE_PITCH1_DEG,
    ARM_PLACE_PITCH2_DEG,
    ARM_PLACE_PITCH3_DEG,
    CAN_BAUDRATE,
    CAN_BUS_ID,
    CAN_RX,
    CAN_TX,
    CAMERA_INIT_ANGLE_DEG,
    CAMERA_L3_ANGLE_DEG,
    CAMERA_UART_BAUD,
    CAMERA_UART_ID,
    CAMERA_UART_RX,
    CAMERA_UART_TX,
    CAMERA_SERVO_ID,
    DEFAULT_ACC_RAD_S2,
    GRIPPER_CLOSED_ANGLE_DEG,
    GRIPPER_OPEN_ANGLE_DEG,
    GRIPPER_SERVO_ID,
    LINE_FOLLOW_BASE_SPEED_RAD_S,
    LINE_FOLLOW_DATA_TIMEOUT_MS,
    LINE_FOLLOW_MAX_STEER_DEG,
    LINE_FOLLOW_STEER_KP,
    MAX_MOTOR_RPM,
    MAX_STEER_ANGLE_DEG,
    MULTI_CENTER_TOLERANCE_PX,
    MULTI_COLUMN_COUNT,
    MULTI_COLUMN_MOVE_MS,
    MULTI_DETECT_TIMEOUT_MS,
    MULTI_ENTRY_ALIGN_TIMEOUT_MS,
    MULTI_HORIZONTAL_SPEED_RAD_S,
    MULTI_HORIZONTAL_STEER_DEG,
    MULTI_GRAB_ROW_POSES,
    MULTI_PLACE_POSES,
    PIVOT_SPEED_SCALE,
    PS2_CLK,
    PS2_CS,
    PS2_DI,
    PS2_DO,
    RUN_MODE,
    SERVO_UART_BAUD,
    SERVO_UART_ID,
    SERVO_UART_RX,
    SERVO_UART_TX,
    RESERVE_SERVO_ENABLED,
    RESERVE_SERVO_IDS,
)
from servo_control import ServoControl, get_all_servo_ids
from servo_lib import ServoBus

# 相机串口共享数据。
camera_data = {"value": None}
_CAMERA_DATA_BUFFER_MAX_CHARS = 2048

# 视觉跟踪参数。delta_x/delta_y 是视觉端计算的中心框与识别框边缘距离。
# 当前视觉分辨率为 640x480，因此最大半宽/半高分别约为 320/240。
CAMERA_X_HALF_SIZE = 320.0
CAMERA_Y_HALF_SIZE = 240.0
# 捕获区及防抖逻辑由视觉端根据两个矩形是否重叠处理，小车端不再提前截断边缘距离。
CAMERA_DEADZONE_X = 0
CAMERA_DEADZONE_Y = 0
CAMERA_TRACK_MAX_SPEED_RAD_S = 3.0
CAMERA_TRACK_MIN_SPEED_RAD_S = 0.15
CAMERA_TRACK_NEAR_GAP_PX = 60
CAMERA_TRACK_NEAR_MAX_SPEED_RAD_S = 0.35
CAMERA_DATA_TIMEOUT_MS = 500
VALID_TASK_COLORS = ("red", "pink", "blue", "purple", "yellow")
_MULTI_MAX_MOTOR_RAD_S = MAX_MOTOR_RPM * 2.0 * math.pi / 60.0
_MULTI_MAX_PIVOT_RAD_S = _MULTI_MAX_MOTOR_RAD_S * PIVOT_SPEED_SCALE
_MULTI_EXECUTE_BUTTON_NAME = "R3"


def clamp(value, low, high):
    return max(low, min(high, value))


def parse_camera_offset(raw_data):
    """解析视觉串口数据。

    兼容两种格式：
    - sx123456       -> delta_x=123,  delta_y=456（3 字符定宽）
    - sx-1230045     -> delta_x=-123, delta_y=45 （4 字符定宽）
    """
    if raw_data is None:
        return None
    if isinstance(raw_data, bytes):
        raw_data = raw_data.decode("utf-8", "replace")

    text = str(raw_data).strip()
    start = text.rfind("sx")
    if start < 0:
        return None

    payload = text[start + 2:]
    # UART 一次可能收到多行，只解析最后一个 sx 后的第一段。
    for separator in ("\r", "\n", " ", "\t"):
        if separator in payload:
            payload = payload.split(separator, 1)[0]

    try:
        if len(payload) >= 8:
            delta_x = int(payload[0:4])
            delta_y = int(payload[4:8])
        elif len(payload) >= 6:
            delta_x = int(payload[0:3])
            delta_y = int(payload[3:6])
        else:
            return None
    except ValueError:
        return None
    return delta_x, delta_y


def parse_line_follow_frame(raw_data):
    """解析巡线视觉数据：ln <dx> <area> 或 ln lost。"""
    if raw_data is None:
        return None
    if isinstance(raw_data, bytes):
        raw_data = raw_data.decode("utf-8", "replace")

    parts = str(raw_data).strip().split()
    if len(parts) < 2 or parts[0] != "ln":
        return None
    if parts[1].lower() == "lost":
        return {
            "lost": True,
            "dx": 0,
            "area": 0,
        }

    try:
        dx = int(parts[1])
        area = int(parts[2]) if len(parts) >= 3 else 0
    except ValueError:
        return None

    return {
        "lost": False,
        "dx": dx,
        "area": area,
    }


def track_camera_target(rover, delta_x, delta_y):
    """将画面偏差转换为底盘的二维移动命令。"""
    delta_x = int(delta_x)
    delta_y = int(delta_y)

    if abs(delta_x) <= CAMERA_DEADZONE_X:
        delta_x = 0
    if abs(delta_y) <= CAMERA_DEADZONE_Y:
        delta_y = 0

    if delta_x == 0 and delta_y == 0:
        rover.stop()
        return 0.0, 0.0

    # delta_x > 0 表示方块在画面左侧，底盘需要向左移动，
    # 因此在底盘坐标中取 lateral < 0。delta_y > 0 表示向前。
    lateral = -clamp(delta_x / CAMERA_X_HALF_SIZE, -1.0, 1.0)
    forward = clamp(delta_y / CAMERA_Y_HALF_SIZE, -1.0, 1.0)
    magnitude = clamp(math.sqrt(lateral * lateral + forward * forward), 0.0, 1.0)

    steer_angle_deg = math.degrees(math.atan2(lateral, forward))
    speed_rad_s = max(
        CAMERA_TRACK_MIN_SPEED_RAD_S,
        magnitude * CAMERA_TRACK_MAX_SPEED_RAD_S,
    )
    # 接近中心框时限制轮速，避免单帧运动跨过整个 30px 捕获框后反向修正。
    if max(abs(delta_x), abs(delta_y)) <= CAMERA_TRACK_NEAR_GAP_PX:
        speed_rad_s = min(speed_rad_s, CAMERA_TRACK_NEAR_MAX_SPEED_RAD_S)

    # 转向舵机限制在 ±90°。目标在车后方时，改用负速度后退。
    if steer_angle_deg > 90.0:
        steer_angle_deg -= 180.0
        speed_rad_s = -speed_rad_s
    elif steer_angle_deg < -90.0:
        steer_angle_deg += 180.0
        speed_rad_s = -speed_rad_s

    rover.drive(speed_rad_s, steer_angle_deg)
    return speed_rad_s, steer_angle_deg


def follow_line_target(rover, line_dx):
    """将巡线水平偏差转换为前进和转向命令。"""
    steer_angle_deg = clamp(
        int(line_dx) * LINE_FOLLOW_STEER_KP,
        -LINE_FOLLOW_MAX_STEER_DEG,
        LINE_FOLLOW_MAX_STEER_DEG,
    )
    rover.drive(LINE_FOLLOW_BASE_SPEED_RAD_S, steer_angle_deg)
    return LINE_FOLLOW_BASE_SPEED_RAD_S, steer_angle_deg


def parse_qrcode_task(payload):
    """解析二维码任务，返回按数量展开后的抓取颜色队列。"""
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

    task_queue = []
    for color, count in zip(colors, counts):
        if count < 0:
            return None
        for _ in range(count):
            task_queue.append(color)
    return task_queue


def run_gripper_grab(rover):
    """闭合夹爪抓取方块。"""
    ok = rover.servo_control.set_reserve_servo_angle(
        GRIPPER_SERVO_ID,
        GRIPPER_CLOSED_ANGLE_DEG,
    )
    if not ok:
        print("夹爪舵机未启用，无法执行抓取。")
    return ok


def run_gripper_release(rover):
    """张开夹爪释放方块。"""
    ok = rover.servo_control.set_reserve_servo_angle(
        GRIPPER_SERVO_ID,
        GRIPPER_OPEN_ANGLE_DEG,
    )
    if not ok:
        print("夹爪舵机未启用，无法释放方块。")
    return ok


def reset_arm_after_place(rover):
    """放置后复位机械臂：先收 pitch1，再恢复完整初始位。"""
    rover.arm.move_joint_pose(
        rover.arm.roll_deg,
        -45.0,
        rover.arm.pitch2_deg,
        rover.arm.pitch3_deg,
    )
    time.sleep_ms(ARM_AUTO_ACTION_DELAY_MS)
    rover.arm.apply_initial_pose()


def execute_grab_place_task(rover, color):
    """执行单个方块的抓取、放置和复位流程。"""
    rover.stop()
    if rover.arm is None:
        print("机械臂未初始化，无法抓取 %s 方块。" % color)
        return False

    try:
        rover.servo_control.set_camera_angle(0.0)
        rover.arm.camera_angle_deg = 0.0
        time.sleep_ms(ARM_AUTO_ACTION_DELAY_MS)
        rover.arm.move_pitch123(
            ARM_GRAB_PITCH1_DEG,
            ARM_GRAB_PITCH2_DEG,
            ARM_GRAB_PITCH3_DEG,
        )
        time.sleep_ms(ARM_AUTO_ACTION_DELAY_MS)
        if not run_gripper_grab(rover):
            return False
        time.sleep_ms(ARM_AUTO_ACTION_DELAY_MS)
        rover.arm.move_pitch123(
            ARM_PLACE_PITCH1_DEG,
            ARM_PLACE_PITCH2_DEG,
            ARM_PLACE_PITCH3_DEG,
        )
        time.sleep_ms(ARM_AUTO_ACTION_DELAY_MS)
        if not run_gripper_release(rover):
            return False
        time.sleep_ms(ARM_AUTO_ACTION_DELAY_MS)
        reset_arm_after_place(rover)
        rover.servo_control.set_camera_angle(CAMERA_INIT_ANGLE_DEG)
        rover.arm.camera_angle_deg = CAMERA_INIT_ANGLE_DEG
        time.sleep_ms(ARM_AUTO_ACTION_DELAY_MS)
    except ArmKinematicsError as err:
        print("机械臂目标无效：%s，%s" % (err.reason, err.message))
        return False
    return True


def arm_debug_loop():
    rover.stop()
    current_gripper_angle = GRIPPER_OPEN_ANGLE_DEG
    print("RUN_MODE=debug，已完成上电复位。")
    print("输入格式：camera=<角度> roll=<角度> pitch1=<角度> pitch2=<角度> pitch3=<角度> gripper=<角度>")
    print("也可按顺序输入最多 6 个数：camera roll pitch1 pitch2 pitch3 gripper。输入 r 复位，q 退出。")

    while True:
        try:
            line = input("debug> ")
        except EOFError:
            print("输入结束，退出 debug 模式。")
            rover.stop()
            return

        text = line.strip()
        if text == "":
            continue
        command = text.lower()
        if command in ("q", "quit", "exit"):
            rover.stop()
            print("退出 debug 模式。")
            return
        if command in ("r", "reset"):
            reset_all_servos()
            rover.arm.roll_deg = ARM_INIT_ROLL_DEG
            rover.arm.pitch1_deg = ARM_INIT_PITCH1_DEG
            rover.arm.pitch2_deg = ARM_INIT_PITCH2_DEG
            rover.arm.pitch3_deg = ARM_INIT_PITCH3_DEG
            rover.arm.camera_angle_deg = CAMERA_INIT_ANGLE_DEG
            current_gripper_angle = GRIPPER_OPEN_ANGLE_DEG
            continue

        parts = text.replace(",", " ").split()
        values = {}
        ordered_names = ("camera", "roll", "pitch1", "pitch2", "pitch3", "gripper")
        try:
            for index, part in enumerate(parts):
                if "=" in part:
                    name, value = part.split("=", 1)
                    name = name.strip().lower()
                    if name not in ordered_names:
                        raise ValueError("unknown")
                    values[name] = float(value)
                else:
                    if index >= len(ordered_names):
                        raise ValueError("too_many")
                    values[ordered_names[index]] = float(part)
        except ValueError:
            print("格式错误。示例：camera=0 roll=0 pitch1=-50 pitch2=-110.6 pitch3=0 gripper=52.8")
            continue

        try:
            if "camera" in values:
                rover.servo_control.set_camera_angle(values["camera"])
                rover.arm.camera_angle_deg = values["camera"]

            target_roll = values.get("roll", rover.arm.roll_deg)
            target_pitch1 = values.get("pitch1", rover.arm.pitch1_deg)
            target_pitch2 = values.get("pitch2", rover.arm.pitch2_deg)
            target_pitch3 = values.get("pitch3", rover.arm.pitch3_deg)
            result = rover.arm.move_joint_pose(
                target_roll,
                target_pitch1,
                target_pitch2,
                target_pitch3,
            )

            if "gripper" in values:
                if not rover.servo_control.set_reserve_servo_angle(
                    GRIPPER_SERVO_ID,
                    values["gripper"],
                ):
                    print("夹爪舵机未启用，无法设置夹爪角度。")
                else:
                    current_gripper_angle = values["gripper"]
        except ArmKinematicsError as err:
            print("机械臂目标无效：%s，%s" % (err.reason, err.message))
            continue

        print(
            "已执行：Camera=%.2f deg, Roll=%.2f deg, Pitch1=%.2f deg, Pitch2=%.2f deg, Pitch3=%.2f deg, Gripper=%.2f deg"
            % (
                rover.arm.camera_angle_deg,
                result["roll_deg"],
                result["pitch1_deg"],
                result["pitch2_deg"],
                result["pitch3_deg"],
                current_gripper_angle,
            )
        )


def parse_multi_detect_frame(raw_data):
    if raw_data is None:
        return None
    if isinstance(raw_data, bytes):
        raw_data = raw_data.decode("utf-8", "replace")

    frames = str(raw_data).strip().splitlines()
    for frame in frames:
        parts = frame.strip().split()
        if len(parts) < 2 or parts[0] != "md":
            continue
        if parts[1] == "none":
            return {"found": False}
        if len(parts) < 4:
            continue
        try:
            return {
                "found": True,
                "color": parts[1],
                "dx": int(parts[2]),
                "dy": int(parts[3]),
            }
        except ValueError:
            continue
    return None


def request_multi_detect():
    return request_multi_detection_frame("multi_detect\n")


def request_multi_anchor_detect():
    return request_multi_detection_frame("multi_anchor\n")


def request_multi_detection_frame(command):
    global camera_data
    camera_data["value"] = None
    send_camera_command(camera_uart, command)
    start_ms = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start_ms) <= MULTI_DETECT_TIMEOUT_MS:
        if camera_data["value"] is not None:
            raw_data = camera_data["value"]
            camera_data["value"] = None
            detection = parse_multi_detect_frame(raw_data)
            if detection is not None:
                return detection
            print("multi：忽略非色块检测串口数据:", raw_data)
        time.sleep_ms(30)

    return None


def align_multi_left_bottom_anchor():
    """将 3x3 左下角色块中心对齐到画面中心红框。"""
    start_ms = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_ms) <= MULTI_ENTRY_ALIGN_TIMEOUT_MS:
        detection = request_multi_anchor_detect()
        if detection is None:
            rover.stop()
            print("multi：左下角基准色块检测超时。")
            time.sleep_ms(50)
            continue
        if not detection.get("found"):
            rover.stop()
            print("multi：未识别到 3x3 左下角基准色块。")
            time.sleep_ms(50)
            continue

        print(
            "multi：左下角基准色块 %s，dx=%d, dy=%d。"
            % (detection["color"], detection["dx"], detection["dy"])
        )
        if multi_detection_centered(detection):
            rover.stop()
            rover.center_chassis_servos()
            print("multi：已对齐 3x3 左下角基准色块，当前位置作为第一列起点。")
            return True

        # md 的 dx/dy 是 blob_center - target_center；track_camera_target 使用 target - blob。
        track_camera_target(rover, -detection["dx"], -detection["dy"])
        time.sleep_ms(80)

    rover.stop()
    print("multi：左下角基准色块对齐超时。")
    return False


def align_multi_current_target_color(target_color):
    """命中目标列后，将当前目标色块中心再次对齐到画面中心红框。"""
    start_ms = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_ms) <= MULTI_ENTRY_ALIGN_TIMEOUT_MS:
        detection = request_multi_detect()
        if detection is None:
            rover.stop()
            print("multi：目标色块 %s 微调检测超时。" % target_color)
            time.sleep_ms(50)
            continue
        if not detection.get("found"):
            rover.stop()
            print("multi：目标色块 %s 微调时未识别到色块。" % target_color)
            time.sleep_ms(50)
            continue
        if detection["color"] != target_color:
            rover.stop()
            print(
                "multi：微调时中心最近色块从 %s 变为 %s，停止本次抓取。"
                % (target_color, detection["color"])
            )
            return False

        print(
            "multi：目标色块 %s 微调，dx=%d, dy=%d。"
            % (target_color, detection["dx"], detection["dy"])
        )
        if multi_detection_centered(detection):
            rover.stop()
            rover.center_chassis_servos()
            print("multi：目标色块 %s 已再次对齐。" % target_color)
            return True

        # md 的 dx/dy 是 blob_center - target_center；track_camera_target 使用 target - blob。
        track_camera_target(rover, -detection["dx"], -detection["dy"])
        time.sleep_ms(80)

    rover.stop()
    print("multi：目标色块 %s 微调对齐超时。" % target_color)
    return False


def multi_detection_centered(detection):
    return (
        detection is not None
        and detection.get("found")
        and abs(detection["dx"]) <= MULTI_CENTER_TOLERANCE_PX
        and abs(detection["dy"]) <= MULTI_CENTER_TOLERANCE_PX
    )


def multi_move_horizontal(direction_steps):
    if direction_steps == 0:
        return
    direction = 1 if direction_steps > 0 else -1
    for _ in range(abs(int(direction_steps))):
        rover.drive(
            MULTI_HORIZONTAL_SPEED_RAD_S * direction,
            MULTI_HORIZONTAL_STEER_DEG,
        )
        time.sleep_ms(MULTI_COLUMN_MOVE_MS)
        rover.stop()
        time.sleep_ms(150)


def set_multi_camera_angle(angle_deg):
    rover.servo_control.set_camera_angle(angle_deg)
    rover.arm.camera_angle_deg = angle_deg


def reset_multi_action_servos(camera_angle_deg=CAMERA_INIT_ANGLE_DEG):
    """单次 multi 抓放动作结束后恢复动作相关舵机。"""
    rover.center_chassis_servos()
    reset_arm_after_place(rover)
    set_multi_camera_angle(camera_angle_deg)
    run_gripper_release(rover)


def execute_multi_single_grab_place_placeholder(
    color,
    column_index,
    row_index,
    place_index,
):
    rover.stop()
    if row_index >= len(MULTI_GRAB_ROW_POSES):
        print("multi：%s 已超过三行可抓数量。" % color)
        return False
    if place_index >= len(MULTI_PLACE_POSES):
        print("multi：放置序号 %d 超出已配置的 6 个放置姿态。" % (place_index + 1))
        return False

    grab_pose_index = len(MULTI_GRAB_ROW_POSES) - 1 - row_index
    grab_roll, grab_pitch1, grab_pitch2, grab_pitch3 = MULTI_GRAB_ROW_POSES[grab_pose_index]
    place_roll, place_pitch1, place_pitch2, place_pitch3 = MULTI_PLACE_POSES[place_index]
    print(
        "multi：命中目标色块 %s，列=%d，行=%d，放置序号=%d。"
        % (color, column_index + 1, grab_pose_index + 1, place_index + 1)
    )

    try:
        set_multi_camera_angle(CAMERA_INIT_ANGLE_DEG)
        time.sleep_ms(ARM_AUTO_ACTION_DELAY_MS)

        rover.arm.move_joint_pose(
            grab_roll,
            grab_pitch1,
            grab_pitch2,
            grab_pitch3,
        )
        time.sleep_ms(ARM_AUTO_ACTION_DELAY_MS)

        if not run_gripper_grab(rover):
            return False
        time.sleep_ms(ARM_AUTO_ACTION_DELAY_MS)

        rover.arm.move_joint_pose(
            place_roll,
            place_pitch1,
            place_pitch2,
            place_pitch3,
        )
        time.sleep_ms(ARM_AUTO_ACTION_DELAY_MS)
        if not run_gripper_release(rover):
            return False
        time.sleep_ms(ARM_AUTO_ACTION_DELAY_MS)
    except ArmKinematicsError as err:
        print("multi：机械臂目标无效：%s，%s" % (err.reason, err.message))
        return False
    finally:
        reset_multi_action_servos()
        time.sleep_ms(ARM_AUTO_ACTION_DELAY_MS)

    return True


def execute_multi_grab_placeholder(task_queue):
    rover.stop()
    print("multi：开始执行三列搜索占位逻辑，任务队列:", task_queue)
    set_multi_camera_angle(CAMERA_L3_ANGLE_DEG)
    time.sleep_ms(200)
    try:
        if not align_multi_left_bottom_anchor():
            return False

        current_column = 0
        grabbed_count_by_color = {}
        for task_index, target_color in enumerate(task_queue):
            row_index = grabbed_count_by_color.get(target_color, 0)
            print("multi：开始寻找第 %d 个任务目标: %s" % (task_index + 1, target_color))
            if current_column != 0:
                multi_move_horizontal(-current_column)
                current_column = 0

            matched = False
            for column_index in range(MULTI_COLUMN_COUNT):
                current_column = column_index
                detection = request_multi_detect()
                if detection is None:
                    print("multi：第 %d 列检测超时。" % (column_index + 1))
                elif not detection.get("found"):
                    print("multi：第 %d 列未识别到当前列基准色块。" % (column_index + 1))
                else:
                    print(
                        "multi：第 %d 列识别到 %s，dx=%d, dy=%d。"
                        % (
                            column_index + 1,
                            detection["color"],
                            detection["dx"],
                            detection["dy"],
                        )
                    )
                    if detection["color"] == target_color:
                        matched = True
                        if not align_multi_current_target_color(target_color):
                            return False
                        if not execute_multi_single_grab_place_placeholder(
                            target_color,
                            column_index,
                            row_index,
                            task_index,
                        ):
                            return False
                        grabbed_count_by_color[target_color] = row_index + 1
                        if task_index < len(task_queue) - 1:
                            set_multi_camera_angle(CAMERA_L3_ANGLE_DEG)
                            time.sleep_ms(200)
                        break

                if column_index < MULTI_COLUMN_COUNT - 1:
                    multi_move_horizontal(1)
                    current_column += 1

            if not matched:
                print("multi：三列中没有找到目标颜色:", target_color)
                return False

        return True
    finally:
        set_multi_camera_angle(CAMERA_INIT_ANGLE_DEG)


def drive_rover_from_ps2_snapshot(ps2, buttons, lx, rx, ry):
    if button_pressed(buttons, ps2.PS2_BTN_R2):
        turn = map_joystick(rx)
        turn_speed = turn / 100.0 * _MULTI_MAX_PIVOT_RAD_S
        if not rover.motors_enabled and abs(turn_speed) > 0.01:
            rover.stop()
            print("multi：电机未使能，无法原地转向。")
            return
        rover.pivot_turn(turn_speed)
        return

    throttle = -map_joystick(ry)
    steer = map_joystick(lx)
    speed_rad_s = throttle / 100.0 * _MULTI_MAX_MOTOR_RAD_S
    steer_angle_deg = steer / 100.0 * MAX_STEER_ANGLE_DEG
    if (
        not rover.motors_enabled
        and (abs(speed_rad_s) > 0.01 or abs(steer_angle_deg) > 0.1)
    ):
        rover.stop()
        print("multi：电机未使能，无法行驶。")
        return
    rover.drive(speed_rad_s, steer_angle_deg)


def parse_multi_camera_task(raw_data):
    if raw_data is None:
        return None
    if isinstance(raw_data, bytes):
        raw_data = raw_data.decode("utf-8", "replace")

    frames = str(raw_data).strip().splitlines()
    for frame in frames:
        frame = frame.strip()
        if frame == "" or frame.startswith("sx") or frame.startswith("ln"):
            continue
        parsed_task = parse_qrcode_task(frame)
        if parsed_task is not None:
            return parsed_task
    return None


def multi_loop(ps2):
    global camera_data
    rover.prepare()
    print(
        "RUN_MODE=multi：二维码任务由视觉端 sekuai.py 自动扫描并发送；"
        "用 PS2 驾驶到抓取位置并停车，按 %s 启动抓取/放置逻辑。"
        % _MULTI_EXECUTE_BUTTON_NAME
    )

    task_queue = []
    task_loaded = False
    prev_buttons = 0

    while True:
        ps2.update()
        fresh, buttons, lx, ly, rx, ry, _ = ps2.snapshot()
        if not fresh:
            rover.stop()
            time.sleep_ms(50)
            continue

        execute_pressed = (
            button_pressed(buttons, ps2.PS2_BTN_R3)
            and not button_pressed(prev_buttons, ps2.PS2_BTN_R3)
        )
        camera_l3_pressed = (
            button_pressed(buttons, ps2.PS2_BTN_L3)
            and not button_pressed(prev_buttons, ps2.PS2_BTN_L3)
        )
        prev_buttons = buttons

        if not task_loaded and camera_data["value"] is not None:
            raw_data = camera_data["value"]
            camera_data["value"] = None
            parsed_task = parse_multi_camera_task(raw_data)
            if parsed_task is None:
                print("multi：等待 sekuai.py 自动二维码任务，忽略串口数据:", raw_data)
            else:
                task_queue = parsed_task
                task_loaded = True
                send_camera_command(camera_uart, "ok\n")
                print("multi：二维码任务加载完成，抓取队列:", task_queue)
            time.sleep_ms(50)
            continue

        if button_pressed(buttons, ps2.PS2_BTN_SELECT):
            rover.stop()
            print("multi：SELECT 退出。")
            return

        if button_pressed(buttons, ps2.PS2_BTN_R1):
            rover.stop()
            time.sleep_ms(50)
            continue

        if camera_l3_pressed:
            rover.servo_control.set_camera_angle(CAMERA_L3_ANGLE_DEG)
            rover.arm.camera_angle_deg = CAMERA_L3_ANGLE_DEG
            print("multi：L3，相机转到 %.1f°。" % CAMERA_L3_ANGLE_DEG)
            time.sleep_ms(200)
            continue

        if execute_pressed:
            rover.stop()
            if not task_loaded:
                print("multi：还没有二维码任务，请等待 sekuai.py 识别二维码并自动发送任务。")
            else:
                execute_multi_grab_placeholder(task_queue)
            time.sleep_ms(200)
            continue

        drive_rover_from_ps2_snapshot(ps2, buttons, lx, rx, ry)
        time.sleep_ms(50)


def send_camera_command(serial, command):
    try:
        serial.write(command)
    except TypeError:
        serial.write(command.encode("utf-8"))

# 硬件配置与初始化。
# 舵机。
servo_uart = UART(
    SERVO_UART_ID,
    SERVO_UART_BAUD,
    tx=SERVO_UART_TX,
    rx=SERVO_UART_RX,
    timeout=64,
)

camera_uart = UART(
    CAMERA_UART_ID,
    CAMERA_UART_BAUD,
    tx=CAMERA_UART_TX,
    rx=CAMERA_UART_RX,
    timeout=64,
)

try:
    can = CAN(
        CAN_BUS_ID,
        mode=CAN.NORMAL,
        baudrate=CAN_BAUDRATE,
        tx=CAN_TX,
        rx=CAN_RX,
    )
except Exception:
    print("CAN硬件占用。触发系统级软复位，别慌，请点击STOP重新连接")
    time.sleep(1)
    machine.reset()
can.clear_rx_queue()


# 相机串口通信线程，储存相机传输的数据。
def re_uart(uart):
    global camera_data, camera_uart
    try:
        while True:
            if uart.any() and uart == camera_uart:
                data = uart.read()
                text = data.decode("utf-8", "replace")
                if camera_data["value"] is None:
                    camera_data["value"] = text
                else:
                    camera_data["value"] = (
                        str(camera_data["value"]) + text
                    )[-_CAMERA_DATA_BUFFER_MAX_CHARS:]
                print("串口1收到数据:", camera_data["value"])
            time.sleep_ms(10)  # 防止形成阻塞。
    except UnicodeError:
        print("【成功拦截乱码】串口1收到一串无法识别的非文本数据:")
        pass
        _thread.start_new_thread(re_uart, (uart,))


motor_bus = MotorBus(can)
servo_bus = ServoBus(servo_uart)
servo_bus.reset_turns_polling(get_all_servo_ids())  # 清除多圈。
servo_bus.lock_all(get_all_servo_ids())  # 舵机锁力。
servo_control = ServoControl(servo_bus)
arm = RobotArm(servo_control)  # 基于运动学的舵机控制接口。
rover = LunarRover(motor_bus, servo_control, arm=arm)


def reset_all_servos():
    """上电后将底盘、机械臂、相机及已启用的预留舵机恢复初始位置。"""
    print("正在执行所有舵机复位，请保持机械结构周围无障碍物。")
    rover.center_chassis_servos()
    rover.arm.apply_initial_pose()
    rover.servo_control.set_camera_angle(CAMERA_INIT_ANGLE_DEG)
    rover.servo_control.init_reserve_servos()
    # API 说明书要求给机械臂足够的动作时间，避免后续指令提前覆盖复位命令。
    time.sleep(2)
    print("所有舵机复位完成。")


# 必须先完成舵机复位，再启动串口线程和其他控制逻辑。
reset_all_servos()

# 打开多线程。
_thread.start_new_thread(re_uart, (camera_uart,))


def main():
    global camera_data  # 解析的串口数据。
    try:
        if RUN_MODE == "debug":
            arm_debug_loop()
            return

        if RUN_MODE == "ps2":
            try:
                rover.prepare()
            finally:
                pass
            ps2_controller = PS2Controller(di=PS2_DI, do=PS2_DO, cs=PS2_CS, clk=PS2_CLK)
            ps2_controller.init_vibration()
            ps2 = PS2Receiver(ps2_controller, 30, True)
            ps2.start()
            try:
                ps2_loop(rover, ps2, camera_data, camera_uart)
            finally:
                ps2.stop()
                rover.disable()
            return

        if RUN_MODE == "multi":
            ps2_controller = PS2Controller(di=PS2_DI, do=PS2_DO, cs=PS2_CS, clk=PS2_CLK)
            ps2_controller.init_vibration()
            ps2 = PS2Receiver(ps2_controller, 30, True)
            ps2.start()
            try:
                multi_loop(ps2)
            finally:
                ps2.stop()
                rover.disable()
            return

        if RUN_MODE == "line_follow":
            rover.prepare()
            print("RUN_MODE=line_follow，等待视觉端巡线数据。")
            line_follow_loop()
            return

        rover.prepare()  # 初始化底盘并使能电机。
        print("RUN_MODE=idle，电机已默认使能。学生可在示例区编写一次性控制程序。")

        # ================= 学生控制示例 =================
        # 使用方法：每次只取消一小段示例代码的注释，确认安全后再运行。
        # 注意：调试底盘前建议先架空车轮，避免小车突然运动。

        # 示例 1：底盘以 2.0 rad/s 前进 1 秒，然后停车。
        # print("示例 1：底盘以 2.0 rad/s 前进 1 秒，然后停车。")
        # rover.drive(speed_rad_s=2.0, steer_angle_deg=0.0)
        # time.sleep(1)  # 必须延迟，让指令有执行时间，避免指令被立即覆盖。
        # rover.stop()
        # time.sleep(1)

        # 示例 2：底盘以 2.0 rad/s、20 度转向角前进 1 秒，然后停车。
        # print("示例 2：底盘以 2.0 rad/s、20 度转向角前进 1 秒，然后停车。")
        # rover.drive(speed_rad_s=2.0, steer_angle_deg=20.0)
        # time.sleep(1)
        # rover.stop()
        # time.sleep(1)

        # 示例 3：相机转到 -30 度，再回到 0 度。
        # print("示例 3：相机转到 -30 度，再回到 0 度。")
        # rover.servo_control.set_camera_angle(-30)
        # time.sleep(1)
        # rover.servo_control.set_camera_angle(0)
        # time.sleep(1)

        # 示例 4：机械臂回初始位。
        # print("示例 4：机械臂回初始位。")
        # rover.arm.apply_initial_pose()
        # time.sleep(2)  # 给机械臂足够时间回初始位，避免后续指令提前覆盖。

        # 示例 5：机械臂单关节点动，分别控制 Roll、Pitch1、Pitch2、Pitch3。
        # 如果前面运行过绝对角度控制，先同步真实舵机角度，再做 jog 增量控制。
        # print("示例 5：机械臂单关节点动，分别控制 Roll、Pitch1、Pitch2、Pitch3。")
        # rover.arm.sync_from_servos()
        # rover.arm.jog_joints(roll_delta_deg=2.0)
        # time.sleep(1)
        # rover.arm.jog_joints(roll_delta_deg=-2.0)
        # time.sleep(1)
        # rover.arm.jog_joints(pitch1_delta_deg=2.0)
        # time.sleep(1)
        # rover.arm.jog_joints(pitch1_delta_deg=-2.0)
        # time.sleep(1)
        # rover.arm.jog_joints(pitch2_delta_deg=2.0)
        # time.sleep(1)
        # rover.arm.jog_joints(pitch2_delta_deg=-2.0)
        # time.sleep(1)
        # rover.arm.jog_joints(pitch3_delta_deg=2.0)
        # time.sleep(1)
        # rover.arm.jog_joints(pitch3_delta_deg=-2.0)
        # time.sleep(1)

        # 示例 6：相机舵机点动，再转回。
        # 如果前面运行过相机绝对角度控制，先同步真实相机角度，再做 jog 增量控制。
        # print("示例 6：相机舵机点动，再转回。")
        # rover.arm.sync_camera_from_servo()  # 使用 jog 控制前，先同步一次当前舵机角度。
        # rover.arm.jog_camera(8.0)
        # time.sleep(1)
        # rover.arm.jog_camera(-8.0)
        # time.sleep(1)

        # 示例 7：四个驱动电机分别设置不同转速，运行 1 秒后停车。
        # 注意：这是直接控制电机，不会自动调整转向舵机角度。
        # print("示例 7：四个驱动电机分别设置不同转速，运行 1 秒后停车。")
        # motor_speeds = (
        #     (1, 1.0),  # 左前电机 ID=1，速度 1.0 rad/s。
        #     (2, 0.5),  # 右前电机 ID=2，速度 0.5 rad/s。
        #     (3, -0.5),  # 左后电机 ID=3，速度 -0.5 rad/s。
        #     (4, -1.0),  # 右后电机 ID=4，速度 -1.0 rad/s。
        # )
        # for motor_id, speed_rad_s in motor_speeds:
        #     rover.motor_bus.set_acc(motor_id, DEFAULT_ACC_RAD_S2)
        #     rover.motor_bus.set_speed(motor_id, speed_rad_s)
        # time.sleep(1)
        # rover.motor_bus.stop_all(rover.motor_ids)
        # time.sleep(1)

        # 示例 8：六个转向舵机分别设置不同角度。
        # 参数顺序：左前、左中、左后、右前、右中、右后。
        # print("示例 8：六个转向舵机分别设置不同角度。")
        # rover.servo_control.set_steering_angles(
        #     20.0, 0.0, -20.0,
        #     -20.0, 0.0, 20.0,
        # )
        # time.sleep(1)
        # rover.center_chassis_servos()
        # time.sleep(1)

        # 示例 9：直接设置机械臂四个关节的目标角度。
        # 参数顺序：Roll、Pitch1、Pitch2、Pitch3。
        # 这是直接舵机角度控制，适合做固定姿态演示。
        # print("示例 9：直接设置机械臂四个关节的目标角度。")
        # rover.servo_control.set_arm_joint_angles(
        #     0.0, 50.0, -140.0, 0.0,
        # )
        # time.sleep(1)

        # 示例 10：读取机械臂四个关节角度。
        # print("示例 10：读取机械臂四个关节角度。")
        # print(rover.servo_control.read_arm_joint_angles())
        # time.sleep(1)

        # 示例 11：预留舵机测试。
        # 需先在 robot_config.py 设置 RESERVE_SERVO_ENABLED = True，并在 RESERVE_SERVO_IDS 中填写 ID。
        # print("示例 11：预留舵机测试。")
        # if not RESERVE_SERVO_ENABLED:
        #     print("预留舵机未启用，请先在 robot_config.py 设置 RESERVE_SERVO_ENABLED = True。")
        # else:
        #     reserve_id = RESERVE_SERVO_IDS[0]
        #     rover.servo_control.set_reserve_servo_angle(reserve_id, 30.0)
        #     time.sleep(1)
        #     angle = rover.servo_control.read_reserve_servo_angle(reserve_id)
        #     print("预留舵机 ID=%d 当前角度：" % reserve_id, angle)
        #     rover.servo_control.set_reserve_servo_angle(reserve_id, -2.0)
        #     time.sleep(1)

        # 示例 12：底盘以 1.0 rad/s 原地右转 1 秒，然后停车。
        # 原地转向：负数左转，正数右转。
        # print("示例 12：底盘以 1.0 rad/s 原地右转 1 秒，然后停车。")
        # rover.pivot_turn(speed_rad_s=1.0)
        # time.sleep(1)
        # rover.stop()
        # time.sleep(1)
    except Exception as e:
        print("错误代码：", e)

    last_camera_data_ms = time.ticks_ms()
    camera_motion_active = False
    grab_queue = []
    current_grab_color = None
    while True:
        if camera_data["value"] is not None:
            raw_data = camera_data["value"]
            camera_data["value"] = None
            if isinstance(raw_data, bytes):
                raw_data = raw_data.decode("utf-8", "replace")
            print(raw_data)

            frames = str(raw_data).strip().splitlines()
            for frame in frames:
                frame = frame.strip()
                if frame == "":
                    continue

                if frame.startswith("sx"):
                    offset = parse_camera_offset(frame)
                    if offset is None:
                        print("视觉偏差格式错误，已忽略:", frame)
                        continue
                    if current_grab_color is None and grab_queue:
                        current_grab_color = grab_queue[0]
                        print("当前抓取目标:", current_grab_color)
                    if current_grab_color is None:
                        rover.stop()
                        camera_motion_active = False
                        continue

                    delta_x, delta_y = offset
                    speed, steer = track_camera_target(rover, delta_x, delta_y)
                    last_camera_data_ms = time.ticks_ms()
                    camera_motion_active = speed != 0.0
                    print(
                        "视觉跟踪 %s: dx=%d, dy=%d, speed=%.2f rad/s, steer=%.1f deg"
                        % (current_grab_color, delta_x, delta_y, speed, steer)
                    )
                    if delta_x == 0 and delta_y == 0:
                        camera_motion_active = False
                        if execute_grab_place_task(rover, current_grab_color):
                            finished = grab_queue.pop(0)
                            print("完成抓取:", finished)
                            current_grab_color = grab_queue[0] if grab_queue else None
                            send_camera_command(camera_uart, "next\n")
                            if current_grab_color is None:
                                print("二维码抓取任务全部完成。")
                        else:
                            rover.stop()
                    continue

                parsed_task = parse_qrcode_task(frame)
                if parsed_task is None:
                    print("串口数据无法识别，已忽略:", frame)
                    continue

                grab_queue = parsed_task
                current_grab_color = grab_queue[0] if grab_queue else None
                camera_motion_active = False
                send_camera_command(camera_uart, "ok\n")
                print("二维码任务加载完成，抓取队列:", grab_queue)
        elif (
            camera_motion_active
            and time.ticks_diff(time.ticks_ms(), last_camera_data_ms)
            > CAMERA_DATA_TIMEOUT_MS
        ):
            print("视觉数据超时，已停车")
            rover.stop()
            camera_motion_active = False

        time.sleep_ms(100)


def line_follow_loop():
    global camera_data
    last_line_data_ms = time.ticks_ms()
    line_motion_active = False

    while True:
        if camera_data["value"] is not None:
            raw_data = camera_data["value"]
            camera_data["value"] = None
            if isinstance(raw_data, bytes):
                raw_data = raw_data.decode("utf-8", "replace")

            frames = str(raw_data).strip().splitlines()
            for frame in frames:
                frame = frame.strip()
                if frame == "":
                    continue

                line_info = parse_line_follow_frame(frame)
                if line_info is None:
                    print("非巡线视觉数据，已忽略:", frame)
                    continue

                last_line_data_ms = time.ticks_ms()
                if line_info["lost"]:
                    rover.stop()
                    line_motion_active = False
                    print("巡线丢失，已停车")
                    continue

                speed, steer = follow_line_target(rover, line_info["dx"])
                line_motion_active = True
                print(
                    "巡线: dx=%d, area=%d, speed=%.2f rad/s, steer=%.1f deg"
                    % (line_info["dx"], line_info["area"], speed, steer)
                )
        elif (
            line_motion_active
            and time.ticks_diff(time.ticks_ms(), last_line_data_ms)
            > LINE_FOLLOW_DATA_TIMEOUT_MS
        ):
            print("巡线视觉数据超时，已停车")
            rover.stop()
            line_motion_active = False

        time.sleep_ms(50)


if __name__ == "__main__":
    main()
         
         