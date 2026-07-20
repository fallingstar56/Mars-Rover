"""
PS2 遥控业务控制逻辑。

本文件负责把 PS2 按键和摇杆映射到底盘、相机和机械臂动作。
ps2_lib.py 只负责手柄底层读取和安全接收。

作者 王笑
日期 20260528
"""

import math
import time

from arm_control import ArmKinematicsError
from robot_config import (
    MAX_MOTOR_RPM,
    MAX_STEER_ANGLE_DEG,
    PIVOT_SPEED_SCALE,
    clamp,
)

_MAX_MOTOR_RAD_S = MAX_MOTOR_RPM * 2.0 * math.pi / 60.0
_MAX_PIVOT_RAD_S = _MAX_MOTOR_RAD_S * PIVOT_SPEED_SCALE
_ARM_JOG_COMMAND_DELAY_MS = 50
_ARM_JOG_STEP_DEG = 8
_last_arm_error_key = None
_last_arm_error_ms = 0


# ==============================================================================
# 核心摇杆数据处理函数
# ==============================================================================


def map_joystick(raw_val, center=128, deadzone=12):
    """
    【摇杆数据映射核心】
    将摇杆的原始 ADC 数据 (通常为 0-255) 转换为 -100 到 100 的百分比数值。

    参数说明:
    - raw_val: 手柄底层读取到的原始摇杆数据 (0~255)
    - center: 摇杆的物理中位值 (默认128)
    - deadzone: 死区范围，摇杆在这个范围内的微小偏移会被忽略，防止摇杆回中不良导致漂移
    """
    # 计算偏移量，例如 128 - 128 = 0，255 - 128 = 127。
    offset = int(raw_val) - center

    # 死区过滤：如果偏移量在死区范围内，说明没有有效拨动，直接返回 0。
    if abs(offset) <= deadzone:
        return 0

    # 确定方向：正向推为 1，反向拉为 -1。
    sign = 1 if offset > 0 else -1

    # 计算有效活动区间，例如 127 - 12 = 115。
    active_range = 127.0 - deadzone

    # 扣除死区后，将实际偏移量映射到 0~100 的百分比，并附加方向符号。
    mapped = int(((abs(offset) - deadzone) / active_range) * 100.0) * sign

    # 安全限制：确保最终输出严格在 -100 到 100 之间。
    return clamp(mapped, -100, 100)


def small_motion(value, threshold):
    # 过滤微小动作，如果计算出的运动增量小于设定阈值，则直接归零。
    return 0.0 if abs(value) < threshold else value


def button_pressed(data, btn):
    # 通过位与运算判断底层复合数据中，某个特定按键是否被按下。
    return (data & btn) == btn


def ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def ticks_diff(a, b):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(a, b)
    return a - b


def print_arm_error(err):
    # 打印机械臂错误，并进行防刷屏处理：1 秒内相同错误只报一次。
    global _last_arm_error_key, _last_arm_error_ms
    now_ms = ticks_ms()
    key = (err.reason, err.message)
    if key == _last_arm_error_key and ticks_diff(now_ms, _last_arm_error_ms) < 1000:
        return
    _last_arm_error_key = key
    _last_arm_error_ms = now_ms
    print("机械臂目标无效：%s，%s" % (err.reason, err.message))


def sync_arm_control_state(rover):
    if rover.arm is None:
        return True
    try:
        rover.arm.sync_from_servos()
        rover.arm.sync_camera_from_servo()
    except ArmKinematicsError as err:
        print_arm_error(err)
        return False
    return True


# ==============================================================================
# 机械臂控制模式（处理传进来的摇杆数据）
# ==============================================================================


def handle_arm_control(rover, ps2, buttons, lx, ly, rx, ry):
    if rover.arm is None:
        return

    # O 键：复位机械臂及相机角度。
    if button_pressed(buttons, ps2.PS2_BTN_CIRCLE):
        try:
            rover.arm.apply_initial_pose()
        except ArmKinematicsError as err:
            print_arm_error(err)
        try:
            rover.arm.jog_camera(-rover.arm.camera_angle_deg)
        except ArmKinematicsError as err:
            print_arm_error(err)
        return

    # 十字键左右：微调相机角度。
    if button_pressed(buttons, ps2.PS2_BTN_LEFT):
        try:
            rover.arm.jog_camera(_ARM_JOG_STEP_DEG)
        except ArmKinematicsError as err:
            print_arm_error(err)
    if button_pressed(buttons, ps2.PS2_BTN_RIGHT):
        try:
            rover.arm.jog_camera(-_ARM_JOG_STEP_DEG)
        except ArmKinematicsError as err:
            print_arm_error(err)

    # 摇杆数据读取与映射：机械臂模式。
    # 机械臂模式的死区设为 20（比底盘严格），避免从底盘切换时发生误碰。
    left_y = map_joystick(ly, deadzone=20)
    # 取反适配机械臂 Roll 轴坐标系。
    right_x = -map_joystick(rx, deadzone=20)
    right_y = map_joystick(ry, deadzone=20)

    roll_delta = 0.0
    pitch1_delta = 0.0
    pitch2_delta = 0.0
    pitch3_delta = 0.0

    # 将百分比 (-100~100) 转化为角度增量。
    # / 100.0 将其变为 -1.0 到 1.0 的比例系数，再乘以最大步进角度。
    if right_x != 0:
        roll_delta = right_x / 100.0 * _ARM_JOG_STEP_DEG
    if right_y != 0:
        pitch1_delta = right_y / 100.0 * _ARM_JOG_STEP_DEG
    if left_y != 0:
        pitch2_delta = left_y / 100.0 * _ARM_JOG_STEP_DEG

    # 十字键上下：按照固定步进修改 Pitch3。
    # if button_pressed(buttons, ps2.PS2_BTN_UP):
    #    pitch3_delta += _ARM_JOG_STEP_DEG
    if button_pressed(buttons, ps2.PS2_BTN_DOWN):
        pitch3_delta -= _ARM_JOG_STEP_DEG

    # 使用 small_motion 过滤掉极微小变化（小于 0.5 度的杂波）。
    roll_delta = small_motion(roll_delta, 0.5)
    pitch1_delta = small_motion(pitch1_delta, 0.5)
    pitch2_delta = small_motion(pitch2_delta, 0.5)
    pitch3_delta = small_motion(pitch3_delta, 0.5)

    if (
        roll_delta != 0.0
        or pitch1_delta != 0.0
        or pitch2_delta != 0.0
        or pitch3_delta != 0.0
    ):
        try:
            rover.arm.jog_joints(
                roll_delta,
                pitch1_delta,
                pitch2_delta,
                pitch3_delta,
            )
        except ArmKinematicsError as err:
            print_arm_error(err)


# ==============================================================================
# 主循环控制：演示如何从底层获取摇杆信息
# ==============================================================================


def ps2_loop(rover, ps2, data, serial):
    print(
        "PS2 控制：X失能，三角使能，R1停车，R2+右摇杆左右原地转向，"
        "L2+O机械臂回初始位并相机回0，L2+方向键左右控制相机，"
        "上下控制Pitch3，L2+左摇杆前后控制Pitch2，右摇杆前后控制Pitch1，"
        "右摇杆左右控制Roll。"
    )
    arm_mode_active = False

    while True:
        # 触发底层更新：要求底层库发起一次 SPI 通信，读取手柄当前状态。
        ps2.update()
        # 获取摇杆信息的关键快照。
        # snapshot() 返回一个包含当前帧所有手柄原始数据的元组
        # fresh: 数据是否有效/最新 (布尔值)
        # buttons: 按键状态码
        # lx: 左摇杆 X 轴原始数据 (0-255)
        # ly: 左摇杆 Y 轴原始数据 (0-255)
        # rx: 右摇杆 X 轴原始数据 (0-255)
        # ry: 右摇杆 Y 轴原始数据 (0-255)
        fresh, buttons, lx, ly, rx, ry, _ = ps2.snapshot()
        #   if button_pressed(buttons, ps2.按键定义名称):
        #   按键定义名称如下：
        #   self.PS2_BTN_SELECT
        #   self.PS2_BTN_L3
        #   self.PS2_BTN_R3
        #   self.PS2_BTN_START
        #   self.PS2_BTN_UP
        #   self.PS2_BTN_RIGHT
        #   self.PS2_BTN_DOWN
        #   self.PS2_BTN_LEFT
        #   self.PS2_BTN_L2
        #   self.PS2_BTN_R2
        #   self.PS2_BTN_L1
        #   self.PS2_BTN_R1
        #   self.PS2_BTN_TRIANGLE
        #   self.PS2_BTN_CIRCLE
        #   self.PS2_BTN_CROSS
        #   self.PS2_BTN_SQUARE
        # 如果获取数据失败（手柄断开或通讯异常），停止动作并重新尝试获取。
        if not fresh:
            rover.stop()
            arm_mode_active = False
            continue

        if button_pressed(buttons, ps2.PS2_BTN_SELECT):
            rover.stop()
            print("SELECT：退出 PS2 控制。")
            break

        if button_pressed(buttons, ps2.PS2_BTN_R1):
            rover.stop()
            arm_mode_active = False
            time.sleep_ms(100)
            continue

        if button_pressed(buttons, ps2.PS2_BTN_CROSS):
            rover.disable()
            arm_mode_active = False
            time.sleep_ms(200)
            continue

        if button_pressed(buttons, ps2.PS2_BTN_TRIANGLE):
            rover.enable_motors()
            arm_mode_active = False
            time.sleep_ms(200)
            continue
        # 按下 UP 到指定位置，然后回到原位。
        if button_pressed(buttons, ps2.PS2_BTN_UP):
            rover.arm.apply_initial_pose()
            time.sleep_ms(3000)

            rover.servo_control.set_arm_joint_angles(
                0.0,
                0.0,
                0.0,
                0.0,
            )
            time.sleep_ms(3000)

            rover.arm.apply_initial_pose()
            time.sleep_ms(3000)
            continue
        # L2 按住时，进入机械臂模式。
        if button_pressed(buttons, ps2.PS2_BTN_L2):
            if not arm_mode_active:
                rover.stop()
                if not sync_arm_control_state(rover):
                    time.sleep_ms(_ARM_JOG_COMMAND_DELAY_MS)
                    continue
                arm_mode_active = True
            # 分配摇杆数据。
            # 将刚刚获取到的 lx, ly, rx, ry 的 0-255 原始值直接透传给机械臂处理逻辑。
            handle_arm_control(rover, ps2, buttons, lx, ly, rx, ry)
            time.sleep_ms(_ARM_JOG_COMMAND_DELAY_MS)
            continue

        arm_mode_active = False

        # R2 按住时，将右摇杆信息分配给底盘进行原地转向
        if button_pressed(buttons, ps2.PS2_BTN_R2):
            # 将 0-255 的 rx（右摇杆 X 轴）转换为 -100~100 的数值。
            turn = map_joystick(rx)
            # 计算目标旋转角速度：摇杆百分比 * 原地旋转最大角速度限制。
            turn_speed = turn / 100.0 * _MAX_PIVOT_RAD_S
            rover.pivot_turn(turn_speed)
            time.sleep_ms(50)
            continue

        # if button_pressed(buttons, ps2.PS2_BTN_UP):
        #    rover.drive_to_position(1.0, 0.0)
        #    time.sleep_ms(300)
        #    rover.drive_to_position(0.0, 0.0)
        #    time.sleep_ms(300)
        #    continue

        # 常规底盘行驶的摇杆读取与使用。
        # ry（右摇杆 Y 轴）作为油门：由于手柄物理构造推到底是 0，拉到底是 255。
        # 这里用 map_joystick 处理后再加个负号 (-)，将其修正为前推为正、后拉为负。
        throttle = -map_joystick(ry)

        # lx（左摇杆 X 轴）作为转向舵：直接映射即可，左推负，右推正。
        steer = map_joystick(lx)

        # 将 -100~100 摇杆百分比按照最大限制缩放到实际车轮角速度和转向角度。
        speed_rad_s = throttle / 100.0 * _MAX_MOTOR_RAD_S
        steer_angle_deg = steer / 100.0 * MAX_STEER_ANGLE_DEG

        # 发送行驶指令到底盘。
        rover.drive(speed_rad_s, steer_angle_deg)
        time.sleep_ms(50)
