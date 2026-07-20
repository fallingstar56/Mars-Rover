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
from ps2_control import ps2_loop
from ps2_lib import PS2Controller, PS2Receiver
from robot_config import (
    ARM_AUTO_ACTION_DELAY_MS,
    ARM_GRAB_PITCH1_DEG,
    ARM_GRAB_PITCH2_DEG,
    ARM_PLACE_PITCH1_DEG,
    ARM_PLACE_PITCH2_DEG,
    CAN_BAUDRATE,
    CAN_BUS_ID,
    CAN_RX,
    CAN_TX,
    CAMERA_INIT_ANGLE_DEG,
    CAMERA_UART_BAUD,
    CAMERA_UART_ID,
    CAMERA_UART_RX,
    CAMERA_UART_TX,
    CAMERA_SERVO_ID,
    DEFAULT_ACC_RAD_S2,
    LINE_FOLLOW_BASE_SPEED_RAD_S,
    LINE_FOLLOW_DATA_TIMEOUT_MS,
    LINE_FOLLOW_MAX_STEER_DEG,
    LINE_FOLLOW_STEER_KP,
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
    """夹爪抓取动作占位。后续在这里补充夹爪舵机/电机控制。"""
    pass


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
        rover.arm.move_pitch12(ARM_GRAB_PITCH1_DEG, ARM_GRAB_PITCH2_DEG)
        time.sleep_ms(ARM_AUTO_ACTION_DELAY_MS)
        run_gripper_grab(rover)
        rover.arm.move_pitch12(ARM_PLACE_PITCH1_DEG, ARM_PLACE_PITCH2_DEG)
        time.sleep_ms(ARM_AUTO_ACTION_DELAY_MS)
        rover.arm.apply_initial_pose()
        rover.servo_control.set_camera_angle(CAMERA_INIT_ANGLE_DEG)
        rover.arm.camera_angle_deg = CAMERA_INIT_ANGLE_DEG
        time.sleep_ms(ARM_AUTO_ACTION_DELAY_MS)
    except ArmKinematicsError as err:
        print("机械臂目标无效：%s，%s" % (err.reason, err.message))
        return False
    return True


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
                camera_data["value"] = data.decode("utf-8", "replace")
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
