"""
月球小车公共配置。

学生通常只需要改这里的运行模式、引脚、速度上限、舵机限位、机械臂尺寸等参数。

作者 王笑
日期 20260528
"""


def clamp(value, low, high):
    return max(low, min(high, value))


# =============================================================================
# 硬件端口与运行模式
# =============================================================================

RUN_MODE = "idle"  # 可选值："idle" | "ps2" | "line_follow"

# UART 接线。
SERVO_UART_ID = 2
SERVO_UART_BAUD = 115200
SERVO_UART_TX = 16
SERVO_UART_RX = 17

# 摄像头接线。
CAMERA_UART_ID = 1
CAMERA_UART_BAUD = 115200
CAMERA_UART_TX = 5
CAMERA_UART_RX = 6

# CAN 接线。
CAN_BUS_ID = 0
CAN_BAUDRATE = 1000000
CAN_TX = 8
CAN_RX = 18

# PS2 手柄接线。
PS2_DI = 9
PS2_DO = 10
PS2_CS = 11
PS2_CLK = 12


# =============================================================================
# 底盘配置
# =============================================================================

MAX_MOTOR_RPM = 200.0  # 最大电机转速。
DEFAULT_ACC_RAD_S2 = 20.0  # 电机加速度，单位 rad/s^2。

MAX_STEER_ANGLE_DEG = 90.0  # 最大转向角度。
PIVOT_SPEED_SCALE = 0.3  # 原地转向速度比例，v_pivot_max = v_max * PIVOT_SPEED_SCALE。

# 巡线配置。
# 视觉端发送格式：ln <dx> <area>，dx 为线中心相对画面中心的水平偏差，单位 px。
LINE_FOLLOW_BASE_SPEED_RAD_S = 0.8
LINE_FOLLOW_MAX_STEER_DEG = 35.0
LINE_FOLLOW_STEER_KP = 0.12
LINE_FOLLOW_DATA_TIMEOUT_MS = 500


# =============================================================================
# 舵机配置
# =============================================================================

BASE_SERVO_IDS = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11)  # 基础舵机 ID，同调试软件配置结果，不要修改。
CAMERA_SERVO_ID = 8  # 相机舵机 ID，同调试软件配置结果，不要修改。

# 预留舵机：未安装时保持 RESERVE_SERVO_ENABLED = False。
RESERVE_SERVO_ENABLED = False  # 默认 False。
RESERVE_SERVO_IDS = (14,)  # 可填多个 ID，如 (13, 14)。
RESERVE_SERVO_SIGNS = {14: 1}  # 每个 ID 的方向符号，1 或 -1，如 {13: 1, 14: -1}。
RESERVE_SERVO_INIT_ANGLE_DEG = {14: 0.0}  # 每个 ID 的初始角，如 {13: 0.0, 14: 10.0}。
RESERVE_SERVO_MIN_DEG = {14: -90.0}  # 每个 ID 的下限，如 {13: -90.0, 14: 0.0}。
RESERVE_SERVO_MAX_DEG = {14: 90.0}  # 每个 ID 的上限，如 {13: 90.0, 14: 45.0}。

# 舵机控制上下限配置。
STEER_ANGLE_MIN_DEG = -90.0  # 转向舵机。
STEER_ANGLE_MAX_DEG = 90.0
CAMERA_ANGLE_MIN_DEG = -90.0  # 相机舵机。
CAMERA_ANGLE_MAX_DEG = 0.0
CAMERA_INIT_ANGLE_DEG = -77.5  # 相机上电复位角度，同时作为点动控制的初始状态。
ARM_ROLL_MIN_DEG = -180.0  # 机械臂 Roll 舵机。
ARM_ROLL_MAX_DEG = 180.0
ARM_PITCH1_MIN_DEG = -90.0  # 机械臂 Pitch1 舵机。
ARM_PITCH1_MAX_DEG = 100.0
ARM_PITCH2_MIN_DEG = -150.0  # 机械臂 Pitch2 舵机。
ARM_PITCH2_MAX_DEG = 150.0
ARM_PITCH3_MIN_DEG = -150.0  # 机械臂 Pitch3 舵机。
ARM_PITCH3_MAX_DEG = 150.0


# =============================================================================
# 机械臂配置
# =============================================================================

ARM_SERVO_SPEED_DEG_S = 60.0  # 机械臂默认运动速度。

# 机械臂初始位配置。
ARM_INIT_ROLL_DEG = 0.0
ARM_INIT_PITCH1_DEG = 50
ARM_INIT_PITCH2_DEG = -140.0
ARM_INIT_PITCH3_DEG = 0.0

# 自动抓取任务姿态配置。
# 二维码任务到位后，控制端依次执行：
# 1. Pitch1/Pitch2 转到抓取姿态
# 2. 执行夹爪抓取占位逻辑
# 3. Pitch1/Pitch2 转到放置姿态
# 4. 机械臂复位
# 这些角度是待标定值，后续只需要改这里。
ARM_GRAB_PITCH1_DEG = -3.92
ARM_GRAB_PITCH2_DEG = -120.25
ARM_PLACE_PITCH1_DEG = 94.39
ARM_PLACE_PITCH2_DEG = 117.98
ARM_AUTO_ACTION_DELAY_MS = 1500

