"""
机械臂组合控制（非底层协议库）。

依赖 servo_control.ServoControl，在舵机功能层之上提供关节点动和相机控制。
底层 UART 协议见 servo_lib.py，单舵机功能见 servo_control.py。

常用学生接口：
- apply_initial_pose()
- jog_joints(roll_delta_deg=0, pitch1_delta_deg=0, pitch2_delta_deg=0, pitch3_delta_deg=0)
- jog_camera(delta_deg)

同步接口（仅在需要主动校准真实姿态时调用）：
- sync_from_servos()
- sync_camera_from_servo()

作者 王笑
日期 20260528
"""

from robot_config import (
    ARM_INIT_PITCH1_DEG,
    ARM_INIT_PITCH2_DEG,
    ARM_INIT_PITCH3_DEG,
    ARM_INIT_ROLL_DEG,
    ARM_PITCH1_MAX_DEG,
    ARM_PITCH1_MIN_DEG,
    ARM_PITCH2_MAX_DEG,
    ARM_PITCH2_MIN_DEG,
    ARM_PITCH3_MAX_DEG,
    ARM_PITCH3_MIN_DEG,
    ARM_ROLL_MAX_DEG,
    ARM_ROLL_MIN_DEG,
    ARM_SERVO_SPEED_DEG_S,
    CAMERA_INIT_ANGLE_DEG,
    CAMERA_ANGLE_MAX_DEG,
    CAMERA_ANGLE_MIN_DEG,
    clamp,
)

_CAMERA_SERVO_SPEED_DEG_S = 60.0


class ArmKinematicsError(Exception):
    """机械臂目标无效时抛出，reason 用于区分错误类型。"""

    def __init__(self, reason, message):
        self.reason = reason
        self.message = message

    def __str__(self):
        return self.message


class RobotArm:
    def __init__(self, servo_control):
        self.servo_control = servo_control
        self.roll_deg = ARM_INIT_ROLL_DEG
        self.pitch1_deg = ARM_INIT_PITCH1_DEG
        self.pitch2_deg = ARM_INIT_PITCH2_DEG
        self.pitch3_deg = ARM_INIT_PITCH3_DEG
        # 与上电复位角保持一致，避免第一次 jog_camera() 跳向错误位置。
        self.camera_angle_deg = clamp(
            CAMERA_INIT_ANGLE_DEG,
            CAMERA_ANGLE_MIN_DEG,
            CAMERA_ANGLE_MAX_DEG,
        )

    def sync_camera_from_servo(self):
        """读取相机舵机真实角度，并更新 camera_angle_deg。"""
        angle = self.servo_control.read_camera_angle()
        if angle is None:
            raise ArmKinematicsError(
                "read_failed",
                "读取相机舵机角度失败，目标不执行。",
            )
        self.camera_angle_deg = clamp(
            float(angle),
            CAMERA_ANGLE_MIN_DEG,
            CAMERA_ANGLE_MAX_DEG,
        )
        return self.camera_angle_deg

    def jog_camera(self, delta_deg, speed_deg_s=_CAMERA_SERVO_SPEED_DEG_S):
        """在内部记录的相机角度基础上增量调整，不依赖读角回包。"""
        self.camera_angle_deg = clamp(
            self.camera_angle_deg + float(delta_deg),
            CAMERA_ANGLE_MIN_DEG,
            CAMERA_ANGLE_MAX_DEG,
        )
        self.servo_control.set_camera_angle(
            self.camera_angle_deg,
            speed_deg_s=speed_deg_s,
        )

    def apply_initial_pose(self, speed_deg_s=ARM_SERVO_SPEED_DEG_S):
        return self._move_to_joint_pose(
            ARM_INIT_ROLL_DEG,
            ARM_INIT_PITCH1_DEG,
            ARM_INIT_PITCH2_DEG,
            ARM_INIT_PITCH3_DEG,
            speed_deg_s=speed_deg_s,
        )

    def jog_joints(self, roll_delta_deg=0.0, pitch1_delta_deg=0.0,
                   pitch2_delta_deg=0.0, pitch3_delta_deg=0.0,
                   speed_deg_s=ARM_SERVO_SPEED_DEG_S):
        return self._move_to_joint_pose(
            self.roll_deg + roll_delta_deg,
            self.pitch1_deg + pitch1_delta_deg,
            self.pitch2_deg + pitch2_delta_deg,
            self.pitch3_deg + pitch3_delta_deg,
            speed_deg_s=speed_deg_s,
        )

    def sync_from_servos(self):
        """
        轮询读取机械臂四个舵机真实角度，并同步内部状态。

        这是可选校准接口；普通运动不会自动调用它。
        如果任意关节读取失败或读取值超出限位，会抛出 ArmKinematicsError。
        """
        values = self.servo_control.read_arm_joint_angles()
        try:
            roll = values["roll"]
            pitch1 = values["pitch1"]
            pitch2 = values["pitch2"]
            pitch3 = values["pitch3"]
        except (KeyError, TypeError):
            raise ArmKinematicsError(
                "read_failed",
                "读取机械臂舵机角度失败，目标不执行。",
            )

        if roll is None or pitch1 is None or pitch2 is None or pitch3 is None:
            raise ArmKinematicsError(
                "read_failed",
                "读取机械臂舵机角度失败，目标不执行。",
            )

        roll = float(roll)
        pitch1 = float(pitch1)
        pitch2 = float(pitch2)
        pitch3 = float(pitch3)
        (self.roll_deg,
         self.pitch1_deg,
         self.pitch2_deg,
         self.pitch3_deg) = self._clamp_joint_pose(
            roll,
            pitch1,
            pitch2,
            pitch3,
        )
        return self._build_move_result()

    def apply_joint_pose(self, speed_deg_s=ARM_SERVO_SPEED_DEG_S):
        (self.roll_deg,
         self.pitch1_deg,
         self.pitch2_deg,
         self.pitch3_deg) = self._clamp_joint_pose(
            self.roll_deg,
            self.pitch1_deg,
            self.pitch2_deg,
            self.pitch3_deg,
        )
        self.servo_control.set_arm_joint_angles(
            self.roll_deg,
            self.pitch1_deg,
            self.pitch2_deg,
            self.pitch3_deg,
            speed_deg_s=speed_deg_s,
        )
        return self._build_move_result(speed_deg_s)

    def _move_to_joint_pose(self, target_roll_deg, target_pitch1_deg,
                            target_pitch2_deg, target_pitch3_deg,
                            speed_deg_s=ARM_SERVO_SPEED_DEG_S):
        (self.roll_deg,
         self.pitch1_deg,
         self.pitch2_deg,
         self.pitch3_deg) = self._clamp_joint_pose(
            target_roll_deg,
            target_pitch1_deg,
            target_pitch2_deg,
            target_pitch3_deg,
        )
        self.servo_control.set_arm_joint_angles(
            self.roll_deg,
            self.pitch1_deg,
            self.pitch2_deg,
            self.pitch3_deg,
            speed_deg_s=speed_deg_s,
        )
        return self._build_move_result(speed_deg_s)

    def _clamp_joint_pose(self, roll_deg, pitch1_deg, pitch2_deg, pitch3_deg):
        return (
            clamp(float(roll_deg), ARM_ROLL_MIN_DEG, ARM_ROLL_MAX_DEG),
            clamp(float(pitch1_deg), ARM_PITCH1_MIN_DEG, ARM_PITCH1_MAX_DEG),
            clamp(float(pitch2_deg), ARM_PITCH2_MIN_DEG, ARM_PITCH2_MAX_DEG),
            clamp(float(pitch3_deg), ARM_PITCH3_MIN_DEG, ARM_PITCH3_MAX_DEG),
        )

    def _require_range(self, name, value, low, high):
        if not self._in_range(value, low, high):
            raise ArmKinematicsError(
                "joint_limit",
                "%s 目标角 %.1f deg 超出限位 %.1f~%.1f deg，目标不执行。"
                % (name, value, low, high),
            )

    @staticmethod
    def _in_range(value, low, high):
        return low <= value <= high

    def _build_move_result(self, speed_deg_s=None):
        return {
            "ok": True,
            "speed_deg_s": speed_deg_s,
            "roll_deg": self.roll_deg,
            "pitch1_deg": self.pitch1_deg,
            "pitch2_deg": self.pitch2_deg,
            "pitch3_deg": self.pitch3_deg,
        }
