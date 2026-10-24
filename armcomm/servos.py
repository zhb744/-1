"""舵机映射：关节 ↔ 舵机 ID ↔ 原始位置 ↔ 角度 ↔ URDF 关节名。

数据来源都是实测读出来的，不做推测：

* ``SERIAL_ANGLE_FACTOR``、各关节的角度正负号与 90° 结构补偿
  —— ``Hiwonder/Arm/src/robot_arm.c`` 的 ``theta2servo()`` / ``robot_arm_claw_set()``
  / ``robot_arm_roll_set()``
* 复位位置 —— ``Hiwonder/Peripherals/Src/global.h``
* 关节限位 —— ``armpi_ultra.urdf``（±2.09 rad）与 ``kinematics.h``（见下方矛盾说明）
* URDF 关节名与轴向 —— ``armpi_ultra_description/urdf/armpi_ultra.urdf``

位置与角度的关系：``position = 500 + (1000/240) * angle``，即 0~1000 对应 ±120°，
1 个位置单位 = 0.24°。固件用 C 的 ``(int)`` 截断，Python 的 ``int()`` 同样向零截断，
两侧逐位一致（有测试覆盖负角度）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List

from .protocol import MAX_DATA_LENGTH

#: 1000/240，固件里的 SERIAL_ANGLE_FACTOR
ANGLE_TO_POSITION_FACTOR = 1000 / 240

#: 舵机原始位置范围（固件 ``serial_servo_set_position`` 只裁上限，下限由上位机保证）
POSITION_MIN = 0
POSITION_MAX = 1000

#: 夹爪舵机（ID 1）的固件硬性上限，防止堵转烧毁电机
CLAW_POSITION_MAX = 700

#: 位置 0~1000 对应的理论半程
ANGLE_SPAN_HALF = 120.0

#: URDF 里每个活动关节都写作 ±2.09 rad
URDF_LIMIT_RAD = 2.09
URDF_LIMIT_DEG = math.degrees(URDF_LIMIT_RAD)


@dataclass(frozen=True)
class Joint:
    servo_id: int
    name: str
    knot: int | None  # 参与逆解的运动学关节序号，夹爪/腕旋为 None
    urdf_joint: str | None  # URDF 里对应的关节名
    min_angle: float  # 滑块允许的角度范围（取自 URDF / robot_arm.h）
    max_angle: float
    reset_position: int
    positive_direction: int  # 角度增大时位置是增大(+1)还是减小(-1)


#: 舵机 ID 1~6。ID 6~3 依次是逆解的 knot0~knot3（底座→大臂→小臂→腕俯仰）。
#:
#: 滑块角度一律取 **位置全域 ±120.0°**（0~1000 的完整行程），而不是 URDF 的 ±119.75°：
#: 后者是 SolidWorks 导出时手填的近似值，用它会白白丢掉 2 个位置单位的行程。
#: URDF 关节对应关系由轴向判断：joint1 与 joint5 的轴接近 Z（回转），joint2~4 接近 -Y（俯仰）。
#: 注意 URDF 把 joint1（底座）导出成了 ``fixed``，仿真前必须改回 ``revolute``，
#: 否则真机的底座在仿真里根本转不动。标定表可用 config.local.json 的
#: ``calibration.servo_angle_limits`` 覆盖这里。
JOINTS: Dict[int, Joint] = {
    1: Joint(1, "夹爪", None, "l_joint", 0.0, 90.0, 226, -1),
    2: Joint(2, "腕旋转", None, "joint5", -90.0, 90.0, 500, -1),
    3: Joint(3, "腕俯仰 (knot3)", 3, "joint4", -ANGLE_SPAN_HALF, ANGLE_SPAN_HALF, 110, +1),
    4: Joint(4, "小臂 (knot2)", 2, "joint3", -ANGLE_SPAN_HALF, ANGLE_SPAN_HALF, 800, +1),
    5: Joint(5, "大臂 (knot1)", 1, "joint2", -ANGLE_SPAN_HALF, ANGLE_SPAN_HALF, 600, +1),
    6: Joint(6, "底座回转 (knot0)", 0, "joint1", -ANGLE_SPAN_HALF, ANGLE_SPAN_HALF, 500, +1),
}

#: 夹爪在 URDF 里是 4 个独立 revolute/fixed 关节（左右各内外两片），真机只有 1 个舵机 ID 1，
#: 所以仿真里这 4 个关节必须联动（mimic），不能各自为政。
GRIPPER_URDF_JOINTS = ("l_joint", "l_in_joint", "r_in_joint", "l_out_joint")

#: ``kinematics.h`` 的 ``MIN/MAX_KNOT3..6_ANGLE``。
#: **不要直接当成滑块限位用**：按舵机 ID 3~6 解释时，它与 ``global.h`` 的复位位置矛盾
#: （例如 ID 4 复位位置 800 对应 +72°，而这里上限只有 20.2°）。三处数据不同源，
#: 说明这套限位的编号约定或所在角度空间与我们的假设不一致，必须以实机标定为准，
#: 标定结果写进 config.local.json 后才启用。tests 里有一条专门盯着这个矛盾。
FIRMWARE_KNOT_LIMITS_DEG: Dict[int, tuple] = {
    3: (-120.2, 120.2),
    4: (-200.2, 20.2),
    5: (-180.2, 0.2),
    6: (-120.2, 120.2),
}

#: ``kinematics.h`` 的连杆长度，单位 cm
LINKAGES_CM = (2.89, 9.50, 9.50, 16.9)

#: ``robot_arm.h`` 的工作空间，单位 cm
WORKSPACE_CM = {"x": (10.0, 20.0), "y": (-10.0, 10.0), "z": (0.0, 25.0)}
DEFAULT_POSE_CM = (15.0, 0.0, 2.0)


def clamp(value: float, low: float, high: float) -> float:
    return low if value < low else (high if value > high else value)


def angle_to_position(angle_deg: float) -> int:
    """通用关节：``pos = 500 + factor * angle``，与固件截断一致。"""
    return 500 + int(ANGLE_TO_POSITION_FACTOR * angle_deg)


def position_to_angle(position: int) -> float:
    return (position - 500) / ANGLE_TO_POSITION_FACTOR


def claw_angle_to_position(open_angle_deg: float) -> int:
    """夹爪：``pos = 700 - 5.555556 * 张开角``，700 全闭、200 全开。"""
    angle = clamp(open_angle_deg, 0.0, 90.0)
    return 700 - int((500 / 90.0) * angle)


def claw_position_to_angle(position: int) -> float:
    return (700 - position) / (500 / 90.0)


def roll_angle_to_position(rotation_angle_deg: float) -> int:
    """腕旋转：``pos = 500 - factor * 角``，方向与普通关节相反。"""
    angle = clamp(rotation_angle_deg, -90.0, 90.0)
    return 500 - int(ANGLE_TO_POSITION_FACTOR * angle)


def roll_position_to_angle(position: int) -> float:
    return (500 - position) / ANGLE_TO_POSITION_FACTOR


def knot_theta_to_servo_position(knot: int, theta_deg: float) -> int:
    """逆解输出的 knot 角度 → 舵机位置。

    固件 ``theta2servo()`` 对 knot1（大臂）做了 ``-(90 - theta)`` 的结构补偿，其余 knot
    直接用；knot0~3 依次落在舵机 6/5/4/3。
    """
    if knot == 1:
        angle = -(90.0 - theta_deg)
    elif knot in (0, 2, 3):
        angle = theta_deg
    else:
        raise ValueError(f"knot{knot} 不参与逆解")
    return angle_to_position(angle)


def servo_angle_to_position(servo_id: int, angle_deg: float) -> int:
    """按关节类型选择正确换算，角度先夹进该关节允许范围。"""
    joint = JOINTS.get(servo_id)
    if joint is None:
        raise ValueError(f"舵机 ID {servo_id} 不在 1~6 范围内")
    angle = clamp(angle_deg, joint.min_angle, joint.max_angle)
    if servo_id == 1:
        return claw_angle_to_position(angle)
    if servo_id == 2:
        return roll_angle_to_position(angle)
    return angle_to_position(angle)


def urdf_joint_to_servo_id() -> Dict[str, int]:
    """给仿真桥接层用的映射表：URDF 关节名 → 舵机 ID。"""
    return {joint.urdf_joint: joint.servo_id for joint in JOINTS.values() if joint.urdf_joint}


def validate_positions(positions: Dict[int, int], errors: List[str]) -> None:
    """把异常位置收集成人类可读的错误列表，界面和自检共用。"""
    for servo_id, position in sorted(positions.items()):
        joint = JOINTS.get(servo_id)
        if joint is None:
            errors.append(f"舵机 ID {servo_id} 不在 1~6 范围内")
            continue
        if not 0 <= position <= POSITION_MAX:
            errors.append(f"{joint.name} 位置 {position} 越界（0~1000）")
        elif servo_id == 1 and position > CLAW_POSITION_MAX:
            errors.append(f"夹爪位置 {position} 超过固件上限 {CLAW_POSITION_MAX}，会被截断")
