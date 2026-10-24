"""舵机映射测试。

这里锁定的不是"Python 自己一致"，而是与 STM32 固件 ``robot_arm.c`` 的整数行为、
以及 URDF 关节对应关系一致。
"""

import math

import pytest

from armcomm.servos import (
    ANGLE_SPAN_HALF,
    ANGLE_TO_POSITION_FACTOR,
    CLAW_POSITION_MAX,
    FIRMWARE_KNOT_LIMITS_DEG,
    GRIPPER_URDF_JOINTS,
    JOINTS,
    LINKAGES_CM,
    POSITION_MAX,
    POSITION_MIN,
    URDF_LIMIT_DEG,
    URDF_LIMIT_RAD,
    angle_to_position,
    claw_angle_to_position,
    claw_position_to_angle,
    clamp,
    knot_theta_to_servo_position,
    position_to_angle,
    roll_angle_to_position,
    servo_angle_to_position,
    urdf_joint_to_servo_id,
    validate_positions,
)


def test_factor_is_firmware_constant():
    """固件 SERIAL_ANGLE_FACTOR = 4.166666666666667f == 1000/240。"""
    assert math.isclose(ANGLE_TO_POSITION_FACTOR, 1000 / 240, rel_tol=0, abs_tol=1e-12)
    assert angle_to_position(120.0) == 1000
    assert angle_to_position(-120.0) == 0
    assert angle_to_position(0.0) == 500


def test_angle_to_position_truncates_toward_zero_like_c():
    """C 的 (int) 向零截断：-45° 得 500-187=313，不是 312。"""
    assert int(ANGLE_TO_POSITION_FACTOR * -45.0) == -187
    assert angle_to_position(-45.0) == 313
    assert angle_to_position(45.0) == 687


@pytest.mark.parametrize(
    "angle, position",
    [
        (0.0, 700),
        (90.0, 200),
        (45.0, 450),
        (100.0, 200),  # 超过 90° 被夹住
        (-10.0, 700),  # 负值夹到 0°
    ],
)
def test_claw_mapping(angle, position):
    assert claw_angle_to_position(angle) == position


def test_claw_roundtrip_within_rounding():
    for angle in range(0, 91, 7):
        position = claw_angle_to_position(float(angle))
        assert abs(claw_position_to_angle(position) - angle) <= 0.25


@pytest.mark.parametrize(
    "angle, position",
    [
        (0.0, 500),
        (90.0, 125),
        (-90.0, 875),
        (200.0, 125),  # 超范围夹到 +90°
    ],
)
def test_roll_mapping_is_inverted(angle, position):
    assert roll_angle_to_position(angle) == position


def test_knot_theta_to_servo_position_applies_the_90_degree_compensation():
    """theta2servo(): knot0/2/3 直用，knot1 做 -(90-theta) 补偿，依次落在 6/5/4/3。"""
    assert knot_theta_to_servo_position(0, 0.0) == 500
    assert knot_theta_to_servo_position(2, 30.0) == angle_to_position(30.0)
    assert knot_theta_to_servo_position(3, -30.0) == angle_to_position(-30.0)
    assert knot_theta_to_servo_position(1, 90.0) == 500
    assert knot_theta_to_servo_position(1, 0.0) == angle_to_position(-90.0)
    with pytest.raises(ValueError):
        knot_theta_to_servo_position(4, 0.0)


def test_knot_to_servo_id_assignment_matches_firmware():
    """固件里舵机号是 6-i，所以 knot0→6, knot1→5, knot2→4, knot3→3。"""
    mapping = {joint.knot: sid for sid, joint in JOINTS.items() if joint.knot is not None}
    assert mapping == {0: 6, 1: 5, 2: 4, 3: 3}


def test_six_servos_defined_with_firmware_reset_positions():
    assert sorted(JOINTS) == [1, 2, 3, 4, 5, 6]
    assert [JOINTS[sid].reset_position for sid in (1, 2, 3, 4, 5, 6)] == [226, 500, 110, 800, 600, 500]
    for joint in JOINTS.values():
        assert POSITION_MIN <= joint.reset_position <= POSITION_MAX


def test_reset_positions_decode_to_sane_angles():
    """4 个主关节的复位位置都应落在 URDF 限位内，否则换算或复位表有一处是错的。"""
    for servo_id in (3, 4, 5, 6):
        joint = JOINTS[servo_id]
        angle = position_to_angle(joint.reset_position)
        assert joint.min_angle <= angle <= joint.max_angle, f"{joint.name} 复位角 {angle:.1f}° 超范围"


def test_slider_range_uses_full_position_travel_not_urdf_approximation():
    """URDF 手填的 ±2.09 rad ≈ ±119.75° 比位置全域 ±120° 窄，会白丢 2 个位置单位。

    桥接层用 ±120°（0~1000 完整行程）作滑块范围，URDF 数值只作为参考保留。
    """
    assert math.isclose(URDF_LIMIT_DEG, math.degrees(URDF_LIMIT_RAD))
    assert 119.7 < URDF_LIMIT_DEG < 119.8
    assert URDF_LIMIT_DEG < ANGLE_SPAN_HALF
    for servo_id in (3, 4, 5, 6):
        assert JOINTS[servo_id].min_angle == -ANGLE_SPAN_HALF
        assert JOINTS[servo_id].max_angle == ANGLE_SPAN_HALF


def test_three_limit_sources_disagree_and_need_calibration():
    """已知矛盾（必须盯住，不能假装看不见）：

    ``kinematics.h`` 的 ``MIN/MAX_KNOT4_ANGLE`` = −200.2~20.2°，若按舵机 ID 解释，
    则 ``global.h`` 给的 ID4 复位位置 800（= +72°）超出它的上限。说明这套数值的编号约定
    或所处角度空间与我们的假设不一致。标定表写进 config.local.json 之前，滑块一律用
    URDF 限位，本模块不启用 FIRMWARE_KNOT_LIMITS_DEG。
    """
    id4_reset_angle = position_to_angle(JOINTS[4].reset_position)
    assert id4_reset_angle > FIRMWARE_KNOT_LIMITS_DEG[4][1]
    assert FIRMWARE_KNOT_LIMITS_DEG[4] == (-200.2, 20.2)
    assert FIRMWARE_KNOT_LIMITS_DEG[5] == (-180.2, 0.2)


@pytest.mark.parametrize(
    "servo_id, angle, expected",
    [
        (6, 999.0, 1000),
        (6, -999.0, 0),
        (2, 180.0, 125),
        (1, 45.0, 450),
        (3, 0.0, 500),
        (5, 30.0, angle_to_position(30.0)),
    ],
)
def test_servo_angle_to_position_clamps_per_joint(servo_id, angle, expected):
    position = servo_angle_to_position(servo_id, angle)
    assert 0 <= position <= 1000
    assert position == expected


def test_servo_angle_to_position_rejects_unknown_id():
    with pytest.raises(ValueError):
        servo_angle_to_position(7, 0.0)
    with pytest.raises(ValueError):
        servo_angle_to_position(0, 0.0)


def test_urdf_joint_to_servo_mapping_is_complete():
    """仿真桥接需要的 6 个自由度都能一一对上 URDF 关节名。"""
    mapping = urdf_joint_to_servo_id()
    assert mapping == {
        "joint1": 6,
        "joint2": 5,
        "joint3": 4,
        "joint4": 3,
        "joint5": 2,
        "l_joint": 1,
    }
    assert set(GRIPPER_URDF_JOINTS).issubset({"l_joint", "l_in_joint", "r_in_joint", "l_out_joint"})


def test_validate_positions_flags_out_of_range_and_claw_cap():
    errors = []
    validate_positions({1: 900, 2: 500, 6: 1500, 7: 500}, errors)
    assert len(errors) == 3
    assert any("夹爪" in e for e in errors)
    assert any("越界" in e for e in errors)
    assert any("不在 1~6" in e for e in errors)


def test_validate_positions_accepts_reset_pose():
    errors = []
    validate_positions({sid: JOINTS[sid].reset_position for sid in JOINTS}, errors)
    assert errors == []


def test_full_angle_scan_produces_integers_in_firmware_range():
    for step in range(-480, 481):
        position = angle_to_position(step * 0.25)
        assert isinstance(position, int)
        assert -1 <= position <= 1001
    assert CLAW_POSITION_MAX == 700


def test_linkage_lengths_come_from_kinematics_header():
    assert LINKAGES_CM == (2.89, 9.50, 9.50, 16.9)


def test_clamp_helper():
    assert clamp(-5, 0, 90) == 0
    assert clamp(200, 0, 90) == 90
    assert clamp(45, 0, 90) == 45
