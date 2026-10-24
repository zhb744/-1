"""URDF 解析测试。

用**厂商给的真文件**跑，不用合成 XML —— 这份 URDF 的坑（包名错、joint1 是 fixed、
关节名与 link 同名）只有对着真实数据才会暴露。
"""

import math
from pathlib import Path

import pytest

from armcomm.servos import JOINTS, urdf_joint_to_servo_id
from armcomm.urdf import (
    PACKAGE_PREFIX,
    find_default_urdf,
    parse_urdf,
    pose_to_matrix,
    resolve_mesh,
    rpy_to_matrix,
    to_web_payload,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
URDF_PATH = find_default_urdf(REPO_ROOT)

requires_real_urdf = pytest.mark.skipif(URDF_PATH is None, reason="找不到厂商 URDF 文件")


def almost(actual, expected, tol=1e-12):
    assert len(actual) == len(expected)
    for a, e in zip(actual, expected):
        assert abs(a - e) <= tol, f"{a} != {e}"


# ---------------------------------------------------------------- 矩阵约定


def test_identity_rpy_gives_identity_rotation():
    assert rpy_to_matrix(0, 0, 0) == [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


def test_yaw_only_matches_z_rotation():
    almost(
        [v for row in rpy_to_matrix(0, 0, math.pi / 2) for v in row],
        [0, -1, 0, 1, 0, 0, 0, 0, 1],
    )


def test_roll_only_matches_x_rotation():
    almost(
        [v for row in rpy_to_matrix(math.pi / 2, 0, 0) for v in row],
        [1, 0, 0, 0, 0, -1, 0, 1, 0],
    )


def test_pitch_only_matches_y_rotation():
    almost(
        [v for row in rpy_to_matrix(0, math.pi / 2, 0) for v in row],
        [0, 0, 1, 0, 1, 0, -1, 0, 0],
    )


def test_rpy_is_extrinsic_so_order_matters():
    """URDF 的 rpy 是固定轴 X→Y→Z，即 R = Rz·Ry·Rx，与先转 Z 的结果不同。"""
    composed = rpy_to_matrix(math.radians(30), math.radians(20), math.radians(10))
    swapped = rpy_to_matrix(math.radians(10), math.radians(20), math.radians(30))
    assert composed != swapped
    # Rx(30) 先，再 Ry(20)，再 Rz(10)
    rx = rpy_to_matrix(math.radians(30), 0, 0)
    ry = rpy_to_matrix(0, math.radians(20), 0)
    rz = rpy_to_matrix(0, 0, math.radians(10))

    def mul(a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    manual = mul(mul(rz, ry), rx)
    for row in range(3):
        almost(composed[row], manual[row], tol=1e-12)


def test_pose_matrix_is_column_major_for_three_js():
    matrix = pose_to_matrix((1.0, 2.0, 3.0), (0, 0, 0))
    assert len(matrix) == 16
    assert matrix[12:16] == [1.0, 2.0, 3.0, 1.0]
    assert matrix[3] == matrix[7] == matrix[11] == 0.0
    assert matrix[15] == 1.0
    # 列主序：第 0 列是旋转矩阵第 0 列
    rot = rpy_to_matrix(0.4, -0.2, 0.9)
    matrix = pose_to_matrix((0, 0, 0), (0.4, -0.2, 0.9))
    assert matrix[0] == pytest.approx(rot[0][0])
    assert matrix[1] == pytest.approx(rot[1][0])
    assert matrix[2] == pytest.approx(rot[2][0])


def test_resolve_mesh_strips_wrong_package_name():
    web, disk, defects = resolve_mesh("package://44 001/meshes/base_link.STL", URDF_PATH)
    assert web == "/meshes/base_link.STL"
    assert disk.exists()
    assert any("44 001" in d for d in defects)


def test_package_prefix_regex_handles_plain_paths():
    assert PACKAGE_PREFIX.sub("", "meshes/x.STL") == "meshes/x.STL"


# ---------------------------------------------------------------- 真实 URDF


@requires_real_urdf
def test_real_urdf_is_parsed():
    model = parse_urdf(URDF_PATH, force_revolute=set(urdf_joint_to_servo_id()))
    assert model.name == "44 001"
    assert model.root_link == "base_link"
    assert len(model.links) == 14
    assert len(model.joints) == 13


@requires_real_urdf
def test_joint_axes_are_read_from_the_axis_element():
    """axis 写成子元素 ``<axis xyz="..."/>``。当成 joint 属性读会全部退化成 (0,0,1)，
    那样俯仰关节会绕竖轴转，模型一眼就散架 —— 这条专门盯住。"""
    model = parse_urdf(URDF_PATH, force_revolute=set(urdf_joint_to_servo_id()))
    assert list(model.joints["joint1"].axis) == [0.0, 0.0, 1.0]  # 全零 axis 已被修正
    for name in ("joint2", "joint3", "joint4"):
        axis = model.joints[name].axis
        assert abs(axis[1]) > 0.99, f"{name} 应当绕 -Y 俯仰，实际 {axis}"
        assert abs(axis[0]) < 0.01 and abs(axis[2]) < 0.01
    assert abs(model.joints["joint5"].axis[2]) > 0.99  # 腕旋转绕 Z


@requires_real_urdf
def test_base_joint_is_defective_and_gets_overridden():
    """joint1（底座回转 = 舵机 6）在 URDF 里是 fixed，必须被覆盖并记录缺陷。"""
    model = parse_urdf(URDF_PATH, force_revolute={"joint1"})
    assert model.joints["joint1"].joint_type == "revolute"
    assert any("joint1" in d and "fixed" in d for d in model.defects)

    without_override = parse_urdf(URDF_PATH)
    assert without_override.joints["joint1"].joint_type == "fixed"


@requires_real_urdf
def test_all_actuated_joints_map_to_exactly_six_servos():
    model = parse_urdf(URDF_PATH, force_revolute=set(urdf_joint_to_servo_id()))
    actuated = model.actuated_joints(urdf_joint_to_servo_id())
    assert [joint.name for joint in actuated] == ["l_joint", "joint5", "joint4", "joint3", "joint2", "joint1"]
    assert [urdf_joint_to_servo_id()[joint.name] for joint in actuated] == [1, 2, 3, 4, 5, 6]


@requires_real_urdf
def test_actuated_joint_limits_are_the_urdf_symmetric_range():
    model = parse_urdf(URDF_PATH, force_revolute=set(urdf_joint_to_servo_id()))
    for joint in model.actuated_joints(urdf_joint_to_servo_id()):
        if joint.name == "l_joint":
            continue  # 夹爪关节限位是 ±1.57 rad，与主臂不同
        assert joint.lower == pytest.approx(-2.09)
        assert joint.upper == pytest.approx(2.09)


@requires_real_urdf
def test_raw_urdf_does_not_invent_mimic_relations():
    """原始 URDF 没有 mimic；不应再从轴向猜一个不完整的联动表。"""
    model = parse_urdf(URDF_PATH)
    assert to_web_payload(model, urdf_joint_to_servo_id())["gripper_mimic"] == {}
    assert "l_out_link" in model.joints  # 原始解析仍忠实保留厂家导出


@requires_real_urdf
def test_every_mesh_referenced_by_the_model_exists_on_disk():
    model = parse_urdf(URDF_PATH, force_revolute=set(urdf_joint_to_servo_id()))
    checked = 0
    for link in model.links.values():
        if link.mesh is None:
            continue
        disk = model.package_root / link.mesh.lstrip("/")
        assert disk.exists(), f"{link.name} 的网格缺失：{disk}"
        checked += 1
    assert checked == 14


@requires_real_urdf
def test_defects_mention_the_package_mismatch():
    model = parse_urdf(URDF_PATH)
    assert any("package://" in d for d in model.defects)
    assert not any("不存在" in d for d in model.defects)


@requires_real_urdf
def test_web_payload_shape():
    model = parse_urdf(URDF_PATH, force_revolute=set(urdf_joint_to_servo_id()))
    payload = to_web_payload(model, urdf_joint_to_servo_id())
    assert payload["root_link"] == "base_link"
    assert len(payload["joints"]) == len(model.joints)
    assert len(payload["links"]) == len(model.links)
    assert payload["gripper_mimic"] == {}
    mapped = {joint["name"]: joint["servo_id"] for joint in payload["joints"] if joint["servo_id"]}
    assert mapped == urdf_joint_to_servo_id()
    for joint in payload["joints"]:
        assert len(joint["matrix"]) == 16
        assert len(joint["axis"]) == 3
    # 每个 link 的父关节都能找到
    children = {joint["child"] for joint in payload["joints"]}
    assert payload["root_link"] not in children
    for link in payload["links"]:
        if link["name"] != payload["root_link"]:
            assert link["name"] in children


@requires_real_urdf
def test_joint_tree_has_no_duplicate_children():
    """运动树必须是一棵树：每个 child 只被一个关节引用，否则前端装配会成环。"""
    model = parse_urdf(URDF_PATH)
    children = [joint.child for joint in model.joints.values()]
    assert len(children) == len(set(children))


@requires_real_urdf
def test_servo_range_covers_urdf_range_so_sliders_are_reachable():
    """滑块用舵机角度空间 ±120°，URDF 的 ±119.75° 应完全落在其中。"""
    model = parse_urdf(URDF_PATH, force_revolute=set(urdf_joint_to_servo_id()))
    for joint in model.actuated_joints(urdf_joint_to_servo_id()):
        servo_id = urdf_joint_to_servo_id()[joint.name]
        if servo_id in (1, 2):
            continue
        limits = (JOINTS[servo_id].min_angle, JOINTS[servo_id].max_angle)
        assert math.radians(joint.lower) >= limits[0] - 1e-9
        assert math.radians(joint.upper) <= limits[1] + 1e-9


def test_find_default_urdf_returns_none_for_missing_root(tmp_path):
    assert find_default_urdf(tmp_path) is None
