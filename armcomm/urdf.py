"""URDF 解析 —— 把 SolidWorks 导出的 ``armpi_ultra.urdf`` 变成浏览器能直接用的模型。

导出的是**列主序 4x4 齐次矩阵**，而不是 rpy 三元组。原因是 URDF 的 ``rpy`` 是固定轴
（extrinsic）X-Y-Z 顺序，即 ``R = Rz(yaw) · Ry(pitch) · Rx(roll)``，而 three.js 的
``Euler('XYZ')`` 是另一种约定；把换算留在 Python 里并写单元测试，前端就只负责乘矩阵，
不会再引入第二套角度约定。

原始 URDF 有两处必须处理才能用，本模块都会检出并记录到 ``model.defects``：

1. 所有网格路径写成 ``package://44 001/meshes/...``（SolidWorks 导出器把带空格的零件名
   当成了包名），与真实包名 ``armpi_ultra_description`` 不符，RViz/Gazebo 里会加载不出模型。
2. ``joint1``（底座回转，对应真机舵机 ID 6）被导出成 ``type="fixed"``，仿真里根本转不动，
   必须按 ``force_revolute`` 覆盖回来。
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET

PACKAGE_PREFIX = re.compile(r"^package://[^/]+/")

#: 夹爪在 URDF 里是 4 个 revolute 关节、真机只有 1 个舵机（ID 1），仿真按此表联动。
#: 驱动关节是 ``l_joint``；注意外侧那片关节在 URDF 里被命名为 ``l_out_link``
#: （和 link 同名，是导出器的笔误），别写成 ``l_out_joint``，否则联动会静默失效。
#: 四个关节的轴都是 (0.99992, -0.0087, -0.0087)，同向；左右对称所以一侧取负号。
#: 符号是轴向推断，视觉上还要再核。
GRIPPER_MIMIC = {"l_in_joint": 1.0, "r_in_joint": -1.0, "l_out_link": 1.0}
GRIPPER_DRIVER_JOINT = "l_joint"


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    """URDF rpy（固定轴 X→Y→Z）→ 3x3 旋转矩阵，行主序。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # Rz · Ry · Rx
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def pose_to_matrix(xyz: Iterable[float], rpy: Iterable[float]) -> list[float]:
    """位姿 → 列主序 4x4（three.js ``Matrix4.fromArray`` 直接可用）。"""
    x, y, z = tuple(xyz)
    rot = rpy_to_matrix(*rpy)
    r00, r01, r02 = rot[0]
    r10, r11, r12 = rot[1]
    r20, r21, r22 = rot[2]
    return [
        r00, r10, r20, 0.0,
        r01, r11, r21, 0.0,
        r02, r12, r22, 0.0,
        x, y, z, 1.0,
    ]


@dataclass
class UrdfLink:
    name: str
    mesh: str | None = None  # 相对包根目录的网格路径
    mesh_origin_matrix: list[float] = field(default_factory=lambda: pose_to_matrix((0, 0, 0), (0, 0, 0)))


@dataclass
class UrdfJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    xyz: tuple
    rpy: tuple
    axis: tuple = (0.0, 0.0, 1.0)
    lower: float = 0.0
    upper: float = 0.0
    matrix: list[float] = field(default_factory=list)

    def __post_init__(self):
        if not self.matrix:
            self.matrix = pose_to_matrix(self.xyz, self.rpy)


@dataclass
class UrdfModel:
    name: str
    package_root: Path
    links: dict
    joints: dict
    root_link: str
    defects: list

    def actuated_joints(self, servo_map: dict) -> list:
        """返回能对上舵机 ID 的关节，按 ID 升序，供界面排滑块。"""
        found = []
        for joint in self.joints.values():
            servo_id = servo_map.get(joint.name)
            if servo_id is not None:
                found.append((servo_id, joint))
        return [joint for _, joint in sorted(found, key=lambda item: item[0])]


def _text(element, tag, default=""):
    child = element.find(tag)
    if child is None:
        return default
    return (child.text or "").strip()


def _link_ref(joint_el, tag):
    """URDF 里 parent/child 是**空元素 + link 属性**，不是文本内容。

    ``<parent link="base_link" />`` —— 直接读 text 会得到空字符串，整棵运动树就散了。
    """
    child = joint_el.find(tag)
    if child is None:
        return ""
    return (child.get("link") or "").strip()


def _attr_vec(element, attr, default):
    """origin/axis 的数值写在元素属性上：``<origin xyz="..." rpy="..."/>``。"""
    if element is None:
        return default
    raw = (element.get(attr) or "").strip()
    if not raw:
        return default
    return tuple(float(v) for v in raw.replace("\n", " ").split())


def resolve_mesh(filename: str, urdf_path: Path) -> tuple[str, Path, list]:
    """把 ``package://44 001/meshes/x.STL`` 解成 (web 路径, 磁盘路径, 缺陷说明)。"""
    defects = []
    stripped = PACKAGE_PREFIX.sub("", filename)
    if stripped != filename:
        package_name = filename[len("package://"):].split("/", 1)[0]
        actual = urdf_path.parent.parent.name
        if package_name != actual:
            defects.append(
                f"网格路径写的是 package://{package_name}/，但实际包名是 {actual}，已按本地目录解析"
            )
    package_root = urdf_path.parent.parent
    disk = package_root / stripped
    return "/" + stripped.replace("\\", "/"), disk, defects


def parse_urdf(urdf_path: Path, force_revolute: Iterable[str] = ()) -> UrdfModel:
    urdf_path = Path(urdf_path)
    root = ET.parse(urdf_path).getroot()
    force = set(force_revolute)

    links: dict[str, UrdfLink] = {}
    joints: dict[str, UrdfJoint] = {}
    defects: list[str] = []

    for link_el in root.findall("link"):
        name = link_el.get("name")
        mesh = None
        mesh_matrix = pose_to_matrix((0, 0, 0), (0, 0, 0))
        visual = link_el.find("visual")
        if visual is not None:
            mesh_el = visual.find("geometry/mesh")
            if mesh_el is not None and mesh_el.get("filename"):
                web_path, disk, mesh_defects = resolve_mesh(mesh_el.get("filename"), urdf_path)
                defects.extend(mesh_defects)
                mesh = web_path
                if not disk.exists():
                    defects.append(f"网格文件不存在：{disk}")
                origin = visual.find("origin")
                if origin is not None:
                    mesh_matrix = pose_to_matrix(
                        _attr_vec(origin, "xyz", (0.0, 0.0, 0.0)),
                        _attr_vec(origin, "rpy", (0.0, 0.0, 0.0)),
                    )
        links[name] = UrdfLink(name=name, mesh=mesh, mesh_origin_matrix=mesh_matrix)

    child_names = set()
    for joint_el in root.findall("joint"):
        name = joint_el.get("name")
        joint_type = joint_el.get("type", "")
        parent = _link_ref(joint_el, "parent")
        child = _link_ref(joint_el, "child")
        origin = joint_el.find("origin")
        xyz = _attr_vec(origin, "xyz", (0.0, 0.0, 0.0))
        rpy = _attr_vec(origin, "rpy", (0.0, 0.0, 0.0))
        # axis 和 origin 一样是子元素（<axis xyz="..."/>），不是 joint 的属性；
        # 读错就会全部退化成默认 (0,0,1)，俯仰关节会变成绕 Z 转，装配完全走样。
        axis = _attr_vec(joint_el.find("axis"), "xyz", (0.0, 0.0, 1.0))
        limit_el = joint_el.find("limit")
        lower = float(limit_el.get("lower", 0.0)) if limit_el is not None else 0.0
        upper = float(limit_el.get("upper", 0.0)) if limit_el is not None else 0.0

        if name in force and joint_type == "fixed":
            defects.append(f"关节 {name} 在 URDF 里是 fixed，已按真机自由度覆盖为 revolute")
            joint_type = "revolute"
            if all(abs(v) < 1e-9 for v in axis):
                # fixed 关节的 axis 被导出成全零，覆盖成 revolute 后必须有旋转轴，
                # 底座/腕这类回转关节取 Z。
                axis = (0.0, 0.0, 1.0)
                defects.append(f"关节 {name} 的 axis 原本是 (0 0 0)，覆盖为 revolute 后改用 Z 轴")

        joints[name] = UrdfJoint(
            name=name, joint_type=joint_type, parent=parent, child=child,
            xyz=xyz, rpy=rpy, axis=axis, lower=lower, upper=upper,
        )
        child_names.add(child)

    roots = [name for name in links if name not in child_names]
    if len(roots) != 1:
        defects.append(f"根 link 判定异常，候选={roots}")
    root_link = roots[0] if roots else next(iter(links))

    return UrdfModel(
        name=root.get("name", urdf_path.stem),
        package_root=urdf_path.parent.parent,
        links=links,
        joints=joints,
        root_link=root_link,
        defects=sorted(set(defects)),
    )


def find_default_urdf(start: Path) -> Path | None:
    """在仓库同级目录里找厂商给的 URDF（用户没在 config 里指定时的兜底）。"""
    start = Path(start).resolve()
    if start.is_file():
        start = start.parent
    candidates = [
        start.parent / "树莓派对应机械臂控制" / "3 功能包文件" / "simulations" / "simulations"
        / "armpi_ultra_description" / "urdf" / "armpi_ultra.urdf",
        start / "sim" / "armpi_ultra.urdf",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def to_web_payload(model: UrdfModel, servo_map: dict) -> dict:
    """导出前端一次就能建好场景的 JSON。"""
    joints = []
    for joint in model.joints.values():
        servo_id = servo_map.get(joint.name)
        joints.append(
            {
                "name": joint.name,
                "type": joint.joint_type,
                "parent": joint.parent,
                "child": joint.child,
                "matrix": joint.matrix,
                "axis": list(joint.axis),
                "lower": joint.lower,
                "upper": joint.upper,
                "servo_id": servo_id,
            }
        )
    links = [
        {
            "name": link.name,
            "mesh": link.mesh,
            "mesh_matrix": link.mesh_origin_matrix,
        }
        for link in model.links.values()
    ]
    return {
        "robot": model.name,
        "root_link": model.root_link,
        "links": links,
        "joints": joints,
        "defects": model.defects,
        "gripper_mimic": GRIPPER_MIMIC,
    }
