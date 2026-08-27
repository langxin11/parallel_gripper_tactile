"""构建自研夹爪的水平固定基座抓取场景。"""

from __future__ import annotations

from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np

from ..profiles import GripperProfile


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_PROFILE = REPOSITORY_ROOT / "configs" / "custom_parallel_gripper.yaml"
GRASP_WORLD_XML = REPOSITORY_ROOT / "assets" / "scenes" / "grasp_world.xml"
TARGET_CUBE_XML = REPOSITORY_ROOT / "assets" / "objects" / "target_cube.xml"
GRIPPER_PREFIX = "gripper/"
CUBE_PREFIX = "cube/"
SUPPORT_GEOM_NAME = "target_cube_support_plate"
DEFAULT_CUBE_HALF_THICKNESS = 0.003
DEFAULT_CUBE_HALF_CONTACT_SIDE = 0.0125
MIN_CUBE_MASS = 0.050
DEFAULT_CUBE_MASS = MIN_CUBE_MASS
PILLAR_ALIGNMENT_OFFSET_IN_BASE = np.array((0.0, 0.0, -0.002))


def _quaternion_rotate(
    quaternion: tuple[float, float, float, float], vector: np.ndarray
) -> np.ndarray:
    """用 MuJoCo ``wxyz`` 四元数旋转 ``vector``。"""
    w, x, y, z = quaternion
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0:
        raise ValueError("mount quaternion must not be zero")
    w, x, y, z = (value / norm for value in (w, x, y, z))
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    return rotation @ vector


def _load_fixed_gripper_spec(mujoco, profile: GripperProfile):
    """加载夹爪，并移除预留给外部安装的 ``base_freejoint``。

    源模型刻意带有 ``base_freejoint``，以便日后安装到机械臂或转接法兰。
    抓取实验需要等价于法兰被刚性紧固的状态，因此这里只移除该自由关节，
    不修改 CAD 基础本体。
    """
    tree = ET.parse(profile.model_path)
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is not None and (meshdir := compiler.get("meshdir")):
        compiler.set("meshdir", str((profile.model_path.parent / meshdir).resolve()))
    base = root.find(".//body[@name='base']")
    if base is None:
        raise ValueError("custom gripper model is missing the reserved base body")
    for freejoint in base.findall("freejoint"):
        base.remove(freejoint)
    return mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))


def _central_taxel_site_name(profile: GripperProfile, side: str) -> str:
    """按 profile 网格尺寸返回中央 taxel 对应 site 的名称。"""
    rows, cols = profile.tactile.rows, profile.tactile.cols
    middle = rows // 2 * cols + cols // 2
    geom_name = profile.tactile.names(side)[middle]
    return geom_name.replace("_geom_", "_", 1)


def tactile_center_in_base(profile: GripperProfile) -> np.ndarray:
    """返回张开状态下两侧中央触觉 site 的中点（base 局部系）。"""
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(profile.model_path))
    data = mujoco.MjData(model)
    drive_qpos = model.jnt_qposadr[model.joint(profile.actuator).id]
    data.qpos[drive_qpos] = profile.open_control
    mujoco.mj_forward(model, data)
    left = data.site_xpos[model.site(_central_taxel_site_name(profile, "left")).id]
    right = data.site_xpos[model.site(_central_taxel_site_name(profile, "right")).id]
    return 0.5 * (left + right)


def _load_cube_spec(
    mujoco,
    cube_half_thickness: float,
    cube_half_contact_side: float,
    cube_mass: float,
):
    """加载 YZ 面 25×25 mm 且能放入夹爪开口的测试块。"""
    if cube_half_thickness <= 0 or cube_half_contact_side <= 0:
        raise ValueError("cube dimensions must be positive")
    if cube_mass < MIN_CUBE_MASS:
        raise ValueError(f"cube_mass must be at least {MIN_CUBE_MASS:g} kg")
    tree = ET.parse(TARGET_CUBE_XML)
    cube = tree.find(".//geom[@name='target_cube_geom']")
    if cube is None:
        raise ValueError("target cube asset is missing target_cube_geom")
    cube.set(
        "size",
        f"{cube_half_thickness:.9g} {cube_half_contact_side:.9g} {cube_half_contact_side:.9g}",
    )
    cube.set("mass", f"{cube_mass:.9g}")
    # 临时支撑板使用碰撞位 2；让方块与它接触，同时保留与夹爪 Pillars 的
    # 常规位 1 接触。
    cube.set("conaffinity", "2")
    return mujoco.MjSpec.from_string(ET.tostring(tree.getroot(), encoding="unicode"))


def build_custom_grasp_spec(
    profile: GripperProfile,
    *,
    cube_half_thickness: float = DEFAULT_CUBE_HALF_THICKNESS,
    cube_half_contact_side: float = DEFAULT_CUBE_HALF_CONTACT_SIDE,
    cube_mass: float = DEFAULT_CUBE_MASS,
):
    """附加固定自研基座、自由方块与临时支撑。

    profile 把滑轨和夹爪本体都映射到地面平面。物理指尖面遵循模型的 YZ
    接触平面约定，因此测试块在 X 方向厚 6 mm，YZ 接触面为 25×25 mm。
    """
    import mujoco

    center_base = tactile_center_in_base(profile)
    mount_position = np.asarray(profile.mount_pos, dtype=np.float64)
    cube_position = mount_position + _quaternion_rotate(profile.mount_quat, center_base)
    # 导出的 Pillar 网格表面略低于其 taxel site 原点。让测试块对齐物理网格
    # 而非可视化 site，保证撤去支撑前两指都已接触。
    cube_position += _quaternion_rotate(profile.mount_quat, PILLAR_ALIGNMENT_OFFSET_IN_BASE)

    world_tree = ET.parse(GRASP_WORLD_XML)
    support = world_tree.find(f".//geom[@name='{SUPPORT_GEOM_NAME}']")
    if support is None:
        raise ValueError(f"grasp world is missing {SUPPORT_GEOM_NAME!r}")
    support.set(
        "pos",
        f"{cube_position[0]:.9g} {cube_position[1]:.9g} "
        f"{cube_position[2] - cube_half_contact_side - 0.0025:.9g}",
    )
    # 共享的 Robotiq 场景使用 50 mm 支撑板，会与本紧凑夹爪的 Pillars 重叠；
    # 这里保持相同的临时支撑语义，但缩小到只覆盖选定的测试块。
    support_half_x = 1.25 * cube_half_thickness
    support_half_y = 1.25 * cube_half_contact_side
    support.set("size", f"{support_half_x:.9g} {support_half_y:.9g} 0.0025")
    # 水平放置时夹爪下部结构会越过支撑板的投影范围。位 2 让支撑板只与方块
    # 接触，避免稳定阶段出现夹爪—支撑板的虚假反力。
    support.set("contype", "2")
    support.set("conaffinity", "2")
    scene = mujoco.MjSpec.from_string(ET.tostring(world_tree.getroot(), encoding="unicode"))

    gripper_mount = scene.worldbody.add_frame(
        name="custom_gripper_mount", pos=list(profile.mount_pos), quat=list(profile.mount_quat)
    )
    cube_mount = scene.worldbody.add_frame(name="custom_cube_mount", pos=cube_position.tolist())
    scene.attach(
        _load_fixed_gripper_spec(mujoco, profile), prefix=GRIPPER_PREFIX, frame=gripper_mount
    )
    scene.attach(
        _load_cube_spec(mujoco, cube_half_thickness, cube_half_contact_side, cube_mass),
        prefix=CUBE_PREFIX,
        frame=cube_mount,
    )
    # 自研执行器是纯力矩源。位置预设应放在 qpos/控制器状态里，
    # 而零力矩是唯一安全的通用 keyframe 命令。
    scene.add_key(name="custom_open", ctrl=[0.0])
    scene.add_key(name="custom_closed", ctrl=[0.0])
    return scene


def build_custom_grasp_model(
    profile: GripperProfile,
    *,
    cube_half_thickness: float = DEFAULT_CUBE_HALF_THICKNESS,
    cube_half_contact_side: float = DEFAULT_CUBE_HALF_CONTACT_SIDE,
    cube_mass: float = DEFAULT_CUBE_MASS,
):
    """编译水平安装的自研夹爪抓取场景。"""
    return build_custom_grasp_spec(
        profile,
        cube_half_thickness=cube_half_thickness,
        cube_half_contact_side=cube_half_contact_side,
        cube_mass=cube_mass,
    ).compile()
