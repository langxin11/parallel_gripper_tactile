"""使用 :class:`mujoco.MjSpec` 在运行时组合 Robotiq 抓取场景。"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GRIPPER_XML = (
    REPOSITORY_ROOT / "assets" / "grippers" / "robotiq_2f85" / "2f85_taxels.xml"
)
DEFAULT_TOUCH_GRID_XML = (
    REPOSITORY_ROOT / "assets" / "grippers" / "robotiq_2f85" / "2f85_touch_grid.xml"
)
GRASP_WORLD_XML = REPOSITORY_ROOT / "assets" / "scenes" / "grasp_world.xml"
TARGET_CUBE_XML = REPOSITORY_ROOT / "assets" / "objects" / "target_cube.xml"
GRIPPER_PREFIX = "gripper/"
CUBE_PREFIX = "cube/"

_GRIPPER_POS = (0.0, 0.0, 0.08)
_GRIPPER_QUAT = (0.70710678, 0.70710678, 0.0, 0.0)
_CUBE_POS = (0.0, -0.155, 0.08)


def prefixed_gripper_name(name: str) -> str:
    """返回 attach 后夹爪实体中对象的名称。"""
    return f"{GRIPPER_PREFIX}{name}"


def _load_gripper_spec(mujoco, gripper_xml: Path):
    """加载夹爪，并移除与场景内存池冲突的旧式容量上限。"""
    tree = ET.parse(gripper_xml)
    root = tree.getroot()
    size = root.find("size")
    if size is not None:
        # ``memory`` 与 ``njmax/nconmax`` 不能共存。组合场景统一采用基础环境
        # 的 memory arena，taxel 资产中的两个旧式上限因此不应参与 attach 冲突。
        size.attrib.pop("njmax", None)
        size.attrib.pop("nconmax", None)
    compiler = root.find("compiler")
    if compiler is not None and (meshdir := compiler.get("meshdir")):
        # from_string 不具备 from_file 的相对路径上下文，故将 meshdir 固化为绝对路径。
        compiler.set("meshdir", str((gripper_xml.parent / meshdir).resolve()))
    return mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))


def gripper_name_in_model(mujoco, model, name: str) -> str:
    """兼容运行时组合模型和 ``--scene`` 传入的旧版完整 MJCF。"""
    prefixed = prefixed_gripper_name(name)
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, prefixed) != -1:
        return prefixed
    return name


def build_grasp_spec(gripper_xml: Path = DEFAULT_GRIPPER_XML):
    """组合基础环境、水平安装的夹爪和自由方块，返回未编译的 ``MjSpec``。"""
    import mujoco

    scene = mujoco.MjSpec.from_file(str(GRASP_WORLD_XML))
    gripper = _load_gripper_spec(mujoco, gripper_xml)
    cube = mujoco.MjSpec.from_file(str(TARGET_CUBE_XML))

    # attach 只能挂到 frame/site；frame 同时定义实体在世界中的安装位姿。
    gripper_mount = scene.worldbody.add_frame(
        name="gripper_mount", pos=list(_GRIPPER_POS), quat=list(_GRIPPER_QUAT)
    )
    cube_mount = scene.worldbody.add_frame(name="cube_mount", pos=list(_CUBE_POS))
    scene.attach(gripper, prefix=GRIPPER_PREFIX, frame=gripper_mount)
    scene.attach(cube, prefix=CUBE_PREFIX, frame=cube_mount)

    # 场景级预设，不依赖任何派生 cube_grasp*.xml 文件。
    scene.add_key(name="open", ctrl=[0.0])
    scene.add_key(name="closed", ctrl=[220.0])
    return scene


def load_grasp_model(scene_xml: Path | None, gripper_xml: Path):
    """加载用户完整场景，或在未指定时编译运行时组合场景。"""
    import mujoco

    if scene_xml is not None:
        return mujoco.MjModel.from_xml_path(str(scene_xml))
    return build_grasp_spec(gripper_xml).compile()
