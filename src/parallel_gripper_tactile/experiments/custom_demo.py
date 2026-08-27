"""构建一个聚焦的自研夹爪触觉接触演示场景。"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_PROFILE = REPOSITORY_ROOT / "configs" / "custom_parallel_gripper.yaml"
CUBE_FREEJOINT = "cube_shear_freejoint"
CUBE_MOCAP = "cube_shear_mocap"
# 经典地面与背景复用 ``assets/scenes/grasp_world.xml`` 里的棋盘地面、skybox 渐变
# 背景与灯光；这些纯视觉元素不参与碰撞，避免影响触觉接触力。
GROUNDPLANE_TEXTURE = "groundplane"
SKYBOX_TEXTURE = "skybox"
FLOOR_GEOM_NAME = "floor"
MAX_PHYSICS_STEPS_PER_UI_TICK = 50


def _parse_taxel(value: str) -> tuple[str, int, int]:
    """解析紧凑的 ``left:11`` 或 ``right:02`` taxel 选择器。"""
    try:
        side, index = value.split(":", maxsplit=1)
        row, column = int(index[0]), int(index[1])
    except (IndexError, ValueError) as error:
        raise ValueError("taxel must have the form left:11 or right:02") from error
    if side not in {"left", "right"} or row not in range(3) or column not in range(3):
        raise ValueError("taxel must be in left/right:00 through left/right:22")
    return side, row, column


def _taxel_position(profile, side: str, row: int, column: int) -> np.ndarray:
    """从准备好的模型中读取命名触觉 site 的默认位置。"""
    source_model = mujoco.MjModel.from_xml_path(str(profile.model_path))
    source_data = mujoco.MjData(source_model)
    mujoco.mj_forward(source_model, source_data)
    geom_name = profile.tactile.names(side)[row * profile.tactile.cols + column]
    site_name = geom_name.replace("_geom_", "_", 1)
    return source_data.site_xpos[source_model.site(site_name).id].copy()


def _inject_world_visual(root: ET.Element) -> None:
    """注入 ``grasp_world.xml`` 的经典棋盘地面与 skybox 渐变背景。

    这些只是视觉元素：地面 geom 设 ``contype/conaffinity=0`` 不参与碰撞，
    只提供经典视角与背景，不改变触觉接触力。
    """
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    if asset.find(f"texture[@name='{GROUNDPLANE_TEXTURE}']") is None:
        ET.SubElement(
            asset,
            "texture",
            name=GROUNDPLANE_TEXTURE,
            type="2d",
            builtin="checker",
            mark="edge",
            rgb1="0.2 0.3 0.4",
            rgb2="0.1 0.2 0.3",
            markrgb="0.8 0.8 0.8",
            width="300",
            height="300",
        )
        ET.SubElement(
            asset,
            "material",
            name=GROUNDPLANE_TEXTURE,
            texture=GROUNDPLANE_TEXTURE,
            texuniform="true",
            texrepeat="5 5",
            reflectance="0.2",
        )
    if asset.find(f"texture[@name='{SKYBOX_TEXTURE}']") is None:
        ET.SubElement(
            asset,
            "texture",
            name=SKYBOX_TEXTURE,
            type="skybox",
            builtin="gradient",
            rgb1="0.3 0.5 0.7",
            rgb2="0 0 0",
            width="512",
            height="3072",
        )
    if root.find("visual") is None:
        visual = ET.SubElement(root, "visual")
        ET.SubElement(
            visual, "headlight", diffuse="0.6 0.6 0.6", ambient="0.3 0.3 0.3", specular="0 0 0"
        )
        ET.SubElement(visual, "rgba", haze="0.15 0.25 0.35 1")
        ET.SubElement(
            visual, "global", azimuth="160", elevation="-20", offwidth="1920", offheight="1080"
        )
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("model is missing worldbody")
    if (
        worldbody.find(f"geom[@name='{FLOOR_GEOM_NAME}']") is None
        and worldbody.find("light") is None
    ):
        ET.SubElement(worldbody, "light", pos="0 0 3.5", dir="0 0 -1", directional="true")
    if worldbody.find(f"geom[@name='{FLOOR_GEOM_NAME}']") is None:
        ET.SubElement(
            worldbody,
            "geom",
            name=FLOOR_GEOM_NAME,
            size="0 0 0.05",
            pos="0 0 0",
            type="plane",
            material=GROUNDPLANE_TEXTURE,
            friction="0.9 0.02 0.001",
            contype="0",
            conaffinity="0",
        )


def build_demo_model(
    profile,
    taxel: tuple[str, int, int],
    cube_half_size: float,
    *,
    shear_axis: str | None = None,
    pillar_type: str = "mesh",
) -> mujoco.MjModel:
    """构建固定基座模型，并在单个触觉 site 前放置静态方块。

    场景沿用 ``assets/scenes/grasp_world.xml`` 的经典棋盘地面与 skybox 渐变
    背景，但只作为纯视觉注入；物理求解器、固定基座与方块保持旧实现，名称
    结构不变（``ContactTaxelReader``/``MITTorqueController`` 仍按原名解析
    taxel geom、site 与执行器）。
    """
    if cube_half_size <= 0:
        raise ValueError("cube_half_size must be positive")
    tree = ET.parse(profile.model_path)
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is not None and (meshdir := compiler.get("meshdir")):
        compiler.set("meshdir", str((profile.model_path.parent / meshdir).resolve()))

    # 导出的根是自由的，以便外部机械臂安装；这个独立的触觉测试改为固定
    # base，让静态方块测试可行。
    base = root.find(".//body[@name='base']")
    if base is None:
        raise ValueError("model is missing base body")
    for freejoint in base.findall("freejoint"):
        base.remove(freejoint)

    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("gravity", "0 0 0")
    _inject_world_visual(root)

    side, row, column = taxel
    if pillar_type not in {"mesh", "sdf"}:
        raise ValueError("pillar_type must be 'mesh' or 'sdf'")
    if pillar_type == "sdf":
        geom_name = profile.tactile.names(side)[row * profile.tactile.cols + column]
        pillar = root.find(f".//geom[@name='{geom_name}']")
        if pillar is None:
            raise ValueError(f"model is missing tactile geom {geom_name!r}")
        pillar.set("type", "sdf")
    taxel_position = _taxel_position(profile, side, row, column)
    cube_position = (0.0, taxel_position[1], taxel_position[2])
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("model is missing worldbody")
    cube = ET.SubElement(
        worldbody,
        "body",
        name="target_cube",
        pos=" ".join(f"{value:.9g}" for value in cube_position),
    )
    if shear_axis is not None:
        if shear_axis not in {"y", "z"}:
            raise ValueError("shear_axis must be 'y', 'z', or None")
        # 仿真循环通过刚性 weld 把该自由体焊到 mocap 测试夹具上，以指定其
        # 位置与切向速度。
        ET.SubElement(cube, "freejoint", name=CUBE_FREEJOINT)
    cube_geom_attributes = {
        "name": "target_cube_geom",
        "type": "box",
        "size": f"0.012 {cube_half_size:.9g} {cube_half_size:.9g}",
        "friction": "0.8 0.02 0.001",
        "rgba": "0.95 0.55 0.1 1",
    }
    ET.SubElement(cube, "geom", **cube_geom_attributes)
    if shear_axis is not None:
        ET.SubElement(
            worldbody,
            "body",
            name=CUBE_MOCAP,
            mocap="true",
            pos=" ".join(f"{value:.9g}" for value in cube_position),
        )
        equality = root.find("equality")
        if equality is None:
            equality = ET.SubElement(root, "equality")
        ET.SubElement(
            equality,
            "weld",
            name="cube_shear_weld",
            body1="target_cube",
            body2=CUBE_MOCAP,
            solref="0.002 1",
            solimp="0.99 0.999 0.0005 0.5 2",
        )
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
