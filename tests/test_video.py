"""视频渲染辅助函数测试。"""

import mujoco
import numpy as np

from parallel_gripper_tactile.video import add_arrow_to_scene


def test_add_arrow_clears_reused_scene_metadata() -> None:
    """自定义箭头不应继承复用槽位中的标签和对象编号。"""
    model = mujoco.MjModel.from_xml_string("<mujoco/>")
    scene = mujoco.MjvScene(model, maxgeom=2)
    slot = scene.geoms[0]
    slot.label = "a single value is returned. Returns"
    slot.objid = 42
    slot.reflectance = 0.73

    used = add_arrow_to_scene(
        scene,
        0,
        np.array([0.0, 0.0, 0.1]),
        np.array([0.0, 0.0, -1.0]),
        scale=0.04,
    )

    assert used == 1
    assert scene.ngeom == 1
    assert slot.label == ""
    assert slot.objid == -1
    assert slot.dataid == -1
    assert slot.reflectance == 0.0
