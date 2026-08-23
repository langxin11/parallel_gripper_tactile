"""闭合抓取方块，并以 Rerun 显示两侧 touch_grid 的切向力与压力。

常见用法::

    uv run scripts/run_touch_grid_demo.py
    uv run scripts/run_touch_grid_demo.py --auto-close
    uv run scripts/run_touch_grid_demo.py --auto-close --no-viewer --no-rerun
    uv run scripts/run_touch_grid_demo.py --render-fps 30
    uv run scripts/run_touch_grid_demo.py --auto-close --record-rrd outputs/robotiq/touch_grid_demo/touch_grid.rrd
"""

from __future__ import annotations

import argparse
from pathlib import Path

from grasp_scene import DEFAULT_TOUCH_GRID_XML, gripper_name_in_model, load_grasp_model
from recording import ForceCsvRecorder, RerunTactileLogger, TactileFrame, run_demo_loop


def _touch_grid_shape(mujoco, model, name: str) -> tuple[int, int]:
    """从 touch_grid 插件配置读取 ``(rows, cols)`` 分辨率。"""
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    plugin_id = model.sensor_plugin[sensor_id]
    get_config = getattr(mujoco, "mj_getPluginConfig", None)
    if get_config is not None:
        size = get_config(model, plugin_id, "size")
    else:
        # Python 绑定 3.10 及更早版本未暴露 mj_getPluginConfig。touch_grid 的
        # 属性注册顺序固定为 nchannel、size、fov、gamma，因此 size 是第二项。
        start = model.plugin_attradr[plugin_id]
        end = (
            model.plugin_attradr[plugin_id + 1]
            if plugin_id + 1 < model.nplugin
            else model.npluginattr
        )
        size = bytes(model.plugin_attr[start:end]).split(b"\0")[1].decode()
    cols, rows = map(int, size.split())
    return rows, cols


def _read_tactile(data, name: str, shape: tuple[int, int]):
    """读取插件输出，并从参考项目的 zxy 顺序转换为 xyz 顺序。"""
    return data.sensor(name).data.reshape((3, *shape))[[1, 2, 0]]


def main() -> None:
    """实时运行手动抓取，并显示左右触觉图。

    默认由 MuJoCo viewer 的控制滑块直接写入 ``fingers_actuator``；本脚本不覆盖
    ``data.ctrl``，因此可在闭合过程中观察 Rerun 触觉图。传入 ``--auto-close``
    时才执行预设的自动闭合轨迹。MuJoCo viewer 使用自由相机，支持鼠标
    旋转、平移和缩放场景。
    """
    try:
        import mujoco
    except ModuleNotFoundError as error:
        raise ModuleNotFoundError("请先使用 `uv sync` 安装项目依赖。") from error

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, help="加载外部完整 MJCF，而非运行时 attach 场景。")
    parser.add_argument("--gripper-xml", type=Path, default=DEFAULT_TOUCH_GRID_XML)
    parser.add_argument("--steps", type=int, default=1500)
    parser.add_argument(
        "--close-control", type=float, default=220, help="最终夹爪控制量，范围 0~255。"
    )
    parser.add_argument("--no-viewer", action="store_true", help="不打开 MuJoCo 交互式 viewer。")
    parser.add_argument("--auto-close", action="store_true", help="按预设轨迹自动闭合夹爪。")
    parser.add_argument(
        "--render-fps", type=float, default=60.0, help="目标 viewer 帧率，默认 60 FPS。"
    )
    parser.add_argument("--record-csv", type=Path, help="将控制量和左右三维力写入 CSV。")
    parser.add_argument("--record-every", type=int, default=1, help="每隔多少物理步记录一次。")
    parser.add_argument("--no-rerun", action="store_true", help="不启动实时 Rerun 触觉仪表盘。")
    parser.add_argument("--record-rrd", type=Path, help="将完整触觉网格写入 Rerun RRD。")
    parser.add_argument(
        "--rerun-hz", type=float, default=100.0, help="Rerun 采样频率，默认 100 Hz。"
    )
    args = parser.parse_args()
    if args.no_viewer and not args.auto_close:
        parser.error("手动模式需要 MuJoCo viewer；无界面运行请同时传入 --auto-close。")
    if args.render_fps <= 0:
        parser.error("--render-fps 必须为正数。")
    if args.record_every <= 0:
        parser.error("--record-every 必须为正整数。")
    if args.rerun_hz <= 0:
        parser.error("--rerun-hz 必须为正数。")
    model = load_grasp_model(args.scene, args.gripper_xml)
    physics_hz = 1.0 / model.opt.timestep
    if args.rerun_hz > physics_hz:
        parser.error(f"--rerun-hz 不能超过 MuJoCo 物理频率 {physics_hz:g} Hz。")
    data = mujoco.MjData(model)
    recorder = ForceCsvRecorder(args.record_csv, args.record_every)
    left_sensor = gripper_name_in_model(mujoco, model, "touch_left")
    right_sensor = gripper_name_in_model(mujoco, model, "touch_right")
    left_shape = _touch_grid_shape(mujoco, model, left_sensor)
    right_shape = _touch_grid_shape(mujoco, model, right_sensor)
    rerun_logger = RerunTactileLogger(
        "robotiq2f85_touch_grid_grasp",
        live=not args.no_rerun,
        path=args.record_rrd,
        hz=args.rerun_hz,
    )

    def sample_frame(step: int) -> TactileFrame:
        return TactileFrame(
            step=step,
            time_s=data.time,
            control=float(data.ctrl[0]),
            left=_read_tactile(data, left_sensor, left_shape),
            right=_read_tactile(data, right_sensor, right_shape),
        )

    def control_at(step: int) -> float:
        return args.close_control * min(1.0, step / max(1, args.steps // 3))

    run_demo_loop(
        model=model,
        data=data,
        steps=args.steps,
        auto_close=args.auto_close,
        no_viewer=args.no_viewer,
        render_fps=args.render_fps,
        control_at=control_at,
        sample_frame=sample_frame,
        recorder=recorder,
        rerun_logger=rerun_logger,
    )


if __name__ == "__main__":
    main()
