"""从迁移前 ROS 源文件生成固定轨迹；显式传入源目录，测试时不执行。"""

import ast
import hashlib
import json
from pathlib import Path
import sys
import types


def generate(source_dir):
    """执行指定原始源目录中的节点数学步骤，返回带来源哈希的固定轨迹。"""
    control_path = source_dir / "control.py"
    node_path = source_dir / "force_tracking_node.py"
    namespace = {"__name__": "_golden_original_control"}
    module = types.ModuleType(namespace["__name__"])
    sys.modules[module.__name__] = module
    exec(compile(control_path.read_text(), str(control_path), "exec"), module.__dict__)
    tree = ast.parse(node_path.read_text())
    cls = next(
        n
        for n in tree.body
        if isinstance(n, ast.ClassDef)
        and any(getattr(x, "name", "") == "_build_command" for x in n.body)
    )
    build = next(n for n in cls.body if getattr(n, "name", "") == "_build_command")
    tick = next(
        n
        for n in cls.body
        if any(
            isinstance(x, ast.Assign)
            and any(isinstance(t, ast.Tuple) for t in x.targets)
            and "self._admittance.step" in ast.unparse(x)
            for x in ast.walk(n)
        )
    )
    # 原样提取跟踪语句，在 ROS 发布命令前停止。
    body = next(
        n.body
        for n in ast.walk(tick)
        if isinstance(n, ast.Try) and any("self._admittance.step" in ast.unparse(x) for x in n.body)
    )
    start = next(
        i
        for i, n in enumerate(body)
        if isinstance(n, ast.Assign) and "self._admittance.step" in ast.unparse(n)
    )
    stop = next(
        i
        for i, n in enumerate(body[start:], start)
        if isinstance(n, ast.Expr) and "self._mit_pub.publish" in ast.unparse(n)
    )
    step_code = "\n".join(ast.unparse(n) for n in body[start:stop])
    env = dict(module.__dict__, MitCommand=types.SimpleNamespace)
    exec(
        compile(
            "from __future__ import annotations\n" + ast.unparse(build), "<original-build>", "exec"
        ),
        env,
    )
    params = dict(
        closing_direction=1,
        position_min_rad=0.0,
        position_max_rad=0.9,
        velocity_limit_rad_s=0.4,
        mit_kp=8.0,
        mit_kd=0.2,
        feedforward_ratio=0.7,
        feedforward_torque_limit_nm=0.2,
        mit_torque_limit_nm=0.6,
        target_normal_force_n=3.0,
    )
    kin = module.CrankSliderKinematics(0.7853981633974483, 0.03, 0.04, 0.021213203435596423)
    scenarios = []
    for direction in (1, -1):
        for timing in ([0.002] * 40, [0.004] * 40, [0.001, 0.004, 0.0025, 0.007, 0.003] * 8):
            params["closing_direction"] = direction
            obj = types.SimpleNamespace(
                _kinematics=kin, _admittance=module.SecondOrderAdmittance(0.8, 40.0, 100.0)
            )
            obj.get_parameter = lambda name: types.SimpleNamespace(value=params[name])
            obj._build_command = types.MethodType(env["_build_command"], obj)
            rows = []
            for i, dt in enumerate(timing):
                left = (i % 9) * 0.6
                right = (i % 7) * 0.7
                q = 0.35 + 0.001 * (i % 5)
                dq = 0.03 * ((i % 3) - 1)
                local = dict(
                    self=obj,
                    reference_position_rad=0.35,
                    motor=types.SimpleNamespace(position_rad=q, velocity_rad_s=dq),
                    target_force_n=3.0,
                    measured_force_n=0.5 * (left + right),
                    dt_s=dt,
                )
                exec(step_code, env, local)
                rows.append(
                    dict(
                        inputs=dict(
                            reference_position_rad=0.35,
                            measured_position_rad=q,
                            measured_velocity_rad_s=dq,
                            left_force_n=left,
                            right_force_n=right,
                            target_force_n=3.0,
                            dt_s=dt,
                        ),
                        command=vars(local["command"]),
                        state=[obj._admittance.displacement_m, obj._admittance.velocity_m_s],
                    )
                )
            scenarios.append(dict(direction=direction, rows=rows))
    return dict(
        source_sha256={
            p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (control_path, node_path)
        },
        scenarios=scenarios,
    )


if __name__ == "__main__":
    Path(__file__).with_name("golden_tracking.json").write_text(
        json.dumps(generate(Path(sys.argv[1])), indent=2) + "\n"
    )
