"""兼容入口：运行力跟踪 PID 模块消融研究。"""

from parallel_gripper_tactile.studies.protocols.force_tracking_ablation import *  # noqa: F403
from parallel_gripper_tactile.studies.protocols.force_tracking_ablation import main


if __name__ == "__main__":
    main()
