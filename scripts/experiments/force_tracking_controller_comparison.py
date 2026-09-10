"""兼容入口：运行力跟踪控制器对比研究。"""

from parallel_gripper_tactile.studies.protocols.force_tracking_controller_comparison import *  # noqa: F403
from parallel_gripper_tactile.studies.protocols.force_tracking_controller_comparison import main
from parallel_gripper_tactile.studies.protocols import (
    force_tracking_controller_comparison as _protocol,
)


_read_tracking_rows = _protocol._read_tracking_rows


if __name__ == "__main__":
    main()
