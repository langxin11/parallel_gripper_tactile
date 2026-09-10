"""兼容入口：运行二阶直接力矩 ADRC 两阶段调参研究。"""

from parallel_gripper_tactile.studies.protocols.force_tracking_torque_adrc_tuning import *  # noqa: F403
from parallel_gripper_tactile.studies.protocols.force_tracking_torque_adrc_tuning import main
from parallel_gripper_tactile.studies.protocols import (
    force_tracking_torque_adrc_tuning as _protocol,
)


_aggregate = _protocol._aggregate


if __name__ == "__main__":
    main()
