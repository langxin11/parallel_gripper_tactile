"""使用 Hydra 组合、预览或执行一次科研实验。"""

from __future__ import annotations

from pathlib import Path

import hydra
from hydra.core.hydra_config import HydraConfig
from omegaconf import DictConfig

from parallel_gripper_tactile.research import execute_research_run, resolve_research_run
from parallel_gripper_tactile.research.hydra_support import (
    composition_provenance,
    register_resolvers,
    resolved_mapping,
)


register_resolvers()


@hydra.main(version_base="1.3", config_path="../../configs/research", config_name="dm_force_track")
def main(config: DictConfig) -> None:
    """完成组合与领域校验后执行计划预览或单次实验。"""
    resolved = resolve_research_run(resolved_mapping(config))
    output_directory = Path(HydraConfig.get().runtime.output_dir)
    outcome = execute_research_run(
        resolved,
        hydra_output_directory=output_directory,
        provenance=composition_provenance(),
    )
    if outcome.mode == "plan":
        print(f"Validated plan: {outcome.output_directory / 'plan.json'}")
    else:
        print(f"Research run: {outcome.run_directory}")


if __name__ == "__main__":
    main()
