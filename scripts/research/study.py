"""使用 Hydra 预览或执行一项完整正式研究。"""

from __future__ import annotations

import json
from pathlib import Path
import sys

import hydra
from hydra.core.hydra_config import HydraConfig
from hydra.types import RunMode
from omegaconf import DictConfig

from parallel_gripper_tactile.research.hydra_support import (
    composition_provenance,
    register_resolvers,
    resolved_mapping,
)
from parallel_gripper_tactile.research.configuration import ResearchConfigurationError
from parallel_gripper_tactile.research.study import (
    ResearchStudySetupError,
    execute_research_study,
    resolve_research_study,
)
from parallel_gripper_tactile.studies.lifecycle import record_study_setup_failure


register_resolvers()


def reject_outer_multirun(arguments: list[str]) -> None:
    """在 Hydra 调度 job 前拒绝正式 study 的外层 Multirun。"""
    if any(
        argument in {"-m", "--multirun"} or argument.upper() == "HYDRA.MODE=MULTIRUN"
        for argument in arguments
    ):
        raise SystemExit("正式 study 内部负责唯一矩阵展开，禁止 Hydra 外层 Multirun。")


@hydra.main(
    version_base="1.3",
    config_path="../../configs",
    config_name="study",
)
def main(config: DictConfig) -> None:
    """在外层 Multirun 防护后解析、计划或执行正式研究。"""
    output_directory = Path(HydraConfig.get().runtime.output_dir)
    if HydraConfig.get().mode == RunMode.MULTIRUN:
        raise ResearchConfigurationError("正式 study 禁止 Hydra 外层 Multirun")
    provenance = composition_provenance()
    try:
        resolved = resolve_research_study(resolved_mapping(config))
    except ResearchStudySetupError as error:
        output_directory.mkdir(parents=True, exist_ok=True)
        provenance_path = output_directory / "composition_provenance.json"
        provenance_path.write_text(
            json.dumps(provenance, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        raw = resolved_mapping(config)
        study = raw.get("study")
        study_kind = study.get("kind") if isinstance(study, dict) else None
        record_study_setup_failure(
            study_directory=output_directory,
            stage=error.stage,
            error=error,
            study_kind=str(study_kind) if study_kind is not None else None,
            artifacts=(provenance_path,),
        )
        raise
    result = execute_research_study(
        resolved,
        hydra_output_directory=output_directory,
        provenance=provenance,
    )
    print(f"Study {resolved.selection.execution.mode}: {result}")


if __name__ == "__main__":
    reject_outer_multirun(sys.argv[1:])
    main()
