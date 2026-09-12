"""正式研究终端进度的中文纯文本展示。"""

from __future__ import annotations

from parallel_gripper_tactile.studies.lifecycle import StudyProgress


def report_study_progress(progress: StudyProgress) -> None:
    """即时输出已落盘的进度，科学失败与执行异常分别显示。"""
    print(
        f"研究进度：已处理 {progress.finished}/{progress.total}，"
        f"科学失败 {progress.scientific_failures}，执行异常 {progress.execution_errors}，"
        f"状态 {progress.state}，目录：{progress.study_directory.resolve()}",
        flush=True,
    )
