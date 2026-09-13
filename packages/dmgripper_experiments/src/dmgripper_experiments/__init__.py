"""DMgripper 纯 Python 真机实验。"""

from .config import (
    ExperimentConfig,
    experiment_config_record,
    load_experiment_config,
    sanitize_directory_component,
)
from .lifecycle import Lifecycle, LifecyclePhase
from .recording import create_run_directory

__version__ = "0.2.0"
__all__ = [
    "ExperimentConfig",
    "Lifecycle",
    "LifecyclePhase",
    "create_run_directory",
    "experiment_config_record",
    "load_experiment_config",
    "sanitize_directory_component",
]
