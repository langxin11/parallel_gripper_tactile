"""仅依赖触觉历史的切向扰动增力策略，不消费外载或物体运动真值。"""

from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, FiniteFloat, model_validator

from dm_grasp_core.grasp.disturbance import (
    DisturbanceCommand as DisturbanceCommand,
    TactileDisturbancePolicy as TactileDisturbancePolicy,
)


class DisturbancePolicyConfig(BaseModel):
    """平均单侧法向目标的单调增力参数。"""

    model_config = ConfigDict(extra="forbid", frozen=True)
    strategy: Literal["constant", "fixed_step", "dynamic_step"] = "dynamic_step"
    detector: Literal["shear_increase", "force_ratio"] = "shear_increase"
    initial_force_n: Annotated[FiniteFloat, Field(gt=0)] = 0.8
    max_force_n: Annotated[FiniteFloat, Field(gt=0)] = 5.0
    max_force_rate_n_s: Annotated[FiniteFloat, Field(gt=0)] = 30.0
    update_period_s: Annotated[FiniteFloat, Field(gt=0)] = 0.01
    filter_tau_s: Annotated[FiniteFloat, Field(gt=0)] = 0.005
    confirm_time_s: Annotated[FiniteFloat, Field(gt=0)] = 0.006
    shear_threshold_n: Annotated[FiniteFloat, Field(gt=0)] = 0.06
    shear_gain: Annotated[FiniteFloat, Field(gt=0)] = 1.0
    fixed_step_n: Annotated[FiniteFloat, Field(gt=0)] = 0.15
    min_step_n: Annotated[FiniteFloat, Field(gt=0)] = 0.03
    max_step_n: Annotated[FiniteFloat, Field(gt=0)] = 0.3
    ratio_window_s: Annotated[FiniteFloat, Field(gt=0)] = 0.06
    normal_delta_floor_n: Annotated[FiniteFloat, Field(gt=0)] = 0.02
    ratio_threshold: FiniteFloat = -0.1
    closure_scale_m: Annotated[FiniteFloat, Field(gt=0)] = 0.01
    contact_floor_n: Annotated[FiniteFloat, Field(gt=0)] = 0.05

    @model_validator(mode="after")
    def check_limits(self):
        """拒绝颠倒的力和步长边界。"""
        if self.max_force_n < self.initial_force_n or self.max_step_n < self.min_step_n:
            raise ValueError("force and step limits must be ordered")
        return self
