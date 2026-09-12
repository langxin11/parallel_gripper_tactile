"""手托杯、撤手接管、倒水和显式释放的纯计算流程。"""

from dm_grasp_core.grasp.disturbance import DisturbancePolicyParameters, TactileDisturbancePolicy


class CupFlow:
    """保留手托杯基线，在撤手前启用增力，结束计时后保持。"""

    def __init__(self, config):
        """以真机专用较慢限幅创建共用增力策略。"""
        self.config = config
        c, g = config.control, config.grip
        self.policy = TactileDisturbancePolicy(
            DisturbancePolicyParameters(
                initial_force_n=c.target_force_n,
                max_force_n=g.max_target_force_n,
                max_force_rate_n_s=g.max_force_rate_n_s,
                update_period_s=1 / c.control_rate_hz,
                filter_tau_s=g.filter_tau_s,
                shear_threshold_n=g.shear_threshold_n,
                shear_gain=g.shear_gain,
                contact_floor_n=c.contact_off_n or 1e-6,
            )
        )
        self.phase = "supported_hold"
        self.stable_since = self.pour_started = None
        self.previous_shear = None
        self.command = None
        self.release_requested = False

    def action(self, action):
        """ready 仅确认已经撤手；release 表示已托住杯子并许可张开。"""
        if action == "release":
            self.release_requested = True
        elif action == "ready" and self.phase == "takeover":
            self.phase = "stabilizing"
            self.stable_since = None

    def update(
        self,
        *,
        time_s,
        dt_s,
        left_normal_n,
        right_normal_n,
        tangential_force_n,
        signed_tangential_force_n,
        closure_m,
    ):
        """每个新的有效触觉快照只更新一次确认窗口与增力状态。"""
        self.command = self.policy.update(
            left_normal_n=left_normal_n,
            right_normal_n=right_normal_n,
            tangential_force_n=tangential_force_n,
            signed_tangential_force_n=signed_tangential_force_n,
            closure_m=closure_m,
            dt=dt_s,
            enabled=self.phase != "supported_hold",
        )
        c, g = self.config.control, self.config.grip
        force_stable = (
            min(left_normal_n, right_normal_n) >= c.contact_on_n
            and abs((left_normal_n + right_normal_n) / 2 - self.command.target_force_n)
            <= g.force_tolerance_n
        )
        shear_stable = (
            self.previous_shear is not None
            and abs(self.command.measured_tangential_force_n - self.previous_shear)
            <= g.shear_threshold_n
        )
        self.previous_shear = self.command.measured_tangential_force_n
        if self.phase in {"supported_hold", "stabilizing"}:
            stable = force_stable and (self.phase == "supported_hold" or shear_stable)
            self.stable_since = (
                (time_s if self.stable_since is None else self.stable_since) if stable else None
            )
            if self.stable_since is not None and time_s - self.stable_since >= g.stable_time_s:
                self.phase = "takeover" if self.phase == "supported_hold" else "pour"
                if self.phase == "pour":
                    self.pour_started = time_s
                self.stable_since = None
        if self.phase == "pour" and time_s - self.pour_started >= c.tracking_duration_s:
            self.phase = "await_release"
        return self.command
