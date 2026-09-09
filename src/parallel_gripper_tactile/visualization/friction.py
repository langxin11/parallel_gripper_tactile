"""展示纯触觉检测证据与独立的仿真评价量。"""

from pathlib import Path

import numpy as np

from .plotstyle import paper_figsize, save_publication_figure, science_pyplot


def plot_summary(path: Path, rows, *, task):
    """以五个共享时间轴面板区分在线输入、检测与离线评价。"""
    plt = science_pyplot()
    times = np.array([r["time_s"] for r in rows], dtype=float)

    def values(key):
        """读取完整轨迹，不平滑实验结果。"""
        return np.array([r[key] for r in rows], dtype=float)

    fig, axes = plt.subplots(5, 1, sharex=True, figsize=paper_figsize(10), layout="constrained")
    axes[0].plot(
        times,
        values("measured_left_normal_n") + values("measured_right_normal_n"),
        label="Total normal force",
    )
    axes[0].plot(
        times,
        values("measured_left_shear_n") + values("measured_right_shear_n"),
        label="Total tangential force",
    )
    axes[0].plot(times, 2 * values("scheduled_target_force_n"), "--", label="Total normal target")
    axes[0].set_ylabel("Force (N)")
    valid = values("valid").astype(bool)
    axes[1].plot(times, np.where(valid, values("rho_global"), np.nan), label=r"$\rho_{global}$")
    axes[1].plot(times, np.where(valid, values("rho_q95"), np.nan), "--", label=r"$\rho_{q95}$")
    axes[1].set_ylabel("Friction utilization")
    axes[2].plot(times, values("incipient_slip_score"), color="k", label="Tactile change score")
    axes[2].plot(times, values("candidate_threshold"), "--", label="Candidate threshold")
    axes[2].plot(times, values("confirm_threshold"), ":", label="Confirm threshold")
    for state, color, label in (
        ("stable", "#eeeeee", "Stable"),
        ("incipient_slip_candidate", "#e5b45b", "Candidate"),
        ("incipient_slip_confirmed", "#87b6a2", "Confirmed (latched)"),
    ):
        mask = np.array([r["slip_state"] == state for r in rows])
        axes[2].fill_between(
            times,
            0,
            1,
            where=mask,
            color=color,
            alpha=0.4,
            label=label,
            transform=axes[2].get_xaxis_transform(),
        )
    axes[2].set_ylabel("Slip score")
    axes[3].plot(times, values("mu_hat"), label=r"$\hat\mu$ (fallback until confirmation)")
    axes[3].plot(
        times, values("mu_true_score_only"), "--", color="0.5", label=r"True $\mu$ (score only)"
    )
    axes[3].set_ylabel("Friction estimate")
    probe = np.array([r["phase"] in {"probe_settle", "probe", "recovery"} for r in rows])
    hold = np.array([r["phase"] == "schedule_load" for r in rows])
    axes[4].plot(
        times,
        np.where(probe, 1000 * values("release_displacement_m"), np.nan),
        label="Probe / recovery",
    )
    axes[4].plot(times, np.where(hold, 1000 * values("hold_displacement_m"), np.nan), label="Hold")
    axes[4].axhline(
        1000 * task.metrics.probe_slip_threshold_m, ls="--", color="0.4", label="Probe limit"
    )
    axes[4].axhline(
        1000 * task.metrics.hold_slip_threshold_m, ls=":", color="0.2", label="Hold limit"
    )
    axes[4].set_ylabel("Displacement (mm)")
    axes[4].set_xlabel("Simulation time (s)")
    axes[4].set_title("Displacement: evaluation only; not used by estimator", fontsize=8)
    detected = next((float(r["time_s"]) for r in rows if r["slip_detected"]), None)
    for ax in axes:
        if detected is not None:
            ax.axvline(detected, ls="--", color="0.4", lw=0.8)
        ax.legend(fontsize=7, ncol=2, loc="upper left")
        ax.grid(alpha=0.15)
    axes[2].set_title(
        "No confirmed event"
        if detected is None
        else f"Incipient-slip proxy confirmed at {detected:.3f} s",
        fontsize=9,
    )
    axes[2].legend(fontsize=6, ncol=3, loc="lower right")
    if detected is not None:
        zoom = axes[2].inset_axes([0.57, 0.5, 0.40, 0.46])
        selection = (times >= detected - 0.06) & (times <= detected + 0.025)
        zoom.plot(times[selection] - detected, values("incipient_slip_score")[selection], color="k")
        zoom.axhline(task.tactile_slip.candidate_threshold, ls="--", color="r", lw=0.7)
        zoom.axhline(task.tactile_slip.confirm_threshold, ls=":", color="b", lw=0.7)
        candidate = np.array([r["slip_state"] == "incipient_slip_candidate" for r in rows])[
            selection
        ]
        zoom.fill_between(
            times[selection] - detected,
            0,
            1,
            where=candidate,
            color="#e5b45b",
            alpha=0.4,
            transform=zoom.get_xaxis_transform(),
        )
        zoom.axvline(0, ls="--", color="0.4", lw=0.7)
        zoom.set_title("Confirmation detail (time relative to event)", fontsize=6)
        zoom.tick_params(labelsize=6)
    save_publication_figure(fig, path)
    plt.close(fig)


def plot_taxel_diagnostics(path: Path, rows, *, task):
    """按左右真实网格排列时间热图与三个事件快照。"""
    plt = science_pyplot()
    times = np.array([r["time_s"] for r in rows], dtype=float)
    coords = sorted(
        tuple(map(int, key.rsplit("_", 2)[-2:]))
        for key in rows[0]
        if key.startswith("left_taxel_normal_")
    )
    shape = (max(c[0] for c in coords) + 1, max(c[1] for c in coords) + 1)
    confirmed = next((i for i, r in enumerate(rows) if r["slip_detected"]), None)
    stable = [
        i for i, r in enumerate(rows) if r["phase"] == "probe" and r["slip_state"] == "stable"
    ]
    snapshots = [
        stable[len(stable) // 2] if stable else 0,
        None if confirmed is None else max(0, confirmed - 1),
        confirmed,
    ]
    fig = plt.figure(figsize=paper_figsize(13), layout="constrained")
    outer = fig.add_gridspec(2, 1, height_ratios=[1, 1.5])
    heat = outer[0].subgridspec(2, 2)
    spaces = outer[1].subgridspec(6, 3)
    for side_index, side in enumerate(("left", "right")):
        matrices = {}
        for field in ("normal", "shear", "ratio"):
            matrices[field] = np.array(
                [[r[f"{side}_taxel_{field}_{a}_{b}"] for a, b in coords] for r in rows], dtype=float
            )
        for j, field in enumerate(("normal", "ratio")):
            ax = fig.add_subplot(heat[j, side_index])
            data = matrices[field]
            finite = data[np.isfinite(data)]
            vmax = max(float(finite.max()), 1e-6) if len(finite) else 1
            im = ax.imshow(
                data.T,
                interpolation="nearest",
                origin="lower",
                aspect="auto",
                extent=[times[0], times[-1], -0.5, len(coords) - 0.5],
                vmin=0,
                vmax=vmax,
            )
            ax.set_yticks(range(len(coords)), [f"{a},{b}" for a, b in coords], fontsize=6)
            ax.set_ylabel("Taxel (row,col)")
            ax.set_xlabel("Time (s)")
            ax.set_title(
                f"{side.title()}: {'normal force' if field == 'normal' else 'friction utilization'}"
            )
            fig.colorbar(im, ax=ax, label="N" if field == "normal" else r"$\rho_i$")
            if confirmed is not None:
                ax.axvline(times[confirmed], color="w", ls="--", lw=0.8)
        for field_index, field in enumerate(("normal", "shear", "ratio")):
            finite = matrices[field][np.isfinite(matrices[field])]
            vmax = max(float(finite.max()), 1e-6) if len(finite) else 1
            for column, (name, index) in enumerate(
                zip(("Stable", "Pre-confirm", "Confirmed"), snapshots, strict=True)
            ):
                ax = fig.add_subplot(spaces[side_index * 3 + field_index, column])
                if index is None:
                    ax.text(
                        0.5,
                        0.5,
                        "No confirmed event",
                        ha="center",
                        va="center",
                        transform=ax.transAxes,
                        fontsize=7,
                    )
                    ax.set_axis_off()
                    continue
                grid = np.full(shape, np.nan)
                for k, coord in enumerate(coords):
                    grid[coord] = matrices[field][index, k]
                im = ax.imshow(grid, origin="lower", vmin=0, vmax=vmax)
                ax.set_title(f"{side.title()} {field}: {name}\nt={times[index]:.3f} s", fontsize=7)
                ax.set_xticks(range(shape[1]))
                ax.set_yticks(range(shape[0]))
                ax.tick_params(labelsize=6)
                fig.colorbar(im, ax=ax, shrink=0.8)
    save_publication_figure(fig, path)
    plt.close(fig)
