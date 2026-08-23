"""Command-line entry points."""

from __future__ import annotations

import argparse
from pathlib import Path

from .profiles import load_profile
from .validation import validate_profile


def check() -> None:
    """Compile a gripper profile from a CLI argument and print the report."""
    parser = argparse.ArgumentParser(description="Compile and validate a gripper profile")
    parser.add_argument("profile", type=Path)
    args = parser.parse_args()
    report = validate_profile(load_profile(args.profile))
    print(
        f"PASS: {report.model_name}; actuator={report.actuator}; "
        f"tactile_channels={report.tactile_channels}; equalities={report.equalities}"
    )
