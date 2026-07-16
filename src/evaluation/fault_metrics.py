"""Fault-response metrics for controller comparison."""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Dict

import numpy as np


@dataclass(frozen=True)
class FaultResponseSummary:
    controller: str
    fault: str
    rms_position_error_m: float
    max_position_error_m: float
    final_position_error_m: float
    max_tilt_deg: float
    max_yaw_error_deg: float
    min_altitude_m: float
    max_altitude_m: float
    failed: bool

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)


def summarize_fault_response(
    results,
    max_allowed_position_error_m: float = 3.0,
    max_allowed_tilt_deg: float = 55.0,
) -> FaultResponseSummary:
    position_errors = np.linalg.norm(results.errors[:, 0:3], axis=1)
    attitude = results.state[:, 6:9]
    tilt = np.linalg.norm(attitude[:, 0:2], axis=1)
    yaw_error = np.abs(results.errors[:, 3])
    altitude = -results.state[:, 2]

    rms_error = float(np.sqrt(np.mean(position_errors * position_errors)))
    max_error = float(np.max(position_errors))
    max_tilt_deg = float(np.rad2deg(np.max(tilt)))
    failed = bool(max_error > max_allowed_position_error_m or max_tilt_deg > max_allowed_tilt_deg)

    return FaultResponseSummary(
        controller=results.controller_name,
        fault=results.fault_name,
        rms_position_error_m=rms_error,
        max_position_error_m=max_error,
        final_position_error_m=float(position_errors[-1]),
        max_tilt_deg=max_tilt_deg,
        max_yaw_error_deg=float(np.rad2deg(np.max(yaw_error))),
        min_altitude_m=float(np.min(altitude)),
        max_altitude_m=float(np.max(altitude)),
        failed=failed,
    )
