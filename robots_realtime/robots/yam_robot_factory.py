"""Factory function for creating YAM robots with proper two-pass offset calibration."""

from __future__ import annotations

import numpy as np
from i2rt.robots.motor_chain_robot import MotorChainRobot
from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.utils import GripperType


def create_yam_robot(
    channel: str = "can0",
    gripper_type: str = "linear_4310",
    gravity_comp_factor: float = 1.3,
    zero_gravity_mode: bool = False,
    kp: list[float] | None = None,
    kd: list[float] | None = None,
    gripper_limits: list[float] | None = None,
    limit_gripper_force: float = 50.0,
) -> MotorChainRobot:
    """Create a YAM robot with proper two-pass offset calibration.

    This wraps i2rt's get_yam_robot() to ensure motor offsets are
    correctly detected before starting the control loop.
    """
    gtype = GripperType.from_string_name(gripper_type)

    kp_arr = np.array(kp, dtype=float) if kp is not None else None
    kd_arr = np.array(kd, dtype=float) if kd is not None else None
    gl_arr = np.array(gripper_limits) if gripper_limits is not None else None

    return get_yam_robot(
        channel=channel,
        gripper_type=gtype,
        zero_gravity_mode=zero_gravity_mode,
        kp_override=kp_arr,
        kd_override=kd_arr,
        gravity_comp_factor=gravity_comp_factor,
        gripper_limits_override=gl_arr,
        limit_gripper_force=limit_gripper_force,
    )
