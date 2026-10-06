"""Helpers for making Cartesian trajectories continuous at joint wraparound."""

import math


def unwrap_continuous_joint_positions(
    joint_names, point_positions, start_positions, continuous_joint_names,
    joint_limits=None,
):
    joint_limits = joint_limits or {}
    normalized = [list(positions) for positions in point_positions]

    for joint_name in continuous_joint_names:
        try:
            joint_index = joint_names.index(joint_name)
            previous = float(start_positions[joint_name])
        except (ValueError, KeyError) as exc:
            raise ValueError(
                f"cannot unwrap continuous joint {joint_name!r} without its "
                "trajectory index and start position"
            ) from exc

        for positions in normalized:
            angle = float(positions[joint_index])
            turns = round((previous - angle) / (2.0 * math.pi))
            candidates = [
                angle + (turns + offset) * 2.0 * math.pi
                for offset in (-1, 0, 1)
            ]
            if joint_name in joint_limits:
                lower, upper = joint_limits[joint_name]
                candidates = [
                    value for value in candidates if lower <= value <= upper]
            if not candidates:
                raise ValueError(
                    f"no equivalent angle for {joint_name!r} is inside its limits")
            positions[joint_index] = min(
                candidates, key=lambda value: abs(value - previous))
            previous = positions[joint_index]

    return normalized


def has_joint_jump(joint_names, point_positions, start_positions, limit=0.15):
    """Reject IK branch switches before a short vertical trajectory is executed."""
    previous = [start_positions[name] for name in joint_names]
    for positions in point_positions:
        if len(positions) != len(previous) or any(
            not math.isfinite(value) or abs(value - before) > limit
            for value, before in zip(positions, previous)
        ):
            return True
        previous = positions
    return False
