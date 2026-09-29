"""Helpers for making Cartesian trajectories continuous at joint wraparound."""

import math


def unwrap_continuous_joint_positions(
    joint_names, point_positions, start_positions, continuous_joint_names
):
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
            delta = (angle - previous + math.pi) % (2.0 * math.pi) - math.pi
            positions[joint_index] = previous + delta
            previous = positions[joint_index]

    return normalized
