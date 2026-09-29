import math

import pytest

from ur_llm_control.trajectory_utils import unwrap_continuous_joint_positions


def test_unwraps_continuous_joint_positions_from_start_state():
    offset = 2.0 * math.pi

    result = unwrap_continuous_joint_positions(
        ["wrist_3_joint", "elbow_joint"],
        [[-3.0 + offset, 0.1], [-2.9 + offset, 0.2]],
        {"wrist_3_joint": -3.0},
        ["wrist_3_joint"],
    )

    assert result[0] == pytest.approx([-3.0, 0.1])
    assert result[1] == pytest.approx([-2.9, 0.2])


def test_unwrap_requires_start_position_for_continuous_joint():
    with pytest.raises(ValueError, match="cannot unwrap continuous joint"):
        unwrap_continuous_joint_positions(
            ["wrist_3_joint"],
            [[0.1]],
            {},
            ["wrist_3_joint"],
        )
