import pytest

from ur_llm_control.skills import SkillExecutor


class FakeMoveIt:
    def __init__(self):
        self.poses = []
        self.attached = []
        self.detached = []
        self.events = []

    def wait_ready(self):
        pass

    def initialize_scene(self, _positions):
        pass

    def move_to_pose(self, position, orientation, label):
        self.poses.append((position, orientation, label))
        self.events.append(label)
        return True, "SUCCESS"

    def remove_world_object(self, _object_id):
        pass

    def restore_world_object(self, *_args):
        pass

    def attach_object(self, *args):
        self.attached.append(args)

    def detach_object(self, *args):
        self.detached.append(args)
        self.events.append("detach")

    def move_home(self):
        return True, "SUCCESS"


class FakeGripper:
    def __init__(self, events):
        self.grasp_offset = None
        self.released_at = None
        self.events = events

    def initialize(self):
        pass

    def grasp(self, _object_id, tcp_offset):
        self.grasp_offset = tcp_offset

    def release(self, position):
        self.released_at = position
        self.events.append("release")


@pytest.fixture
def skill_executor():
    config = {
        "objects": {"red_cube": {"position": [0.28, -0.18, 0.18], "size": 0.05}},
        "zones": {"zone_b": {"position": [0.28, 0.18, 0.18]}},
        "motion": {
            "approach_height": 0.06,
            "tool_length": 0.176,
            "grasp_height_offset": 0.016,
            "grasp_orientation": [0.70710678, 0.70710678, 0.0, 0.0],
        },
    }
    moveit = FakeMoveIt()
    gripper = FakeGripper(moveit.events)
    executor = SkillExecutor(moveit, gripper, config, lambda *_args: None)
    return executor, moveit, gripper


def test_pick_targets_grasp_band_inside_cube(skill_executor):
    executor, moveit, gripper = skill_executor

    assert executor.pick("red_cube") == "SUCCESS"

    assert moveit.poses[1][0] == pytest.approx([0.28, -0.18, 0.372])
    assert moveit.poses[1][1] == pytest.approx([0.70710678, 0.70710678, 0.0, 0.0])
    assert moveit.attached[0][2] == pytest.approx(0.192)
    assert gripper.grasp_offset == pytest.approx(0.192)


def test_place_centers_cube_on_zone(skill_executor):
    executor, moveit, gripper = skill_executor
    executor.held = "red_cube"

    assert executor.place("red_cube", "zone_b") == "SUCCESS"

    assert moveit.poses[1][0] == pytest.approx([0.28, 0.18, 0.372])
    assert moveit.detached[0][1] == pytest.approx([0.28, 0.18, 0.18])
    assert gripper.released_at == pytest.approx([0.28, 0.18, 0.18])
    assert moveit.events.index("release") < moveit.events.index("retreat from zone_b")
    assert moveit.events.index("retreat from zone_b") < moveit.events.index("detach")
