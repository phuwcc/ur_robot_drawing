from ur_llm_control import moveit_client


class FakeNode:
    def create_client(self, *_args):
        return object()


def test_attached_object_allows_contact_with_configured_gripper_links(monkeypatch):
    monkeypatch.setattr(moveit_client, "ActionClient", lambda *_args: object())
    config = {
        "frame_id": "base_link",
        "ee_link": "tool0",
        "group_name": "ur_manipulator",
        "motion": {},
        "gripper": {"touch_links": ["left_finger_link", "right_finger_link"]},
    }
    client = moveit_client.MoveItClient(FakeNode(), config)
    scenes = []
    client._apply = lambda scene, _label: scenes.append(scene)

    client.attach_object("red_cube", 0.05, 0.201)

    attached = scenes[0].robot_state.attached_collision_objects[0]
    assert attached.touch_links == [
        "tool0",
        "left_finger_link",
        "right_finger_link",
    ]
