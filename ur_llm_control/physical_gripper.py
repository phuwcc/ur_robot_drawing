"""ROS 2 Robotiq motion plus Gazebo detachable-joint grasping."""

import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from std_msgs.msg import Empty, String
from trajectory_msgs.msg import JointTrajectoryPoint


class PhysicalGripper:
    def __init__(self, node, object_names, config):
        self.node = node
        self.object_names = tuple(object_names)
        self.command_client = ActionClient(
            node, FollowJointTrajectory,
            config.get(
                "action_name",
                "/robotiq_gripper_controller/follow_joint_trajectory",
            ),
        )
        self.open_position = float(config.get("open_position", 0.0))
        self.closed_position = float(config.get("closed_position", 0.65))
        self.motion_duration = float(config.get("motion_duration", 2.0))
        self.command_timeout = float(config.get("command_timeout", 8.0))
        self.joint_states = {name: None for name in self.object_names}
        self.attach_publishers = {
            name: node.create_publisher(Empty, f"/gripper/{name}/attach", 1)
            for name in self.object_names
        }
        self.detach_publishers = {
            name: node.create_publisher(Empty, f"/gripper/{name}/detach", 1)
            for name in self.object_names
        }
        self.state_subscriptions = [
            node.create_subscription(
                String,
                f"/model/{name}/detachable_joint/state",
                lambda message, object_id=name: self._update_state(object_id, message),
                10,
            )
            for name in self.object_names
        ]
        self.held = None
        self.initialized = False

    def _update_state(self, object_id, message):
        self.joint_states[object_id] = message.data


    def _wait_future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.02)
        raise RuntimeError("timed out waiting for gripper controller")

    def _command(self, position):
        if not self.command_client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("Robotiq gripper trajectory server is unavailable")

        angle = float(position)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            "robotiq_85_left_knuckle_joint",
            "robotiq_85_right_knuckle_joint",
            "robotiq_85_left_inner_knuckle_joint",
            "robotiq_85_right_inner_knuckle_joint",
            "robotiq_85_left_finger_tip_joint",
            "robotiq_85_right_finger_tip_joint",
        ]
        point = JointTrajectoryPoint()
        point.positions = [angle, -angle, angle, -angle, -angle, angle]
        seconds = int(self.motion_duration)
        point.time_from_start = Duration(
            sec=seconds,
            nanosec=int((self.motion_duration - seconds) * 1e9),
        )
        goal.trajectory.points = [point]

        goal_handle = self._wait_future(
            self.command_client.send_goal_async(goal), self.command_timeout
        )
        if not goal_handle.accepted:
            raise RuntimeError("Robotiq gripper trajectory was rejected")

        response = self._wait_future(
            goal_handle.get_result_async(), self.command_timeout
        )
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"Robotiq gripper trajectory failed with status {response.status}"
            )

    def _wait_for_bridge(self, timeout=15.0):
        deadline = time.monotonic() + timeout
        publishers = (
            tuple(self.attach_publishers.values())
            + tuple(self.detach_publishers.values())
        )
        while time.monotonic() < deadline:
            if all(publisher.get_subscription_count() > 0 for publisher in publishers):
                return
            time.sleep(0.05)
        raise RuntimeError("Gazebo attach/detach topic bridge is unavailable")

    def _set_joint(self, object_id, attached, timeout=3.0):
        publishers = self.attach_publishers if attached else self.detach_publishers
        publisher = publishers.get(object_id)
        if publisher is None:
            raise RuntimeError(f"no Gazebo joint configured for {object_id}")

        expected = "attached" if attached else "detached"
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            publisher.publish(Empty())
            wait_deadline = min(deadline, time.monotonic() + 0.4)
            while time.monotonic() < wait_deadline:
                if self.joint_states[object_id] == expected:
                    return
                time.sleep(0.02)
        action = "attach" if attached else "detach"
        raise RuntimeError(f"Gazebo did not confirm {action} for {object_id}")

    def initialize(self):
        if self.initialized:
            return
        self._wait_for_bridge()
        self._command(self.open_position)
        # DetachableJoint starts attached. Explicitly detach every cube before
        # executing the first plan and verify the reported state.
        for object_id in self.object_names:
            self._set_joint(object_id, attached=False)
        self.initialized = True

    def grasp(self, object_id, _tcp_offset):
        if self.held is not None:
            raise RuntimeError(f"gripper already holds {self.held}")
        self._command(self.closed_position)
        self._set_joint(object_id, attached=True)
        self.held = object_id

    def release(self, _position):
        if self.held is None:
            raise RuntimeError("gripper is empty")
        object_id = self.held
        self._command(self.open_position)
        self._set_joint(object_id, attached=False)
        self.held = None
