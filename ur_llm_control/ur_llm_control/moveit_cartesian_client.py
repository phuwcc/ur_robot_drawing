"""MoveIt client with Cartesian motion for short vertical pick/place segments."""

import threading

from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

from .moveit_client import MoveItClient as PoseGoalMoveItClient
from .trajectory_utils import unwrap_continuous_joint_positions


class MoveItClient(PoseGoalMoveItClient):
    def __init__(self, node, config):
        super().__init__(node, config)
        self.cartesian_client = node.create_client(
            GetCartesianPath, "/compute_cartesian_path")
        self.execute_client = ActionClient(
            node, ExecuteTrajectory, "/execute_trajectory")
        self._joint_state_lock = threading.Lock()
        self._joint_positions = {}
        self._joint_state_subscription = node.create_subscription(
            JointState, "/joint_states", self._update_joint_positions, 10)

    def _update_joint_positions(self, message):
        with self._joint_state_lock:
            self._joint_positions.update(zip(message.name, message.position))

    def wait_ready(self, timeout=30.0):
        super().wait_ready(timeout)
        if not self.cartesian_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/compute_cartesian_path is unavailable")
        if not self.execute_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("/execute_trajectory is unavailable")

    def move_to_pose(self, position, orientation, label):
        if label.startswith(("descend", "lift", "retreat")):
            return self._move_cartesian(position, orientation, label)
        return super().move_to_pose(position, orientation, label)

    def _move_cartesian(self, position, orientation, label):
        target = Pose()
        target.position.x, target.position.y, target.position.z = map(float, position)
        (target.orientation.x, target.orientation.y,
         target.orientation.z, target.orientation.w) = map(float, orientation)

        request = GetCartesianPath.Request()
        request.header.frame_id = self.frame_id
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.start_state.is_diff = True
        request.group_name = self.group_name
        request.link_name = self.ee_link
        request.waypoints = [target]
        request.max_step = 0.005
        request.jump_threshold = 0.0
        request.revolute_jump_threshold = 0.0
        request.avoid_collisions = True
        response = self._wait(
            self.cartesian_client.call_async(request), 30.0, f"plan {label}")
        if response.error_code.val != MoveItErrorCodes.SUCCESS or response.fraction < 0.995:
            return False, "PLANNING_FAILED"
        trajectory = response.solution
        if not trajectory.joint_trajectory.points:
            return False, "PLANNING_FAILED"

        with self._joint_state_lock:
            start_positions = dict(self._joint_positions)
        joint_trajectory = trajectory.joint_trajectory
        normalized_positions = unwrap_continuous_joint_positions(
            joint_trajectory.joint_names,
            [point.positions for point in joint_trajectory.points],
            start_positions,
            self.motion["continuous_joints"],
        )
        for point, positions in zip(
                joint_trajectory.points, normalized_positions):
            point.positions = positions

        scale = float(self.motion["velocity_scale"])
        for point in joint_trajectory.points:
            duration = point.time_from_start
            nanoseconds = round(
                (duration.sec * 1_000_000_000 + duration.nanosec) / scale)
            duration.sec, duration.nanosec = divmod(nanoseconds, 1_000_000_000)
            point.velocities = [value * scale for value in point.velocities]
            point.accelerations = [value * scale * scale for value in point.accelerations]

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        handle = self._wait(
            self.execute_client.send_goal_async(goal), 30.0, f"execute {label}")
        if not handle.accepted:
            return False, "EXECUTION_FAILED"
        duration = joint_trajectory.points[-1].time_from_start
        timeout = max(60.0, duration.sec + duration.nanosec / 1e9 + 30.0)
        result = self._wait(handle.get_result_async(), timeout, f"execute {label}")
        if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, "EXECUTION_FAILED"
        return True, "SUCCESS"
