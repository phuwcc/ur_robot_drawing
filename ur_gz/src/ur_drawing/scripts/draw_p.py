#!/usr/bin/env python3
"""Draw a Cartesian letter P with MoveIt."""

import copy
import math
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
)
from moveit_msgs.srv import GetCartesianPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformException, TransformListener


def letter_p(x, y, z, width, height, step):
    """Bottom-to-top stem, then a right-hand half ellipse, in the Y-Z plane."""
    if not all(math.isfinite(v) for v in (x, y, z, width, height, step)):
        raise ValueError("Coordinates and dimensions must be finite")
    if min(width, height, step) <= 0:
        raise ValueError("width, height and step must be positive")
    stem_count = max(2, math.ceil(height / step))
    arc_count = max(12, math.ceil(math.pi * max(width, height / 4) / step))
    points = [(x, y, z + height * i / stem_count) for i in range(stem_count + 1)]
    points.extend(
        (x, y + width * math.sin(math.pi * i / arc_count),
         z + 0.75 * height + 0.25 * height * math.cos(math.pi * i / arc_count))
        for i in range(1, arc_count + 1)
    )
    return points


class DrawP(Node):
    def __init__(self):
        super().__init__("draw_p")
        defaults = {
            "frame_id": "base_link", "ee_link": "tool0", "group_name": "ur_manipulator",
            "x": 0.30, "y": -0.08, "z": 0.18, "width": 0.06, "height": 0.12,
            "step": 0.004, "speed_scale": 0.125, "execute": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        self.cfg["use_sim_time"] = self.get_parameter("use_sim_time").value
        if not 0 < self.cfg["speed_scale"] <= 1:
            raise ValueError("speed_scale must be in (0, 1]")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.target_pub = self.create_publisher(Path, "/draw_p/target_path", qos)
        self.trace_pub = self.create_publisher(Path, "/draw_p/actual_path", qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.controller_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.trace = Path()
        self.trace.header.frame_id = self.cfg["frame_id"]
        self.tracing = False
        self.create_timer(0.05, self.sample_trace)

    def tool_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.cfg["frame_id"], self.cfg["ee_link"], rclpy.time.Time())
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def sample_trace(self):
        if not self.tracing:
            return
        try:
            pose = self.tool_pose()
        except TransformException:
            return
        if self.trace.poses:
            old = self.trace.poses[-1].pose.position
            new = pose.position
            if math.dist((old.x, old.y, old.z), (new.x, new.y, new.z)) < 0.001:
                return
        stamped = PoseStamped()
        stamped.header.frame_id = self.cfg["frame_id"]
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose = pose
        self.trace.poses.append(stamped)
        self.trace.header.stamp = stamped.header.stamp
        self.trace_pub.publish(self.trace)

    def wait_result(self, future, timeout=60.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise RuntimeError("MoveIt request timed out; check move_group and simulation")
        return future.result()

    def compute_cartesian(self, poses, header, label):
        request = GetCartesianPath.Request()
        request.header = header
        request.start_state.is_diff = True
        request.group_name = self.cfg["group_name"]
        request.link_name = self.cfg["ee_link"]
        request.waypoints = poses
        request.max_step = self.cfg["step"]
        request.jump_threshold = 5.0
        request.revolute_jump_threshold = 0.3
        request.avoid_collisions = True
        if hasattr(request, "max_velocity_scaling_factor"):
            request.max_velocity_scaling_factor = self.cfg["speed_scale"]
            request.max_acceleration_scaling_factor = self.cfg["speed_scale"]
        response = self.wait_result(self.cartesian.call_async(request))
        self.get_logger().info(f"{label}: {response.fraction:.1%}")
        if response.error_code.val != MoveItErrorCodes.SUCCESS or response.fraction < 0.999:
            raise RuntimeError(
                f"{label} is incomplete; robot will not move. "
                "Adjust x/y/z/width/height to keep the letter reachable and collision-free.")
        if not response.solution.joint_trajectory.points:
            raise RuntimeError(f"MoveIt returned an empty trajectory for {label}")
        return response.solution

    def move_to_start(self):
        """Use OMPL to reach a deterministic, collision-free writing pose."""
        if not self.controller_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError(
                "joint_trajectory_controller action server is not ready; "
                "check the controller spawner")
        if not self.move_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("/move_action unavailable; MoveIt did not start")
        # The controller may be active before MoveIt's own action client has
        # completed DDS discovery. Give that separate process a short grace period.
        self.get_logger().info("Controller action server is ready")
        time.sleep(1.0)
        # This collision-free UR3 configuration was verified in Gazebo. A fixed
        # joint goal avoids random IK branches whose wrapped angles Gazebo may
        # interpret differently on separate runs. The letter is anchored at the
        # measured TCP after this motion, so FK/model variations do not matter.
        joint_names = (
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        )
        start_positions = (
            -0.8062525553, -0.2823116024, 1.2251390614,
            1.2194131064, 1.0917290302, -1.0695566106,
        )
        constraints = Constraints()
        constraints.name = "letter_start"
        constraints.joint_constraints = [
            JointConstraint(
                joint_name=name,
                position=value,
                tolerance_above=0.01,
                tolerance_below=0.01,
                weight=1.0,
            )
            for name, value in zip(joint_names, start_positions)
        ]

        goal = MoveGroup.Goal()
        goal.request.group_name = self.cfg["group_name"]
        # ur_moveit_config registers the OMPL pipeline under the "move_group"
        # key on ROS 2 Humble (see the move_group startup log).
        goal.request.pipeline_id = "move_group"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        # A conservative approach speed gives Gazebo time to settle at the
        # first waypoint. The Cartesian drawing still uses speed_scale.
        approach_scale = min(self.cfg["speed_scale"], 0.15)
        goal.request.max_velocity_scaling_factor = approach_scale
        goal.request.max_acceleration_scaling_factor = approach_scale
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        handle = self.wait_result(self.move_client.send_goal_async(goal), timeout=30.0)
        if not handle.accepted:
            raise RuntimeError("MoveIt rejected the approach goal")
        result = self.wait_result(handle.get_result_async(), timeout=90.0)
        if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            if result.result.error_code.val == MoveItErrorCodes.CONTROL_FAILED:
                raise RuntimeError(
                    "The approach was planned, but Gazebo could not track its final goal; "
                    "see the joint_trajectory_controller tolerance error above")
            raise RuntimeError(
                f"MoveIt could not reach the letter start (code "
                f"{result.result.error_code.val}); adjust x/y/z")
        self.get_logger().info("Reached the first waypoint using OMPL")

    def make_target_path(self, points, orientation):
        target = Path()
        target.header.frame_id = self.cfg["frame_id"]
        target.header.stamp = self.get_clock().now().to_msg()
        for x, y, z in points:
            stamped = PoseStamped()
            stamped.header = copy.deepcopy(target.header)
            stamped.pose.position.x, stamped.pose.position.y, stamped.pose.position.z = x, y, z
            stamped.pose.orientation = copy.deepcopy(orientation)
            target.poses.append(stamped)
        self.target_pub.publish(target)
        return target

    def execute_trajectory(self, trajectory, trace=False):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        handle = self.wait_result(self.execute_client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError("MoveIt rejected trajectory execution")
        self.tracing = trace
        if trace:
            self.sample_trace()
        result_future = handle.get_result_async()
        try:
            duration = trajectory.joint_trajectory.points[-1].time_from_start
            result = self.wait_result(
                result_future, max(60.0, duration.sec + duration.nanosec / 1e9 + 30.0))
        except (RuntimeError, KeyboardInterrupt):
            self.wait_result(handle.cancel_goal_async(), timeout=5.0)
            raise
        finally:
            if trace:
                self.sample_trace()
            self.tracing = False
        if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"Execution failed: MoveIt code {result.result.error_code.val}")

    def run(self):
        if not self.cfg["use_sim_time"]:
            raise RuntimeError("This node is for simulation; set use_sim_time:=true")
        points = letter_p(*(self.cfg[k] for k in ("x", "y", "z", "width", "height", "step")))
        deadline = time.monotonic() + 30.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                current = self.tool_pose()
                break
            except TransformException:
                if time.monotonic() > deadline:
                    raise RuntimeError("No tool TF: check robot_state_publisher and joint_states")
        else:
            return
        target = self.make_target_path(points, current.orientation)
        self.get_logger().info("Published letter P on /draw_p/target_path")
        if not self.cartesian.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("/compute_cartesian_path unavailable; MoveIt did not start")
        if not self.cfg["execute"]:
            self.get_logger().info("Preview only; execute:=false, so robot will not move")
            return
        # Let OMPL choose a reachable orientation, then hold that orientation
        # throughout the Cartesian letter.
        self.move_to_start()
        current = self.tool_pose()
        if not self.execute_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("/execute_trajectory unavailable")
        # Move from the deterministic joint pose to the configured drawing
        # origin while preserving its known-reachable tool orientation.
        origin_pose = Pose()
        origin_pose.position.x = self.cfg["x"]
        origin_pose.position.y = self.cfg["y"]
        origin_pose.position.z = self.cfg["z"]
        origin_pose.orientation = copy.deepcopy(current.orientation)
        positioning = self.compute_cartesian(
            [origin_pose], target.header, "Positioning path")
        self.execute_trajectory(positioning, trace=False)
        current = self.tool_pose()
        reached = (current.position.x, current.position.y, current.position.z)
        configured = (self.cfg["x"], self.cfg["y"], self.cfg["z"])
        position_error = math.dist(reached, configured)
        self.get_logger().info(f"Drawing-origin error: {position_error:.4f} m")
        if position_error > 0.01:
            raise RuntimeError(
                f"TCP stopped {position_error:.3f} m from the configured drawing origin")
        points = letter_p(
            self.cfg["x"], self.cfg["y"], self.cfg["z"],
            self.cfg["width"], self.cfg["height"], self.cfg["step"])
        target = self.make_target_path(points, current.orientation)
        # Keep the reached orientation while tracing the Cartesian letter.
        drawing = self.compute_cartesian(
            [p.pose for p in target.poses], target.header, "Letter P path")
        self.execute_trajectory(drawing, trace=True)
        self.get_logger().info("Finished drawing P. Paths remain visible while this node runs.")


def main():
    rclpy.init()
    node = None
    try:
        node = DrawP()
        try:
            node.run()
        except (RuntimeError, ValueError) as exc:
            node.get_logger().error(str(exc))
            return
        rclpy.spin(node)  # Keep transient-local paths available for late RViz subscribers.
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
