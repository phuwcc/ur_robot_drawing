"""MoveIt client with Cartesian motion for short vertical pick/place segments."""

import threading

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

from .moveit_client import MoveItClient as PoseGoalMoveItClient
from .trajectory_utils import unwrap_continuous_joint_positions, has_joint_jump


class MoveItClient(PoseGoalMoveItClient):
    def __init__(self, node, config):
        super().__init__(node, config)
        self.cartesian_client = node.create_client(
            GetCartesianPath, "/compute_cartesian_path")
        self.fk_client = node.create_client(GetPositionFK, "/compute_fk")
        self.ik_client = node.create_client(GetPositionIK, "/compute_ik")
        self.execute_client = ActionClient(
            node, ExecuteTrajectory, "/execute_trajectory")
        self._joint_state_lock = threading.Lock()
        self._joint_positions = {}
        self._joint_state_subscription = node.create_subscription(
            JointState, "/joint_states", self._update_joint_positions, 10)

    def _update_joint_positions(self, message):
        with self._joint_state_lock:
            self._joint_positions.update(zip(message.name, message.position))

    def reachable_position(self, object_id, position):
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('MoveIt IK service unavailable')
        offset = self.motion['tool_length'] + self.motion['grasp_height_offset']
        for height in (self.motion['approach_height'], 0.0):
            request = GetPositionIK.Request()
            ik = request.ik_request
            ik.group_name = self.group_name
            ik.ik_link_name = self.ee_link
            ik.avoid_collisions = True
            ik.timeout = Duration(sec=1)
            ik.pose_stamped.header.frame_id = self.frame_id
            ik.pose_stamped.pose.position.x = float(position[0])
            ik.pose_stamped.pose.position.y = float(position[1])
            ik.pose_stamped.pose.position.z = float(position[2] + offset + height + self.motion['release_clearance'])
            q = ik.pose_stamped.pose.orientation
            q.x, q.y, q.z, q.w = map(float, self.motion['grasp_orientation'])
            with self._joint_state_lock:
                ik.robot_state.joint_state.name = list(self._joint_positions)
                ik.robot_state.joint_state.position = list(self._joint_positions.values())
            ik.robot_state.is_diff = True
            response = self._wait(self.ik_client.call_async(request), 4.0, 'check temporary IK')
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                return False
            joints = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
            for name, (lo, hi) in self.motion.get('pose_joint_limits', {}).items():
                if name not in joints or not lo <= joints[name] <= hi:
                    return False
        return True

    def verify_grasp_pose(self, target, orientation):
        import math
        with self._joint_state_lock:
            names = list(self._joint_positions)
            values = [self._joint_positions[n] for n in names]
        request = GetPositionFK.Request()
        request.header.frame_id = self.frame_id
        request.fk_link_names = [self.ee_link]
        request.robot_state.joint_state.name = names
        request.robot_state.joint_state.position = values
        response = self._wait(self.fk_client.call_async(request), 10.0, 'verify grasp FK')
        if response.error_code.val != MoveItErrorCodes.SUCCESS or not response.pose_stamped:
            raise RuntimeError('Cannot verify gripper proximity')
        pose = response.pose_stamped[0].pose
        actual = [pose.position.x, pose.position.y, pose.position.z]
        q = pose.orientation
        dot = abs(sum(a*b for a, b in zip([q.x, q.y, q.z, q.w], orientation)))
        if math.dist(actual, target) > 0.012 or dot < math.cos(0.05 / 2):
            raise RuntimeError('Gripper is not at the observed grasp pose; refusing attachment')
        # The accepted MoveIt goal has position and orientation tolerances.
        # Return FK so the attached collision object can preserve its measured
        # world pose instead of assuming that the tool reached the nominal pose.
        return pose

    def wait_ready(self, timeout=30.0):
        super().wait_ready(timeout)
        if not self.cartesian_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/compute_cartesian_path is unavailable")
        if not self.fk_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/compute_fk is unavailable")
        if not self.ik_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/compute_ik is unavailable")
        if not self.execute_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("/execute_trajectory is unavailable")

    def move_to_pose(self, position, orientation, label):
        if label.startswith(("descend", "lift", "retreat")):
            return self._move_cartesian(position, orientation, label)
        return self._move_to_near_seed_ik(position, orientation, label)

    def move_home(self):
        """Return home through a high, collision-free Cartesian posture."""
        transit = self.motion.get("home_transit_position")
        if transit is not None:
            ok, status = self._move_to_near_seed_ik(
                transit,
                self.motion["grasp_orientation"],
                "move to home transit",
            )
            if not ok:
                return False, status
        # The UR arm can represent the same home posture with several joints
        # shifted by 2*pi. A literal zero target can therefore command an
        # unnecessary long wrist rotation that sweeps the gripper through the
        # forearm. Reuse the same nearest-equivalent normalization as pose IK.
        with self._joint_state_lock:
            current_state = dict(self._joint_positions)
        if not current_state:
            raise RuntimeError("No joint state is available for home planning")
        solution = {
            name: float(value)
            for name, value in self.motion["home_joints"].items()
        }
        return self._plan_joint_solution(solution, current_state, "home")

    def _compute_ik(self, position, orientation, seed_state, label, warn=True):
        """Return one collision-aware IK solution seeded by ``seed_state``."""
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.group_name
        ik.ik_link_name = self.ee_link
        ik.avoid_collisions = True
        ik.timeout = Duration(sec=2)
        ik.pose_stamped.header.frame_id = self.frame_id
        ik.pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        (ik.pose_stamped.pose.position.x, ik.pose_stamped.pose.position.y,
         ik.pose_stamped.pose.position.z) = map(float, position)
        (ik.pose_stamped.pose.orientation.x, ik.pose_stamped.pose.orientation.y,
         ik.pose_stamped.pose.orientation.z,
         ik.pose_stamped.pose.orientation.w) = map(float, orientation)
        ik.robot_state.joint_state.name = list(seed_state)
        ik.robot_state.joint_state.position = list(seed_state.values())
        ik.robot_state.is_diff = True

        response = self._wait(
            self.ik_client.call_async(request), 5.0, f"compute IK for {label}")
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            if not warn:
                return None
            self.node.get_logger().warning(
                f"IK failed for '{label}' (MoveIt code {response.error_code.val})")
            return None
        return dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position,
        ))

    def _plan_joint_solution(self, solution, current_state, label):
        """Normalize and collision-check an exact arm-joint goal."""
        arm_joint_names = list(self.motion["home_joints"])
        missing = [name for name in arm_joint_names if name not in solution]
        if missing:
            raise RuntimeError(
                f"IK response is missing arm joints: {', '.join(missing)}")
        missing_current = [name for name in arm_joint_names if name not in current_state]
        if missing_current:
            raise RuntimeError(
                f"Joint state is missing arm joints: {', '.join(missing_current)}")

        targets = [solution[name] for name in arm_joint_names]
        current = {name: current_state[name] for name in arm_joint_names}
        targets = unwrap_continuous_joint_positions(
            arm_joint_names,
            [targets],
            current,
            self.motion.get(
                "equivalent_angle_joints", self.motion["continuous_joints"]),
            self.motion.get("equivalent_angle_joint_limits"),
        )[0]

        max_delta = float(self.motion.get("max_pose_joint_delta", 2.8))
        excessive = {
            name: abs(target - current[name])
            for name, target in zip(arm_joint_names, targets)
            if abs(target - current[name]) > max_delta
        }
        if excessive:
            detail = ", ".join(
                f"{name}: {current[name]:.2f} -> "
                f"{targets[arm_joint_names.index(name)]:.2f} "
                f"(delta={delta:.2f} rad)"
                for name, delta in excessive.items())
            self.node.get_logger().warning(
                f"Rejected distant IK branch for '{label}': {detail}")
            return False, "PLANNING_FAILED"

        constraints = Constraints()
        tolerance = float(self.motion.get("ik_joint_tolerance", 0.01))
        for name, target in zip(arm_joint_names, targets):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(target)
            joint.tolerance_above = tolerance
            joint.tolerance_below = tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return self._run(constraints, label)

    def move_to_approach(self, approach, target, orientation, label):
        """Reach an approach pose on an IK branch that can also reach target.

        Solving the lower target first and using that solution as the seed for
        the approach prevents a short vertical segment from crossing between
        two UR analytic/KDL branches near a joint limit.
        """
        with self._joint_state_lock:
            current_state = dict(self._joint_positions)
        if not current_state:
            raise RuntimeError("No joint state is available for IK planning")
        target_solution = self._compute_ik(
            target, orientation, current_state, f"{label} target", warn=False)
        if target_solution is None:
            # Some KDL seeds can reach the approach pose but return NO_IK for
            # the lower pose. Use the reachable approach branch as a second
            # seed, then solve the approach again from that lower solution.
            approach_hint = self._compute_ik(
                approach, orientation, current_state,
                f"{label} approach hint", warn=False)
            if approach_hint is not None:
                target_solution = self._compute_ik(
                    target, orientation, approach_hint,
                    f"{label} target from approach", warn=False)
        if target_solution is None:
            # Final deterministic retry from the configured central/home arm
            # posture, while preserving non-arm joints from the live state.
            central_seed = dict(current_state)
            central_seed.update({
                name: float(value)
                for name, value in self.motion["home_joints"].items()
            })
            target_solution = self._compute_ik(
                target, orientation, central_seed,
                f"{label} target from central seed", warn=False)
        if target_solution is None:
            self.node.get_logger().warning(
                f"IK failed for '{label} target' after live, approach, "
                "and central seeds")
            return False, "PLANNING_FAILED"
        approach_solution = self._compute_ik(
            approach, orientation, target_solution, f"{label} approach")
        if approach_solution is None:
            return False, "PLANNING_FAILED"
        return self._plan_joint_solution(approach_solution, current_state, label)

    def _move_to_near_seed_ik(self, position, orientation, label):
        """Plan to the IK solution nearest the current joint configuration.

        A pose-only MoveGroup goal may be satisfied by several UR IK branches.
        Selecting the goal joints first, with the live state as the IK seed,
        prevents a planner from choosing an equivalent solution that turns a
        wrist (or the shoulder) almost a full revolution.
        """
        with self._joint_state_lock:
            current_state = dict(self._joint_positions)
        if not current_state:
            raise RuntimeError("No joint state is available for IK planning")
        solution = self._compute_ik(position, orientation, current_state, label)
        if solution is None:
            return False, "PLANNING_FAILED"
        return self._plan_joint_solution(solution, current_state, label)

    def _move_cartesian(self, position, orientation, label):
        # A pose goal is allowed a small position/orientation tolerance. Starting
        # the Cartesian segment with the nominal pose would therefore combine a
        # vertical move with a correction in X/Y/orientation and can make local
        # IK jump to a dead end. Use FK so these short segments change only Z.
        with self._joint_state_lock:
            joint_names = list(self._joint_positions)
            joint_positions = [self._joint_positions[name] for name in joint_names]
        if not joint_names:
            raise RuntimeError("No joint state is available for Cartesian planning")

        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = self.frame_id
        fk_request.header.stamp = self.node.get_clock().now().to_msg()
        fk_request.fk_link_names = [self.ee_link]
        fk_request.robot_state.joint_state.name = joint_names
        fk_request.robot_state.joint_state.position = joint_positions
        fk_response = self._wait(
            self.fk_client.call_async(fk_request), 10.0, f"compute FK for {label}")
        if (fk_response.error_code.val != MoveItErrorCodes.SUCCESS
                or not fk_response.pose_stamped):
            raise RuntimeError(f"MoveIt could not compute FK for {label}")
        current = fk_response.pose_stamped[0].pose

        target = Pose()
        target.position.x = current.position.x
        target.position.y = current.position.y
        target.position.z = float(position[2])
        target.orientation = current.orientation

        request = GetCartesianPath.Request()
        request.header.frame_id = self.frame_id
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.start_state.is_diff = True
        request.start_state.joint_state.name = joint_names
        request.start_state.joint_state.position = joint_positions
        request.group_name = self.group_name
        request.link_name = self.ee_link
        request.waypoints = [target]
        request.max_step = 0.005
        request.jump_threshold = 2.0
        request.revolute_jump_threshold = 0.0
        request.avoid_collisions = True
        response = self._wait(
            self.cartesian_client.call_async(request), 30.0, f"plan {label}")
        if response.error_code.val != MoveItErrorCodes.SUCCESS or response.fraction < 0.995:
            self.node.get_logger().warning(
                f"Cartesian planning for '{label}' reached "
                f"{response.fraction * 100.0:.1f}% "
                f"(MoveIt code {response.error_code.val}); retrying with "
                "collision-aware pose planning"
            )
            current_orientation = [
                current.orientation.x,
                current.orientation.y,
                current.orientation.z,
                current.orientation.w,
            ]
            return self._move_to_near_seed_ik(
                [target.position.x, target.position.y, target.position.z],
                current_orientation,
                f"{label} fallback",
            )
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
            self.motion.get(
                "equivalent_angle_joints", self.motion["continuous_joints"]),
            self.motion.get("equivalent_angle_joint_limits"),
        )
        if has_joint_jump(joint_trajectory.joint_names, normalized_positions, start_positions):
            self.node.get_logger().warning(f"Rejected Cartesian IK branch jump for {label}")
            return self._move_to_near_seed_ik(
                [target.position.x, target.position.y, target.position.z],
                [current.orientation.x, current.orientation.y, current.orientation.z, current.orientation.w],
                f"{label} continuous fallback")
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
        timeout = max(120.0, (duration.sec + duration.nanosec / 1e9) * 10.0 + 30.0)
        try:
            result = self._wait(handle.get_result_async(), timeout, f"execute {label}")
        except RuntimeError:
            handle.cancel_goal_async()
            raise
        if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, "EXECUTION_FAILED"
        return True, "SUCCESS"
