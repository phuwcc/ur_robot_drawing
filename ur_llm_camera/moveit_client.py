"""Minimal MoveIt 2 client built from ROS messages available in Humble."""

import time

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive


class MoveItClient:
    def __init__(self, node, config):
        self.node = node
        self.config = config
        self.frame_id = config["frame_id"]
        self.ee_link = config["ee_link"]
        self.touch_links = list(dict.fromkeys([
            self.ee_link,
            *config.get("gripper", {}).get("touch_links", []),
        ]))
        self.group_name = config["group_name"]
        self.motion = config["motion"]
        self.move_client = ActionClient(node, MoveGroup, "/move_action")
        self.scene_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")

    @staticmethod
    def _wait(future, timeout, label):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            raise RuntimeError(f"{label} timed out")
        result = future.result()
        if result is None:
            raise RuntimeError(f"{label} returned no result")
        return result

    def wait_ready(self, timeout=30.0):
        if not self.move_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("/move_action is unavailable; start MoveIt first")
        if not self.scene_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/apply_planning_scene is unavailable; start MoveIt first")

    def _run(self, constraints, label):
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.pipeline_id = "move_group"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = float(self.motion["planning_time"])
        goal.request.max_velocity_scaling_factor = float(self.motion["velocity_scale"])
        goal.request.max_acceleration_scaling_factor = float(self.motion["acceleration_scale"])
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        handle = self._wait(self.move_client.send_goal_async(goal), 30.0, label)
        if not handle.accepted:
            return False, "PLANNING_FAILED"
        try:
            result = self._wait(handle.get_result_async(), 600.0, label)
        except RuntimeError:
            handle.cancel_goal_async()
            raise
        code = result.result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            time.sleep(0.5)
            return True, "SUCCESS"
        if code == MoveItErrorCodes.CONTROL_FAILED:
            return False, "EXECUTION_FAILED"
        return False, "PLANNING_FAILED"

    def move_to_pose(self, position, orientation, label):
        header = self.node.get_clock().now().to_msg()

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.frame_id
        position_constraint.header.stamp = header
        position_constraint.link_name = self.ee_link
        position_constraint.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.006]
        target = Pose()
        target.position.x, target.position.y, target.position.z = map(float, position)
        target.orientation.w = 1.0
        position_constraint.constraint_region.primitives = [sphere]
        position_constraint.constraint_region.primitive_poses = [target]

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.frame_id
        orientation_constraint.header.stamp = header
        orientation_constraint.link_name = self.ee_link
        (orientation_constraint.orientation.x, orientation_constraint.orientation.y,
         orientation_constraint.orientation.z, orientation_constraint.orientation.w) = map(
            float, orientation)
        orientation_tolerance = float(self.motion["orientation_tolerance"])
        orientation_constraint.absolute_x_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = orientation_tolerance
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [position_constraint]
        constraints.orientation_constraints = [orientation_constraint]
        # Equivalent IK solutions can differ by a full revolution. Keep selected
        # joints away from their +/-2*pi bounds so the following Cartesian move
        # still has room to descend or retreat.
        for name, limits in self.motion.get("pose_joint_limits", {}).items():
            lower, upper = map(float, limits)
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = (lower + upper) / 2.0
            joint.tolerance_below = joint.position - lower
            joint.tolerance_above = upper - joint.position
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return self._run(constraints, label)

    def move_home(self):
        constraints = Constraints()
        for name, value in self.motion["home_joints"].items():
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = 0.015
            joint.tolerance_below = 0.015
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return self._run(constraints, "home")

    @staticmethod
    def _box(object_id, frame_id, position, dimensions, orientation=None):
        collision = CollisionObject()
        collision.header.frame_id = frame_id
        collision.id = object_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in dimensions]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, position)
        if orientation is None:
            pose.orientation.w = 1.0
        else:
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w) = map(float, orientation)
        collision.primitives = [primitive]
        collision.primitive_poses = [pose]
        collision.operation = CollisionObject.ADD
        return collision

    def _apply(self, scene, label):
        scene.is_diff = True
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self._wait(self.scene_client.call_async(request), 10.0, label)
        if not response.success:
            raise RuntimeError(f"MoveIt rejected planning-scene update: {label}")

    def initialize_scene(self, object_positions):
        scene = PlanningScene()
        table = self.config["table"]
        scene.world.collision_objects.append(
            self._box(table["id"], self.frame_id, table["position"], table["size"])
        )
        for name, data in self.config["objects"].items():
            position = object_positions[name]
            size = float(data["size"])
            scene.world.collision_objects.append(
                self._box(name, self.frame_id, position, [size, size, size])
            )
        self._apply(scene, "initialize scene")

    def allow_grasp_contact(self, object_id, allowed):
        from moveit_msgs.srv import GetPlanningScene
        from moveit_msgs.msg import PlanningSceneComponents, AllowedCollisionEntry
        if not hasattr(self, '_get_scene'):
            self._get_scene = self.node.create_client(GetPlanningScene, '/get_planning_scene')
        if not self._get_scene.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('Planning scene query unavailable')
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        matrix = self._wait(self._get_scene.call_async(request), 10.0, 'read contact matrix').scene.allowed_collision_matrix
        for name in [object_id] + list(self.touch_links):
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)
                for row in matrix.entry_values:
                    row.enabled.append(False)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
        index = matrix.entry_names.index(object_id)
        for link in self.touch_links:
            other = matrix.entry_names.index(link)
            matrix.entry_values[index].enabled[other] = allowed
            matrix.entry_values[other].enabled[index] = allowed
        scene = PlanningScene()
        scene.allowed_collision_matrix = matrix
        self._apply(scene, 'update grasp contact permission')

    def remove_world_object(self, object_id):
        scene = PlanningScene()
        collision = CollisionObject()
        collision.header.frame_id = self.frame_id
        collision.id = object_id
        collision.operation = CollisionObject.REMOVE
        scene.world.collision_objects = [collision]
        self._apply(scene, f"remove {object_id}")

    def restore_world_object(self, object_id, position, size):
        scene = PlanningScene()
        scene.world.collision_objects = [
            self._box(object_id, self.frame_id, position, [size, size, size])
        ]
        self._apply(scene, f"restore {object_id}")

    @staticmethod
    def _object_pose_in_link(world_position, link_pose):
        """Preserve an axis-aligned object's measured pose when attaching it."""
        q = link_pose.orientation
        norm = (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) ** 0.5
        if norm == 0.0:
            raise RuntimeError("Cannot attach an object to an invalid FK pose")
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        dx = float(world_position[0]) - link_pose.position.x
        dy = float(world_position[1]) - link_pose.position.y
        dz = float(world_position[2]) - link_pose.position.z

        # R(q)^T transforms a world-frame displacement into the link frame.
        relative_position = [
            (1-2*(y*y+z*z))*dx + 2*(x*y+z*w)*dy + 2*(x*z-y*w)*dz,
            2*(x*y-z*w)*dx + (1-2*(x*x+z*z))*dy + 2*(y*z+x*w)*dz,
            2*(x*z+y*w)*dx + 2*(y*z-x*w)*dy + (1-2*(x*x+y*y))*dz,
        ]
        # The camera model supports axis-aligned cubes, so q^-1 is the cube's
        # orientation relative to the current link pose.
        relative_orientation = [-x, -y, -z, w]
        return relative_position, relative_orientation

    def attach_object(self, object_id, size, world_position, link_pose):
        attached = AttachedCollisionObject()
        attached.link_name = self.ee_link
        attached.touch_links = self.touch_links
        relative_position, relative_orientation = self._object_pose_in_link(
            world_position, link_pose)
        attached.object = self._box(
            object_id, self.ee_link, relative_position, [size, size, size],
            relative_orientation)
        attached.object.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        self._apply(scene, f"attach {object_id}")

    def detach_object(self, object_id, position, size):
        detached = AttachedCollisionObject()
        detached.link_name = self.ee_link
        detached.object.id = object_id
        detached.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [detached]
        scene.world.collision_objects = [
            self._box(object_id, self.frame_id, position, [size, size, size])
        ]
        self._apply(scene, f"detach {object_id}")
