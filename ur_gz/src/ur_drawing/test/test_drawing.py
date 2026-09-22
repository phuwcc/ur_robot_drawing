"""Run with ROS sourced: python3 -m unittest discover -s ur_gz/src/ur_drawing/test."""
import importlib.util
import math
from pathlib import Path
import unittest
from unittest.mock import Mock

from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header

spec = importlib.util.spec_from_file_location(
    'drawing', Path(__file__).parents[1] / 'scripts' / 'draw_p.py')
drawing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(drawing)


class DrawingTests(unittest.TestCase):
    def test_circle_is_closed_and_on_requested_plane(self):
        points = drawing.circle(0.3, -0.08, 0.18, 0.04, 0.004)
        self.assertEqual(points[0], points[-1])
        self.assertAlmostEqual(points[0][2], 0.14)
        for x, y, z in points:
            self.assertEqual(x, 0.3)
            self.assertAlmostEqual(math.hypot(y + 0.08, z - 0.18), 0.04)
        self.assertLessEqual(max(math.dist(a, b) for a, b in zip(points, points[1:])), 0.004)

    def test_letter_endpoints_and_spacing(self):
        points = drawing.letter_p(0.3, -0.08, 0.18, 0.06, 0.12, 0.004)
        self.assertEqual(points[0], (0.3, -0.08, 0.18))
        self.assertAlmostEqual(points[-1][1], -0.08)
        self.assertAlmostEqual(points[-1][2], 0.24)
        self.assertLessEqual(max(math.dist(a, b) for a, b in zip(points, points[1:])), 0.004 + 1e-12)

    def test_invalid_circle_dimensions(self):
        for radius in (0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                drawing.circle(0.3, 0, 0.2, radius, 0.004)

    def test_cartesian_speed_and_partial_path_rejection(self):
        node = Mock()
        node.cfg = {'ee_link': 'tool0', 'group_name': 'ur_manipulator',
                    'step': 0.004, 'speed_scale': 0.5}
        response = GetCartesianPath.Response()
        response.error_code.val = MoveItErrorCodes.SUCCESS
        response.fraction = 1.0
        point = JointTrajectoryPoint(velocities=[2.0], accelerations=[4.0])
        point.time_from_start.sec = 1
        response.solution.joint_trajectory.points = [point]
        node.wait_result.return_value = response
        trajectory = drawing.DrawingNode.compute_cartesian(node, [Pose()], Header(), 'test')
        self.assertEqual(trajectory.joint_trajectory.points[0].time_from_start.sec, 2)
        self.assertEqual(list(point.velocities), [1.0])
        self.assertEqual(list(point.accelerations), [1.0])
        response.fraction = 0.5
        with self.assertRaises(RuntimeError):
            drawing.DrawingNode.compute_cartesian(node, [Pose()], Header(), 'test')

    def test_approach_targets_requested_point_without_joint_goal(self):
        node = Mock()
        node.cfg = {'ee_link': 'tool0', 'group_name': 'ur_manipulator', 'speed_scale': 0.125}
        handle = Mock(accepted=True)
        result = Mock()
        result.result.error_code.val = MoveItErrorCodes.SUCCESS
        node.wait_result.side_effect = [handle, result]
        start = Pose()
        start.position.y = -0.08
        start.position.z = 0.14
        start.orientation.w = 1.0
        drawing.DrawingNode.move_to_start(node, start, Header(frame_id='base_link'))
        goal = node.move_client.send_goal_async.call_args.args[0]
        constraint = goal.request.goal_constraints[0]
        self.assertEqual(constraint.joint_constraints, [])
        self.assertEqual(constraint.orientation_constraints, [])
        self.assertEqual(constraint.position_constraints[0].constraint_region.primitive_poses[0], start)
        self.assertFalse(goal.planning_options.plan_only)


if __name__ == '__main__':
    unittest.main()
