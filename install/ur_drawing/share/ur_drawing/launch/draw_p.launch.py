"""Start the drawing node and RViz after the robot stack is running."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = {
        "x": "0.30",
        "y": "-0.08",
        "z": "0.18",
        "shape": "p",
        "radius": "0.04",
        "width": "0.06",
        "height": "0.12",
        "step": "0.004",
        "speed_scale": "0.125",
        "execute": "true",
        "launch_rviz": "false",
    }
    arguments = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    parameters = {
        k: ParameterValue(LaunchConfiguration(k), value_type=float)
        for k in ("x", "y", "z", "width", "height", "radius", "step", "speed_scale")
    }
    parameters.update({
        "shape": ParameterValue(LaunchConfiguration("shape"), value_type=str),
        "use_sim_time": True,
        "execute": ParameterValue(LaunchConfiguration("execute"), value_type=bool),
    })
    return LaunchDescription(arguments + [
        SetEnvironmentVariable("LC_ALL", "C.UTF-8"),
        Node(
            package="ur_drawing",
            executable="draw_p.py",
            output="screen",
            parameters=[parameters],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz_draw_p",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("ur_drawing"),
                "config",
                "draw_p.rviz",
            ])],
            parameters=[{"use_sim_time": True}],
            additional_env={"LC_ALL": "C.UTF-8"},
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        ),
    ])
