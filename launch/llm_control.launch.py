"""Start the natural-language planner, validator and skill executor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = {
        "student_name": "YOUR_NAME",
        "student_id": "00000000",
        "gemini_model": "gemini-2.5-flash",
        "gemini_api_key_env": "GEMINI_API_KEY",
        "gemini_timeout": "30.0",
    }
    arguments = [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
    world_config = PathJoinSubstitution([
        FindPackageShare("ur_llm_camera"), "config", "world.yaml"])
    node = Node(
        package="ur_llm_camera",
        executable="llm_skill_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "world_config": world_config,
            "student_name": ParameterValue(LaunchConfiguration("student_name"), value_type=str),
            "student_id": ParameterValue(LaunchConfiguration("student_id"), value_type=str),
            "gemini_model": ParameterValue(
                LaunchConfiguration("gemini_model"), value_type=str),
            "gemini_api_key_env": ParameterValue(
                LaunchConfiguration("gemini_api_key_env"), value_type=str),
            "gemini_timeout": ParameterValue(
                LaunchConfiguration("gemini_timeout"), value_type=float),
        }],
    )
    return LaunchDescription(arguments + [node])
