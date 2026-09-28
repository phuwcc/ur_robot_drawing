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
        "router_base_url": "http://127.0.0.1:20128/v1",
        "router_model": "CHANGE_ME",
        "router_api_key_env": "NINE_ROUTER_API_KEY",
        "mock_llm": "false",
    }
    arguments = [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
    world_config = PathJoinSubstitution([
        FindPackageShare("ur_llm_control"), "config", "world.yaml"])
    node = Node(
        package="ur_llm_control",
        executable="llm_skill_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "world_config": world_config,
            "student_name": ParameterValue(LaunchConfiguration("student_name"), value_type=str),
            "student_id": ParameterValue(LaunchConfiguration("student_id"), value_type=str),
            "router_base_url": ParameterValue(LaunchConfiguration("router_base_url"), value_type=str),
            "router_model": ParameterValue(LaunchConfiguration("router_model"), value_type=str),
            "router_api_key_env": ParameterValue(
                LaunchConfiguration("router_api_key_env"), value_type=str),
            "mock_llm": ParameterValue(LaunchConfiguration("mock_llm"), value_type=bool),
        }],
    )
    return LaunchDescription(arguments + [node])
