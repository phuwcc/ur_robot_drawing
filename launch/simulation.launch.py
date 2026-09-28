"""Start the custom Gazebo world, UR3e, MoveIt and an optional RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Gazebo resolves model:// URIs relative to directories in its resource path.
    robotiq_models_path = os.path.dirname(
        get_package_share_directory("robotiq_description")
    )
    ignition_resource_path = SetEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH",
        [robotiq_models_path, os.pathsep,
         EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value="")],
    )
    gz_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        [robotiq_models_path, os.pathsep,
         EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
    )
    world = PathJoinSubstitution([FindPackageShare("ur_llm_control"), "worlds", "pick_place.sdf"])
    gripper_topics = [
        f"/gripper/{name}/{action}@std_msgs/msg/Empty]ignition.msgs.Empty"
        for name in ("red_cube", "yellow_cube", "blue_cube")
        for action in ("attach", "detach")
    ]
    gripper_topics.extend([
        f"/model/{name}/detachable_joint/state@std_msgs/msg/String[ignition.msgs.StringMsg"
        for name in ("red_cube", "yellow_cube", "blue_cube")
    ])
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"])),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "world_file": world,
            "runtime_config_package": "ur_llm_control",
            "controllers_file": "ur_controllers.yaml",
            "description_package": "ur_llm_control",
            "description_file": "ur3e_robotiq_2f85.urdf.xacro",
            "launch_rviz": "false",
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "description_package": "ur_llm_control",
            "description_file": "ur3e_robotiq_2f85.urdf.xacro",
            "moveit_config_file": PathJoinSubstitution([
                FindPackageShare("ur_llm_control"),
                "srdf",
                "ur3e_robotiq_2f85.srdf.xacro",
            ]),
            "use_sim_time": "true",
            "launch_rviz": LaunchConfiguration("launch_rviz"),
        }.items(),
    )
    gripper_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=gripper_topics,
        output="screen",
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        ignition_resource_path,
        gz_resource_path,
        control,
        gripper_controller_spawner,
        moveit,
        gripper_bridge,
    ])
