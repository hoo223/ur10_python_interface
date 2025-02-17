import os
from sys import prefix
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur10_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml") # scaled_joint_trajectory_controller
        .moveit_cpp(
            file_path=get_package_share_directory("ur10_python_interface")
            + "/config/motion_planning_parameteres.yaml"
        )
        .to_moveit_configs()
    )    

    move_group_launch = generate_move_group_launch(moveit_config)

    mode_manager = Node(
        package="ur10_python_interface",
        executable="mode_manager",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True}, # to match time stamp of "/joint_states" topic with gazebo
        ],
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True), # set "use_sim_time" to true for all instances in the launch file
        move_group_launch,
        mode_manager,
    ])