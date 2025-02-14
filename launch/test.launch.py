import os
from sys import prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "ur10_python_interface"  # 패키지 이름 입력

    # Launch 인자로 설정
    env_arg = DeclareLaunchArgument(
        "env",
        default_value='gazebo',
        description="Environment to use"
    )
    prefix_arg = DeclareLaunchArgument(
        "prefix",
        default_value="",
        description="Prefix for the node"
    )

    # 실행할 노드 (setup.py에서 등록한 이름과 동일해야 함)
    node = Node(
        package=package_name,
        executable="config",  # 🔹 실행할 노드 이름 확인 필요
        name="configurable_node",
        output="screen",
        arguments=[
            "--prefix", LaunchConfiguration("prefix"),
            "--env", LaunchConfiguration("env")
        ]
    )

    return LaunchDescription([
        prefix_arg,
        env_arg,
        node
    ])