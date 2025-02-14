import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "ur10_python_interface"  # 패키지 이름 입력
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "config_gazebo.yaml"
    )

    # 파일 존재 여부 확인
    if not os.path.exists(config_file_path):
        raise FileNotFoundError(f"Config file not found: {config_file_path}")

    # Launch 인자로 설정
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=config_file_path,
        description="Path to the configuration file"
    )

    # 실행할 노드 (setup.py에서 등록한 이름과 동일해야 함)
    node = Node(
        package=package_name,
        executable="config",  # 🔹 실행할 노드 이름 확인 필요
        name="configurable_node",
        output="screen",
        arguments=["--config", LaunchConfiguration("config")]
    )

    return LaunchDescription([
        config_arg,
        LogInfo(msg=f"Using config file: {config_file_path}"),
        node
    ])