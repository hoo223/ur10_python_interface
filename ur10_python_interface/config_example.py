import argparse
import yaml
import rclpy
from rclpy.node import Node

class ConfigurableNode(Node):
    def __init__(self, config_path):
        super().__init__('configurable_node')

        # YAML 파일 로드
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        # 파라미터 적용
        for key, value in config.items():
            self.declare_parameter(key, value)
            self.get_logger().info(f"Loaded param: {key} = {value}")

def main(args=None):
    rclpy.init(args=args)

    # ROS2의 --ros-args를 무시하도록 처리
    parser = argparse.ArgumentParser(description="Load YAML config for ROS2 node")
    parser.add_argument("--config", type=str, required=True, help="Path to config.yaml file")
    known_args, _ = parser.parse_known_args()  # 🔹 `parse_known_args()`를 사용하여 ROS2 인자 무시

    node = ConfigurableNode(known_args.config)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()