import argparse
import yaml
import rclpy
from rclpy.node import Node

class ConfigurableNode(Node):
    def __init__(self, args):
        super().__init__('configurable_node')

        # YAML 파일 로드
        with open(args.config, 'r') as file:
            config = yaml.safe_load(file)

        # 파라미터 적용
        for key, value in config.items():
            self.declare_parameter(key, value)
            self.get_logger().info(f"Loaded param: {key} = {value}")
        
        self.get_logger().info(f"prefix: {args.prefix}")

def main(args=None):
    rclpy.init(args=args)

    # ROS2의 --ros-args를 무시하도록 처리
    parser = argparse.ArgumentParser(description="Load YAML config for ROS2 node")
    parser.add_argument("--prefix", type=str, default='', help="Prefix for the node")
    parser.add_argument("--config", type=str, required=True, help="Path to config.yaml file")
    args, _ = parser.parse_known_args()  # 🔹 `parse_known_args()`를 사용하여 ROS2 인자 무시
    

    node = ConfigurableNode(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()