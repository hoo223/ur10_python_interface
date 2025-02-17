import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers

class ControllerListClient(Node):
    def __init__(self):
        super().__init__('controller_list_client')
        self.client = self.create_client(ListControllers, '/controller_manager/list_controllers')

    def get_controller_list(self):
        """ 현재 로드된 컨트롤러 목록을 가져오는 함수 """
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /controller_manager/list_controllers is not available!')
            return []

        req = ListControllers.Request()

        try:
            # 🟢 동기 서비스 호출
            future = self.client.call(req)

            if future is not None:
                controllers = future.controller
                for c in controllers:
                    self.get_logger().info(f'Controller: {c.name}, Type: {c.type}, State: {c.state}')
                return controllers
            else:
                self.get_logger().error('Received empty response from service!')
                return []
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            return []

def main(args=None):
    rclpy.init(args=args)
    client = ControllerListClient()
    controllers = client.get_controller_list()
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()