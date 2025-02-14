#!/usr/bin/python
# -*- coding: utf8 -*- 

## standard library
from math import *
import argparse
import yaml
import os

## ros library
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
#from tf.transformations import *
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController, ListControllers
from moveit.planning import MoveItPy
from builtin_interfaces.msg import Duration


# mode
INIT = 0
TELEOP = 1
TASK_CONTROL = 2
JOINT_CONTROL = 3
RSA = 4
MOVEIT = 5
IDLE = 6

mode_dict = {
    0: "INIT",
    1: "TELEOP",
    2: "TASK_CONTROL",
    3: "JOINT_CONTROL",
    4: "RSA",
    5: "MOVEIT",
    6: "IDLE"
}

def get_package_dir(package_name):
    share_dir = get_package_share_directory(package_name)
    package_dir = share_dir.replace('install', 'src').removesuffix(f'/share/{package_name}')
    return package_dir

class ModeManager(Node):
    def __init__(self, args):
        super().__init__('mode_manager')

        # config 파일 로드
        config_path = os.path.join(get_package_dir("ur10_python_interface"), 'config', f"config_{args.env}.yaml")
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        # Upload parameters
        for key, value in config.items():
            self.declare_parameter(key, value)
            self.get_logger().info(f"Loaded param: {key} = {value}")
            
        # initialize
        self.button = 0.0
        self.mode = INIT
        self.base_controller = config['base_controller']
        self.velocity_controller = config['velocity_controller']
            
        # publisher & subscriber
        self.joystick_command_sub = self.create_subscription(Float64MultiArray, 'joystick_command', self.joystick_command_callback, 10)
        self.keyboard_command_sub = self.create_subscription(Float64MultiArray, 'keyboard_command', self.keyboard_command_callback, 10)
        
        # service
        self.switch_controller_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.list_controller_client = self.create_client(ListControllers, '/controller_manager/list_controllers')

        
        # mode loop
        self.timer = self.create_timer(0.01, self.timer_callback)
        
    def joystick_command_callback(self, msg):
        self.joystick_command = msg.data
        self.button = self.joystick_command[-1]
    
    def keyboard_command_callback(self, msg):
        self.keyboard_command = msg.data
        self.button = self.keyboard_command[-1]

    def timer_callback(self):
        if self.button == 6.0:
            self.set_parameters([rclpy.parameter.Parameter('mode', rclpy.Parameter.Type.INTEGER, INIT)])
        elif self.button == 7.0:
            self.set_parameters([rclpy.parameter.Parameter('mode', rclpy.Parameter.Type.INTEGER, TELEOP)])
        
        mode = self.get_parameter('mode').get_parameter_value().integer_value
        #self.get_logger().info(f'Mode {mode}')
        
        if mode is not self.mode:
            print(f"{mode_dict[mode]} mode")
            if mode in [INIT, MOVEIT]:
                #self.change_to_base_controller()
                #if mode == INIT:  self.mgi.init_pose()
                conlist = self.get_active_controllers()
                print(conlist)
                pass
            elif mode in [TELEOP, TASK_CONTROL, JOINT_CONTROL, IDLE, RSA]:
                #self.change_to_velocity_controller()
                conlist = self.get_active_controllers()
                print(conlist)
                pass
            self.mode = mode
            print(f"Current mode: {mode_dict[self.mode]}")
        
        '''
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % my_param)

        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)
        '''

    def change_to_velocity_controller(self):
        print("change to ", self.velocity_controller)
        res = self.controller_change(self.base_controller, self.velocity_controller)
        return res
    
    def change_to_base_controller(self):
        print("change to ", self.base_controller)
        res = self.controller_change(self.velocity_controller, self.base_controller)
        return res
    
    # def controller_change(self, current_controller, target_controller):
    #     self.get_logger().info(f'Calling /controller_manager/switch_controller service')

    #     if not self.client.wait_for_service(timeout_sec=5.0):  # 🔹 대기 시간 5초로 증가
    #         self.get_logger().error('Service /controller_manager/switch_controller is not available!')
    #         return False

    #     req = SwitchController.Request()
    #     req.activate_controllers = [target_controller]
    #     req.deactivate_controllers = [current_controller]
    #     req.strictness = SwitchController.Request.BEST_EFFORT
    #     req.timeout = Duration(sec=2, nanosec=0)  # 🔹 Gazebo에서 2초 대기하도록 설정

    #     # 🔹 비동기 호출 후 최대 10초까지 대기
    #     future = self.client.call_async(req)
    #     rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)  # 🔹 10초까지 대기

    #     if future.result() is not None:
    #         res = future.result()
    #         if res.ok:
    #             self.get_logger().info(f'Controller changed from {current_controller} to {target_controller}')
    #             return True
    #         else:
    #             self.get_logger().error('Failed to change controller')
    #             return False
    #     else:
    #         self.get_logger().error('Service call failed or timed out!')
    #         return False
    
    def get_active_controllers(self):
        if not self.list_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /controller_manager/list_controllers is not available!')
            return []
        self.get_logger().info('Service /controller_manager/list_controllers is available!')
        
        req = ListControllers.Request()

        try:
            # 🟢 동기 서비스 호출 (10초 대기)
            future = self.list_controller_client.call(req)

            if future is not None:
                controllers = future.controller
                active_controllers = [c.name for c in controllers if c.state == 'active']
                self.get_logger().info(f'Active Controllers: {active_controllers}')
                return active_controllers
            else:
                self.get_logger().error('Received empty response from service!')
                return []
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            return []


        # future = self.list_controller_client.call_async(ListControllers.Request())
        # rclpy.spin_until_future_complete(self, future)

        # if future.result() is not None:
        #     controllers = future.result().controller
        #     active_controllers = [c.name for c in controllers if c.state == 'active']
        #     self.get_logger().info(f'Active Controllers: {active_controllers}')
        #     return active_controllers
        # else:
        #     self.get_logger().error('Failed to get controller list!')
        #     return []
    
    def controller_change(self, current_controller, target_controller):
        self.get_logger().info(f'Calling /controller_manager/switch_controller service')

        if not self.switch_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /controller_manager/switch_controller is not available!')
            return False

        req = SwitchController.Request()
        req.activate_controllers = [target_controller]
        req.deactivate_controllers = [current_controller]
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.timeout = Duration(sec=2, nanosec=0)  # 요청 대기 시간 설정

        future = self.switch_controller_client.call_async(req)

        # 🔹 응답을 기다리지 않고 일정 시간 후 그냥 넘어감
        timeout = 5.0  # 최대 대기 시간 (초)
        elapsed_time = 0.0
        while not future.done() and elapsed_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            elapsed_time += 0.5

        if future.done() and future.result() is not None:
            res = future.result()
            if res.ok:
                self.get_logger().info(f'Controller changed from {current_controller} to {target_controller}')
                return True
            else:
                self.get_logger().error('Failed to change controller')
                return False
        else:
            self.get_logger().warn(f'No response received after {timeout} seconds, continuing execution...')
            return True  # 🚀 응답이 없어도 다음 코드로 진행

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="Load YAML config for ROS2 node")
    parser.add_argument("--prefix", type=str, default='', help="Prefix for the node")
    parser.add_argument("--env", type=str, default='gazebo', help="Path to config.yaml file")
    args, _ = parser.parse_known_args()  # 🔹 `parse_known_args()`를 사용하여 ROS2 인자 무시
    
    mode_manager = ModeManager(args)
    rclpy.spin(mode_manager)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mode_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    