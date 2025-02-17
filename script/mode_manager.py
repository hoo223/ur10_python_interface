#!/usr/bin/python
# -*- coding: utf8 -*- 

## standard library
from logging import config
from math import *
import argparse
import yaml
import os
from pathlib import Path
import time

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
READY = -1

mode_dict = {
    0: "INIT",
    1: "TELEOP",
    2: "TASK_CONTROL",
    3: "JOINT_CONTROL",
    4: "RSA",
    5: "MOVEIT",
    6: "IDLE",
    -1: "READY"
}

def get_package_dir(package_name):
    share_dir = get_package_share_directory(package_name)
    package_dir = share_dir.replace('install', 'src').removesuffix(f'/share/{package_name}')
    return package_dir

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)
    logger.info("Completed execution")

class ModeManager(Node):
    def __init__(self, args, ur10):
        super().__init__('mode_manager')
        self.ur10 = ur10
        self.ur10_arm = ur10.get_planning_component("ur10_arm")
        self.get_logger().info(f"MoveItPy instance created, {type(self.ur10_arm)}")

        # config 파일 로드
        config_path = os.path.join(get_package_dir("ur10_python_interface"), 'config', f"config_{args.env}.yaml")
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        # Upload parameters
        for key, value in config.items():
            self.declare_parameter(key, value)
            # self.get_logger().info(f"Loaded param: {key} = {value}")
            
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
        
        # initialize pose
        self.move_to_config_pose('init', sleep_time=5.0)
        
        # mode loop
        self.timer = self.create_timer(0.001, self.timer_callback)
        
    def joystick_command_callback(self, msg):
        self.joystick_command = msg.data
        self.button = self.joystick_command[-1]
    
    def keyboard_command_callback(self, msg):
        self.keyboard_command = msg.data
        self.button = self.keyboard_command[-1]

    def timer_callback(self):
        # mode change by input
        if self.button == 6.0:
            self.set_parameters([rclpy.parameter.Parameter('mode', rclpy.Parameter.Type.INTEGER, INIT)])
        elif self.button == 7.0:
            self.set_parameters([rclpy.parameter.Parameter('mode', rclpy.Parameter.Type.INTEGER, TELEOP)])
        # read current mode
        mode = self.get_parameter('mode').get_parameter_value().integer_value
        
        # if mode changed
        if mode is not self.mode:
            # switch controller interface
            if mode in [INIT, MOVEIT]:
                #conlist = self.get_active_controllers()
                #print(conlist)
                self.change_to_base_controller()
            elif mode in [TELEOP, TASK_CONTROL, JOINT_CONTROL, IDLE, RSA]:
                self.change_to_velocity_controller()
                
            # move pose
            if mode == INIT:
                self.move_to_config_pose('init')
            elif mode == READY:
                self.move_to_config_pose('ready')
            
            # update mode
            self.mode = mode
            print(f"Current mode: {mode_dict[self.mode]}")
        
    def move_to_config_pose(self, config_pose, sleep_time=2.0):
        assert config_pose in ['up', 'init', 'ready'], f"Invalid pose {config_pose}"
        self.ur10_arm.set_start_state_to_current_state()
        self.ur10_arm.set_goal_state(configuration_name=config_pose)
        #current_state = self.ur10_arm.get_start_state()
        #self.get_logger().info(f"Current state!!!!!!!!!!!!!!!: {current_state.joint_positions}")
        plan_and_execute(self.ur10, self.ur10_arm, self.get_logger(), sleep_time=sleep_time)        
        
    def change_to_velocity_controller(self):
        print("change to ", self.velocity_controller)
        res = self.switch_controller(self.base_controller, self.velocity_controller)
        return res
    
    def change_to_base_controller(self):
        print("change to ", self.base_controller)
        res = self.switch_controller(self.velocity_controller, self.base_controller)
        return res
    
    def switch_controller(self, deactivate_controllers, activate_controllers, strictness=2):
        """ 컨트롤러 변경 함수 """
        if not self.switch_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /controller_manager/switch_controller is not available!')
            return False

        req = SwitchController.Request()
        req.activate_controllers = [activate_controllers]
        req.deactivate_controllers = [deactivate_controllers]
        req.strictness = strictness  # 2 = STRICT, 1 = BEST_EFFORT

        # Duration 설정 (필수)
        req.timeout = Duration()
        req.timeout.sec = 2
        req.timeout.nanosec = 0

        future = self.switch_controller_client.call_async(req)
        # self.get_logger().info(f"future: {future.done()} {future.result()}")
        return True
        #rclpy.spin_until_future_complete(self, future)

        # if future.done() and future.result() is not None:
        #     if future.result().ok:
        #         self.get_logger().info(f'Successfully switched controllers: Activated {activate_controllers}, Deactivated {deactivate_controllers}')
        #         return True
        #     else:
        #         self.get_logger().error('Controller switch failed!')
        #         return False
        # else:
        #     self.get_logger().error('Service call timed out or failed.')
        #     return False
    
    def get_active_controllers(self):
        if not self.list_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /controller_manager/list_controllers is not available!')
            return []
        self.get_logger().info('Service /controller_manager/list_controllers is available!')
        
        req = ListControllers.Request()
        future = self.list_controller_client.call_async(req)
        self.get_logger().info(f"future: {future.done()} {future.result()}")
        # controllers = future.result().controller
        # active_controllers = [c.name for c in controllers if c.state == 'active']
        # self.get_logger().info(f'Active Controllers: {active_controllers}')
        return True

        # try:
        #     # 🟢 동기 서비스 호출 (10초 대기)
        #     future = self.list_controller_client.call(req)
        #     if future is not None:
        #         controllers = future.controller
        #         active_controllers = [c.name for c in controllers if c.state == 'active']
        #         self.get_logger().info(f'Active Controllers: {active_controllers}')
        #         return active_controllers
        #     else:
        #         self.get_logger().error('Received empty response from service!')
        #         return []
        # except Exception as e:
        #     self.get_logger().error(f'Service call failed: {str(e)}')
        #     return []

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

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="Load YAML config for ROS2 node")
    parser.add_argument("--prefix", type=str, default='', help="Prefix for the node")
    parser.add_argument("--env", type=str, default='gazebo', help="Path to config.yaml file")
    args, _ = parser.parse_known_args()  # 🔹 `parse_known_args()`를 사용하여 ROS2 인자 무시
    
    # instantiate MoveItPy -> should be done outside of Node class!!
    ur10 = MoveItPy(node_name="test")    
    mode_manager = ModeManager(args, ur10)
    rclpy.spin(mode_manager)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mode_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    