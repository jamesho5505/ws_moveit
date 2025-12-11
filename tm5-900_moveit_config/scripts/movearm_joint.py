#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import JointConstraint, Constraints, MotionPlanRequest
from builtin_interfaces.msg import Duration

# ---!!! 請修改這裡 !!!---
# 1. 您的六個關節名稱 (請確認與您的URDF和MoveIt Config一致)
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
# 2. 您的 MoveIt 規劃群組名稱
GROUP_NAME = "tmr_arm" 

# 您的目標關節角度 (度)
# TARGET_DEGREES = [29.76,-24.07,109.28,4.73,89.79,119.56]
TARGET_DEGREES = [55.57, -15.99,104.32,6.77,63.35,223.57]
# TARGET_DEGREES = [-65.98,26.81,-91.97,-43.98,-72.44,-76.57]
# TARGET_DEGREES = [-62.25,15.02,-100.97,-13.06,-110.64,-83.71]
# TARGET_DEGREES = [-31.05,66.13,-90.65,-29.41,-80.62,-147.11]
# TARGET_DEGREES = [71.52,-63.55,66.23,69.52,73.53,120.34]
# TARGET_DEGREES = [113.93,-23.69,56.57,61.19,87.73,166.33]
# TARGET_DEGREES = [55.44,9.99,72.31,35.96,116.24,-38.40]

class MoveGroupActionClient(Node):
    def __init__(self):
        super().__init__('moveit_planner_client')
        # MoveIt 2 的規劃/執行動作伺服器名稱
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('MoveGroup Action Client initialized.')

    def degrees_to_radians(self, degrees):
        """將角度清單轉換為弧度清單"""
        return np.radians(degrees).tolist()

    def create_goal(self):
        """建立 MoveGroup Action Goal 訊息"""
        
        goal_msg = MoveGroup.Goal()
        
        # 1. 建立運動規劃請求 (MotionPlanRequest)
        req = MotionPlanRequest()
        req.group_name = GROUP_NAME
        req.num_planning_attempts = 1  # 嘗試次數
        req.allowed_planning_time = 5.0 # 規劃時間 (秒)
        req.planner_id = "RRTConnect" # 可選：指定規劃器
        
        # 2. 建立目標約束 (Goal Constraints)
        constraints = Constraints()
        
        target_radians = self.degrees_to_radians(TARGET_DEGREES)
        self.get_logger().info(f"Target (Radians): {target_radians}")

        for i, (name, position) in enumerate(zip(JOINT_NAMES, target_radians)):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.001  # 容許誤差 (弧度)
            jc.tolerance_below = 0.001
            jc.weight = 1.0 # 權重
            constraints.joint_constraints.append(jc)
        
        req.goal_constraints.append(constraints)
        
        # 3. 將請求放入 Goal 訊息中
        goal_msg.request = req
        
        # 4. 設定規劃選項 (讓它規劃完畢後直接執行)
        goal_msg.planning_options.plan_only = False
        # goal_msg.planning_options.planning_time = 5.0
        
        return goal_msg

    def send_goal(self):
        """發送 MoveGroup Action 請求"""
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = self.create_goal()
        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """處理目標響應"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """處理動作結果"""
        result = future.result().result
        
        if result.error_code.val == 1: # MoveItErrorCodes.SUCCESS
            self.get_logger().info('✅ 運動規劃與執行成功！')
        else:
            self.get_logger().error(f'❌ 運動規劃或執行失敗。錯誤碼: {result.error_code.val}')
            # 錯誤碼 1: 成功; -31: 找不到逆向運動學解; -1: 規劃失敗; 等

        rclpy.shutdown()


def main(args=None):
    try:
        rclpy.init(args=args)
        action_client = MoveGroupActionClient()
        action_client.send_goal()
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
