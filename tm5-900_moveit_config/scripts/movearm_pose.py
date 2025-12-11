#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import JointConstraint, Constraints, MotionPlanRequest
from moveit_msgs.msg import PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
import tf_transformations

rx, ry, rz = 180.0, 0.0, 180.0
# rx, ry, rz = -136.2, 0.0, 180.0
# rx, ry, rz = 135.9, 0.0, 180.0
# rx, ry, rz = -137.7, 29.3, -155.9
# rx, ry, rz = 175.0, -26.3, -160.6
# rx, ry, rz = 172.9, 38.1, -143.5
# rx, ry, rz = -147.1, -17.0, -157.0
# rx, ry, rz = -166.6, 41.3, -158.0
# rx, ry, rz = -169.8, 10.9, -164.9

# rx, ry, rz = 180.0, 0.0, 180.0
# rx, ry, rz = -136.2, 0.0, 180.0
# rx, ry, rz = 135.9, 0.0, 180.0
# rx, ry, rz = -137.7, 29.3, 180.0
# rx, ry, rz = -137.7, -29.3, 180.0
# rx, ry, rz = 135.9, -30.0, 180.0
# rx, ry, rz = -147.1, 40.0, -150.0
# rx, ry, rz = -166.6, -40.0, -150.0
# rx, ry, rz = 169.8, 10.9, -164.9
# x,y,z = -0.00737,-0.68338,0.1  # use tcp
x,y,z = 0.10, -0.6, 0.35 # use tcp observe
# x,y,z = 0.008212221145629884, -0.7441826171875, 0.03035211181640625 # use tcp observe
# rx, ry, rz = -120.0, 0.0, 178.8
# x,y,z = -0.0, -0.4, 0.6 # use flange
# rx, ry, rz = -150.0, 30.0, -180.0
GROUP_NAME = "tmr_arm" 
TARGET_POSITION = [x, y, z]  # 位置 (x, y, z)
# TARGET_POSITION = [-0.24072930908203125, -0.3410655517578125, 0.5430317382812501]  # 位置 (x, y, z)
TARGET_RPY = [rx*np.pi/180, ry*np.pi/180, rz*np.pi/180]         # 姿態 (rx, ry, rz)，單位：弧度


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
        goal_msg = MoveGroup.Goal()
        
        req = MotionPlanRequest()
        req.group_name = GROUP_NAME
        req.num_planning_attempts = 1
        req.allowed_planning_time = 5.0
        req.planner_id = "RRTConnect"

        # Set velocity scaling factor
        req.max_velocity_scaling_factor = 1.0  # Scale to 100% of maximum velocity
        req.max_acceleration_scaling_factor = 1.0 # Scale to 100% of maximum acceleration

        # === 1. 建立 Pose 目標 ===
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base"  # 請依你的 robot frame 改

        pose_stamped.pose.position.x = TARGET_POSITION[0]
        pose_stamped.pose.position.y = TARGET_POSITION[1]
        pose_stamped.pose.position.z = TARGET_POSITION[2]

        q = tf_transformations.quaternion_from_euler(
            TARGET_RPY[0], TARGET_RPY[1], TARGET_RPY[2]
        )
        pose_stamped.pose.orientation.x = q[0]
        pose_stamped.pose.orientation.y = q[1]
        pose_stamped.pose.orientation.z = q[2]
        pose_stamped.pose.orientation.w = q[3]

        # === 2. 將 Pose 轉成 constraints ===
        constraints = Constraints()
        pos_constraint = constraints.position_constraints
        ori_constraint = constraints.orientation_constraints

        pc = PositionConstraint()
        pc.header.frame_id = "base"
        # pc.link_name = "flange"  # 請依你的末端 link 改
        pc.link_name = "tcp"  # 請依你的末端 link 改
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.001, 0.001, 0.001]  # 1mm cube
        pc.constraint_region.primitives.append(box)
        pc.constraint_region.primitive_poses.append(pose_stamped.pose)
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = "base"
        # oc.link_name = "flange"
        oc.link_name = "tcp"
        oc.orientation = pose_stamped.pose.orientation
        oc.absolute_x_axis_tolerance = 0.001
        oc.absolute_y_axis_tolerance = 0.001
        oc.absolute_z_axis_tolerance = 0.001
        oc.weight = 1.0

        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)

        req.goal_constraints.append(constraints)

        # === 3. 讓 MoveIt 規劃後直接執行 ===
        goal_msg.request = req
        goal_msg.planning_options.plan_only = False

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
