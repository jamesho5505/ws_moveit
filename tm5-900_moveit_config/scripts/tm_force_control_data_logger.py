#!/usr/bin/env python3
"""
TM Robot Force Control Data Logger

記錄以下數據：
- Time (timestamp)
- Joint positions (6 joints)
- TCP position (X, Y, Z, Rx, Ry, Rz)
- TCP speed (6 DOF)
- Force data (Fx, Fy, Fz in sensor frame and base frame)
- Normal force magnitude
- Normal direction unit vector
- Tangential direction unit vector
- Control mode
- Velocity commands
"""

import math
import csv
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Float64MultiArray
from tm_msgs.msg import FeedbackState
import numpy as np
from scipy.spatial.transform import Rotation


class TMForceControlDataLogger(Node):
    """數據記錄節點"""

    def __init__(self):
        super().__init__("tm_force_control_data_logger")

        # 創建輸出目錄
        self.output_dir = Path.home() / "tm_force_control_logs"
        self.output_dir.mkdir(exist_ok=True)
        
        # 創建 CSV 檔案
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = self.output_dir / f"force_control_log_{timestamp}.csv"
        
        # CSV 欄位
        self.csv_headers = [
            # Time
            "timestamp",
            "ros_time_sec",
            "ros_time_nsec",
            
            # Joint positions (rad)
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
            
            # TCP position (mm and deg)
            "tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz",
            
            # TCP velocity (mm/s and deg/s)
            "tcp_vel_x", "tcp_vel_y", "tcp_vel_z", 
            "tcp_vel_rx", "tcp_vel_ry", "tcp_vel_rz",
            
            # Desired trajectory point (mm) - 期望軌跡點
            "desired_traj_x", "desired_traj_y",
            
            # Corrected trajectory point (mm) - 修正後軌跡點
            "corrected_traj_x", "corrected_traj_y",
            
            # Target position (mm) - 最終目標位置
            "target_x", "target_y", "target_z",
            
            # Force in sensor frame (N)
            "force_sensor_x", "force_sensor_y", "force_sensor_z",
            
            # Force in base frame (N)
            "force_base_x", "force_base_y", "force_base_z",
            
            # Torque in sensor frame (N-m)
            "torque_sensor_x", "torque_sensor_y", "torque_sensor_z",
            
            # Normal force
            "normal_force_magnitude",
            "normal_direction_x", "normal_direction_y",
            
            # Tangential direction
            "tangential_direction_x", "tangential_direction_y",
            
            # Control mode
            "control_mode",

            # Delta_L1, Delta_L2, Theta
            "delta_L1", "delta_L2", "theta",
            
            # Velocity commands (mm/s and deg/s)
            # "velocity_cmd_x", "velocity_cmd_y", "velocity_cmd_z",
            # "velocity_cmd_rx", "velocity_cmd_ry", "velocity_cmd_rz",
        ]
        
        # 初始化 CSV 檔案
        with open(self.csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(self.csv_headers)
        
        self.get_logger().info(f"Data logging to: {self.csv_filename}")
        
        # 數據儲存
        self.latest_feedback = None
        self.latest_wrench = None
        self.force_base = np.array([0.0, 0.0, 0.0])
        self.normal_force_magnitude = 0.0
        self.normal_direction_unit = np.array([0.0, 0.0])
        self.tangential_direction_unit = np.array([0.0, 0.0])
        # self.velocity_command = [0.0] * 6
        
        # 軌跡資料
        self.desired_trajectory_point = [0.0, 0.0]      # 期望軌跡點 (X, Y)
        self.corrected_trajectory_point = [0.0, 0.0]    # 修正後軌跡點 (X, Y)
        self.target_position = [0.0, 0.0, 0.0]          # 最終目標位置 (X, Y, Z)
        self.control_mode = "UNKNOWN"                   # 控制模式
        self.delta_L1 = 0.0
        self.delta_L2 = 0.0
        self.theta = 0.0
        
        # 訂閱者
        self.feedback_subscription = self.create_subscription(
            FeedbackState,
            "feedback_states",
            self.feedback_callback,
            10
        )
        
        self.force_subscription = self.create_subscription(
            WrenchStamped,
            "/ft_compensated",
            self.force_callback,
            10
        )

        self.sub_desired = self.create_subscription(
            PointStamped,
            "/force_control/desired_point",
            self.desired_callback,
            10
        )

        self.sub_corrected = self.create_subscription(
            PointStamped,
            "/force_control/corrected_point",
            self.corrected_callback,
            10
        )

        self.sub_corrected_data = self.create_subscription(
            Float64MultiArray,
            "/force_control/corrected_data",
            self.corrected_data_callback,
            10
        )

        self.sub_target = self.create_subscription(
            PointStamped,
            "/force_control/target_point",
            self.target_callback,
            10
        )

        self.sub_state = self.create_subscription(
            String,
            "/force_control/control_state",
            self.state_callback,
            10
        )
        
        self.log_timer = self.create_timer(0.01, self.log_data_callback)

    def desired_callback(self, msg):
        self.desired_trajectory_point = [msg.point.x, msg.point.y]

    def corrected_callback(self, msg):
        self.corrected_trajectory_point = [msg.point.x, msg.point.y]
    
    def corrected_data_callback(self, msg):
        if len(msg.data) >= 3:
            self.delta_L1 = float(msg.data[0])
            self.delta_L2 = float(msg.data[1])
            self.theta    = float(msg.data[2])

    def target_callback(self, msg):
        self.target_position = [msg.point.x, msg.point.y, msg.point.z]

    def state_callback(self, msg):
        self.control_mode = msg.data

    def feedback_callback(self, msg: FeedbackState) -> None:
        """儲存 FeedbackState 數據"""
        self.latest_feedback = msg
        
        # 計算 base frame 的力
        if len(msg.tool_pose) >= 6:
            meters_to_mm = 1000.0
            rad_to_deg = 180.0 / math.pi
            
            rx_deg = msg.tool_pose[3] * rad_to_deg
            ry_deg = msg.tool_pose[4] * rad_to_deg
            rz_deg = msg.tool_pose[5] * rad_to_deg
            
            # Tool frame 到 Base frame 的旋轉矩陣
            rotation_tool_to_base = Rotation.from_euler(
                'zyx',
                [rz_deg, ry_deg, rx_deg],
                degrees=True
            ).as_matrix()
            
            # 如果有力數據，轉換到 base frame
            if self.latest_wrench is not None:
                force_sensor = np.array([
                    self.latest_wrench.wrench.force.x,
                    self.latest_wrench.wrench.force.y,
                    self.latest_wrench.wrench.force.z
                ])
                
                self.force_base = rotation_tool_to_base @ force_sensor
                
                # 計算法向力（XY 平面）
                force_xy = np.array([self.force_base[0], self.force_base[1]])
                self.normal_force_magnitude = np.linalg.norm(force_xy)
                
                if self.normal_force_magnitude > 1e-6:
                    self.normal_direction_unit = force_xy / self.normal_force_magnitude
                    
                    # 切向 = 法向逆時針旋轉 90 度
                    self.tangential_direction_unit = np.array([
                        -self.normal_direction_unit[1],
                        self.normal_direction_unit[0]
                    ])
                else:
                    self.normal_direction_unit = np.array([0.0, 0.0])
                    self.tangential_direction_unit = np.array([0.0, 0.0])

    def force_callback(self, msg: WrenchStamped) -> None:
        """儲存力/力矩數據"""
        self.latest_wrench = msg

    def log_data_callback(self) -> None:
        """定時記錄數據到 CSV"""
        if self.latest_feedback is None or self.latest_wrench is None:
            return
        
        try:
            # 準備數據行
            current_time = self.get_clock().now()
            timestamp = datetime.now().isoformat()
            
            meters_to_mm = 1000.0
            rad_to_deg = 180.0 / math.pi
            
            # Joint positions
            joint_pos = list(self.latest_feedback.joint_pos) if len(self.latest_feedback.joint_pos) >= 6 else [0.0] * 6
            
            # TCP position
            tcp_pose = list(self.latest_feedback.tool_pose) if len(self.latest_feedback.tool_pose) >= 6 else [0.0] * 6
            tcp_x = tcp_pose[0] * meters_to_mm
            tcp_y = tcp_pose[1] * meters_to_mm
            tcp_z = tcp_pose[2] * meters_to_mm
            tcp_rx = tcp_pose[3] * rad_to_deg
            tcp_ry = tcp_pose[4] * rad_to_deg
            tcp_rz = tcp_pose[5] * rad_to_deg
            
            # TCP velocity
            tcp_speed = list(self.latest_feedback.tcp_speed) if len(self.latest_feedback.tcp_speed) >= 6 else [0.0] * 6
            tcp_vel_x = tcp_speed[0] * meters_to_mm
            tcp_vel_y = tcp_speed[1] * meters_to_mm
            tcp_vel_z = tcp_speed[2] * meters_to_mm
            tcp_vel_rx = tcp_speed[3] * rad_to_deg
            tcp_vel_ry = tcp_speed[4] * rad_to_deg
            tcp_vel_rz = tcp_speed[5] * rad_to_deg
            
            # Force in sensor frame
            force_sensor_x = self.latest_wrench.wrench.force.x
            force_sensor_y = self.latest_wrench.wrench.force.y
            force_sensor_z = self.latest_wrench.wrench.force.z
            
            # Force in base frame
            force_base_x = self.force_base[0]
            force_base_y = self.force_base[1]
            force_base_z = self.force_base[2]
            
            # Torque in sensor frame
            torque_sensor_x = self.latest_wrench.wrench.torque.x
            torque_sensor_y = self.latest_wrench.wrench.torque.y
            torque_sensor_z = self.latest_wrench.wrench.torque.z
            
            # 組合數據行
            data_row = [
                # Time
                timestamp,
                current_time.seconds_nanoseconds()[0],
                current_time.seconds_nanoseconds()[1],
                
                # Joint positions
                *joint_pos,
                
                # TCP position
                tcp_x, tcp_y, tcp_z, tcp_rx, tcp_ry, tcp_rz,
                
                # TCP velocity
                tcp_vel_x, tcp_vel_y, tcp_vel_z, tcp_vel_rx, tcp_vel_ry, tcp_vel_rz,
                
                # Desired trajectory point
                self.desired_trajectory_point[0],
                self.desired_trajectory_point[1],
                
                # Corrected trajectory point
                self.corrected_trajectory_point[0],
                self.corrected_trajectory_point[1],
                
                # Target position
                self.target_position[0],
                self.target_position[1],
                self.target_position[2],
                
                # Force sensor frame
                force_sensor_x, force_sensor_y, force_sensor_z,
                
                # Force base frame
                force_base_x, force_base_y, force_base_z,
                
                # Torque sensor frame
                torque_sensor_x, torque_sensor_y, torque_sensor_z,
                
                # Normal force
                self.normal_force_magnitude,
                self.normal_direction_unit[0],
                self.normal_direction_unit[1],
                
                # Tangential direction
                self.tangential_direction_unit[0],
                self.tangential_direction_unit[1],
                
                # Control mode
                self.control_mode,

                self.delta_L1,
                self.delta_L2,
                self.theta,
                
                # Velocity commands
                # *self.velocity_command,
            ]
            
            # 寫入 CSV
            with open(self.csv_filename, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(data_row)
                
        except Exception as e:
            self.get_logger().error(f"Error logging data: {e}")


def main(args=None):
    rclpy.init(args=args)
    logger = TMForceControlDataLogger()
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        logger.get_logger().info("Data logging stopped by user.")
    finally:
        logger.get_logger().info(f"Data saved to: {logger.csv_filename}")
        logger.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

# #!/usr/bin/env python3
# """
# TM Robot Force Control Data Logger

# 記錄以下數據：
# - Time (timestamp)
# - Joint positions (6 joints)
# - TCP position (X, Y, Z, Rx, Ry, Rz)
# - TCP speed (6 DOF)
# - Force data (Fx, Fy, Fz in sensor frame and base frame)
# - Normal force magnitude
# - Normal direction unit vector
# - Tangential direction unit vector
# - Control mode
# - Velocity commands
# """

# import math
# import csv
# import time
# from datetime import datetime
# from pathlib import Path

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import WrenchStamped
# from tm_msgs.msg import FeedbackState
# import numpy as np
# from scipy.spatial.transform import Rotation


# class TMForceControlDataLogger(Node):
#     """數據記錄節點"""

#     def __init__(self):
#         super().__init__("tm_force_control_data_logger")

#         # 創建輸出目錄
#         self.output_dir = Path.home() / "tm_force_control_logs"
#         self.output_dir.mkdir(exist_ok=True)
        
#         # 創建 CSV 檔案
#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         self.csv_filename = self.output_dir / f"force_control_log_{timestamp}.csv"
        
#         # CSV 欄位
#         self.csv_headers = [
#             # Time
#             "timestamp",
#             "ros_time_sec",
#             "ros_time_nsec",
            
#             # Joint positions (rad)
#             "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
            
#             # TCP position (mm and deg)
#             "tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz",
            
#             # TCP velocity (mm/s and deg/s)
#             "tcp_vel_x", "tcp_vel_y", "tcp_vel_z", 
#             "tcp_vel_rx", "tcp_vel_ry", "tcp_vel_rz",
            
#             # Force in sensor frame (N)
#             "force_sensor_x", "force_sensor_y", "force_sensor_z",
            
#             # Force in base frame (N)
#             "force_base_x", "force_base_y", "force_base_z",
            
#             # Torque in sensor frame (N-m)
#             "torque_sensor_x", "torque_sensor_y", "torque_sensor_z",
            
#             # Normal force
#             "normal_force_magnitude",
#             "normal_direction_x", "normal_direction_y",
            
#             # Tangential direction
#             "tangential_direction_x", "tangential_direction_y",
            
            
#             # Velocity commands (mm/s and deg/s)
#             "velocity_cmd_x", "velocity_cmd_y", "velocity_cmd_z",
#             "velocity_cmd_rx", "velocity_cmd_ry", "velocity_cmd_rz",
#         ]
        
#         # 初始化 CSV 檔案
#         with open(self.csv_filename, 'w', newline='') as f:
#             writer = csv.writer(f)
#             writer.writerow(self.csv_headers)
        
#         self.get_logger().info(f"Data logging to: {self.csv_filename}")
        
#         # 數據儲存
#         self.latest_feedback = None
#         self.latest_wrench = None
#         self.force_base = np.array([0.0, 0.0, 0.0])
#         self.normal_force_magnitude = 0.0
#         self.normal_direction_unit = np.array([0.0, 0.0])
#         self.tangential_direction_unit = np.array([0.0, 0.0])
#         self.velocity_command = [0.0] * 6
        
#         # 訂閱者
#         self.feedback_subscription = self.create_subscription(
#             FeedbackState,
#             "feedback_states",
#             self.feedback_callback,
#             10
#         )
        
#         self.force_subscription = self.create_subscription(
#             WrenchStamped,
#             "/ft_compensated",
#             self.force_callback,
#             10
#         )
        
#         # 定時器：以 100 Hz 記錄數據
#         self.log_timer = self.create_timer(0.01, self.log_data_callback)
        
#         self.get_logger().info("Data logger initialized. Starting to record...")

#     def feedback_callback(self, msg: FeedbackState) -> None:
#         """儲存 FeedbackState 數據"""
#         self.latest_feedback = msg
        
#         # 計算 base frame 的力
#         if len(msg.tool_pose) >= 6:
#             meters_to_mm = 1000.0
#             rad_to_deg = 180.0 / math.pi
            
#             rx_deg = msg.tool_pose[3] * rad_to_deg
#             ry_deg = msg.tool_pose[4] * rad_to_deg
#             rz_deg = msg.tool_pose[5] * rad_to_deg
            
#             # Tool frame 到 Base frame 的旋轉矩陣
#             rotation_tool_to_base = Rotation.from_euler(
#                 'zyx',
#                 [rz_deg, ry_deg, rx_deg],
#                 degrees=True
#             ).as_matrix()
            
#             # 如果有力數據，轉換到 base frame
#             if self.latest_wrench is not None:
#                 force_sensor = np.array([
#                     self.latest_wrench.wrench.force.x,
#                     self.latest_wrench.wrench.force.y,
#                     self.latest_wrench.wrench.force.z
#                 ])
                
#                 self.force_base = rotation_tool_to_base @ force_sensor
                
#                 # 計算法向力（XY 平面）
#                 force_xy = np.array([self.force_base[0], self.force_base[1]])
#                 self.normal_force_magnitude = np.linalg.norm(force_xy)
                
#                 if self.normal_force_magnitude > 1e-6:
#                     self.normal_direction_unit = force_xy / self.normal_force_magnitude
                    
#                     # 切向 = 法向逆時針旋轉 90 度
#                     self.tangential_direction_unit = np.array([
#                         -self.normal_direction_unit[1],
#                         self.normal_direction_unit[0]
#                     ])
#                 else:
#                     self.normal_direction_unit = np.array([0.0, 0.0])
#                     self.tangential_direction_unit = np.array([0.0, 0.0])

#     def force_callback(self, msg: WrenchStamped) -> None:
#         """儲存力/力矩數據"""
#         self.latest_wrench = msg

#     def log_data_callback(self) -> None:
#         """定時記錄數據到 CSV"""
#         if self.latest_feedback is None or self.latest_wrench is None:
#             return
        
#         try:
#             # 準備數據行
#             current_time = self.get_clock().now()
#             timestamp = datetime.now().isoformat()
            
#             meters_to_mm = 1000.0
#             rad_to_deg = 180.0 / math.pi
            
#             # Joint positions
#             joint_pos = list(self.latest_feedback.joint_pos) if len(self.latest_feedback.joint_pos) >= 6 else [0.0] * 6
            
#             # TCP position
#             tcp_pose = list(self.latest_feedback.tool_pose) if len(self.latest_feedback.tool_pose) >= 6 else [0.0] * 6
#             tcp_x = tcp_pose[0] * meters_to_mm
#             tcp_y = tcp_pose[1] * meters_to_mm
#             tcp_z = tcp_pose[2] * meters_to_mm
#             tcp_rx = tcp_pose[3] * rad_to_deg
#             tcp_ry = tcp_pose[4] * rad_to_deg
#             tcp_rz = tcp_pose[5] * rad_to_deg
            
#             # TCP velocity
#             tcp_speed = list(self.latest_feedback.tcp_speed) if len(self.latest_feedback.tcp_speed) >= 6 else [0.0] * 6
#             tcp_vel_x = tcp_speed[0] * meters_to_mm
#             tcp_vel_y = tcp_speed[1] * meters_to_mm
#             tcp_vel_z = tcp_speed[2] * meters_to_mm
#             tcp_vel_rx = tcp_speed[3] * rad_to_deg
#             tcp_vel_ry = tcp_speed[4] * rad_to_deg
#             tcp_vel_rz = tcp_speed[5] * rad_to_deg
            
#             # Force in sensor frame
#             force_sensor_x = self.latest_wrench.wrench.force.x
#             force_sensor_y = self.latest_wrench.wrench.force.y
#             force_sensor_z = self.latest_wrench.wrench.force.z
            
#             # Force in base frame
#             force_base_x = self.force_base[0]
#             force_base_y = self.force_base[1]
#             force_base_z = self.force_base[2]
            
#             # Torque in sensor frame
#             torque_sensor_x = self.latest_wrench.wrench.torque.x
#             torque_sensor_y = self.latest_wrench.wrench.torque.y
#             torque_sensor_z = self.latest_wrench.wrench.torque.z
            
#             # 組合數據行
#             data_row = [
#                 # Time
#                 timestamp,
#                 current_time.seconds_nanoseconds()[0],
#                 current_time.seconds_nanoseconds()[1],
                
#                 # Joint positions
#                 *joint_pos,
                
#                 # TCP position
#                 tcp_x, tcp_y, tcp_z, tcp_rx, tcp_ry, tcp_rz,
                
#                 # TCP velocity
#                 tcp_vel_x, tcp_vel_y, tcp_vel_z, tcp_vel_rx, tcp_vel_ry, tcp_vel_rz,
                
#                 # Force sensor frame
#                 force_sensor_x, force_sensor_y, force_sensor_z,
                
#                 # Force base frame
#                 force_base_x, force_base_y, force_base_z,
                
#                 # Torque sensor frame
#                 torque_sensor_x, torque_sensor_y, torque_sensor_z,
                
#                 # Normal force
#                 self.normal_force_magnitude,
#                 self.normal_direction_unit[0],
#                 self.normal_direction_unit[1],
                
#                 # Tangential direction
#                 self.tangential_direction_unit[0],
#                 self.tangential_direction_unit[1],
                
                
#                 # Velocity commands
#                 *self.velocity_command,
#             ]
            
#             # 寫入 CSV
#             with open(self.csv_filename, 'a', newline='') as f:
#                 writer = csv.writer(f)
#                 writer.writerow(data_row)
                
#         except Exception as e:
#             self.get_logger().error(f"Error logging data: {e}")


# def main(args=None):
#     rclpy.init(args=args)
#     logger = TMForceControlDataLogger()
    
#     try:
#         rclpy.spin(logger)
#     except KeyboardInterrupt:
#         logger.get_logger().info("Data logging stopped by user.")
#     finally:
#         logger.get_logger().info(f"Data saved to: {logger.csv_filename}")
#         logger.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == "__main__":
#     main()