#!/usr/bin/env python3
"""
Hybrid Position/Force Control using PVT with Trajectory Projection
使用 PVT 但控制位置，速度設為 0
- 預先計算圓形軌跡點
- 使用投影方法修正軌跡
- 實現力控制下的軌跡跟隨
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Float64MultiArray
from tm_msgs.msg import FeedbackState
from tm_msgs.srv import SendScript
import numpy as np
from scipy.spatial.transform import Rotation as Rotation


class HybridPvtForceNode(Node):
    """
    Hybrid Position/Force Control using PVT with Trajectory Projection
    PVT 模式但速度設為 0，直接控制位置
    """

    def __init__(self):
        super().__init__("hybrid_pvt_force_node")

        # === 1. 初始化與通訊 ===
        self.send_script_client = self.create_client(SendScript, "send_script")
        self.get_logger().info("Waiting for /send_script service...")
        self.send_script_client.wait_for_service()
        self.get_logger().info("/send_script service is ready.")

        # === 2. 核心控制參數 (PVT 框架) ===
        self.pvt_duration_seconds = 1.0 / 100.0  # 100 Hz 控制頻率

        self.has_tool_pose = False
        self.current_tool_pose = [0.0] * 6       # 機器人實際位置
        self.target_position = [0.0] * 6         # 目標位置（直接控制）
        self.reference_position = [0.0] * 6      # 參考位置
        
        # === 3. 狀態機與力控參數 ===
        self.control_mode = "APPROACH"  # "APPROACH" → "FORCE_CONTROL"
        self.latest_force = np.array([0.0, 0.0, 0.0])
        self.latest_torque = np.array([0.0, 0.0, 0.0])
        self.force_x_base = 0.0
        self.force_y_base = 0.0
        self.force_z_base = 0.0
        
        # 力控制參數
        self.desired_normal_force_newton = 5.0   # 目標法向力 (N)
        self.max_force_limit = 20.0              # 最大力限制 (N)
        self.normal_force_threshold = 3.00        # 接觸檢測門檻 (N)
        self.min_normal_force_eps = 1e-6
        
        # 位置限制
        self.min_y_position = -421.0   # mm
        self.min_z_position = 45.0     # mm
        
        # 軌跡參數
        self.circle_center = np.array([135.0, -515.0])  # 圓心 (X, Y)
        self.circle_radius = 20.0                       # 半徑 (mm)
        self.num_trajectory_points = 3600                # 軌跡點數量
        
        # 繞圓速度設定（兩種方式擇一）
        # 方式 1: 直接設定角速度 (rad/cycle)
        # self.angular_velocity = 0.02
        
        # 方式 2: 根據期望的線速度計算角速度（推薦）
        self.desired_tangential_speed = -1.0  # 期望的切向速度 (mm/s)
        # 計算角速度: ω = v / r，並轉換到每週期，除以 100 是因為控制頻率是 100 Hz
        self.angular_velocity = self.desired_tangential_speed / (self.circle_radius * 100.0)        
        # 預先計算的軌跡
        self.circle_trajectory = []      # 圓形軌跡點列表 [(X, Y), ...]
        self.trajectory_slopes = []      # 每個軌跡點的切線斜率 m2
        self.current_trajectory_index_float = 0.0  # 當前軌跡點索引（浮點數，支援小步長）
        
        # 接近模式的插值參數
        self.approach_mode = "MOVE_TO_CIRCLE"  # "MOVE_TO_CIRCLE" → "ON_CIRCLE"
        self.approach_start_position = None     # 起始位置
        self.approach_target_position = None    # 圓上的目標位置
        self.approach_interpolation_step = 0.05  # 每個週期移動的距離 (mm)
        self.approach_tolerance = 0.05           # 到達圓軌跡的容忍度 (mm)
        
        # 初始化軌跡
        self._initialize_trajectory()

        # PI 控制器參數（輸出為位移，不是速度）
        self.Kp_normal_force = 0.0001     # 比例增益 (mm/N)
        self.Ki_normal_force = 0.0001    # 積分增益 (mm/(N*s))
        self.normal_force_deadband_newton = 0.5
        self.normal_force_integral_limit = 10.0
        self.normal_force_error_integral = 0.0
        self.max_normal_correction_mm = 5.0  # 法向修正的最大位移 (mm)
        
        # 法向和切向資訊
        self.normal_force_magnitude = 0.0
        self.normal_direction_unit = np.array([0.0, 0.0])
        self.tangential_direction_unit = np.array([0.0, 0.0])
        
        # 日誌計數器
        self.log_counter = 0
        
        # === 4. 訂閱者 ===
        self.force_subscription = self.create_subscription(
            WrenchStamped, "/ft_compensated", 
            self.force_torque_callback, 10
        )
        self.feedback_subscription = self.create_subscription(
            FeedbackState, "feedback_states", 
            self.feedback_callback, 10
        )

        self.pub_desired_point = self.create_publisher(
            PointStamped, "/force_control/desired_point", 10
        )
        self.pub_corrected_point = self.create_publisher(
            PointStamped, "/force_control/corrected_point", 10
        )
        self.pub_corrected_data = self.create_publisher(
            Float64MultiArray, "/force_control/corrected_data", 10
        )
        self.pub_target_point = self.create_publisher(
            PointStamped, "/force_control/target_point", 10
        )

        self.pub_control_state = self.create_publisher(
            String, "/force_control/control_state", 10
        )


        # === 5. 啟動 PVT 模式與定時器 ===
        self.send_script_command("PVTEnter(1)")
        self.get_logger().info("Sent PVTEnter(1), starting in APPROACH mode.")
        
        self.pvt_timer = self.create_timer(
            self.pvt_duration_seconds, 
            self.pvt_timer_callback
        )

    def _initialize_trajectory(self) -> None:
        """預先計算圓形軌跡的所有點和切線斜率"""
        self.get_logger().info("Initializing circular trajectory...")
        
        for i in range(self.num_trajectory_points):
            # 計算角度（完整一圈 2π）
            # theta = self.angular_velocity * math.pi * i / self.num_trajectory_points
            theta = 2.0 * math.pi * i / self.num_trajectory_points
            
            # 計算圓周上的點
            x = self.circle_center[0] + self.circle_radius * math.cos(theta)
            y = self.circle_center[1] + self.circle_radius * math.sin(theta)
            self.circle_trajectory.append(np.array([x, y]))
                    
        self.get_logger().info(
            f"Trajectory initialized: {self.num_trajectory_points} points, "
            f"Center=({self.circle_center[0]:.2f}, {self.circle_center[1]:.2f}), "
            f"Radius={self.circle_radius:.2f}mm"
        )
        np.save("circle_trajectory.npy", np.array(self.circle_trajectory))
        # print(self.circle_trajectory)
        
        # 計算並顯示實際速度資訊
        circumference = 2 * math.pi * self.circle_radius  # 圓周長 (mm)
        time_per_revolution = (2 * math.pi) / (self.angular_velocity * 100.0)  # 一圈所需時間 (秒)
        actual_tangential_speed = self.angular_velocity * self.circle_radius * 100.0  # 實際線速度 (mm/s)
        
        self.get_logger().info(
            f"Circle motion parameters: "
            f"Circumference={circumference:.2f}mm, "
            f"Angular velocity={self.angular_velocity:.4f} rad/cycle, "
            f"Tangential speed={actual_tangential_speed:.2f} mm/s, "
            f"Time per revolution={time_per_revolution:.2f} seconds"
        )

    def force_torque_callback(self, msg: WrenchStamped) -> None:
        """力/力矩數據回呼"""
        self.latest_force = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z
        ])

        self.latest_torque = np.array([
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z
        ])

    def feedback_callback(self, msg: FeedbackState) -> None:
        """機器人狀態回呼"""
        meters_to_mm = 1000.0
        rad_to_deg = 180.0 / math.pi
        
        raw = list(msg.tool_pose)
        
        # 單位轉換
        x_mm  = raw[0] * meters_to_mm
        y_mm  = raw[1] * meters_to_mm
        z_mm  = raw[2] * meters_to_mm
        rx_deg = raw[3] * rad_to_deg
        ry_deg = raw[4] * rad_to_deg
        rz_deg = raw[5] * rad_to_deg
        
        self.current_tool_pose = [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
    
        if not self.has_tool_pose:
            # 第一次初始化：目標位置 = 當前位置
            self.target_position = self.current_tool_pose.copy()
            self.reference_position = self.current_tool_pose.copy()
            self.has_tool_pose = True
            
            # 找到最接近當前位置的軌跡點索引
            current_xy = np.array([x_mm, y_mm])
            min_dist = float('inf')
            closest_index = 0
            for i, point in enumerate(self.circle_trajectory):
                dist = np.linalg.norm(point - current_xy)
                if dist < min_dist:
                    min_dist = dist
                    closest_index = i
            
            self.current_trajectory_index_float = float(closest_index)
            
            self.get_logger().info(
                f"Initialized at position: "
                f"X={x_mm:.2f}, Y={y_mm:.2f}, Z={z_mm:.2f}mm | "
                f"Orientation: Rx={rx_deg:.2f}, Ry={ry_deg:.2f}, Rz={rz_deg:.2f}° | "
                f"Starting at trajectory index: {closest_index}"
            )

    def get_next_trajectory_point(self) -> np.ndarray:
        """獲取下一個軌跡點"""
        # 根據角速度更新索引（浮點數累加）
        index_step = self.angular_velocity * self.num_trajectory_points / (2 * math.pi)
        self.current_trajectory_index_float += index_step
        
        # 保持在 [0, num_trajectory_points) 範圍內
        self.current_trajectory_index_float = self.current_trajectory_index_float % self.num_trajectory_points
        
        # 取整數索引用於查表
        index = int(self.current_trajectory_index_float)
        
        return self.circle_trajectory[index]

    def get_current_trajectory_slope(self) -> float:
        """獲取當前軌跡點的切線斜率 m2"""
        index = int(self.current_trajectory_index_float)
        return self.trajectory_slopes[index]

    # def calculate_projection_correction(self, current_pos: np.ndarray, 
    #                                    desired_point: np.ndarray) -> tuple:
    #     """
    #     計算軌跡投影修正
        
    #     參數:
    #         current_pos: 當前位置 (X, Y)
    #         desired_point: 期望軌跡點 B (X2, Y2)
        
    #     返回:
    #         (corrected_point, delta_L1, delta_L2, theta): 修正後的點 C 和相關參數
    #     """
    #     # 當前位置 A (X1, Y1)
    #     X1, Y1 = current_pos[0], current_pos[1]
        
    #     # 期望點 B (X2, Y2)
    #     X2, Y2 = desired_point[0], desired_point[1]
        
    #     # 計算 AB 的斜率 m1
    #     delta_x = X2 - X1
    #     delta_y = Y2 - Y1
        
    #     if abs(delta_x) < 1e-6:
    #         m1 = float('inf') if delta_y > 0 else float('-inf')
    #     else:
    #         m1 = delta_y / delta_x
        
    #     # 計算 AB 的長度 ΔL1
    #     delta_L1 = math.sqrt(delta_x**2 + delta_y**2)
        
    #     # 獲取障礙物邊界的斜率 m2（切線方向）
    #     # m2 = self.get_current_trajectory_slope()
    #     # 從切向單位向量計算斜率：m2 = dy/dx
    #     tangent_vec = self.tangential_direction_unit
    #     if abs(tangent_vec[0]) < 1e-6:
    #         # 切線接近垂直
    #         m2 = float('inf') if tangent_vec[1] > 0 else float('-inf')
    #     else:
    #         m2 = tangent_vec[1] / tangent_vec[0]
        
    #     # 計算夾角 θ
    #     # tan(θ) = |m1 - m2| / |1 + m1*m2|
    #     if math.isinf(m1) or math.isinf(m2):
    #         # 處理垂直線的情況
    #         if math.isinf(m1) and math.isinf(m2):
    #             theta = 0.0  # 兩條線平行
    #         elif math.isinf(m1):
    #             theta = abs(math.atan(1.0 / m2))
    #         else:  # math.isinf(m2)
    #             theta = abs(math.atan(m1))
    #     else:
    #         denominator = 1.0 + m1 * m2
    #         if abs(denominator) < 1e-6:
    #             theta = math.pi / 2.0  # 垂直
    #         else:
    #             numerator = m1 - m2
    #             theta = abs(math.atan(numerator / denominator))
        
    #     # 計算投影長度 ΔL2 = ΔL1 * cos(θ)
    #     delta_L2 = delta_L1 * math.cos(theta)
        
    #     # 計算修正點 C (X3, Y3)
    #     # C 是沿著 L2（切線方向）從 A 移動 ΔL2 距離
    #     if math.isinf(m2):
    #         # 切線垂直
    #         X3 = X1
    #         Y3 = Y1 + delta_L2 if Y2 > Y1 else Y1 - delta_L2
    #     else:
    #         # 切線方向單位向量
    #         # direction_length = math.sqrt(1 + m2**2)
    #         # direction_x = 1.0 / direction_length
    #         # direction_y = m2 / direction_length

    #         # 使用切向單位向量（更準確）
    #         direction_x = tangent_vec[0]
    #         direction_y = tangent_vec[1]
    #         # 確保方向與 AB 方向一致
    #         # if delta_x * direction_x + delta_y * direction_y < 0:
    #         #     direction_x = -direction_x
    #         #     direction_y = -direction_y
            
    #         X3 = X1 + delta_L2 * direction_x
    #         Y3 = Y1 + delta_L2 * direction_y
        
    #     corrected_point = np.array([X3, Y3])
        
    #     return corrected_point, delta_L1, delta_L2, theta

    def calculate_projection_correction(
        self,
        current_position_xy: np.ndarray,
        desired_point_xy: np.ndarray,
    ) -> tuple[np.ndarray, float, float, float]:
        """
        A = current_position_xy
        B = desired_point_xy
        t_hat = tangential_direction_unit (unit vector)
        C = A + proj_t(AB)
        """
        point_A_x = float(current_position_xy[0])
        point_A_y = float(current_position_xy[1])
        point_B_x = float(desired_point_xy[0])
        point_B_y = float(desired_point_xy[1])

        vector_AB = np.array([point_B_x - point_A_x, point_B_y - point_A_y], dtype=float)
        delta_L1 = float(np.linalg.norm(vector_AB))

        tangential_unit = self.tangential_direction_unit.astype(float)
        tangential_norm = float(np.linalg.norm(tangential_unit))
        if tangential_norm < 1e-9 or delta_L1 < 1e-9:
            corrected_point_C = np.array([point_A_x, point_A_y], dtype=float)
            delta_L2 = 0.0
            theta_rad = 0.0
            return corrected_point_C, delta_L1, delta_L2, theta_rad

        tangential_unit = tangential_unit / tangential_norm

        # signed projection length onto tangential direction
        delta_L2 = float(np.dot(vector_AB, tangential_unit))

        corrected_point_C = np.array([point_A_x, point_A_y], dtype=float) + delta_L2 * tangential_unit

        # optional: compute theta for logging only (0..pi)
        cos_theta = float(np.clip(delta_L2 / delta_L1, -1.0, 1.0))
        theta_rad = float(np.arccos(cos_theta))

        return corrected_point_C, delta_L1, delta_L2, theta_rad

    # def calculate_projection_correction(self, current_pos, next_point, F_contact) -> tuple:
        
    #     # 當前位置 A (X1, Y1), 期望點 B (X2, Y2)
    #     X1, Y1 = current_pos[0], current_pos[1]
    #     X2, Y2 = next_point[0], next_point[1]
        
    #     # 計算 AB 向量與斜率 m1
    #     delta_x, delta_y = X2 - X1, Y2 - Y1
        
    #     m1 = delta_y / delta_x if abs(delta_x) > 1e-6 else float('inf')
    #     delta_L1 = math.sqrt(delta_x**2 + delta_y**2)
        
    #     # m2 由力方向 F_contact 決定
    #     phi = np.arctan2(F_contact[1], F_contact[0])
    #     print(f"Force angle phi (radians): {phi}, (degrees): {np.degrees(phi)}")
    #     # 1. 先定出一個標準切線方向 (例如逆時針 90 度)
    #     tangent_phi = phi + np.pi / 2
    #     tangent_vec = np.array([math.cos(tangent_phi), math.sin(tangent_phi)])
    #     if np.array([delta_x, delta_y]) @ tangent_vec > 0:
    #         tangent_vec = -tangent_vec 
            
    #     m2 = tangent_vec[1] / tangent_vec[0] if abs(tangent_vec[0]) > 1e-6 else float('inf')
        
    #     # 計算夾角 (AB 與 F 之間的夾角)
    #     denominator = np.sqrt(1 + m1**2) * np.sqrt(1 + m2**2)
    #     numerator = m1 - m2
    #     theta = np.arcsin(np.clip(numerator / denominator, -1.0, 1.0))
        
    #     # 計算投影長度 L2
    #     delta_L2 = delta_L1 * math.cos(theta)
        
    #     # 修正點 C: 沿著接觸面移動 
    #     X3 = X1 + delta_L2 * tangent_vec[0]
    #     Y3 = Y1 + delta_L2 * tangent_vec[1]
        
    #     corrected_point = np.array([X3, Y3])
    #     return corrected_point, delta_L1, delta_L2, theta, tangent_vec, m1, m2

    def pvt_timer_callback(self) -> None:
        """主控制迴圈 - 使用 PVT 但速度設為 0"""
        if not self.has_tool_pose:
            return
        
        # === 座標轉換：Tool frame → Base frame ===
        rx_deg, ry_deg, rz_deg = self.current_tool_pose[3:6]
        rotation_tool_to_base = Rotation.from_euler(
            'zyx',
            [rz_deg, ry_deg, rx_deg],
            degrees=True
        ).as_matrix()
        
        force_in_base = rotation_tool_to_base @ self.latest_force
        self.force_x_base = force_in_base[0]
        self.force_y_base = force_in_base[1]
        self.force_z_base = force_in_base[2]
        
        # 計算 XY 平面的法向力
        force_xy_vector = np.array([self.force_x_base, self.force_y_base])
        self.normal_force_magnitude = np.linalg.norm(force_xy_vector)

        # 計算法向和切向方向
        if self.normal_force_magnitude < self.min_normal_force_eps:
            self.normal_direction_unit = np.array([0.0, 0.0])
            self.tangential_direction_unit = np.array([0.0, 0.0])
        else:
            # 法向單位向量（指向推力方向）
            self.normal_direction_unit = force_xy_vector / self.normal_force_magnitude
            
            # 切向單位向量（逆時針旋轉 90 度）
            self.tangential_direction_unit = np.array([
                -self.normal_direction_unit[1],
                self.normal_direction_unit[0]
            ])
        
        # === 安全檢查 ===
        if self.normal_force_magnitude > self.max_force_limit:
            self.get_logger().error(
                f"Force exceeded limit: {self.normal_force_magnitude:.2f}N > {self.max_force_limit}N"
            )
            self.send_script_command("PVTExit()")
            self.pvt_timer.cancel()
            return

        if self.current_tool_pose[2] < self.min_z_position:
            self.get_logger().error(
                f"Z position limit reached: {self.current_tool_pose[2]:.2f}mm < {self.min_z_position}mm"
            )
            self.send_script_command("PVTExit()")
            self.pvt_timer.cancel()
            return
        
        dt = self.pvt_duration_seconds
        
        # === 狀態機控制邏輯 ===
        if self.control_mode == "APPROACH":
            # control state
            self.pub_control_state.publish(String(data=self.control_mode))
            # 接近模式：分為兩個子狀態
            if self.approach_mode == "MOVE_TO_CIRCLE":
                # 子狀態1: 從當前位置移動到圓形軌跡
                
                # 第一次進入此狀態，設定起始和目標位置
                if self.approach_start_position is None:
                    self.approach_start_position = np.array([
                        self.current_tool_pose[0], 
                        self.current_tool_pose[1]
                    ])
                    
                    # 找到圓上最近的點作為目標（使用整數索引）
                    index = int(self.current_trajectory_index_float)
                    self.approach_target_position = self.circle_trajectory[index]
                    
                    distance = np.linalg.norm(self.approach_target_position - self.approach_start_position)
                    
                    self.get_logger().info(
                        f"Starting MOVE_TO_CIRCLE: "
                        f"From ({self.approach_start_position[0]:.2f}, {self.approach_start_position[1]:.2f}) "
                        f"to ({self.approach_target_position[0]:.2f}, {self.approach_target_position[1]:.2f}), "
                        f"Distance: {distance:.2f}mm"
                    )
                
                # 計算當前位置到目標的向量
                current_xy = np.array([self.current_tool_pose[0], self.current_tool_pose[1]])
                to_target = self.approach_target_position - current_xy
                distance_to_target = np.linalg.norm(to_target)
                
                # 檢查是否已經到達圓形軌跡
                if distance_to_target < self.approach_tolerance:
                    self.approach_mode = "ON_CIRCLE"
                    self.get_logger().info(
                        f"Reached circle trajectory! Switching to ON_CIRCLE mode. "
                        f"Distance to target: {distance_to_target:.2f}mm"
                    )
                else:
                    # 計算插值移動
                    if distance_to_target > self.approach_interpolation_step:
                        # 沿著方向移動固定步長
                        direction = to_target / distance_to_target
                        step_displacement = direction * self.approach_interpolation_step
                    else:
                        # 直接到達目標
                        step_displacement = to_target
                    
                    # 更新目標位置
                    self.target_position[0] += step_displacement[0]
                    self.target_position[1] += step_displacement[1]
                
                # 日誌（降低頻率）
                self.log_counter += 1
                if self.log_counter % 10 == 0:
                    self.get_logger().info(
                        f"APPROACH [MOVE_TO_CIRCLE] | "
                        f"Current: X={self.current_tool_pose[0]:.2f}, Y={self.current_tool_pose[1]:.2f}mm | "
                        f"Target: X={self.target_position[0]:.2f}, Y={self.target_position[1]:.2f}mm | "
                        f"Distance: {distance_to_target:.2f}mm | F={self.normal_force_magnitude:.2f}N"
                    )
            
            elif self.approach_mode == "ON_CIRCLE":
                # 子狀態2: 沿著圓形軌跡移動
                desired_point_xy = self.get_next_trajectory_point()
                # 發布期望點
                now = self.get_clock().now().to_msg()
                msg_desired = PointStamped()
                msg_desired.header.stamp = now
                msg_desired.header.frame_id = "base"
                msg_desired.point.x = float(desired_point_xy[0])
                msg_desired.point.y = float(desired_point_xy[1])
                msg_desired.point.z = 0.0
                self.pub_desired_point.publish(msg_desired)
                # 更新目標位置的 X, Y（圓形軌跡）
                # self.target_position[0] = desired_point_xy[0]
                # self.target_position[1] = desired_point_xy[1]
                # 更新目標位置的 X, Y（圓形軌跡）with feedback
                desired_x = float(desired_point_xy[0])
                desired_y = float(desired_point_xy[1])

                actual_x = float(self.current_tool_pose[0])
                actual_y = float(self.current_tool_pose[1])

                position_error_x = desired_x - actual_x
                position_error_y = desired_y - actual_y
                outer_loop_kp = 0.2  # (無單位) 建議從 0.05~0.3 試
                max_outer_loop_correction_mm = 1.0

                outer_correction_x = float(np.clip(outer_loop_kp * position_error_x,
                                                -max_outer_loop_correction_mm,
                                                +max_outer_loop_correction_mm))
                outer_correction_y = float(np.clip(outer_loop_kp * position_error_y,
                                                -max_outer_loop_correction_mm,
                                                +max_outer_loop_correction_mm))

                self.target_position[0] = desired_x + outer_correction_x
                self.target_position[1] = desired_y + outer_correction_y

                
                # 日誌（降低頻率）
                self.log_counter += 1
                if self.log_counter % 10 == 0:
                    self.get_logger().info(
                        f"APPROACH [ON_CIRCLE] | Index={self.current_trajectory_index_float:.1f} | "
                        f"X={self.current_tool_pose[0]:.2f}, Y={self.current_tool_pose[1]:.2f}mm | "
                        f"Target X={self.target_position[0]:.2f}, Y={self.target_position[1]:.2f}mm | "
                        f"F={self.normal_force_magnitude:.2f}N"
                    )
            
            # Z 和姿態保持在參考位置（兩個子狀態共用）
            self.target_position[2] = self.reference_position[2]
            self.target_position[3:6] = self.reference_position[3:6]
            
            # 檢測接觸（兩個子狀態都檢測）
            if self.normal_force_magnitude > self.normal_force_threshold:
                self.control_mode = "FORCE_CONTROL"
                self.reference_position = self.current_tool_pose.copy()
                self.target_position = self.current_tool_pose.copy()
                self.normal_force_error_integral = 0.0
                
                self.get_logger().info(
                    f"Contact detected! F={self.normal_force_magnitude:.2f}N | "
                    f"Position: X={self.current_tool_pose[0]:.2f}, Y={self.current_tool_pose[1]:.2f}mm | "
                    f"Switching to FORCE_CONTROL mode"
                )
        
        elif self.control_mode == "FORCE_CONTROL":
            # 力控制模式：使用投影修正方法
            
            # 1. 獲取下一個期望軌跡點 B
            desired_point_B = self.get_next_trajectory_point()
            
            # 2. 當前位置 A
            current_pos_A = np.array([self.current_tool_pose[0], self.current_tool_pose[1]])
            
            # 3. 計算投影修正點 C
            corrected_point_C, delta_L1, delta_L2, theta = self.calculate_projection_correction(
                current_pos_A, desired_point_B
            )
            corrected_data = [delta_L1, delta_L2, math.degrees(theta)]
            # 發布期望點
            now = self.get_clock().now().to_msg()
            msg_desired = PointStamped()
            msg_desired.header.stamp = now
            msg_desired.header.frame_id = "base"
            msg_desired.point.x = float(desired_point_B[0])
            msg_desired.point.y = float(desired_point_B[1])
            msg_desired.point.z = 0.0
            self.pub_desired_point.publish(msg_desired)
            # 發布修正點
            now = self.get_clock().now().to_msg()
            msg_corrected = PointStamped()
            msg_corrected.header.stamp = now
            msg_corrected.header.frame_id = "base"
            msg_corrected.point.x = float(corrected_point_C[0])
            msg_corrected.point.y = float(corrected_point_C[1])
            msg_corrected.point.z = 0.0
            self.pub_corrected_point.publish(msg_corrected)
            # 發布修正點(delta_L1,L2,theta)
            now = self.get_clock().now().to_msg()
            msg_corrected_data = Float64MultiArray()
            # msg_corrected_data.header.stamp = now
            # msg_corrected_data.header.frame_id = "base"
            msg_corrected_data.data = corrected_data
            self.pub_corrected_data.publish(msg_corrected_data)
            
            # 4. 力控制計算 (PI 控制器)
            normal_force_error = self.desired_normal_force_newton - self.normal_force_magnitude
            
            # PI 控制器（輸出為位移修正量）
            if abs(normal_force_error) > self.normal_force_deadband_newton:
                # 比例項
                proportional_correction = self.Kp_normal_force * normal_force_error
                
                # 積分項
                self.normal_force_error_integral += normal_force_error * dt
                self.normal_force_error_integral = float(np.clip(
                    self.normal_force_error_integral,
                    -self.normal_force_integral_limit,
                    self.normal_force_integral_limit
                ))
                integral_correction = self.Ki_normal_force * self.normal_force_error_integral
                
                normal_correction_mm = proportional_correction + integral_correction
            else:
                # 在死區內
                normal_correction_mm = 0.0
                self.normal_force_error_integral *= 0.95  # 積分衰減
            
            # 限制法向修正量
            normal_correction_mm = float(np.clip(
                normal_correction_mm,
                -self.max_normal_correction_mm,
                self.max_normal_correction_mm
            ))
            
            # 5. 法向位移修正（垂直於軌跡方向）
            # 使用力的方向作為法向
            normal_displacement_xy = -normal_correction_mm * self.normal_direction_unit
            
            # 6. 最終目標位置 = 修正點 C + 法向修正
            self.target_position[0] = corrected_point_C[0] + normal_displacement_xy[0]
            self.target_position[1] = corrected_point_C[1] + normal_displacement_xy[1]
            
            # Z 與姿態保持在參考位置
            self.target_position[2] = self.reference_position[2]
            self.target_position[3:6] = self.reference_position[3:6]

            # target (實際送 PVT)
            msg_target = PointStamped()
            msg_target.header.stamp = now
            msg_target.header.frame_id = "base"
            msg_target.point.x = float(self.target_position[0])
            msg_target.point.y = float(self.target_position[1])
            msg_target.point.z = float(self.target_position[2])
            self.pub_target_point.publish(msg_target)
            # control state
            self.pub_control_state.publish(String(data=self.control_mode))
            
            # 日誌
            self.log_counter += 1
            if self.log_counter % 10 == 0:
                self.get_logger().info(
                    f"FORCE_CONTROL | "
                    f"F={self.normal_force_magnitude:.2f}N (target={self.desired_normal_force_newton:.2f}) | "
                    f"Error={normal_force_error:.2f} | "
                    f"Normal_Corr={normal_correction_mm:.3f}mm | "
                    f"ΔL1={delta_L1:.2f}, ΔL2={delta_L2:.2f}, θ={math.degrees(theta):.1f}° | "
                    f"Pos: X={self.current_tool_pose[0]:.2f}, Y={self.current_tool_pose[1]:.2f} | "
                    f"Target: X={self.target_position[0]:.2f}, Y={self.target_position[1]:.2f}"
                )

        # === 產生 PVTPoint 指令（速度設為 0）===
        position_string = "{" + ", ".join(f"{v:.4f}" for v in self.target_position) + "}"
        
        # 關鍵：速度全部設為 0
        velocity_command = [0.0] * 6
        velocity_string = "{" + ", ".join(f"{v:.4f}" for v in velocity_command) + "}"
        
        script_command = f"PVTPoint({position_string}, {velocity_string}, {dt})"
        
        # 送給 TM Robot
        self.send_script_command(script_command)

    def send_script_command(self, script_text: str) -> None:
        """發送 TMScript 指令"""
        script_request = SendScript.Request()
        script_request.id = str(int(time.time() * 1000) % 100000)
        script_request.script = script_text
        self.send_script_client.call_async(script_request)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HybridPvtForceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.send_script_command("PVTExit()")
            node.get_logger().info("Sent PVTExit().")
        
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

