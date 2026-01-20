#!/usr/bin/env python3
"""
Hybrid Position/Force Control - Continuous Contour Following
連續輪廓跟隨控制（類似 Simulink 模型的連續軌跡）

主要改進：
1. 移除離散點到點邏輯
2. 實現連續軌跡生成
3. 使用連續的位置/速度控制
4. 類似 C 程式的連續 PID 控制
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, PointStamped
from std_msgs.msg import String, Float64MultiArray
from tm_msgs.msg import FeedbackState
from tm_msgs.srv import SendScript
import numpy as np


class HybridContourFollowingNode(Node):
    """
    混合位置/力控制 - 連續輪廓跟隨
    """

    def __init__(self):
        super().__init__("hybrid_contour_following_node")

        # === 1. 通訊設置 ===
        self.send_script_client = self.create_client(SendScript, "send_script")
        self.get_logger().info("Waiting for /send_script service...")
        self.send_script_client.wait_for_service()
        self.get_logger().info("/send_script service is ready.")

        # === 2. 核心控制參數 ===
        self.control_frequency = 100.0  # 100 Hz 控制頻率（類似 C 程式的 100Hz）
        self.dt = 1.0 / self.control_frequency
        self.pvt_duration_seconds = self.dt

        self.has_tool_pose = False
        self.current_tool_pose = [0.0] * 6       # 當前位置
        self.target_position = [0.0] * 6         # 目標位置
        self.target_velocity = [0.0] * 6         # 目標速度（關鍵：不再設為 0）
        self.reference_position = [0.0] * 6      # 參考位置
        
        # === 3. 狀態機 ===
        self.control_mode = "APPROACH"  # "APPROACH" → "FORCE_CONTROL" → "STOPPED"
        self.latest_force = np.array([0.0, 0.0, 0.0])
        self.latest_torque = np.array([0.0, 0.0, 0.0])
        self.min_z_position = 45.0
        
        # === 4. 力控制參數 ===
        self.desired_normal_force_newton = 3.50
        self.force_deadband_newton = 0.5
        self.max_force_limit = 15.0
        self.normal_force_threshold = 2.50
        
        
        # === 5. 連續軌跡參數 ===
        self.trajectory_type = "line"  # "line" 或 "circle"
        self.trajectory_velocity = 0.10  # 軌跡速度 (mm/s)
        self.trajectory_start_position = None
        self.trajectory_time = 0.0  # 軌跡時間參數
        
        # 直線軌跡參數（沿 y=x 方向）
        self.line_direction = np.array([1.0, 1.0]) / np.sqrt(2.0)  # 45° 方向單位向量
        self.line_length = 50.0  # 軌跡長度 (mm)
        
        # 圓形軌跡參數
        self.circle_radius = 10.0  # 半徑 (mm)
        self.circle_center = None
        self.circle_angular_velocity = 0.1  # rad/s
        
        # === 6. PID 控制器參數 ===
        # 位置控制器
        self.Kp_position = np.array([0.01, 0.01])  # 比例增益
        self.Ki_position = np.array([0.01, 0.01])  # 積分增益
        self.Kd_position = np.array([0.0, 0.0])  # 微分增益
        
        self.position_error_integral = np.array([0.0, 0.0])
        self.position_error_prev = np.array([0.0, 0.0])
        self.position_integral_limit = 10.0
        
        # 力控制器（PID，類似 C 程式）
        self.Kp_force = 0.03  # 比例增益 (mm/N)
        self.Ki_force = 0.005  # 積分增益 (mm/(N*s))
        self.Kd_force = 0.0005  # 微分增益 (mm/(N/s))
        
        self.force_error_integral = 0.0
        self.force_error_prev = 0.0
        self.force_integral_limit = 10.0
        self.max_force_correction = 2.0  # 最大力修正量 (mm)
        
        # === 7. 法向和切向資訊 ===
        self.normal_force_magnitude = 0.0
        self.normal_direction_unit = np.array([0.0, 0.0])
        self.tangential_direction_unit = np.array([0.0, 0.0])
        
        # === 8. 日誌 ===
        self.log_counter = 0
        
        # === 9. 訂閱者與發布者 ===
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
        self.pub_control_data = self.create_publisher(
            Float64MultiArray, "/force_control/control_data", 10
        )

        # === 10. 啟動控制 ===
        self.send_script_command("PVTEnter(1)")
        self.get_logger().info("Sent PVTEnter(1), starting APPROACH mode.")
        
        self.control_timer = self.create_timer(
            self.pvt_duration_seconds, 
            self.control_loop
        )

    def force_torque_callback(self, msg: WrenchStamped) -> None:
        """力/力矩回呼"""
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
        
        x_mm  = raw[0] * meters_to_mm
        y_mm  = raw[1] * meters_to_mm
        z_mm  = raw[2] * meters_to_mm
        rx_deg = raw[3] * rad_to_deg
        ry_deg = raw[4] * rad_to_deg
        rz_deg = raw[5] * rad_to_deg
        
        self.current_tool_pose = [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
        
        if not self.has_tool_pose:
            self.has_tool_pose = True
            self.reference_position = self.current_tool_pose.copy()
            self.target_position = self.current_tool_pose.copy()
            self.get_logger().info(
                f"Initial position: X={x_mm:.2f}, Y={y_mm:.2f}, Z={z_mm:.2f}mm"
            )

    def generate_trajectory_point(self) -> tuple:
        """
        生成連續軌跡點（類似 C 程式的在線回歸）
        返回: (期望位置 [x, y], 期望速度 [vx, vy])
        """
        if self.trajectory_type == "line":
            # 直線軌跡：沿 y=x 方向
            distance = self.trajectory_velocity * self.trajectory_time
            
            # 限制軌跡長度
            if distance > self.line_length:
                distance = self.line_length
                
            # 計算期望位置
            desired_position = (
                self.trajectory_start_position + 
                self.line_direction * distance
            )
            
            # 期望速度（沿軌跡方向）
            if distance < self.line_length:
                desired_velocity = self.line_direction * self.trajectory_velocity
            else:
                desired_velocity = np.array([0.0, 0.0])  # 到達終點停止
            
            return desired_position, desired_velocity
            
        elif self.trajectory_type == "circle":
            # 圓形軌跡
            angle = self.circle_angular_velocity * self.trajectory_time
            
            # 期望位置
            desired_position = self.circle_center + np.array([
                self.circle_radius * np.cos(angle),
                self.circle_radius * np.sin(angle)
            ])
            
            # 期望速度（切向）
            desired_velocity = self.circle_radius * self.circle_angular_velocity * np.array([
                -np.sin(angle),
                np.cos(angle)
            ])
            
            return desired_position, desired_velocity
            
        else:
            return self.trajectory_start_position, np.array([0.0, 0.0])

    def position_pid_controller(self, position_error: np.ndarray) -> np.ndarray:
        """
        位置 PID 控制器（類似 C 程式的位置控制器）
        輸入: 位置誤差 (mm)
        輸出: 位置修正量 (mm)
        """
        # 比例項
        p_term = self.Kp_position * position_error
        
        # 積分項
        self.position_error_integral += position_error * self.dt
        self.position_error_integral = np.clip(
            self.position_error_integral,
            -self.position_integral_limit,
            self.position_integral_limit
        )
        i_term = self.Ki_position * self.position_error_integral
        
        # 微分項
        d_error = (position_error - self.position_error_prev) / self.dt
        d_term = self.Kd_position * d_error
        
        # 更新前一次誤差
        self.position_error_prev = position_error.copy()
        
        # PID 輸出
        correction = p_term + i_term + d_term
        
        return correction

    def force_pid_controller(self, force_error: float) -> float:
        """
        力 PID 控制器（類似 C 程式的力控制器）
        輸入: 力誤差 (N)
        輸出: 位置修正量 (mm)
        """
        # 比例項
        p_term = self.Kp_force * force_error
        
        # 積分項
        self.force_error_integral += force_error * self.dt
        self.force_error_integral = float(np.clip(
            self.force_error_integral,
            -self.force_integral_limit,
            self.force_integral_limit
        ))
        i_term = self.Ki_force * self.force_error_integral
        
        # 微分項
        d_error = (force_error - self.force_error_prev) / self.dt
        d_term = self.Kd_force * d_error
        
        # 更新前一次誤差
        self.force_error_prev = force_error
        
        # PID 輸出
        correction = p_term + i_term + d_term
        
        # 限制修正量
        correction = float(np.clip(
            correction,
            -self.max_force_correction,
            self.max_force_correction
        ))
        
        return correction
    
    def calculate_projection_correction(
        self,
        current_position_xy: np.ndarray,
        desired_point_xy: np.ndarray,
    ) -> tuple[np.ndarray, float, float, float]:
        """
        投影修正：將期望點投影到切向方向上
        
        幾何關係：
        A = current_position_xy（當前位置）
        B = desired_point_xy（期望位置）
        t_hat = tangential_direction_unit（切向單位向量）
        C = A + proj_t(AB)（投影後的修正點）
        
        返回：
        - corrected_point_C: 投影修正後的點
        - delta_L1: AB 的距離
        - delta_L2: AB 在切向上的投影長度（有符號）
        - theta_rad: AB 與切向的夾角
        """
        point_A_x = float(current_position_xy[0])
        point_A_y = float(current_position_xy[1])
        point_B_x = float(desired_point_xy[0])
        point_B_y = float(desired_point_xy[1])

        # 向量 AB
        vector_AB = np.array([point_B_x - point_A_x, point_B_y - point_A_y], dtype=float)
        delta_L1 = float(np.linalg.norm(vector_AB))

        # 切向單位向量
        tangential_unit = self.tangential_direction_unit.astype(float)
        tangential_norm = float(np.linalg.norm(tangential_unit))
        
        # 邊界情況：切向向量或 AB 太小
        if tangential_norm < 1e-9 or delta_L1 < 1e-9:
            corrected_point_C = np.array([point_A_x, point_A_y], dtype=float)
            delta_L2 = 0.0
            theta_rad = 0.0
            return corrected_point_C, delta_L1, delta_L2, theta_rad

        # 歸一化切向向量
        tangential_unit = tangential_unit / tangential_norm

        # AB 在切向上的有符號投影長度
        delta_L2 = float(np.dot(vector_AB, tangential_unit))

        # 修正點 C = A + delta_L2 * t_hat
        corrected_point_C = np.array([point_A_x, point_A_y], dtype=float) + delta_L2 * tangential_unit

        # 計算夾角（用於日誌記錄）
        cos_theta = float(np.clip(delta_L2 / delta_L1 if delta_L1 > 1e-9 else 0.0, -1.0, 1.0))
        theta_rad = float(np.arccos(cos_theta))

        return corrected_point_C, delta_L1, delta_L2, theta_rad

    def control_loop(self) -> None:
        """主控制迴圈（類似 C 程式的 step1 函數）"""
        if not self.has_tool_pose:
            return
        
        t0 = time.time()
        
        # === 計算法向力 ===
        force_xy = np.array([self.latest_force[0], self.latest_force[1]])
        self.normal_force_magnitude = np.linalg.norm(force_xy)
        
        if self.normal_force_magnitude > 1e-6:
            self.normal_direction_unit = force_xy / self.normal_force_magnitude
            # 切向方向（垂直於法向）
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
            self.control_timer.cancel()
            return

        if self.current_tool_pose[2] < self.min_z_position:
            self.get_logger().error(
                f"Z position limit reached: {self.current_tool_pose[2]:.2f}mm < {self.min_z_position}mm"
            )
            self.send_script_command("PVTExit()")
            self.control_timer.cancel()
            return
        
        # === 狀態機 ===
        if self.control_mode == "APPROACH":
            # 接近模式：沿 45° 方向接近工件
            approach_velocity = 1.0  # mm/s
            approach_direction = np.array([1.0, 1.0]) / np.sqrt(2.0)
            
            # 更新目標位置
            self.target_position[0] += approach_direction[0] * approach_velocity * self.dt
            self.target_position[1] += approach_direction[1] * approach_velocity * self.dt
            self.target_position[2] = self.reference_position[2]
            self.target_position[3:6] = self.reference_position[3:6]
            
            # 目標速度（用於 PVT）
            self.target_velocity[0] = approach_direction[0] * approach_velocity
            self.target_velocity[1] = approach_direction[1] * approach_velocity
            self.target_velocity[2:6] = [0.0] * 4
            
            # 檢查接觸
            if self.normal_force_magnitude > self.normal_force_threshold:
                self.control_mode = "FORCE_CONTROL"
                
                # 初始化軌跡
                self.trajectory_start_position = np.array([
                    self.current_tool_pose[0],
                    self.current_tool_pose[1]
                ])
                self.trajectory_time = 0.0
                
                # 設置圓心（如果是圓形軌跡）
                self.circle_center = self.trajectory_start_position.copy()
                
                self.get_logger().info(
                    f"Contact detected! F={self.normal_force_magnitude:.2f}N | "
                    f"Switching to FORCE_CONTROL mode | "
                    f"Start: X={self.trajectory_start_position[0]:.2f}, "
                    f"Y={self.trajectory_start_position[1]:.2f}mm"
                )
        
        elif self.control_mode == "FORCE_CONTROL":
            # === 連續輪廓跟隨模式 ===
            
            # 1. 生成軌跡參考點和速度
            desired_position, desired_velocity = self.generate_trajectory_point()
            
            # 當前位置（XY 平面）
            current_position = np.array([
                self.current_tool_pose[0],
                self.current_tool_pose[1]
            ])
            
            # 2. 投影修正：將期望點投影到切向上
            corrected_desired_position, delta_L1, delta_L2, theta = \
                self.calculate_projection_correction(current_position, desired_position)
            corrected_data = [delta_L1, delta_L2, math.degrees(theta)]

            # 3. 位置誤差（使用投影修正後的點）
            position_error = corrected_desired_position - current_position
            
            # 4. 位置 PID 控制器
            position_correction = self.position_pid_controller(position_error)
            
            # 5. 力 PID 控制器
            force_error = self.desired_normal_force_newton - self.normal_force_magnitude
            force_correction_scalar = self.force_pid_controller(force_error)
            
            # 法向修正（逆著力的方向）
            force_correction = -force_correction_scalar * self.normal_direction_unit
            
            # 6. 混合控制：位置修正 + 力修正
            total_correction = position_correction + force_correction
            
            # 6. 更新目標位置
            self.target_position[0] = current_position[0] + total_correction[0]
            self.target_position[1] = current_position[1] + total_correction[1]
            self.target_position[2] = self.reference_position[2]
            self.target_position[3:6] = self.reference_position[3:6]
            
            # 7. 更新目標速度（前饋 + 反饋）
            self.target_velocity[0] = desired_velocity[0] + total_correction[0] / self.dt
            self.target_velocity[1] = desired_velocity[1] + total_correction[1] / self.dt
            self.target_velocity[2:6] = [0.0] * 4
            
            # 8. 更新軌跡時間
            self.trajectory_time += self.dt
            
            # 9. 發布數據
            now = self.get_clock().now().to_msg()
            
            # 期望點
            msg_desired = PointStamped()
            msg_desired.header.stamp = now
            msg_desired.header.frame_id = "base"
            msg_desired.point.x = float(desired_position[0])
            msg_desired.point.y = float(desired_position[1])
            msg_desired.point.z = 0.0
            self.pub_desired_point.publish(msg_desired)

            # 修正點
            msg_corrected = PointStamped()
            msg_corrected.header.stamp = now
            msg_corrected.header.frame_id = "base"
            msg_corrected.point.x = float(corrected_desired_position[0])
            msg_corrected.point.y = float(corrected_desired_position[1])
            msg_corrected.point.z = 0.0
            self.pub_corrected_point.publish(msg_corrected)
            
            # 發布修正數據(delta_L1, L2, theta)
            msg_corrected_data = Float64MultiArray()
            msg_corrected_data.data = corrected_data
            self.pub_corrected_data.publish(msg_corrected_data)
            
            # 目標點
            msg_target = PointStamped()
            msg_target.header.stamp = now
            msg_target.header.frame_id = "base"
            msg_target.point.x = float(self.target_position[0])
            msg_target.point.y = float(self.target_position[1])
            msg_target.point.z = float(self.target_position[2])
            self.pub_target_point.publish(msg_target)
            
            # 控制數據
            control_data = Float64MultiArray()
            control_data.data = [
                float(force_error),
                float(force_correction_scalar),
                float(self.normal_force_magnitude),
                float(np.linalg.norm(position_error)),
                float(self.trajectory_time)
            ]
            self.pub_control_data.publish(control_data)
            
            # 日誌
            self.log_counter += 1
            if self.log_counter % 20 == 0:
                self.get_logger().info(
                    f"FORCE_CONTROL | t={self.trajectory_time:.2f}s | "
                    f"F={self.normal_force_magnitude:.2f}N (err={force_error:.2f}) | "
                    f"Pos_err={np.linalg.norm(position_error):.2f}mm | "
                    f"F_corr={force_correction_scalar:.3f}mm | "
                    f"Proj: ΔL1={delta_L1:.2f}, ΔL2={delta_L2:.2f}, θ={np.degrees(theta):.1f}° | "
                    f"Vel=[{self.target_velocity[0]:.2f}, {self.target_velocity[1]:.2f}]"
                )
            
            # 檢查軌跡結束
            if self.trajectory_type == "line" and self.trajectory_time * self.trajectory_velocity > self.line_length:
                self.control_mode = "STOPPED"
                self.get_logger().info("Trajectory completed! Switching to STOPPED mode.")
        
        elif self.control_mode == "STOPPED":
            # 停止模式：保持當前位置
            self.target_position = self.current_tool_pose.copy()
            self.target_velocity = [0.0] * 6
        
        # 發布狀態
        self.pub_control_state.publish(String(data=self.control_mode))
        
        # 計算實際執行時間
        t1 = time.time()
        actual_dt = t1 - t0
        
        # === 發送 PVT 指令 ===
        position_string = "{" + ", ".join(f"{v:.4f}" for v in self.target_position) + "}"
        velocity_string = "{" + ", ".join(f"{v:.4f}" for v in self.target_velocity) + "}"
        
        script_command = f"PVTPoint({position_string}, {velocity_string}, {actual_dt})"
        self.send_script_command(script_command)

    def send_script_command(self, script_text: str) -> None:
        """發送 TMScript 指令"""
        script_request = SendScript.Request()
        script_request.id = str(int(time.time() * 1000) % 100000)
        script_request.script = script_text
        self.send_script_client.call_async(script_request)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HybridContourFollowingNode()
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