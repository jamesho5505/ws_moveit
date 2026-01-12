#!/usr/bin/env python3
"""
Hybrid Position/Force Control using PVT (Position Tracking)
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from tm_msgs.msg import FeedbackState
from tm_msgs.srv import SendScript
import numpy as np
from scipy.spatial.transform import Rotation as Rotation


class HybridPvtForceNode(Node):
    """
    Hybrid Position/Force Control using PVT
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
        self.desired_normal_force_newton = 3.0   # 目標法向力 (N)
        self.max_force_limit = 12.0              # 最大力限制 (N)
        self.normal_force_threshold = 1.50        # 接觸檢測門檻 (N)
        self.min_normal_force_eps = 1e-6
        
        # 位置限制
        self.min_y_position = -421.0   # mm
        self.min_z_position = 45.0     # mm
        
        # 運動參數
        self.approach_step_x = 0.0    # 每個週期接近的步長 (mm)
        self.approach_step_y = 0.01    # 每個週期接近的步長 (mm)
        self.tangential_step = 0.001    # 切向移動步長 (mm)


        # PI 控制器參數（輸出為位移，不是速度）
        self.Kp_normal_force = 0.001     # 比例增益 (mm/N)
        self.Ki_normal_force = 0.0    # 積分增益 (mm/(N*s))
        self.normal_force_deadband_newton = 0.5
        self.normal_force_integral_limit = 10.0
        self.normal_force_error_integral = 0.0
        self.max_normal_correction_mm = 3.0  # 法向修正的最大位移 (mm)
        
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

        # === 5. 啟動 PVT 模式與定時器 ===
        self.send_script_command("PVTEnter(1)")
        self.get_logger().info("Sent PVTEnter(1), starting in APPROACH mode.")
        
        self.pvt_timer = self.create_timer(
            self.pvt_duration_seconds, 
            self.pvt_timer_callback
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
            self.get_logger().info(
                f"Initialized at position: "
                f"X={x_mm:.2f}, Y={y_mm:.2f}, Z={z_mm:.2f}mm | "
                f"Orientation: Rx={rx_deg:.2f}, Ry={ry_deg:.2f}, Rz={rz_deg:.2f}°"
            )

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

        # if self.current_tool_pose[1] < self.min_y_position:
        #     self.get_logger().error(
        #         f"Y position limit reached: {self.current_tool_pose[1]:.2f}mm < {self.min_y_position}mm"
        #     )
        #     self.send_script_command("PVTExit()")
        #     self.pvt_timer.cancel()
        #     return

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
            # 接近模式：沿 Y 軸移動
            self.target_position[1] += self.approach_step_y
            
            # 其他軸保持在參考位置
            self.target_position[0] = self.reference_position[0]
            self.target_position[2] = self.reference_position[2]
            self.target_position[3:6] = self.reference_position[3:6]
            
            # 日誌（降低頻率）
            self.log_counter += 1
            if self.log_counter % 10 == 0:
                self.get_logger().info(
                    f"APPROACH | Y={self.current_tool_pose[1]:.2f}mm | "
                    f"Target Y={self.target_position[1]:.2f}mm | "
                    f"F={self.normal_force_magnitude:.2f}N"
                )
            
            # 檢測接觸
            if self.normal_force_magnitude > self.normal_force_threshold:
                self.control_mode = "FORCE_CONTROL"
                self.reference_position = self.current_tool_pose.copy()
                self.target_position = self.current_tool_pose.copy()
                self.normal_force_error_integral = 0.0
                
                self.get_logger().info(
                    f"Contact detected! F={self.normal_force_magnitude:.2f}N | "
                    f"Position: Y={self.current_tool_pose[1]:.2f}mm | "
                    f"Switching to FORCE_CONTROL mode"
                )
        
        elif self.control_mode == "FORCE_CONTROL":
            # 力控制模式
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

            # 法向位移（注意負號：力的方向 → 往外退；力的反方向 → 往內推）
            # normal_direction_unit 指向力的方向
            # 當力太大(error < 0)，correction < 0，要往力的方向後退
            # 當力太小(error > 0)，correction > 0，要往力的反方向推入
            normal_displacement_xy = -normal_correction_mm * self.normal_direction_unit

            # 切向位移
            tangential_displacement_xy = self.tangential_step * self.tangential_direction_unit

            # 更新目標位置（累加位移）
            self.target_position[0] += float(normal_displacement_xy[0] + tangential_displacement_xy[0])
            self.target_position[1] += float(normal_displacement_xy[1] + tangential_displacement_xy[1])

            # Z 與姿態保持在參考位置
            self.target_position[2] = self.reference_position[2]
            self.target_position[3:6] = self.reference_position[3:6]
            
            # 日誌
            self.log_counter += 1
            if self.log_counter % 10 == 0:
                self.get_logger().info(
                    f"FORCE_CONTROL | "
                    f"F={self.normal_force_magnitude:.2f}N (target={self.desired_normal_force_newton:.2f}) | "
                    f"Error={normal_force_error:.2f} | "
                    f"Correction={normal_correction_mm:.3f}mm | "
                    f"Position: X={self.current_tool_pose[0]:.2f}, Y={self.current_tool_pose[1]:.2f} | "
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