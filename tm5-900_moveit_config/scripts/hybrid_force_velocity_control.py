#!/usr/bin/env python3
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
    Hybrid Position/Force Control using PVT (Velocity Tracking)
    - Automatically starts in APPROACH mode and switches to FORCE_CONTROL upon contact detection.
    """

    def __init__(self):
        super().__init__("hybrid_pvt_force_node")

        # === 1. 初始化與通訊 ===
        self.send_script_client = self.create_client(SendScript, "send_script")
        self.get_logger().info("Waiting for /send_script service...")
        self.send_script_client.wait_for_service()
        self.get_logger().info("/send_script service is ready.")

        # === 2. 核心控制參數 (PVT 框架) ===
        self.pvt_duration_seconds = 1.0 / 100.0 

        self.has_tool_pose = False
        self.current_tool_pose = [0.0] * 6       # 機器人實際位置（從 feedback）
        self.target_tool_pose = [0.0] * 6        # PVT 軌跡積分目標
        self.reference_position = [0.0] * 6      # 接觸時的參考位置 (X, Y 固定用)
        
        # === 3. 狀態機與力控參數 ===
        self.control_mode = "APPROACH"  # "APPROACH" → "FORCE_CONTROL"
        self.latest_force = np.array([0.0, 0.0, 0.0])  # 最新力傳感器讀數 (X, Y, Z)
        self.latest_torque = np.array([0.0, 0.0, 0.0])  # 最新力矩傳感器讀數 (X, Y, Z)
        self.force_x_base = 0.0
        self.force_y_base = 0.0
        self.force_z_base = 0.0
        self.torque_x_base = 0.0
        self.torque_y_base = 0.0
        self.torque_z_base = 0.0
        self.force_threshold_newton = 1.0       # 接觸門檻
        self.desired_normal_force_newton = 3.0     # 目標法向力 (N)
        self.max_force_limit = 12.0  # N
        self.min_y_position = -421.0   # mm (絕對下限)
        self.min_z_position = 48.0   # mm (絕對下限)
        self.tangential_speed_mm_s = 0.10        # 切向移動速度
        self.normal_force_threshold = 2.0       # 判斷接觸用
        self.min_normal_force_eps = 1e-6      

        
        # PI 增益
        self.Kp_normal_force = 0.050
        self.Ki_normal_force = 0.010
        self.force_deadband = 0.1
        self.normal_force_deadband_newton = 0.1
        self.normal_force_integral_limit = 5.0
        self.normal_force_error_integral = 0.0
        self.max_normal_correction_speed_mm_s = 2.0  # 法向修正速度上限，避免震盪
                
        # 接近速度 (mm/s)
        self.approach_speed_y = 0.50 
        
        # 速度限制
        self.max_linear_speed_mm_s = 1.0
        self.max_angular_speed_deg_s = 1.0
        
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

    # ========== 回呼：力矩傳感器 ==========
    def force_torque_callback(self, msg: WrenchStamped) -> None:
        # Z 軸力 (向下為正)
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


    # ========== 回呼：TCP 姿態 ==========
    def feedback_callback(self, msg: FeedbackState) -> None:
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
            # 第一次初始化：目標位置 = 實際位置
            self.target_tool_pose = self.current_tool_pose.copy()
            self.reference_position = self.current_tool_pose.copy()
            self.has_tool_pose = True
            self.get_logger().info(
                f"Initialized at position: "
                f"X={x_mm:.2f}, Y={y_mm:.2f}, Z={z_mm:.2f}mm"
            )

    # ========== PVT 主控制迴圈 ==========
    def pvt_timer_callback(self) -> None:
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
        force_xy_vector = np.array([self.force_x_base, self.force_y_base])
        normal_force_magnitude = np.linalg.norm(force_xy_vector)

        if normal_force_magnitude < self.min_normal_force_eps:
            normal_direction_unit = np.array([0.0, 0.0])
            tangential_direction_unit = np.array([0.0, 0.0])
        else:
            # 法向單位向量
            normal_direction_unit = force_xy_vector / normal_force_magnitude

            # 切向 = 法向逆時針轉 90 度
            tangential_direction_unit = np.array([
                -normal_direction_unit[1],
                normal_direction_unit[0]
            ])
        self.get_logger().info(
            f"Force base: Fx={force_in_base[0]:.2f}, "
            f"Fy={force_in_base[1]:.2f}, "
            f"Fz={force_in_base[2]:.2f}"
            f" | Normal force magnitude: {normal_force_magnitude:.2f}N"
        )


        if not self.has_tool_pose:
            return
        
        if normal_force_magnitude > self.max_force_limit:
            self.get_logger().error(f"Force exceeded limit: {normal_force_magnitude:.2f}N")
            self.send_script_command("PVTExit()")
            self.pvt_timer.cancel()
            return

        # if self.current_tool_pose[1] > self.min_y_position:
        #     self.get_logger().error(f"Y position too High: {self.current_tool_pose[1]:.2f}mm")
        #     self.send_script_command("PVTExit()")
        #     self.pvt_timer.cancel()
        #     return

        if self.current_tool_pose[2] < self.min_z_position:
            self.get_logger().error(f"Z position too Low: {self.current_tool_pose[2]:.2f}mm")
            self.send_script_command("PVTExit()")
            self.pvt_timer.cancel()
            return
        
        dt = self.pvt_duration_seconds
        
        # 初始化速度指令 (全部為 0)
        velocity_command = [0.0] * 6
        
        # === 狀態機控制邏輯 ===
        if self.control_mode == "APPROACH":
            velocity_command[1] = self.approach_speed_y  # 沿 Y 軸接近
            self.get_logger().info("APPROACH mode: moving along Y axis.")
            velocity_command[3] = 0.0  # Rx
            velocity_command[4] = 0.0  # Ry
            velocity_command[5] = 0.0  # Rz
            if normal_force_magnitude > self.normal_force_threshold:
                self.control_mode = "FORCE_CONTROL"
                print("normal_force_magnitude:", normal_force_magnitude)
                self.reference_position = self.current_tool_pose.copy()
                self.normal_force_error_integral = 0.0

                self.get_logger().info(
                    f"Contact detected | "
                    f"Pos: X={self.current_tool_pose[0]:.2f}, "
                    f"Y={self.current_tool_pose[1]:.2f}, "
                    f"Z={self.current_tool_pose[2]:.2f} | "
                    f"Orient: Rx={self.current_tool_pose[3]:.2f}, "
                    f"Ry={self.current_tool_pose[4]:.2f}, "
                    f"Rz={self.current_tool_pose[5]:.2f}"
                )


        elif self.control_mode == "FORCE_CONTROL":
            # self.get_logger().info("FORCE_CONTROL mode.")
            normal_force_error = self.desired_normal_force_newton - normal_force_magnitude

            if abs(normal_force_error) > self.normal_force_deadband_newton:
                proportional_term = self.Kp_normal_force * normal_force_error

                self.normal_force_error_integral += normal_force_error * dt
                self.normal_force_error_integral = float(np.clip(
                    self.normal_force_error_integral,
                    -self.normal_force_integral_limit,
                    self.normal_force_integral_limit
                ))
                integral_term = self.Ki_normal_force * self.normal_force_error_integral

                normal_correction_speed_mm_s = proportional_term + integral_term
            else:
                normal_correction_speed_mm_s = 0.0
                self.normal_force_error_integral *= 0.95

            normal_correction_speed_mm_s = float(np.clip(
                normal_correction_speed_mm_s,
                -self.max_normal_correction_speed_mm_s,
                self.max_normal_correction_speed_mm_s
            ))

            # 法向位移（注意負號：力的方向 → 往外退；力的反方向 → 往內推）
            # normal_direction_unit 指向力的方向
            # 當力太大(error < 0)，correction < 0，要往力的方向後退
            # 當力太小(error > 0)，correction > 0，要往力的反方向推入
            velocity_normal_xy = -normal_correction_speed_mm_s * normal_direction_unit

            #  切向掃描：固定速度沿 t_hat 
            velocity_tangential_xy = self.tangential_speed_mm_s * tangential_direction_unit

            #  合成 XY 速度 
            velocity_command[0] = float(velocity_normal_xy[0] + velocity_tangential_xy[0])
            velocity_command[1] = float(velocity_normal_xy[1] + velocity_tangential_xy[1])
            self.get_logger().info(f"velocity_command XY:  {velocity_command[0]:.2f}, {velocity_command[1]:.2f} mm/s")

            for axis_index in [2, 3, 4, 5]:
                velocity_command[axis_index] = 0.0  # 完全鎖定不動


        # === 速度限制 ===
        for i in range(3):
            velocity_command[i] = max(
                -self.max_linear_speed_mm_s,
                min(self.max_linear_speed_mm_s, velocity_command[i])
            )
        for i in range(3, 6):
            velocity_command[i] = max(
                -self.max_angular_speed_deg_s,
                min(self.max_angular_speed_deg_s, velocity_command[i])
            )
        
        # === PVT 積分：計算下一個目標位置 ===
        next_tool_pose = []
        for i in range(6):
            next_position = self.target_tool_pose[i] + velocity_command[i] * dt
            next_tool_pose.append(next_position)
        
        # === 產生 PVTPoint 指令 ===
        position_string = "{" + ", ".join(f"{v:.4f}" for v in next_tool_pose) + "}"
        velocity_string = "{" + ", ".join(f"{v:.4f}" for v in velocity_command) + "}"
        script_command = f"PVTPoint({position_string}, {velocity_string}, {dt})"
        
        # 送給 TM Robot
        self.send_script_command(script_command)
        
        # 更新下一輪積分基準
        self.target_tool_pose = next_tool_pose.copy()

    # ========== SendScript 封裝 ==========
    def send_script_command(self, script_text: str) -> None:
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
