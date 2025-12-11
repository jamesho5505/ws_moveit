#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, PoseStamped, Twist
from tm_msgs.msg import FeedbackState
from tm_msgs.srv import SendScript


class HybridPvtForceNode(Node):
    """
    Hybrid Position/Force Control using PVT (Velocity Tracking)
    - X, Y, Rx, Ry, Rz axes: Position/Velocity Control (from /cmd_vel)
    - Z axis: Force Compliance Control (from PI controller)
    """

    def __init__(self):
        super().__init__("hybrid_pvt_force_node")

        # === 1. 初始化與通訊 ===
        self.send_script_client = self.create_client(SendScript, "send_script")
        self.get_logger().info("Waiting for /send_script service...")
        self.send_script_client.wait_for_service()
        self.get_logger().info("/send_script service is ready.")

        # === 2. 核心控制參數 (PVT 框架) ===
        self.pvt_duration_seconds = 1.0 / 60.0  # 60 Hz 週期
        self.velocity_command_timeout_seconds = 0.50 

        self.has_tool_pose = False
        self.has_velocity_command = False
        self.current_tool_pose = [0.0] * 6       # 機器人實際位置（從 feedback）
        self.target_tool_pose = [0.0] * 6        # PVT 軌跡積分目標 (P_old)
        self.cartesian_velocity_command = [0.0] * 6  # 外部 /cmd_vel 速度 (mm/s, deg/s)
        self.last_velocity_command_time = None
        
        # === 3. 混合控制/PI 參數 (從 IncrementalApproachNode 繼承) ===
        self.control_mode = "position"             # "position" 或 "force"
        self.latest_force_z = 0.0                  # 實際偵測的 Z 軸力 (N)
        self.force_threshold_newton = 2.0          # 接觸門檻 (N)
        self.desired_contact_force = 3.0           # 目標接觸力 (N)
        
        # PI 增益 (使用 IncrementalApproachNode 的值)
        self.Kp_force = 0.00485
        self.Ki_force = 0.0005
        self.force_deadband = 0.1
        self.integral_limit = 5.0
        self.force_error_integral = 0.0
        self.reset_integral_on_mode_change = True
        
        self.approach_speed_z = -0.5  # Position mode 接近速度 (mm/s)
        
        # === 4. 訂閱者 (Subscribers) ===
        self.force_subscription = self.create_subscription(
            WrenchStamped, "/ft_compensated", self.force_torque_callback, 10,
        )
        self.feedback_subscription = self.create_subscription(
            FeedbackState, "feedback_states", self.feedback_callback, 10,
        )
        self.twist_subscription = self.create_subscription(
            Twist, "/cmd_vel", self.twist_callback, 10,
        )

        # === 5. 啟動 PVT 模式與定時器 ===
        self.send_script_command("PVTEnter(1)")
        self.get_logger().info("Sent PVTEnter(1), Cartesian PVT mode enabled.")
        self.pvt_timer = self.create_timer(self.pvt_duration_seconds, self.pvt_timer_callback)

    # ========== 回呼：力矩傳感器 (Force/Torque Callback) ==========

    def force_torque_callback(self, msg: WrenchStamped) -> None:
        # Z 軸力 Fz，將其轉換為向下的正值 (如果您的傳感器設定是向內為負)
        self.latest_force_z = -msg.wrench.force.z
        
        # 1. 模式切換與積分計算 (只在有速度指令時執行)
        if not self.has_velocity_command:
            return

        if self.control_mode == "position":
            # 偵測接觸
            if self.latest_force_z > self.force_threshold_newton:
                self.get_logger().info(
                    f"Contact detected (Force: {self.latest_force_z:.2f} N), switching to force control"
                )
                self.control_mode = "force"
                if self.reset_integral_on_mode_change:
                    self.force_error_integral = 0.0
                    self.get_logger().info("Reset integral term")
                
        # elif self.control_mode == "force":
        #     # 2. PI 力控制核心
        #     force_error = self.desired_contact_force - self.latest_force_z
            
        #     if abs(force_error) > self.force_deadband:
        #         proportional_term = self.Kp_force * force_error
                
        #         # 積分項：累積誤差
        #         self.force_error_integral += force_error * self.pvt_duration_seconds
                
        #         # 積分限幅 (Anti-windup)
        #         self.force_error_integral = max(-self.integral_limit, min(self.integral_limit, self.force_error_integral))
                
        #         integral_term = self.Ki_force * self.force_error_integral
                
        #         # 3. PI 輸出: 步進位移 (delta_z)
        #         # 輸出為目標步進位移量 (mm)，負號使正誤差 (F_des > F_curr) 導致向下位移 (負 delta_z)
        #         delta_z_mm = -(proportional_term + integral_term)
                
        #         # 4. 轉換為 Z 軸順應性速度 (V_force)
        #         # V_force = delta_z / control_period
        #         V_force_z = delta_z_mm / self.pvt_duration_seconds
                
        #         # 5. 將 Z 軸速度注入 cartesian_velocity_command
        #         # Z 軸 (index 2) 被 PI 控制器接管
        #         self.cartesian_velocity_command[2] = V_force_z
                
        #         self.get_logger().debug(
        #             f"[PI] E={force_error:.2f}N, P={proportional_term:.4f}, I={integral_term:.4f}, Vz={V_force_z:.3f}"
        #         )
        #     else:
        #         # 力量在死區內，停止 Z 軸運動，並緩慢減少積分項
        #         self.cartesian_velocity_command[2] = 0.0
        #         self.force_error_integral *= 0.95


    # ========== 回呼：TCP 姿態 (Feedback) ==========

    def feedback_callback(self, msg: FeedbackState) -> None:
        meters_per_second_to_mm_per_second = 1000.0
        radians_per_second_to_deg_per_second = 180.0 / math.pi
        
        raw = list(msg.tool_pose)
        
        # 單位轉換：m -> mm, rad -> deg
        x_mm  = raw[0] * meters_per_second_to_mm_per_second
        y_mm  = raw[1] * meters_per_second_to_mm_per_second
        z_mm  = raw[2] * meters_per_second_to_mm_per_second
        rx_deg = raw[3] * radians_per_second_to_deg_per_second
        ry_deg = raw[4] * radians_per_second_to_deg_per_second
        rz_deg = raw[5] * radians_per_second_to_deg_per_second
        
        self.current_tool_pose = [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
    
        if not self.has_tool_pose:
            # 第一次初始化時，目標位置 = 實際位置 (用於 PVT 積分起點)
            self.target_tool_pose = self.current_tool_pose.copy()
            self.has_tool_pose = True

    # ========== 回呼：Twist 外部速度指令 (Command Velocity) ==========

    def twist_callback(self, twist_message: Twist) -> None:
        meters_per_second_to_mm_per_second = 1000.0
        radians_per_second_to_deg_per_second = 180.0 / math.pi
        had_velocity_command_before = self.has_velocity_command

        # 接收外部的 X, Y, Z, Rx, Ry, Rz 速度指令
        linear_x_mm_per_second = twist_message.linear.x * meters_per_second_to_mm_per_second
        linear_y_mm_per_second = twist_message.linear.y * meters_per_second_to_mm_per_second
        linear_z_mm_per_second = twist_message.linear.z * meters_per_second_to_mm_per_second

        angular_rx_deg_per_second = twist_message.angular.x * radians_per_second_to_deg_per_second
        angular_ry_deg_per_second = twist_message.angular.y * radians_per_second_to_deg_per_second
        angular_rz_deg_per_second = twist_message.angular.z * radians_per_second_to_deg_per_second

        # 將外部指令存入，但 Z 軸會被 force_torque_callback 覆寫 (Hybrid Logic)
        self.cartesian_velocity_command = [
            linear_x_mm_per_second, linear_y_mm_per_second, linear_z_mm_per_second,
            angular_rx_deg_per_second, angular_ry_deg_per_second, angular_rz_deg_per_second,
        ]
        
        current_time_seconds = time.time()
        self.last_velocity_command_time = current_time_seconds
        
        is_moving_now = any(abs(v) > 1e-6 for v in self.cartesian_velocity_command)
        self.has_velocity_command = is_moving_now
        
        if self.control_mode == "position":
            # Position 模式下，Z 軸的接近速度
            self.cartesian_velocity_command[2] = self.approach_speed_z
        
        # 只有從 False → True 的那一瞬間才重送 PVTEnter(1)
        if self.has_velocity_command and not had_velocity_command_before:
            self.get_logger().info("New velocity command after stop, send PVTEnter(1) again.")
            self.send_script_command("PVTEnter(1)")


    # ------------------------------------------------------------------
    # 週期送出一個新的 PVTPoint (Hybrid Control Core)
    # ------------------------------------------------------------------
    def pvt_timer_callback(self) -> None:
        if not self.has_tool_pose:
            return
        
        if self.last_velocity_command_time is not None:
            elapsed_seconds = time.time() - self.last_velocity_command_time
            if elapsed_seconds > self.velocity_command_timeout_seconds:
                # 超過 timeout 才清掉，並送一次 PVTExit
                self.cartesian_velocity_command = [0.0] * 6
                if self.has_velocity_command:
                    # 只有在 PVTExit() 前才顯示 log
                    self.get_logger().info("Velocity timeout, send PVTExit().")
                    self.send_script_command("PVTExit()")
                    # 讓下次啟動時，current_tool_pose 必須從 feedback 重新取得
                
                self.has_velocity_command = False

        if not self.has_velocity_command:
            return
            
        dt = self.pvt_duration_seconds 
        max_linear_speed_mm_s = 100.0
        max_angular_speed_deg_s = 30.0
        
        V_cmd_final = self.cartesian_velocity_command.copy()
        if self.control_mode == "force":
            force_error = self.desired_contact_force - self.latest_force_z
            
            if abs(force_error) > self.force_deadband:
                proportional_term = self.Kp_force * force_error
                
                # 更新積分項 (確保積分週期與定時器一致)
                self.force_error_integral += force_error * dt
                self.force_error_integral = max(-self.integral_limit, min(self.integral_limit, self.force_error_integral))
                integral_term = self.Ki_force * self.force_error_integral
                
                # PI 輸出 (delta_z) / dt = V_force
                delta_z_mm = -(proportional_term + integral_term)
                V_force_z = delta_z_mm / dt 

                # 覆寫 Z 軸速度
                V_cmd_final[2] = V_force_z
                
                # 執行極限檢查
                # (如果實際力量超過極限，這裡應加入安全停止邏輯)

            else:
                # 力量在死區內，Z 軸速度歸零
                V_cmd_final[2] = 0.0
                self.force_error_integral *= 0.95 # 緩慢減少積分項
        
        elif self.control_mode == "position":
            # Position Mode 下，Z 軸使用接近速度
            V_cmd_final[2] = self.approach_speed_z 
            
        # --- PVT 積分與發送 (原程式碼邏輯) ---
        next_tool_pose = []
        for axis_index in range(6):
            current_position = self.target_tool_pose[axis_index]
            raw_velocity = V_cmd_final[axis_index] # 使用經過 Hybrid 處理的 V_cmd_final

            if axis_index < 3:
                clamped_velocity = max(-max_linear_speed_mm_s,
                                    min(max_linear_speed_mm_s, raw_velocity))
            else:
                clamped_velocity = max(-max_angular_speed_deg_s,
                                    min(max_angular_speed_deg_s, raw_velocity))

            next_position = current_position + clamped_velocity * dt
            
            V_cmd_final[axis_index] = clamped_velocity
            next_tool_pose.append(next_position)

        position_string = "{" + ", ".join(f"{value:.4f}" for value in next_tool_pose) + "}"
        velocity_string = "{" + ", ".join(f"{value:.4f}" for value in V_cmd_final) + "}"
        script_command = f"PVTPoint({position_string}, {velocity_string}, {dt})"

        # 送給 TM
        self.send_script_command(script_command)

        # 更新下一輪積分基準
        self.target_tool_pose = next_tool_pose.copy() # 更新積分基準

    # ------------------------------------------------------------------
    # SendScript 封裝
    # ------------------------------------------------------------------
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

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from tm_msgs.srv import SendScript
# from geometry_msgs.msg import WrenchStamped, PoseStamped

# class IncrementalApproachNode(Node):
#     def __init__(self):
#         super().__init__("incremental_approach_node")

#         # /send_script client
#         self.send_script_client = self.create_client(SendScript, "send_script")
#         self.get_logger().info("Waiting for /send_script service...")
#         self.send_script_client.wait_for_service()
#         self.get_logger().info("/send_script service ready")

#         # 力訊號訂閱
#         self.force_subscription = self.create_subscription(
#             WrenchStamped,
#             "/ft_compensated",
#             self.force_torque_callback,
#             10,
#         )

#         # TCP 位姿訂閱
#         self.tool_pose_subscription = self.create_subscription(
#             PoseStamped,
#             "/tool_pose",
#             self.tool_pose_callback,
#             10,
#         )

#         # === 力量相關變數 ===
#         self.latest_force_z = 0.0
#         self.force_threshold_newton = 2.0
        
#         # === 控制模式 ===
#         self.control_mode = "position"  # "position" 或 "force"
        
#         # === PI 力控制參數 ===
#         self.desired_contact_force = 3.0       # 目標接觸力 (N)
#         self.Kp_force = 0.00485                  # 比例增益
#         self.Ki_force = 0.0005                   # 積分增益
#         self.force_deadband = 0.1              # 力控制死區 (N)
        
#         # 積分項相關
#         self.force_error_integral = 0.0       # 力誤差積分累積
#         self.integral_limit = 5.0             # 積分限幅
#         self.reset_integral_on_mode_change = True
        
#         # === 位置控制參數 ===
#         self.approach_speed = 0.5            # 接近速度 (mm/step)

#         # === 位姿相關變數 ===
#         self.latest_tool_pose_msg: PoseStamped | None = None
#         self.contact_tool_pose_msg: PoseStamped | None = None
#         self.contact_position_z: float | None = None

#         # 當前 TCP 姿態 [X, Y, Z, Rx, Ry, Rz]
#         self.current_tcp_pose_xyzrpy = None

#         # === 步進控制變數 ===
#         self.step_delta_z = -0.05
#         self.step_target_z_mm = None
#         self.step_in_progress = False
#         self.position_tolerance_mm = 0.05

#         # === 控制迴圈參數 ===
#         self.control_period_s = 0.05
#         self.max_step_count = 800
#         self.current_step_count = 0
#         self.pending_future = None

#         # 啟動控制定時器
#         self.control_timer = self.create_timer(
#             self.control_period_s,
#             self.control_loop_callback,
#         )

#     # ========== 回呼：tool pose ==========

#     def tool_pose_callback(self, msg: PoseStamped) -> None:
#         self.latest_tool_pose_msg = msg

#         if self.step_in_progress and self.step_target_z_mm is not None:
#             current_z_mm = msg.pose.position.z * 1000.0

#             if abs(current_z_mm - self.step_target_z_mm) < self.position_tolerance_mm:
#                 self.step_in_progress = False

#     # ========== 回呼：force / torque (PI 控制) ==========

#     def force_torque_callback(self, msg: WrenchStamped) -> None:
#         self.latest_force_z = -msg.wrench.force.z

#         if self.control_mode == "position":
#             # 接近階段：偵測接觸
#             if self.latest_force_z > self.force_threshold_newton:
#                 self.get_logger().info(
#                     f"Contact detected (Force: {self.latest_force_z:.2f} N), "
#                     "switching to force control"
#                 )
#                 self.control_mode = "force"
#                 if self.current_tcp_pose_xyzrpy is not None:
#                     self.contact_position_z = self.current_tcp_pose_xyzrpy[2]
                
#                 # 切換到力控模式時重置積分項
#                 if self.reset_integral_on_mode_change:
#                     self.force_error_integral = 0.0
#                     self.get_logger().info("Reset integral term")
                
#         elif self.control_mode == "force":
#             # === PI 力控制 ===
#             force_error = self.desired_contact_force - self.latest_force_z
            
#             if abs(force_error) > self.force_deadband:
#                 # 比例項
#                 proportional_term = self.Kp_force * force_error
                
#                 # 積分項：累積誤差
#                 self.force_error_integral += force_error * self.control_period_s
                
#                 # 積分限幅 (Anti-windup)
#                 if self.force_error_integral > self.integral_limit:
#                     self.force_error_integral = self.integral_limit
#                 elif self.force_error_integral < -self.integral_limit:
#                     self.force_error_integral = -self.integral_limit
                
#                 integral_term = self.Ki_force * self.force_error_integral
                
#                 # PI 控制輸出
#                 self.step_delta_z = -(proportional_term + integral_term)
                
#                 self.get_logger().info(
#                     f"PI Control: Error={force_error:.2f}N, "
#                     f"P={proportional_term:.4f}, I={integral_term:.4f}, "
#                     f"Integral_sum={self.force_error_integral:.3f}, "
#                     f"Output={self.step_delta_z:.3f}mm"
#                 )
#             else:
#                 # 力量在死區內
#                 self.step_delta_z = 0
#                 # 慢慢減少積分項，避免不必要的累積
#                 self.force_error_integral *= 0.95
#                 self.get_logger().debug(
#                     f"Force in deadband. Integral={self.force_error_integral:.3f}"
#                 )

#     # ========== 控制主迴圈 ==========

#     def control_loop_callback(self):
#         # 第一次初始化
#         if self.current_tcp_pose_xyzrpy is None:
#             if self.latest_tool_pose_msg is None:
#                 self.get_logger().info("Waiting for first /tool_pose...")
#                 return

#             pose = self.latest_tool_pose_msg.pose
#             tcp_x_mm = pose.position.x * 1000.0
#             tcp_y_mm = pose.position.y * 1000.0
#             tcp_z_mm = pose.position.z * 1000.0

#             tcp_rx_deg = -180.0
#             tcp_ry_deg = 0.0
#             tcp_rz_deg = -180.0

#             self.current_tcp_pose_xyzrpy = [
#                 tcp_x_mm, tcp_y_mm, tcp_z_mm,
#                 tcp_rx_deg, tcp_ry_deg, tcp_rz_deg,
#             ]
#             self.get_logger().info(
#                 f"Initialized TCP: ({tcp_x_mm:.2f}, {tcp_y_mm:.2f}, {tcp_z_mm:.2f}) mm"
#             )
#             return

#         # 步數保護
#         if self.current_step_count >= self.max_step_count:
#             self.get_logger().warn(f"Reached max steps ({self.max_step_count})")
#             self.send_stop_and_clear_buffer()
#             self.control_timer.cancel()
#             return

#         # 等待條件
#         if self.step_in_progress:
#             return
#         if self.pending_future is not None and not self.pending_future.done():
#             return

#         # 根據控制模式決定移動量
#         if self.control_mode == "position":
#             self.step_delta_z = -self.approach_speed
#         # force 模式的 step_delta_z 已在 force_torque_callback 中計算

#         # 只有在有明顯移動時才發送
#         if abs(self.step_delta_z) < 0.001:
#             return

#         # 更新目標位置並發送指令
#         self.current_tcp_pose_xyzrpy[2] += self.step_delta_z
#         self.current_step_count += 1

#         (tcp_x, tcp_y, tcp_z, tcp_rx, tcp_ry, tcp_rz) = self.current_tcp_pose_xyzrpy

#         tm_script_command = (
#             'PTP("CPP",'
#             f"{tcp_x:.3f},{tcp_y:.3f},{tcp_z:.3f},"
#             f"{tcp_rx:.3f},{tcp_ry:.3f},{tcp_rz:.3f},"
#             "5,50,0,false)"
#         )

#         script_request = SendScript.Request()
#         script_request.id = "1"
#         script_request.script = tm_script_command

#         self.step_target_z_mm = tcp_z
#         self.step_in_progress = True

#         self.get_logger().info(
#             f"[{self.control_mode.upper()}] Step {self.current_step_count}: "
#             f"Z={tcp_z:.3f}mm, Force={self.latest_force_z:.2f}N"
#         )
        
#         self.pending_future = self.send_script_client.call_async(script_request)
#         self.pending_future.add_done_callback(self.step_response_callback)

#     # ========== 輔助函數 ==========

#     def send_stop_and_clear_buffer(self):
#         stop_request = SendScript.Request()
#         stop_request.id = "stop"
#         stop_request.script = "StopAndClearBuffer(0)"
#         self.send_script_client.call_async(stop_request)
#         self.get_logger().info("Sent StopAndClearBuffer(0)")

#     def step_response_callback(self, future):
#         try:
#             response = future.result()
#             self.get_logger().debug(f"SendScript response.ok = {response.ok}")
#         except Exception as error:
#             self.get_logger().error(f"SendScript failed: {error}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = IncrementalApproachNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Node stopped by KeyboardInterrupt.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()