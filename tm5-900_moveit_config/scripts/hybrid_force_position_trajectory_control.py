#!/usr/bin/env python3
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tm_msgs.msg import FeedbackState
from tm_msgs.srv import SendScript
from geometry_msgs.msg import WrenchStamped, PointStamped
from std_msgs.msg import String, Float64MultiArray
from scipy.spatial.transform import Rotation as Rotation


# =========================================================
# Utility
# =========================================================

def is_contact_detected(force_n: float, threshold: float) -> bool:
    return force_n > threshold


# =========================================================
# Main Node
# =========================================================

class HybridContourFollowingNode(Node):

    def __init__(self):
        super().__init__("hybrid_contour_following_node")

        # ---------------- ROS ----------------
        self.send_script_client = self.create_client(SendScript, "send_script")
        self.send_script_client.wait_for_service()

        self.create_subscription(WrenchStamped, "/ft_compensated", self.force_cb, 10)
        self.create_subscription(FeedbackState, "feedback_states", self.feedback_cb, 10)

        self.pub_state = self.create_publisher(String, "/force_control/control_state", 10)
        self.pub_control_parameters = self.create_publisher(
            Float64MultiArray, "/force_control/control_parameters", 10
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
        self.log_counter = 0

        # ---------------- Timing ----------------
        self.ctrl_hz = 60.0
        self.dt = 1.0 / self.ctrl_hz
        self.delay_steps = 1   # <<< 關鍵：等效 k+2
        self.last_time = time.time()

        # ---------------- Robot state ----------------
        self.has_pose = False
        self.pose = np.zeros(6)          # [mm, mm, mm, deg, deg, deg]
        self.target_pose = np.zeros(6)
        self.target_vel = np.zeros(6)

        # ---------------- Force ----------------
        self.force = np.zeros(3)
        self.force_base = np.zeros(3)
        self.prev_force_error = 0.0
        self.reference_force = 3.5
        self.max_force_limit = 12.0
        self.force_threshold_on = 1.80   # 進入 Hybrid 的門檻
        self.force_threshold_off = 0.50  # 退出 Hybrid 的門檻
        self.is_in_hybrid = False       # 紀錄當前是否處於力控狀態
        self.force_established = False  # 記錄是否已經建立穩定接觸
        self.force_stable_time = 0.0
        self.force_stable_required = 0.05  # 力穩定所需時間（秒）

        # ---------------- Admittance (Normal) ----------------
        self.Kf = 0.050          # mm / N
        self.Df = 0.000         # mm / (N/s)
        self.force_deadband = 0.3  # N
        self.max_normal_step = 0.005
        self.force_correction = np.zeros(2)

        # ---------------- Trajectory ----------------
        self.traj_speed = 0.40   # mm/s
        self.traj_dir = np.array([math.sqrt(3)/2, 0.5])
        # self.traj_dir = np.array([0.0, 1.0])
        self.traj_dir /= np.linalg.norm(self.traj_dir)
        self.traj_length = 50.0
        self.traj_time = 0.0
        self.traj_start = None
        self.min_z_position = 45.0

        # ---------------- Contact frame ----------------
        self.ref_tangent = None

        # ---------------- Start ----------------
        self.send_script("PVTEnter(1)")
        # self.timer = self.create_timer(self.dt, self.control_loop)
        self.timer = self.create_timer(1/60, self.control_loop)
        msg_control_parameters = Float64MultiArray()
        msg_control_parameters.data = [
            float(self.ctrl_hz), 
            float(self.reference_force), 
            float(self.force_threshold_on), 
            float(self.force_threshold_off), 
            float(self.max_force_limit), 
            float(self.Kf), 
            float(self.Df), 
            float(self.force_deadband), 
            float(self.max_normal_step), 
            float(self.traj_speed), 
            float(self.traj_dir[0]),      # traj_dir_x 分量
            float(self.traj_dir[1]),      # traj_dir_y 分量
            float(self.traj_length), 
            float(self.force_stable_required)
        ]
        self.pub_control_parameters.publish(msg_control_parameters)


        

    # =====================================================
    # Callbacks
    # =====================================================

    def force_cb(self, msg: WrenchStamped):
        self.force[:] = [
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z
        ]

    def feedback_cb(self, msg: FeedbackState):
        p = msg.tool_pose
        self.pose[:] = [
            p[0] * 1000.0,
            p[1] * 1000.0,
            p[2] * 1000.0,
            p[3] * 180.0 / math.pi,
            p[4] * 180.0 / math.pi,
            p[5] * 180.0 / math.pi,
        ]

        if not self.has_pose:
            self.has_pose = True
            self.target_pose[:] = self.pose
            self.traj_start = self.pose[:2].copy()

    # =====================================================
    # Trajectory
    # =====================================================

    def generate_trajectory(self):
        s = min(self.traj_speed * self.traj_time, self.traj_length)
        pos = self.traj_start + self.traj_dir * s
        vel = self.traj_dir * self.traj_speed
        return pos, vel

    # =====================================================
    # Projection
    # =====================================================
    def project_onto_traj(self, reference_pos, reference_vel, t_hat: np.ndarray):

        predicted_pos = (
            reference_pos
            + self.delay_steps * reference_vel * self.dt
        )

        # ===============================
        # (1) Projection：只要接觸就算
        # ===============================
        cur_pos = self.pose[:2]
        AB = predicted_pos - cur_pos
        delta_L1 = np.linalg.norm(AB)
        delta_L2 = np.dot(AB, t_hat)

        # 禁止切向回頭
        delta_L2 = max(delta_L2, 0.0)

        cos_theta = float(
            np.clip(delta_L2 / delta_L1 if delta_L1 > 1e-9 else 0.0, -1.0, 1.0)
        )
        theta_rad = float(np.arccos(cos_theta))

        corrected_data = [delta_L1, delta_L2, math.degrees(theta_rad)]
        projected_pos = cur_pos + delta_L2 * t_hat
        return projected_pos, corrected_data


    # =====================================================
    # Control Loop
    # =====================================================

    def control_loop(self):
        if not self.has_pose:
            return

        current_time = time.time()
        dt_measured = current_time - self.last_time
        self.last_time = current_time

        dt_effective = max(dt_measured, 1.0 / self.ctrl_hz)
        self.dt = dt_effective

        cur_pos = self.pose[:2]
        corrected_data = np.zeros(3)
        force_stable = False  # 預設值

        # === 座標轉換：Tool frame → Base frame ===
        rx_deg, ry_deg, rz_deg = self.pose[3:6]
        rotation_tool_to_base = Rotation.from_euler(
            'zyx',
            [rz_deg, ry_deg, rx_deg],
            degrees=True
        ).as_matrix()
        
        self.force_base = rotation_tool_to_base @ self.force

        # ---------- Force frame ----------
        f_xy = self.force_base[:2]
        force_n = np.linalg.norm(f_xy)

        # if force_n > 1e-6:
        #     n_hat = f_xy / force_n
        #     t_hat = np.array([-n_hat[1], n_hat[0]])
        # else:
        #     n_hat = np.zeros(2)
        #     t_hat = self.traj_dir.copy()

        force_n_raw = np.linalg.norm(f_xy)

        if self.is_in_hybrid and force_n_raw > 0.5:
            # 1. 取得力的原始方向
            f_hat_raw = f_xy / force_n_raw
            
            # 2. 【核心修正】將力向量投影到「預期法向」上
            # 假設您的 traj_dir 是前進方向，那麼 [-y, x] 就是預期法向
            expected_n = np.array([-self.traj_dir[1], self.traj_dir[0]])
            
            # 3. 限制法向量的變動範圍 (例如只允許在預期方向的 ±30 度內變動)
            # 或者簡單地使用低通濾波：
            if not hasattr(self, 'smooth_n_hat'): self.smooth_n_hat = expected_n
            alpha = 0.05 # 濾波係數，越小越穩定
            self.smooth_n_hat = (1-alpha) * self.smooth_n_hat + alpha * f_hat_raw
            self.smooth_n_hat /= np.linalg.norm(self.smooth_n_hat)
            
            n_hat = self.smooth_n_hat
            t_hat = np.array([-n_hat[1], n_hat[0]])
            if np.dot(t_hat, self.traj_dir) < 0:
                t_hat = -t_hat
        else:
            n_hat = np.array([-self.traj_dir[1], self.traj_dir[0]])
            t_hat = self.traj_dir.copy()

        # ==================================================
        # ★ 遲滯判斷邏輯 (Hysteresis Logic)
        # ==================================================
        if not self.is_in_hybrid:
            if force_n > self.force_threshold_on:
                self.is_in_hybrid = True
                self.force_established = False
                self.force_stable_time = 0.0
                self.target_vel[:] = 0.0
                self.traj_time = 0.0
                self.traj_start = cur_pos.copy()
                self.get_logger().info(f">>> Switch to HYBRID_CONTROL (F={force_n:.2f}N)")
        else:
            if force_n < self.force_threshold_off:
                self.is_in_hybrid = False
                self.force_established = False
                self.ref_tangent = None
                self.force_stable_time = 0.0
                self.get_logger().info(f"<<< Switch to POSITION (F={force_n:.2f}N)")

        # === 安全檢查 ===
        if force_n > self.max_force_limit:
            self.get_logger().error(
                f"Force exceeded limit: {force_n:.2f}N > {self.max_force_limit}N"
            )
            self.send_script("PVTExit()")
            self.timer.cancel()
            return

        if self.pose[2] < self.min_z_position:
            self.get_logger().error(
                f"Z position limit reached: {self.pose[2]:.2f}mm < {self.min_z_position}mm"
            )
            self.send_script("PVTExit()")
            self.timer.cancel()
            return

        

        state = "HYBRID_CONTROL" if self.is_in_hybrid else "POSITION"
        self.pub_state.publish(String(data=state))

        

        if self.is_in_hybrid:
            # ============================================
            # Hybrid Mode: Force Control (Normal direction)
            # ============================================
            force_error = self.reference_force - force_n
            force_error_dot = (force_error - self.prev_force_error) / self.dt
            self.prev_force_error = force_error # 更新舊誤差

            if abs(force_error) <= self.force_deadband:
                self.force_stable_time += self.dt
            else:
                self.force_stable_time = 0.0
            
            delta = self.Kf * force_error + self.Df * force_error_dot
            delta = np.clip(delta, -self.max_normal_step, self.max_normal_step)
            self.force_correction = -n_hat * delta

            force_stable = self.force_stable_time >= self.force_stable_required

            if force_stable and (not self.force_established):
                self.force_established = True
                # 力穩定時，重設軌跡起點
                self.traj_start = cur_pos.copy()
                self.traj_time = 0.0
                self.get_logger().info("Force established, trajectory reset to current position")

            if force_stable:
                # ============================================
                # Force Stable: 切向移動 + 法向修正
                # ============================================
                self.traj_time += self.dt
                reference_pos, reference_vel = self.generate_trajectory()
                projected_pos, corrected_data = self.project_onto_traj(reference_pos, reference_vel, t_hat)
                
                # 使用 projection 作為切向參考
                new_reference_pos = projected_pos + self.force_correction
            else:
                # ============================================
                # Force Unstable: 只法向修正，不走切向
                # ============================================
                # 維持當前位置，只加法向修正
                new_reference_pos = cur_pos + self.force_correction

        else:
            # ============================================
            # Position Mode: 自由空間運動
            # ============================================
            self.force_stable_time = 0.0
            self.force_established = False
            self.force_correction = np.zeros(2)
            self.traj_time += self.dt
            new_reference_pos, _ = self.generate_trajectory()
            position_error = new_reference_pos - cur_pos

        position_error = new_reference_pos - cur_pos

        # ---------- Publish debug info ----------
        now = self.get_clock().now().to_msg()
        
        # 期望點（只在 force stable 時有意義）
        if self.is_in_hybrid and force_stable:
            reference_pos, _ = self.generate_trajectory()
        else:
            reference_pos = cur_pos  # 沒有切向移動時，顯示當前位置
        
        msg_reference = PointStamped()
        msg_reference.header.stamp = now
        msg_reference.header.frame_id = "base"
        msg_reference.point.x = float(reference_pos[0])
        msg_reference.point.y = float(reference_pos[1])
        msg_reference.point.z = 0.0
        self.pub_desired_point.publish(msg_reference)
        
        # 修正點
        msg_corrected = PointStamped()
        msg_corrected.header.stamp = now
        msg_corrected.header.frame_id = "base"
        msg_corrected.point.x = float(new_reference_pos[0])
        msg_corrected.point.y = float(new_reference_pos[1])
        msg_corrected.point.z = 0.0
        self.pub_corrected_point.publish(msg_corrected)

        # 發布修正數據
        msg_corrected_data = Float64MultiArray()
        msg_corrected_data.data = list(corrected_data)
        self.pub_corrected_data.publish(msg_corrected_data)

        # ---------- Command ----------
        self.target_pose[:2] = new_reference_pos
        self.target_pose[2] = self.pose[2]
        self.target_pose[3:] = self.pose[3:]

        # self.target_vel[:2] = 0.0
        self.target_vel[:2] = self.traj_dir * self.traj_speed
        self.target_vel[2:] = 0.0

        self.log_counter += 1
        if self.log_counter % 20 == 0:
            stable_str = "STABLE" if force_stable else "UNSTABLE"
            self.get_logger().info(
                f"{state}({stable_str}) | t={self.traj_time:.2f}s | "
                f"F={force_n:.2f}N (err={self.reference_force - force_n:.2f}) | "
                f"error=[{position_error[0]:.5f}, {position_error[1]:.5f}] mm | "
                f"vel={np.linalg.norm(self.target_vel):.2f} mm/s"
            )

        self.send_pvt()

        # 目標點
        msg_target = PointStamped()
        msg_target.header.stamp = now
        msg_target.header.frame_id = "base"
        msg_target.point.x = float(self.target_pose[0])
        msg_target.point.y = float(self.target_pose[1])
        msg_target.point.z = float(self.target_pose[2])
        self.pub_target_point.publish(msg_target)


    # =====================================================
    # TM Command
    # =====================================================

    def send_pvt(self):
        pvt_time = 0.01
        pos = "{" + ",".join(f"{v:.4f}" for v in self.target_pose) + "}"
        vel = "{" + ",".join(f"{v:.4f}" for v in self.target_vel) + "}"
        self.send_script(f"PVTPoint({pos},{vel},{pvt_time})")

    def send_script(self, script: str):
        req = SendScript.Request()
        req.id = str(int(time.time() * 1000) % 100000)
        req.script = script
        self.send_script_client.call_async(req)


# =========================================================
# Main
# =========================================================

def main():
    rclpy.init()
    node = HybridContourFollowingNode()
    try:
        rclpy.spin(node)
    finally:
        node.send_script("PVTExit()")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()