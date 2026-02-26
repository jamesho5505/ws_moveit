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
# Fuzzy Logic Controller (Simplified - Kf Only)
# =========================================================

class FuzzyKfController:
    """
    Mamdani 模糊推論系統 + 重心法（COG）去模糊化

    ── 輸入隸屬度函數（force_error 與 force_error_rate 共用）────────────
      NB: trapezoidal (-10, -10, -1.5, -0.5)
      NS: triangular  (-1.5, -0.5, 0)
      ZE: triangular  (-0.3, 0, 0.3)
      PS: triangular  (0, 0.5, 1.5)
      PB: trapezoidal (0.5, 1.5, 10, 10)

    ── 輸出 Kf 隸屬度函數（mm/N，範圍 [0.050, 0.100]）──────────────────
      VS: trapezoidal (0.050, 0.050, 0.055, 0.063)
      S : triangular  (0.055, 0.063, 0.073)
      M : triangular  (0.063, 0.073, 0.085)
      B : triangular  (0.073, 0.085, 0.095)
      VB: trapezoidal (0.085, 0.093, 0.100, 0.100)

    ── 規則表（行: force_error / 列: force_error_rate）─────────────────
               fer_NB  fer_NS  fer_ZE  fer_PS  fer_PB
      fe_NB  [  VS,     VS,     VS,     VS,     VS  ]
      fe_NS  [  S,      S,      VS,     VS,     VS  ]
      fe_ZE  [  M,      M,      M,      S,      S   ]
      fe_PS  [  B,      B,      M,      M,      S   ]
      fe_PB  [  VB,     VB,     B,      B,      M   ]

    ── 推論方式 ─────────────────────────────────────────────────────────
      AND 運算子 : min
      激活方式   : Clipping（以激活度截切輸出 MF）
      聚合方式   : max（取各規則截切後的最大包絡）
      去模糊化   : 重心法 COG（數值積分）
                   Kf = Σ(x · μ_agg(x)) · Δx / Σ(μ_agg(x)) · Δx
    """

    # ── 輸出宇集離散化（解析度 0.0001 mm/N）────────────────────────────
    KF_MIN      = 0.050
    KF_MAX      = 0.100
    KF_STEP     = 0.0001
    KF_UNIVERSE = np.arange(0.050, 0.100 + KF_STEP, KF_STEP)

    # ── 規則表（行: fe / 列: fer）────────────────────────────────────────
    #            fer_NB  fer_NS  fer_ZE  fer_PS  fer_PB
    # RULE_TABLE = [
    #     # fe_NB
    #     ['VS', 'VS', 'VS', 'VS', 'VS'],
    #     # fe_NS
    #     ['S',  'S',  'VS', 'VS', 'VS'],
    #     # fe_ZE
    #     ['M',  'M',  'M',  'S',  'S' ],
    #     # fe_PS
    #     ['B',  'B',  'M',  'M',  'S' ],
    #     # fe_PB
    #     ['VB', 'VB', 'B',  'B',  'M' ],
    # ]
    RULE_TABLE = [
        # fe_NB
        ['S',  'S',  'S',  'M',  'B'],
        # fe_NS
        ['S',  'S',  'S', 'S', 'S'],
        # fe_ZE
        ['M',  'M',  'M',  'S',  'S' ],
        # fe_PS
        ['B',  'B',  'M',  'M',  'S' ],
        # fe_PB
        ['B', 'B', 'B',  'B',  'M' ],
    ]

    INPUT_LABELS  = ['NB', 'NS', 'ZE', 'PS', 'PB']
    OUTPUT_LABELS = ['VS', 'S',  'M',  'B',  'VB']

    def __init__(self):
        # 預先計算輸出 MF 矩陣，避免重複運算（shape: 5 × len(KF_UNIVERSE)）
        self._output_mf = self._precompute_output_mf()

    # ── 輸入隸屬度函數 ───────────────────────────────────────────────────

    def _tri(self, x, a, b, c):
        """三角形 MF"""
        if x <= a or x >= c:
            return 0.0
        elif a < x <= b:
            return (x - a) / (b - a)
        else:
            return (c - x) / (c - b)
        

    def _trap(self, x, a, b, c, d):
        """梯形 MF（支援飽和端 a==b 或 c==d）"""
        if x < a or x > d:
            return 0.0
        elif b <= x <= c:
            return 1.0
        elif x < b:
            return (x - a) / (b - a) if b != a else 1.0
        else:
            return (d - x) / (d - c) if d != c else 1.0

    def _fuzzify_force_error_input(self, value):
        """
        輸入模糊化（force_error 與 force_error_rate 共用相同 MF）
        返回 dict {語言值: 隸屬度}
        """
        return {
            'NB': self._trap(value, -5.0, -5.0, -1.5, -0.5),
            'NS': self._tri (value, -1.5, -0.5,  0.0),
            'ZE': self._tri (value, -0.3,  0.0,  0.3),
            'PS': self._tri (value,  0.0,  0.5,  1.5),
            'PB': self._trap(value,  0.5,  1.5,  5.0,  5.0),
        }

    def _fuzzify_force_error_rate_input(self, value):
        """
        輸入模糊化（force_error 與 force_error_rate 共用相同 MF）
        返回 dict {語言值: 隸屬度}
        """
        return {
            'NB': self._trap(value, -30.0, -30.0, -8.0, -3.0),
            'NS': self._tri (value, -8.0, -3.0,  0.0),
            'ZE': self._tri (value, -1.5,  0.0,  1.5),
            'PS': self._tri (value,  0.0,  3.0,  8.0),
            'PB': self._trap(value,  3.0,  8.0,  30.0,  30.0),
        }


    # ── 輸出 MF（向量化，預先計算）──────────────────────────────────────

    def _precompute_output_mf(self):
        """
        預計算輸出 Kf 的各語言值隸屬度向量
        shape: {label: np.ndarray(len(KF_UNIVERSE))}

        VS: trapezoidal (0.050, 0.050, 0.055, 0.063)
        S : triangular  (0.055, 0.063, 0.073)
        M : triangular  (0.063, 0.073, 0.085)
        B : triangular  (0.073, 0.085, 0.095)
        VB: trapezoidal (0.085, 0.093, 0.100, 0.100)
        """
        u = self.KF_UNIVERSE
        mf = {}
        # VS
        mf['VS'] = np.array([self._trap(x, 0.050, 0.050, 0.055, 0.063) for x in u])
        # S
        mf['S']  = np.array([self._tri (x, 0.055, 0.063, 0.073) for x in u])
        # M
        mf['M']  = np.array([self._tri (x, 0.063, 0.073, 0.085) for x in u])
        # B
        mf['B']  = np.array([self._tri (x, 0.073, 0.085, 0.095) for x in u])
        # VB
        mf['VB'] = np.array([self._trap(x, 0.085, 0.093, 0.100, 0.100) for x in u])
        
        return mf

    # ── 推論 + 聚合 ──────────────────────────────────────────────────────

    def _infer_and_aggregate(self, fe_mf, fer_mf):
        """
        Mamdani 推論：
          1. 計算各規則激活度 w = min(fe_mf, fer_mf)
          2. Clipping：截切輸出 MF → min(w, MF(x))
          3. 聚合：max 運算，得到聚合模糊集 μ_agg(x)

        返回: np.ndarray，shape=(len(KF_UNIVERSE),)
        """
        aggregated = np.zeros(len(self.KF_UNIVERSE))

        for i, fe_label in enumerate(self.INPUT_LABELS):
            for j, fer_label in enumerate(self.INPUT_LABELS):
                activation = min(fe_mf[fe_label], fer_mf[fer_label])
                if activation <= 0.0:
                    continue
                out_label   = self.RULE_TABLE[i][j]
                clipped_mf  = np.minimum(activation, self._output_mf[out_label])
                aggregated  = np.maximum(aggregated, clipped_mf)

        return aggregated

    # ── COG 去模糊化 ────────────────────────────────────────────────────

    def _cog(self, aggregated):
        """
        重心法 (Center of Gravity)
        Kf = Σ(x · μ_agg(x)) / Σ(μ_agg(x))
        """
        denominator = np.sum(aggregated)
        if denominator < 1e-9:
            return 0.073  # 無激活時回傳 M 的中心值
        return float(np.sum(self.KF_UNIVERSE * aggregated) / denominator)

    # ── 主計算入口 ───────────────────────────────────────────────────────

    def compute_kf(self, force_error, force_error_rate):
        """
        Mamdani 模糊推論 + COG 去模糊化

        Step 1: 模糊化兩個輸入
        Step 2: 規則推論 + Clipping + 聚合（max）
        Step 3: COG 去模糊化

        返回: Kf (mm/N)
        """
        # Step 1
        fe_mf  = self._fuzzify_force_error_input(force_error)
        fer_mf = self._fuzzify_force_error_rate_input(force_error_rate)

        # Step 2
        aggregated = self._infer_and_aggregate(fe_mf, fer_mf)
        
        # Step 3
        kf_value = self._cog(aggregated)
        return float(np.clip(kf_value, self.KF_MIN, self.KF_MAX))



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

        # ---------------- Fuzzy Controller (Kf Only) ----------------
        self.fuzzy = FuzzyKfController()
        
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
        # 發布模糊參數
        self.pub_fuzzy_params = self.create_publisher(
            Float64MultiArray, "/force_control/fuzzy_kf", 10
        )
        
        self.log_counter = 0

        # ---------------- Timing ----------------
        self.ctrl_hz = 60.0
        self.dt = 1.0 / self.ctrl_hz
        self.delay_steps = 1
        self.last_time = time.time()

        # ---------------- Robot state ----------------
        self.has_pose = False
        self.pose = np.zeros(6)
        self.target_pose = np.zeros(6)
        self.target_vel = np.zeros(6)

        # ---------------- Force ----------------
        self.force = np.zeros(3)
        self.force_base = np.zeros(3)
        self.reference_force = 3.5
        self.max_force_limit = 12.0
        self.force_threshold_on = 1.80
        self.force_threshold_off = 0.50
        self.is_in_hybrid = False
        self.force_established = False
        self.force_stable_time = 0.0
        self.force_stable_required = 0.05
        # ---------------- Force error filtering (for fer) ----------------
        self.filtered_force_error = 0.0
        self.previous_filtered_force_error = 0.0
        # 0~1, 越大越跟得快但越吃噪聲；60Hz 下建議 0.15~0.3
        self.force_error_lowpass_alpha = 0.2
        # 配合你的 fer universe / MF 範圍
        self.force_error_rate_limit = 30.0

        # ---------------- Admittance (Normal) ----------------
        self.Kf_base = 0.070          # 基礎增益（會被模糊邏輯調整）
        self.Kf = self.Kf_base        # 當前使用的值
        self.Df = 0.000               # 阻尼（不調整）
        
        # Kf 範圍限制
        self.Kf_min = 0.05            # Kf 最小值 (mm/N)
        self.Kf_max = 0.10            # Kf 最大值 (mm/N)
        
        self.force_deadband = 0.3
        self.max_normal_step = 0.005
        self.force_correction = np.zeros(2)

        # ---------------- Trajectory ----------------
        self.traj_speed = 0.40        # 固定速度（不調整）
        self.traj_dir = np.array([math.sqrt(3)/2, 0.5])
        self.traj_dir /= np.linalg.norm(self.traj_dir)
        self.traj_length = 50.0
        self.traj_time = 0.0
        self.traj_start = None
        self.min_z_position = 45.0

        # ---------------- Contact frame ----------------
        self.ref_tangent = None
        
        # ---------------- Force tracking ----------------
        self.prev_force_error = 0.0
        self.prev_force_n = 0.0

        # ---------------- Start ----------------
        self.send_script("PVTEnter(1)")
        self.timer = self.create_timer(1/60, self.control_loop)
        
        msg_control_parameters = Float64MultiArray()
        msg_control_parameters.data = [
            float(self.ctrl_hz), 
            float(self.reference_force), 
            float(self.force_threshold_on), 
            float(self.force_threshold_off), 
            float(self.max_force_limit), 
            float(self.Kf_base), 
            float(self.Df), 
            float(self.force_deadband), 
            float(self.max_normal_step), 
            float(self.traj_speed), 
            float(self.traj_dir[0]),
            float(self.traj_dir[1]),
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

        cur_pos = self.pose[:2]
        AB = predicted_pos - cur_pos
        delta_L1 = np.linalg.norm(AB)
        delta_L2 = np.dot(AB, t_hat)

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

        # ---------------- Force ----------------
        f_xy = self.force[:2]
        force_n_raw = np.linalg.norm(f_xy)
        force_n = max(force_n_raw, 0.0)  # 正向接觸力（絕對值）
        

        # ==================================================
        # Normal & Tangent
        # ==================================================
        if force_n > 0.3:
            f_hat_raw = f_xy / force_n_raw
            
            expected_n = np.array([-self.traj_dir[1], self.traj_dir[0]])
            
            if not hasattr(self, 'smooth_n_hat'): 
                self.smooth_n_hat = expected_n
            # alpha = 0.05  # 平滑參數，越大越跟得快但越吃噪聲
            alpha = 0.02  # 平滑參數，越大越跟得快但越吃噪聲
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
        # Hysteresis Logic
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

        # === Safety ===
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

        # ==================================================
        # ★ Mamdani 模糊推論 + COG → Kf
        # ==================================================
        force_error      = self.reference_force - force_n
        # 先對 force_error 低通濾波（避免微分放大噪聲）
        self.filtered_force_error = (
            (1.0 - self.force_error_lowpass_alpha) * self.filtered_force_error
            + self.force_error_lowpass_alpha * force_error
        )

        # 再對濾波後的誤差做微分
        force_error_rate = (
            (self.filtered_force_error - self.previous_filtered_force_error) / self.dt
            if self.dt > 1e-6 else 0.0
        )
        self.previous_filtered_force_error = self.filtered_force_error

        # 限幅到 MF universe 範圍，避免偶發尖峰直接打飽和
        force_error_rate = float(np.clip(force_error_rate, -self.force_error_rate_limit, self.force_error_rate_limit))

        if self.is_in_hybrid:
            self.Kf = self.fuzzy.compute_kf(force_error, force_error_rate)

            # 發布模糊參數用於監控
            # [0] force_error  [1] force_error_rate  [2] Kf
            # [3] force_n      [4] reference_force
            msg_fuzzy = Float64MultiArray()
            msg_fuzzy.data = [
                float(force_error),         # [0] 力誤差 (N)
                float(force_error_rate),    # [1] 力誤差變化率 (N/s)
                float(self.Kf),             # [2] COG 輸出 Kf (mm/N)
                float(force_n),             # [3] 當前接觸力 (N)
                float(self.reference_force) # [4] 目標力 (N)
            ]
            self.pub_fuzzy_params.publish(msg_fuzzy)
        else:
            # 位置模式下使用基礎值
            self.Kf = self.Kf_base

        if self.is_in_hybrid:
            # ============================================
            # Hybrid Mode
            # ============================================
            force_error_dot = (force_error - self.prev_force_error) / self.dt
            self.prev_force_error = force_error

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
                self.traj_start = cur_pos.copy()
                self.traj_time = 0.0
                self.get_logger().info("Force established, trajectory reset")

            if force_stable:
                self.traj_time += self.dt
                reference_pos, reference_vel = self.generate_trajectory()
                projected_pos, corrected_data = self.project_onto_traj(reference_pos, reference_vel, t_hat)
                new_reference_pos = projected_pos + self.force_correction
            else:
                new_reference_pos = cur_pos + self.force_correction

        else:
            # ============================================
            # Position Mode
            # ============================================
            self.force_stable_time = 0.0
            self.force_established = False
            self.force_correction = np.zeros(2)
            self.traj_time += self.dt
            new_reference_pos, _ = self.generate_trajectory()

        position_error = new_reference_pos - cur_pos

        # ---------- Publish debug info ----------
        now = self.get_clock().now().to_msg()
        
        if self.is_in_hybrid and force_stable:
            reference_pos, _ = self.generate_trajectory()
        else:
            reference_pos = cur_pos
        
        msg_reference = PointStamped()
        msg_reference.header.stamp = now
        msg_reference.header.frame_id = "base"
        msg_reference.point.x = float(reference_pos[0])
        msg_reference.point.y = float(reference_pos[1])
        msg_reference.point.z = 0.0
        self.pub_desired_point.publish(msg_reference)
        
        msg_corrected = PointStamped()
        msg_corrected.header.stamp = now
        msg_corrected.header.frame_id = "base"
        msg_corrected.point.x = float(new_reference_pos[0])
        msg_corrected.point.y = float(new_reference_pos[1])
        msg_corrected.point.z = 0.0
        self.pub_corrected_point.publish(msg_corrected)

        msg_corrected_data = Float64MultiArray()
        msg_corrected_data.data = list(corrected_data)
        self.pub_corrected_data.publish(msg_corrected_data)

        # ---------- Command ----------
        self.target_pose[:2] = new_reference_pos
        self.target_pose[2] = self.pose[2]
        self.target_pose[3:] = self.pose[3:]

        self.target_vel[:2] = self.traj_dir * self.traj_speed
        self.target_vel[2:] = 0.0

        self.log_counter += 1
        if self.log_counter % 20 == 0:
            force_stable = self.force_stable_time >= self.force_stable_required
            stable_str = "STABLE" if force_stable else "UNSTABLE"
            
            if self.is_in_hybrid:
                self.get_logger().info(
                    f"{state}({stable_str}) | t={self.traj_time:.2f}s | "
                    f"F={force_n:.2f}N (err={force_error:.2f}) | "
                    f"Kf={self.Kf:.5f} | "
                    f"fer={force_error_rate:.2f} N/s | "
                    f"pos_err=[{position_error[0]:.3f}, {position_error[1]:.3f}] mm"
                )
            else:
                self.get_logger().info(
                    f"{state} | t={self.traj_time:.2f}s | "
                    f"pos_err=[{position_error[0]:.3f}, {position_error[1]:.3f}] mm"
                )

        self.send_pvt()

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