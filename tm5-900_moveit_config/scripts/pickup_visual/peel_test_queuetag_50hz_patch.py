#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
peel_test.py + 視覺定位整合版
----------------------------------------------------------------------
新增功能（來自 peeling_node_visual.py）：
  - 移動到觀測位姿 VIEW_POSE_CPP
  - 使用 YOLO OBB + RealSense + TF2 估測夾取點與 TCP 姿態
  - 移動到翹取點正上方（Z = VIEW_POSE_CPP["z"]）後停止

執行方式（視覺定位測試）：
  python3 peel_test_vision.py
  → visual_only=True（預設），不進入 PVT 控制

若日後要整合完整剝牙流程：
  node = HybridContourFollowingNode(visual_only=False)
  → 視覺定位完成後自動啟動 PVTEnter + control_loop
"""

import argparse
import math
import os
import sys
import time
import threading
from pathlib import Path
from typing import Dict, Optional, Tuple
import traceback
import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
import tf_transformations
import torch
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from tm_msgs.msg import FeedbackState
from tm_msgs.msg import StaResponse  
from tm_msgs.srv import SendScript
from geometry_msgs.msg import WrenchStamped, PointStamped
from scipy.spatial.transform import Rotation as Rotation
from tf2_ros import Buffer, TransformListener

# ── YOLO（支援本地 ultralytics repo）──────────────────────────
current_script_directory = Path(__file__).resolve().parent
ultralytics_repository_root = current_script_directory / "ultralytics"
local_ultralytics_init_file = ultralytics_repository_root / "ultralytics" / "__init__.py"
if local_ultralytics_init_file.exists():
    sys.path.insert(0, str(ultralytics_repository_root))

from ultralytics import YOLO

try:
    import open3d as o3d
except ImportError:
    o3d = None


# =========================================================
# ── 視覺定位常數 ──────────────────────────────────────────
# =========================================================

# DEFAULT_MODEL_PATH = (
#     '/home/jamesho5055/ws_moveit/src/tm5-900_moveit_config/'
#     'scripts/pickup_visual/yolosbest.pt'
# )
DEFAULT_MODEL_PATH = (
    '/home/jamesho5055/ws_moveit/src/tm5-900_moveit_config/'
    'scripts/pickup_visual/final_obb.pt'
)

VIEW_POSE_CPP = {
    'x': -140.0, 'y': -520.0, 'z': 190.0,
    'rx': 180.0, 'ry': 0.0,   'rz': 90.0,
}
VIEW_MOVE_SPEED           = 500
VIEW_MOVE_ACCELERATION_MS = 10

CAMERA_WIDTH              = 640
CAMERA_HEIGHT             = 480
CAMERA_FPS                = 30
NEEDED_VISION_SAMPLES     = 10
MAX_VISION_FRAMES         = 300
DEPTH_PATCH_RADIUS_PIXELS = 4
VISION_SAMPLE_INTERVAL_S  = 0.1
SHOW_VISION_WINDOW        = True
VISION_WINDOW_NAME        = 'YOLO OBB Detection'
TCP_X_OFFSET_MM = 6.50
TCP_Y_OFFSET_MM = -10.50
PEELING_X_OFFSET = -10.0
PEELING_RX_OFFSET = -160.0
PEELING_RY_OFFSET = 0.0
PEELING_Z_OFFSET =  89.0 #94.0 

CALIBRATION_Z_OFFSET_MM   = 0.0
# PCA_ROI_RADIUS_PIXELS      = 10
# PCA_MIN_RAW_POINTS         = 120
# PCA_DEPTH_BAND_MM          = 8.0
# PCA_NEIGHBOR_RADIUS_MM     = 3.0
# PCA_MIN_NEIGHBOR_POINTS    = 80
# PCA_USE_SOR                = False
# PCA_SOR_NEIGHBORS          = 20
# PCA_SOR_STD_RATIO          = 2.0
# PCA_PLANE_OUTLIER_MM       = 1.0
# PCA_PLANE_REFINE_ITERATIONS = 2
# FORCE_TCP_NORMAL_DOWNWARD  = True
AXIS_DRAW_LENGTH_MM        = 30.0

ARRIVAL_TOL_MM             = 0.5    # 等待到位容許誤差 [mm]


# =========================================================
# Fuzzy Logic Controller（不變）
# =========================================================

class FuzzyKfController:
    """
    Mamdani 模糊推論系統 + 重心法（COG）去模糊化
    """

    KF_MIN      = 0.005
    KF_MAX      = 0.020 #0.025
    KF_STEP     = 0.00005
    KF_UNIVERSE = np.arange(KF_MIN, KF_MAX + KF_STEP, KF_STEP)

    # RULE_TABLE = [
    #     ['S', 'S', 'S', 'M', 'B'],
    #     ['S', 'S', 'S', 'S', 'S'],
    #     ['M', 'M', 'M', 'S', 'S'],
    #     ['B', 'B', 'M', 'M', 'S'],
    #     ['B', 'B', 'B', 'B', 'M'],
    # ]
    RULE_TABLE = [
        ['B', 'M', 'M', 'S', 'S'],
        ['M', 'M', 'S', 'S', 'S'],
        ['M', 'S', 'S', 'S', 'M'],
        ['S', 'S', 'S', 'M', 'M'],
        ['S', 'S', 'M', 'M', 'B'],
    ]
    INPUT_LABELS = ['NB', 'NS', 'ZE', 'PS', 'PB']

    def __init__(self):
        self._output_mf = self._precompute_output_mf()

    def _tri(self, x, a, b, c):
        if x <= a or x >= c:
            return 0.0
        if a < x <= b:
            return (x - a) / (b - a)
        return (c - x) / (c - b)

    def _trap(self, x, a, b, c, d):
        if x < a or x > d:
            return 0.0
        if b <= x <= c:
            return 1.0
        if x < b:
            return (x - a) / (b - a) if b != a else 1.0
        return (d - x) / (d - c) if d != c else 1.0

    def _fuzzify_force_error_input(self, value):
        return {
            'NB': self._trap(value, -5.0, -5.0, -2.0, -0.8),
            'NS': self._tri (value, -2.0, -0.8,  0.0),
            'ZE': self._tri (value, -0.3,  0.0,  0.3),
            'PS': self._tri (value,  0.0,  0.8,  2.0),
            'PB': self._trap(value,  0.8,  2.0,  5.0,  5.0),
        }

    def _fuzzify_force_error_rate_input(self, value):
        return {
            'NB': self._trap(value, -30.0, -30.0, -8.0, -3.0),
            'NS': self._tri (value,  -8.0,  -3.0,  0.0),
            'ZE': self._tri (value,  -1.5,   0.0,  1.5),
            'PS': self._tri (value,   0.0,   3.0,  8.0),
            'PB': self._trap(value,   3.0,   8.0, 30.0, 30.0),
        }

    # def _fuzzify_force_error_input(self, value):
    #     # 調整重疊度，讓相鄰三角形完美交疊
    #     return {
    #         'NB': self._trap(value, -5.0, -5.0, -3.0, -0.8),
    #         'NS': self._tri (value, -3.0, -0.8,  0.0),
    #         'ZE': self._tri (value, -0.8,  0.0,  0.8), # 稍微拉寬穩態 ZE 的範圍，避免微小擾動太敏感
    #         'PS': self._tri (value,  0.0,  0.8,  3.0),
    #         'PB': self._trap(value,  0.8,  3.0,  5.0,  5.0),
    #     }

    # def _fuzzify_force_error_input(self, value):
    #     # 調整重疊度，讓相鄰三角形完美交疊
    #     return {
    #         'NB': self._trap(value, -5.0, -5.0, -3.0, -0.8),
    #         'NS': self._tri (value, -3.0, -0.8,  0.0),
    #         'ZE': self._tri (value, -0.8,  0.0,  0.8), # 稍微拉寬穩態 ZE 的範圍，避免微小擾動太敏感
    #         'PS': self._tri (value,  0.0,  0.8,  3.0),
    #         'PB': self._trap(value,  0.8,  3.0,  5.0,  5.0),
    #     }

    # def _fuzzify_force_error_rate_input(self, value):
    #     # 修正重點：將 NB 的飽和區往外推（-8 -> -20），並讓斜坡從 -2 開始就與 NS 交疊
    #     return {
    #         'NB': self._trap(value, -30.0, -30.0, -15.0, -5.0),
    #         'NS': self._tri (value, -15.0,  -5.0,   0.0),
    #         'ZE': self._tri (value,  -5.0,   0.0,   5.0), # 加強與 NS, PS 的重疊
    #         'PS': self._tri (value,   0.0,   5.0,  15.0),
    #         'PB': self._trap(value,   5.0,  15.0,  30.0, 30.0),
    #     }

    # def _precompute_output_mf(self):
    #     universe = self.KF_UNIVERSE
    #     return {
    #         # 1.5 mm/s FOLLOW_SURFACE 時，原本 B≈0.017 會讓法向修正太積極。
    #         # 先降低輸出增益，不改 rule table，保留模糊控制邏輯。
    #         'S' : np.array([self._tri (x, 0.0055, 0.0065, 0.0080) for x in universe]),
    #         'M' : np.array([self._tri (x, 0.0075, 0.0090, 0.0110) for x in universe]),
    #         'B' : np.array([self._tri (x, 0.0100, 0.0120, 0.0140) for x in universe]),
    #     }
    def _precompute_output_mf(self):
        universe = self.KF_UNIVERSE
        return {
            'S' : np.array([self._tri (x, 0.005, 0.008, 0.011) for x in universe]),
            'M' : np.array([self._tri (x, 0.008, 0.012, 0.016) for x in universe]),
            'B' : np.array([self._tri (x, 0.012, 0.016, 0.020) for x in universe]),
        }

    def _infer_and_aggregate(self, fe_membership, fer_membership):
        aggregated = np.zeros(len(self.KF_UNIVERSE))
        for fe_idx, fe_label in enumerate(self.INPUT_LABELS):
            for fer_idx, fer_label in enumerate(self.INPUT_LABELS):
                activation = min(fe_membership[fe_label], fer_membership[fer_label])
                if activation <= 0.0:
                    continue
                output_label = self.RULE_TABLE[fe_idx][fer_idx]
                clipped_mf = np.minimum(activation, self._output_mf[output_label])
                aggregated = np.maximum(aggregated, clipped_mf)
        return aggregated

    def _cog(self, aggregated):
        denominator = np.sum(aggregated)
        if denominator < 1e-9:
            return (self.KF_MIN + self.KF_MAX) / 2.0
        return float(np.sum(self.KF_UNIVERSE * aggregated) / denominator)

    def compute_kf(self, force_error, force_error_rate):
        fe_membership  = self._fuzzify_force_error_input(force_error)
        fer_membership = self._fuzzify_force_error_rate_input(force_error_rate)
        aggregated = self._infer_and_aggregate(fe_membership, fer_membership)
        return float(np.clip(self._cog(aggregated), self.KF_MIN, self.KF_MAX))


# =========================================================
# Main Node
# =========================================================

class HybridContourFollowingNode(Node):
    """
    修改說明（相對於原始 peel_test.py）：

    新增參數 visual_only（預設 True）：
      - True  → 只執行視覺定位測試（不進入 PVTEnter / control_loop）
      - False → 視覺定位完成後自動啟動 PVTEnter + control_loop（完整流程）

    新增的視覺定位能力全部來自 peeling_node_visual.py，
    所有原有力控邏輯、FuzzyKfController 均未修改。
    """

    PHASE_APPROACH         = 0
    PHASE_FORCE_REGULATION = 1
    PHASE_FOLLOW_SURFACE   = 2
    PHASE_TRENCH_TRAVERSE  = 3
    PHASE_DONE             = 4

    def __init__(
        self,
        visual_only: bool = True,
        model_path: str = DEFAULT_MODEL_PATH,
        speed_override_ratio: float = 0.05,
        pvt_duration_sec: float = 0.01,
        fast_actual_approach_speed_mm_s: float = 10.0,
        slow_actual_force_speed_mm_s: float = 0.5,
        follow_actual_speed_mm_s: float = 0.5,
        fast_max_command_lead_mm: float = 2.5,
        slow_max_command_lead_mm: float = 0.3,
        pvt_target_settle_error_mm: float = 0.05,
        logic_rate_hz: float = 100.0,
        pvt_buffer_limit: int = 3,
        hybrid_pvt_buffer_limit: int = 1,
        queue_tag_timeout_sec: float = 0.30,
    ):
        super().__init__("hybrid_contour_following_node")

        self.visual_only = visual_only

        # ---------------- Fuzzy Controller ----------------
        self.fuzzy = FuzzyKfController()

        # ---------------- ROS ----------------
        self.send_script_client = self.create_client(SendScript, "send_script")
        self.send_script_client.wait_for_service()

        self.create_subscription(WrenchStamped, "/ft_compensated", self.force_cb, 10)
        self.create_subscription(FeedbackState, "feedback_states", self.feedback_cb, 10)
        self.create_subscription(StaResponse, "/sta_response", self.tm_sta_response_cb, 10)

        self.pub_state            = self.create_publisher(String,            "/force_control/control_state",      10)
        self.pub_phase            = self.create_publisher(String,            "/force_control/control_phase",      10)
        self.pub_control_parameters = self.create_publisher(Float64MultiArray, "/force_control/control_parameters", 10)
        self.pub_desired_point    = self.create_publisher(PointStamped,      "/force_control/desired_point",      10)
        self.pub_corrected_point  = self.create_publisher(PointStamped,      "/force_control/corrected_point",    10)
        self.pub_corrected_data   = self.create_publisher(Float64MultiArray, "/force_control/corrected_data",     10)
        self.pub_target_point     = self.create_publisher(PointStamped,      "/force_control/target_point",       10)
        self.pub_control_data     = self.create_publisher(Float64MultiArray, "/force_control/control_data",       10)
        self.pub_fuzzy_params     = self.create_publisher(Float64MultiArray, "/force_control/fuzzy_kf",           10)

        self.log_counter = 0

        # ---------------- Timing / PVT under TMFlow speed override ----------------
        # TMFlow speed override 固定 5% 時：
        #   actual_speed ~= command_speed * speed_override_ratio
        #   actual_segment_time ~= pvt_duration_sec / speed_override_ratio
        self.speed_override_ratio = float(speed_override_ratio)
        self.pvt_duration_sec = float(pvt_duration_sec)
        self.ctrl_hz = float(logic_rate_hz)
        self.dt = 1.0 / self.ctrl_hz
        self.delay_steps = 1
        self.last_time = time.time()
        self.current_tag = 1          # 目前要發送的標籤編號 (1~15 循環)
        self.sent_tags_in_buffer = {}  # 紀錄哪些標籤已經送出，尚未收到完成通知

        # APPROACH 使用較大的 buffer，避免單一 QueueTag 延遲就讓接近動作停住。
        # 接觸後的力控使用較小 buffer，避免舊的力修正命令排隊造成振盪。
        self.position_pvt_buffer_limit = int(np.clip(pvt_buffer_limit, 1, 15))
        self.hybrid_pvt_buffer_limit = int(np.clip(hybrid_pvt_buffer_limit, 1, 15))
        self.buffer_limit = self.position_pvt_buffer_limit
        self.queue_tag_timeout_sec = float(queue_tag_timeout_sec)

        self.fast_actual_approach_speed_mm_s = float(fast_actual_approach_speed_mm_s)
        self.slow_actual_force_speed_mm_s = float(slow_actual_force_speed_mm_s)
        self.follow_actual_speed_mm_s = float(follow_actual_speed_mm_s)

        self.fast_command_approach_speed_mm_s = (
            self.fast_actual_approach_speed_mm_s / max(self.speed_override_ratio, 1e-6)
        )
        self.slow_command_force_speed_mm_s = (
            self.slow_actual_force_speed_mm_s / max(self.speed_override_ratio, 1e-6)
        )
        self.follow_command_speed_mm_s = (
            self.follow_actual_speed_mm_s / max(self.speed_override_ratio, 1e-6)
        )

        self.fast_approach_step_mm = self.fast_command_approach_speed_mm_s * self.pvt_duration_sec
        self.slow_force_step_mm = self.slow_command_force_speed_mm_s * self.pvt_duration_sec
        self.follow_step_mm = self.follow_command_speed_mm_s * self.pvt_duration_sec

        self.fast_max_command_lead_mm = float(fast_max_command_lead_mm)
        self.slow_max_command_lead_mm = float(slow_max_command_lead_mm)
        self.pvt_target_settle_error_mm = float(pvt_target_settle_error_mm)

        self.active_pvt_target_pose = None
        self.last_pvt_send_time = 0.0
        self.last_command_lead_mm = 0.0
        self.pvt_point_counter = 0
        self.script_sequence_number = 0
        self.pending_script_futures = []
        self.last_real_pvt_send_time = None

        # ---------------- Robot state ----------------
        self.has_pose             = False
        self.pose                 = np.zeros(6)
        self.target_pose          = np.zeros(6)
        self.target_vel           = np.zeros(6)
        self.rotation_tool_to_base = np.eye(3)
        self.approach_direction_xz = np.array([1.0, 0.0], dtype=float)
        self.done = False
        self.failed = False

        # ---------------- Trajectory ----------------
        self.traj_speed     = 1.0
        traj_angle_deg      = 80.0
        self.traj_dir = np.array([
            np.cos(np.radians(traj_angle_deg)),
            np.sin(np.radians(traj_angle_deg))
        ], dtype=float)
        self.traj_dir /= np.linalg.norm(self.traj_dir)
        self.traj_length = 15.0
        self.traj_time   = 0.0
        self.traj_start  = None

        # ---------------- Force ----------------
        self.force      = np.zeros(3)
        self.force_base = np.zeros(3)

        self.reference_force     = 4.0
        self.max_force_limit     = 50.0
        self.force_threshold_on  = 2.25
        self.force_threshold_off = 1.25
        self.force_xz_norm   = 0.0
        self.fz_base         = 0.0
        self.n_hat           = np.array([-self.traj_dir[1], self.traj_dir[0]], dtype=float)
        self.t_hat           = self.traj_dir.copy()
        self.force_error     = 0.0
        self.force_error_rate = 0.0
        self.contact_confirm_time = 0.0
        self.contact_confirm_required = 0.100
        self.contact_lost_time = 0.0
        self.contact_lost_required = 0.20

        # ---------------- Trench detection ----------------
        self.trench_fz_threshold    = -3.0
        self.trench_backoff_step_mm = 0.8
        self.trench_traj_start = None
        self.trench_traj_time  = 0.0
        self.traj_end_z        = None
        self.trench_record_start_time = None
        self.trench_record_duration_sec = 3.0

        # ---------------- Phase / hybrid ----------------
        self.phase          = self.PHASE_APPROACH
        self.is_in_hybrid   = False

        self.force_established    = False
        self.force_stable_time    = 0.0
        self.force_stable_required = 0.1

        # ---------------- Force error filtering ----------------
        self.filtered_force_error          = 0.0
        self.previous_filtered_force_error = 0.0
        self.force_error_lowpass_alpha     = 0.2
        self.force_error_rate_limit        = 30.0

        # ---------------- Admittance (P + I) ----------------
        self.Kf_base = 0.012
        self.Kf      = self.Kf_base
        self.Df      = 0.000
        self.Ki                   = 0.0015   # 0.01
        self.force_integral       = 0.0 
        self.force_integral_limit = 0.008  # 0.05

        self.force_deadband     = 0.2     # 0.3
        self.max_normal_step    = 0.01     # 0.2
        self.force_correction   = np.zeros(2)

        self.approach_speed_mm_s = self.fast_command_approach_speed_mm_s
        self.trench_actual_speed_mm_s = 100.0
        self.min_z_position = 88.50
        self.prev_force_error = 0.0

        # ── 視覺定位元件 ─────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.model_path   = model_path
        self.model        = YOLO(self.model_path)
        self.model_device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.model_device)

        self.camera_pipeline  = None
        self.camera_aligner   = None
        self.camera_started   = False
        self.calibration_z_offset_m = CALIBRATION_Z_OFFSET_MM / 1000.0

        self.detected_peel_pose = None   # 視覺定位結果（mm，dict）
        self.visual_test_done   = False

        # ---------------- Force thread ----------------
        self._stop_event    = threading.Event()
        self._force_thread  = threading.Thread(
            target=self._force_process_loop, daemon=True
        )
        self._force_thread.start()

        # ── 依模式啟動 ────────────────────────────────────────
        if self.visual_only:
            self.get_logger().info('[INIT] visual_only=True → 啟動視覺定位測試執行緒')
            threading.Thread(
                target=self._run_visual_test_sequence,
                daemon=True,
                name='visual_test',
            ).start()
        else:
            # 完整流程：視覺定位後自動開始力控
            self.get_logger().info('[INIT] visual_only=False → 啟動視覺定位後進入 PVT 控制')
            threading.Thread(
                target=self._run_visual_then_pvt,
                daemon=True,
                name='visual_then_pvt',
            ).start()

        # 發布控制參數（供 rqt 監視）
        msg = Float64MultiArray()
        msg.data = [
            float(self.ctrl_hz), float(self.reference_force),
            float(self.force_threshold_on), float(self.force_threshold_off),
            float(self.max_force_limit), float(self.Kf_base),
            float(self.Df), float(self.force_deadband),
            float(self.max_normal_step), float(self.traj_speed),
            float(self.traj_dir[0]), float(self.traj_dir[1]),
            float(self.traj_length), float(self.force_stable_required),
            float(self.trench_fz_threshold), float(self.trench_backoff_step_mm),
            float(self.Ki), float(self.force_integral_limit),
        ]
        self.pub_control_parameters.publish(msg)

        self.get_logger().info(
            '[PVT_5_PERCENT] parameters: '
            f'override={self.speed_override_ratio:.3f}, '
            f'pvt_duration={self.pvt_duration_sec:.4f}s, '
            f'fast_actual={self.fast_actual_approach_speed_mm_s:.3f}mm/s, '
            f'fast_command={self.fast_command_approach_speed_mm_s:.3f}mm/s, '
            f'fast_step={self.fast_approach_step_mm:.3f}mm, '
            f'slow_actual={self.slow_actual_force_speed_mm_s:.3f}mm/s, '
            f'slow_command={self.slow_command_force_speed_mm_s:.3f}mm/s, '
            f'slow_step={self.slow_force_step_mm:.3f}mm, '
            f'logic_rate={self.ctrl_hz:.1f}Hz, '
            f'position_buffer={self.position_pvt_buffer_limit}, '
            f'hybrid_buffer={self.hybrid_pvt_buffer_limit}, '
            f'queue_tag_timeout={self.queue_tag_timeout_sec:.2f}s'
        )

    # =========================================================
    # Callbacks
    # =========================================================

    def force_cb(self, msg: WrenchStamped):
        self.force[:] = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]

    def feedback_cb(self, msg: FeedbackState):
        p = msg.tool_pose
        self.pose[:] = [
            p[0] * 1000.0, p[1] * 1000.0, p[2] * 1000.0,
            p[3] * 180.0 / math.pi,
            p[4] * 180.0 / math.pi,
            p[5] * 180.0 / math.pi,
        ]
        if not self.has_pose:
            self.has_pose = True
            self.target_pose[:] = self.pose
            self.traj_start = np.array([self.pose[0], self.pose[2]], dtype=float)
        self.rotation_tool_to_base = Rotation.from_euler(
            'xyz', [self.pose[3], self.pose[4], self.pose[5]], degrees=True
        ).as_matrix()

    def tm_sta_response_cb(self, msg: StaResponse):
        if msg.subcmd.strip() != "01":
            return

        subdata_text = msg.subdata.strip()

        try:
            tag_text, status_text = subdata_text.split(",", 1)
            completed_tag_number = int(tag_text)
            tag_completed = status_text.strip().lower() == "true"
        except Exception:
            self.get_logger().warn(
                f"[QueueTag] cannot parse STA: subcmd={msg.subcmd}, subdata={msg.subdata}"
            )
            return

        if not tag_completed:
            return

        if completed_tag_number in self.sent_tags_in_buffer:
            del self.sent_tags_in_buffer[completed_tag_number]
        #     self.get_logger().info(
        #         f"[QueueTag] release tag={completed_tag_number}, "
        #         f"buffer={len(self.sent_tags_in_buffer)}/{self.buffer_limit}"
        #     )

    # =========================================================
    # Force processing thread（不變）
    # =========================================================

    def _force_process_loop(self):
        FORCE_CONTROL_HZ = 100.0
        interval = 1.0 / FORCE_CONTROL_HZ
        while not self._stop_event.is_set():
            t0 = time.time()

            force_base = self.rotation_tool_to_base @ self.force
            self.force_base = force_base

            f_xz = np.array([force_base[0], force_base[2]], dtype=float)
            force_xz_norm = float(np.linalg.norm(f_xz))
            self.force_xz_norm = force_xz_norm
            self.fz_base = float(force_base[2])

            if force_xz_norm > 1e-9:
                f_hat_raw = f_xz / force_xz_norm
            else:
                f_hat_raw = np.zeros(2)

            if force_xz_norm > 0.3:
                expected_n = np.array([-self.traj_dir[1], self.traj_dir[0]], dtype=float)
                if not hasattr(self, 'smooth_n_hat'):
                    self.smooth_n_hat = expected_n.copy()
                self.smooth_n_hat = 0.95 * self.smooth_n_hat + 0.05 * f_hat_raw
                self.smooth_n_hat /= max(np.linalg.norm(self.smooth_n_hat), 1e-9)
                n_hat = self.smooth_n_hat.copy()
                t_hat = np.array([n_hat[1], -n_hat[0]], dtype=float)
                if float(np.dot(t_hat, self.traj_dir)) < 0.0:
                    t_hat = -t_hat
            else:
                n_hat = np.array([-self.traj_dir[1], self.traj_dir[0]], dtype=float)
                t_hat = self.traj_dir.copy()

            self.n_hat = n_hat
            self.t_hat = t_hat

            raw_force_error = self.reference_force - force_xz_norm

            previous_force_error = self.force_error
            self.force_error = raw_force_error

            rate = (self.force_error - previous_force_error) / interval
            self.force_error_rate = float(np.clip(
                rate,
                -self.force_error_rate_limit,
                self.force_error_rate_limit
            ))


            # raw_force_error = self.reference_force - force_xz_norm
            # prev = self.filtered_force_error
            # self.filtered_force_error = 0.8 * prev + 0.2 * raw_force_error
            # rate = (self.filtered_force_error - self.previous_filtered_force_error) / interval
            # self.previous_filtered_force_error = self.filtered_force_error
            # self.force_error      = self.filtered_force_error
            # self.force_error_rate = float(np.clip(rate, -self.force_error_rate_limit,
            #                                             self.force_error_rate_limit))

            elapsed = time.time() - t0
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    # =========================================================
    # ── 視覺定位測試執行序列 ──────────────────────────────────
    # =========================================================

    def _run_visual_test_sequence(self):
        """
        visual_only=True 時執行：
          1. 等待 TF 和 pose 就緒
          2. 移到觀測位姿
          3. 執行視覺定位
          4. 移到翹取點正上方（Z = VIEW_POSE_CPP["z"]）停止
        """
        try:
            self.get_logger().info('[VISUAL_TEST] 等待機器人 pose 就緒...')
            for _ in range(200):
                if self.has_pose:
                    break
                time.sleep(0.05)
            if not self.has_pose:
                self.get_logger().error('[VISUAL_TEST] 超時：未收到 pose')
                return

            self.get_logger().info('[VISUAL_TEST] 切換 TCP → tcp_obb_208')
            self._send_script_blocking('ChangeTCP("tcp_obb_208")')

            self.get_logger().info('[VISUAL_TEST] 移動到觀測位姿...')
            self._send_ptp_blocking(VIEW_POSE_CPP,
                                    speed=VIEW_MOVE_SPEED,
                                    acc_ms=VIEW_MOVE_ACCELERATION_MS)
            self._wait_arrival_visual(VIEW_POSE_CPP)
            self.get_logger().info('[VISUAL_TEST] 已到達觀測位姿')

            self.get_logger().info('[VISUAL_TEST] 開始視覺定位...')
            peel_pose, _ = self._localize_peel_pose_with_vision()

            self.detected_peel_pose = dict(peel_pose)
            self.get_logger().info(
                '[VISUAL_TEST] ✅ 視覺定位完成\n'
                f'  peel_pose = x={peel_pose["x"]:.2f} mm, '
                f'y={peel_pose["y"]:.2f} mm, '
                f'z={peel_pose["z"]:.2f} mm\n'
                f'  rx={peel_pose["rx"]:.2f}°, '
                f'ry={peel_pose["ry"]:.2f}°, '
                f'rz={peel_pose["rz"]:.2f}°'
            )

            tcp_x_base = peel_pose['tcp_x_base']
            tcp_y_base = peel_pose['tcp_y_base']

            offset_vector = (
                TCP_X_OFFSET_MM * tcp_x_base
                + TCP_Y_OFFSET_MM * tcp_y_base
            )

            offset_vector[2] = 0.0  # 僅在 base XY 平面偏移


            peel_pose_with_offset = dict(peel_pose)
            peel_pose_with_offset['x'] += float(offset_vector[0])
            peel_pose_with_offset['y'] += float(offset_vector[1])
            peel_pose_with_offset['z'] += float(offset_vector[2])

            # 移到翹取點正上方（保持視角高度）
            above_peel = dict(peel_pose_with_offset)
            # above_peel['x'] = peel_pose['x'] - PEELING_X_OFFSET
            # above_peel['y'] = peel_pose['y'] 
            above_peel['z'] = VIEW_POSE_CPP['z']
            above_peel['rx'] = PEELING_RX_OFFSET
            above_peel['ry'] = PEELING_RY_OFFSET

            self.get_logger().info(
                '[VISUAL_TEST] 移動到翹取點正上方 '
                f'(x={above_peel["x"]:.2f}, y={above_peel["y"]:.2f}, z={above_peel["z"]:.2f})...'
            )
            self.get_logger().info('[VISUAL_TEST] 切換 TCP → tcp_peel')
            self._send_script_blocking('ChangeTCP("tcp_peel_new")')
            self._send_line_blocking(
                x=above_peel['x'], y=above_peel['y'], z=above_peel['z'],
                rx=above_peel['rx'], ry=above_peel['ry'], rz=above_peel['rz'],
                velocity=500, acceleration=50,
            )
            self._wait_arrival_visual(above_peel)
            self.get_logger().info('[VISUAL_TEST] ✅ 已到達翹取點正上方。')

            # peel_pose['x']   -= PEELING_X_OFFSET
            peel_pose = dict(peel_pose_with_offset)
            peel_pose['rx']   = PEELING_RX_OFFSET
            peel_pose['ry']   = PEELING_RY_OFFSET
            peel_pose['z']    = PEELING_Z_OFFSET  # 視覺定位的 Z 是基於觀測位姿的相對值，這裡直接套用實驗中測得的值 # 130 無接觸

            if peel_pose['z'] < self.min_z_position:
                peel_pose['z'] = self.min_z_position

            self.get_logger().info(
                '[VISUAL_TEST] 移動到翹取點'
                f'(x={peel_pose["x"]:.2f}, y={peel_pose["y"]:.2f}, z={peel_pose["z"]:.2f})...'
            )
            self._send_line_blocking(
                x=peel_pose['x'], y=peel_pose['y'], z=peel_pose['z'],
                rx=peel_pose['rx'], ry=peel_pose['ry'], rz=peel_pose['rz'],
                velocity=200, acceleration=100,
            )
            self._wait_arrival_visual(peel_pose)
            self.get_logger().info('[VISUAL_TEST] ✅ 已到達翹取點。')
            self.visual_test_done = True

        except Exception as exc:
            self.get_logger().error(f'[VISUAL_TEST] 發生錯誤: {exc}')
            self.get_logger().error(traceback.format_exc())
        finally:
            try:
                self.cam_stop()
            except Exception:
                pass
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

    def _run_visual_then_pvt(self):
        """
        visual_only=False 時執行：
          視覺定位完成後自動啟動 PVTEnter + control_loop timer
        """
        self._run_visual_test_sequence()
        if not self.visual_test_done:
            self.get_logger().error('[VISUAL→PVT] 視覺定位失敗，不啟動力控')
            return
        time.sleep(1.0)
        self.get_logger().info('[VISUAL→PVT] 啟動 PVTEnter + feedback-gated PVT control')

        # 進入 PVT 前，先同步 target_pose，避免第一個 PVTPoint 跳到舊位置
        self.target_pose[:] = self.pose
        self.target_vel[:] = 0.0
        self.active_pvt_target_pose = None
        self.last_pvt_send_time = 0.0
        self.last_command_lead_mm = 0.0
        self.pvt_point_counter = 0

        self.traj_start = np.array([self.pose[0], self.pose[2]], dtype=float)
        self.traj_time = 0.0
        self.last_time = time.time()

        self.phase = self.PHASE_APPROACH
        self.is_in_hybrid = False
        self.force_established = False
        self.force_stable_time = 0.0
        self.force_integral = 0.0
        self.prev_force_error = self.force_error

        time.sleep(0.1)

        self.send_script("PVTEnter(1)")
        self.timer = self.create_timer(1.0 / self.ctrl_hz, self.control_loop)

    # =========================================================
    # ── 阻塞式運動工具（供視覺執行緒使用）────────────────────
    # =========================================================

    def _send_script_blocking(self, script: str) -> bool:
        """以同步方式送出 TM script，等待 service 回應。"""
        req = SendScript.Request()
        req.id     = 'visual'
        req.script = script
        future = self.send_script_client.call_async(req)
        while not future.done():
            time.sleep(0.005)
        return True

    def _send_ptp_blocking(self, target_pose: Dict[str, float],
                           speed: int = 200, acc_ms: int = 150):
        script = (
            'PTP("CPP",'
            f'{target_pose["x"]:.4f},{target_pose["y"]:.4f},{target_pose["z"]:.4f},'
            f'{target_pose["rx"]:.4f},{target_pose["ry"]:.4f},{target_pose["rz"]:.4f},'
            f'{speed},{acc_ms},0,true)'
        )
        self._send_script_blocking(script)

    def _send_line_blocking(self, x: float, y: float, z: float,
                            rx: float, ry: float, rz: float,
                            velocity: int = 100, acceleration: int = 100,
                            blend: int = 0):
        target_str = f'{{{x:.3f},{y:.3f},{z:.3f},{rx:.3f},{ry:.3f},{rz:.3f}}}'
        script = f'Line("CPP", {target_str}, {velocity}, {acceleration}, {blend}, false)'
        self._send_script_blocking(script)

    def _wait_arrival_visual(self, target_pose: Dict[str, float],
                             tol: float = ARRIVAL_TOL_MM):
        """輪詢 self.pose 直到 XYZ 距離 < tol mm。"""
        tx, ty, tz = target_pose['x'], target_pose['y'], target_pose['z']
        while True:
            cx, cy, cz = self.pose[0], self.pose[1], self.pose[2]
            if math.sqrt((cx-tx)**2 + (cy-ty)**2 + (cz-tz)**2) < tol:
                return
            time.sleep(0.02)

    # =========================================================
    # ── 視覺定位核心（移植自 peeling_node_visual.py）──────────
    # =========================================================

    def cam_start(self):
        if self.camera_started:
            return
        self.camera_pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.bgr8, CAMERA_FPS)
        cfg.enable_stream(rs.stream.depth, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.z16, CAMERA_FPS)
        self.camera_pipeline.start(cfg)
        self.camera_aligner = rs.align(rs.stream.color)
        self.camera_started = True

    def cam_stop(self):
        if not self.camera_started:
            return
        try:
            self.camera_pipeline.stop()
        except Exception:
            pass
        self.camera_started = False
        self.camera_pipeline = None
        self.camera_aligner  = None

    def get_base_from_camera_transform(self) -> Optional[np.ndarray]:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                'base', 'camera_color_optical_frame', rclpy.time.Time()
            )
            t  = tf_msg.transform.translation
            q  = tf_msg.transform.rotation
            mat = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
            mat[:3, 3] = [t.x, t.y, t.z]
            return mat
        except Exception:
            return None

    def transform_from_tcp_obb_208_to_tcp_peel(self, obb_pose: Dict[str, float]) -> Dict[str, float]:
        peel_pose = dict(obb_pose)
        peel_pose['x'] += PEELING_X_OFFSET
        peel_pose['rx'] += PEELING_RX_OFFSET
        return peel_pose

    def backproject(self, px: float, py: float, depth_m: float, intrinsics) -> np.ndarray:
        cx = (px - intrinsics.ppx) / intrinsics.fx * depth_m
        cy = (py - intrinsics.ppy) / intrinsics.fy * depth_m
        return np.array([cx, cy, depth_m], dtype=float)

    def median_depth(self, depth_frame, cx: int, cy: int, r: int) -> float:
        h, w = depth_frame.get_height(), depth_frame.get_width()
        vals = []
        for py in range(max(cy-r, 0), min(cy+r, h-1)+1):
            for px in range(max(cx-r, 0), min(cx+r, w-1)+1):
                d = depth_frame.get_distance(px, py)
                if d > 0:
                    vals.append(d)
        return float(np.median(vals)) if vals else 0.0

    def _normalize(self, v: np.ndarray, eps: float = 1e-12) -> Optional[np.ndarray]:
        n = float(np.linalg.norm(v))
        return v / n if n >= eps else None

    def _angle_deg(self, a: np.ndarray, b: np.ndarray, eps: float = 1e-12) -> float:
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na < eps or nb < eps:
            return 180.0
        c = float(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0))
        return math.degrees(math.acos(c))

    def _project_base_to_pixel(self, pt_base: np.ndarray,
                                T: np.ndarray, intrinsics):
        R = T[:3, :3]
        t = T[:3, 3]
        pc = R.T @ (np.asarray(pt_base, float) - t)
        z  = float(pc[2])
        if z <= 1e-6:
            return None
        u = int(round(intrinsics.fx * float(pc[0]) / z + intrinsics.ppx))
        v = int(round(intrinsics.fy * float(pc[1]) / z + intrinsics.ppy))
        return u, v

    def _draw_tcp_axes(self, image, T, intrinsics, origin_base,
                       x_base, y_base, z_base):
        o_px = self._project_base_to_pixel(origin_base, T, intrinsics)
        if o_px is None:
            return
        L = AXIS_DRAW_LENGTH_MM / 1000.0
        for axis_vec, color, label in [
            (x_base, (0, 0, 255), 'X'),
            (y_base, (0, 255, 0), 'Y'),
            (z_base, (255, 0, 0), 'Z'),
        ]:
            ep = self._project_base_to_pixel(origin_base + L * axis_vec, T, intrinsics)
            if ep is not None:
                cv2.arrowedLine(image, o_px, ep, color, 2, tipLength=0.2)
                cv2.putText(image, label, ep, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.drawMarker(image, o_px, (255, 255, 255), cv2.MARKER_CROSS, 12, 2)

    def _obb_axis_vectors_base(self, obb: np.ndarray, depth_m: float,
                                intrinsics, T: np.ndarray):
        cx, cy, bw, bh, theta = map(float, obb[:5])
        if depth_m <= 0:
            return []
        s1 = max(8.0, 0.5 * max(abs(bw), abs(bh)))
        s2 = max(8.0, 0.5 * min(abs(bw), abs(bh)))
        R = T[:3, :3]
        result = []
        for name, pdir, step in [
            ('obb_axis_theta',        np.array([ math.cos(theta),  math.sin(theta)]), s1),
            ('obb_axis_theta_plus_90', np.array([-math.sin(theta), math.cos(theta)]), s2),
        ]:
            pp = self.backproject(cx + step * pdir[0], cy + step * pdir[1], depth_m, intrinsics)
            pm = self.backproject(cx - step * pdir[0], cy - step * pdir[1], depth_m, intrinsics)
            ax = self._normalize(R @ (pp - pm))
            if ax is not None:
                result.append((name, ax))
        return result

    def compute_tcp_rpy_sxyz_from_yvec_and_tooth2_obb(
        self,
        y_axis_vector_base: np.ndarray,
        oriented_box_tooth2: np.ndarray,
        depth_tooth2_meters: float,
        intrinsics,
        base_from_camera_transform: np.ndarray,
    ):
        tcp_y = self._normalize(np.asarray(y_axis_vector_base, float))
        if tcp_y is None:
            return None, None, {'reason': 'bad_y_axis'}

        down = np.array([0., 0., -1.], float)
        candidates = []
        for name, raw_ax in self._obb_axis_vectors_base(
                oriented_box_tooth2, depth_tooth2_meters,
                intrinsics, base_from_camera_transform):
            px_ax = self._normalize(raw_ax - float(np.dot(raw_ax, tcp_y)) * tcp_y)
            if px_ax is None:
                continue
            perp_q = abs(float(np.dot(raw_ax, tcp_y)))
            candidates.append((perp_q, name, px_ax, raw_ax))

        if candidates:
            candidates.sort(key=lambda x: x[0])
            _, sel_name, tcp_x, raw_ax = candidates[0]
            fallback = False
        else:
            tcp_x = self._normalize(np.cross(tcp_y, down))
            if tcp_x is None:
                tcp_x = self._normalize(np.cross(tcp_y, np.array([1., 0., 0.])))
            if tcp_x is None:
                return None, None, {'reason': 'cannot_build_x_axis'}
            sel_name, raw_ax, fallback = 'fallback_cross_y_down', None, True

        tcp_z = self._normalize(np.cross(tcp_x, tcp_y))
        if tcp_z is None:
            return None, None, {'reason': 'cannot_build_z_axis'}

        if float(np.dot(tcp_z, down)) < 0.0:
            tcp_x = -tcp_x
            tcp_z = -tcp_z

        tcp_x = self._normalize(np.cross(tcp_y, tcp_z))
        if tcp_x is None:
            return None, None, {'reason': 'cannot_rebuild_x_axis'}

        R4 = np.eye(4, dtype=float)
        R4[:3, 0] = tcp_x
        R4[:3, 1] = tcp_y
        R4[:3, 2] = tcp_z

        roll_r, pitch_r, yaw_r = tf_transformations.euler_from_matrix(R4, axes='sxyz')
        debug = {
            'reason': 'ok',
            'axis_name': sel_name,
            'fallback_used': bool(fallback),
            'dot_xy': float(np.dot(tcp_x, tcp_y)),
            'dot_z_down': float(np.dot(tcp_z, down)),
            'raw_obb_axis_base': raw_ax,
        }
        return (
            math.degrees(roll_r),
            math.degrees(pitch_r),
            math.degrees(yaw_r),
        ), R4, debug

    def compute_tcp_rpy_sxyz_from_xvec(
        self,
        x_axis_vector_base: np.ndarray,
    ):
        """
        New pickup-pose rule:
        - x_tcp is fixed by the vector tooth3 -> tooth2
        - z_tcp is forced as close as possible to base -Z
        - y_tcp = z_tcp × x_tcp
        """
        tcp_x = self._normalize(np.asarray(x_axis_vector_base, float))
        if tcp_x is None:
            return None, None, {'reason': 'bad_x_axis'}

        down = np.array([0.0, 0.0, -1.0], dtype=float)

        # 將 base -Z 投影到垂直 x_tcp 的平面，避免 z_tcp 與 x_tcp 不垂直
        tcp_z = down - float(np.dot(down, tcp_x)) * tcp_x
        tcp_z = self._normalize(tcp_z)

        if tcp_z is None:
            # 如果 x_tcp 幾乎平行 -Z，改用 base Y 當備援
            fallback_axis = np.array([0.0, 1.0, 0.0], dtype=float)
            tcp_z = fallback_axis - float(np.dot(fallback_axis, tcp_x)) * tcp_x
            tcp_z = self._normalize(tcp_z)

        if tcp_z is None:
            return None, None, {'reason': 'cannot_build_z_axis'}

        tcp_y = np.cross(tcp_z, tcp_x)
        tcp_y = self._normalize(tcp_y)
        if tcp_y is None:
            return None, None, {'reason': 'cannot_build_y_axis'}

        # 重新計算 z，確保三軸完全正交
        tcp_z = np.cross(tcp_x, tcp_y)
        tcp_z = self._normalize(tcp_z)
        if tcp_z is None:
            return None, None, {'reason': 'cannot_rebuild_z_axis'}

        R4 = np.eye(4, dtype=float)
        R4[:3, 0] = tcp_x
        R4[:3, 1] = tcp_y
        R4[:3, 2] = tcp_z

        roll_r, pitch_r, yaw_r = tf_transformations.euler_from_matrix(R4, axes='sxyz')

        debug = {
            'reason': 'ok',
            'axis_rule': 'x_tcp = tooth3_to_tooth2',
            'dot_x_down': float(np.dot(tcp_x, down)),
            'dot_z_down': float(np.dot(tcp_z, down)),
            'dot_xy': float(np.dot(tcp_x, tcp_y)),
            'dot_xz': float(np.dot(tcp_x, tcp_z)),
            'dot_yz': float(np.dot(tcp_y, tcp_z)),
        }

        return (
            math.degrees(roll_r),
            math.degrees(pitch_r),
            math.degrees(yaw_r),
        ), R4, debug

    def capture_one_detection(self):
        """
        抓一幀影像，跑 YOLO OBB，回傳：
          (pick_point_m, orientation_deg, vis_image, detection_output)
        任何步驟失敗時回傳 None。
        """
        T = self.get_base_from_camera_transform()
        if T is None:
            return None, None, None, None

        try:
            fs = self.camera_pipeline.wait_for_frames(timeout_ms=1000)
            afs = self.camera_aligner.process(fs)
            depth_frame = afs.get_depth_frame()
            color_frame = afs.get_color_frame()
            if not depth_frame or not color_frame:
                return None, None, None, None
            color_img = np.asanyarray(color_frame.get_data())
        except Exception:
            return None, None, None, None

        h, w = color_img.shape[:2]
        mid_y = h / 2.0
        intr  = color_frame.profile.as_video_stream_profile().intrinsics

        results = self.model.predict(color_img, verbose=False)
        if not results:
            return None, None, color_img, None

        res  = results[0]
        vis  = res.plot(labels=False, conf=False)

        if (getattr(res, 'obb', None) is None
                or getattr(res.obb, 'xywhr', None) is None
                or len(res.obb.xywhr) == 0):
            return None, None, vis, None

        obbs = res.obb.xywhr.detach().cpu().numpy()
        ctrs = obbs[:, :2]

        # 過濾 tooth-s8ie 類別
        if getattr(res.obb, 'cls', None) is not None:
            cls_ids = res.obb.cls.detach().cpu().numpy().astype(int)
            tooth_cls = None
            for cid, cname in self.model.names.items():
                if str(cname).strip().lower() == 'tooth-s8ie':
                    tooth_cls = int(cid)
                    break
            if tooth_cls is not None:
                valid = np.where(cls_ids == tooth_cls)[0]
                if len(valid) == 0:
                    return None, None, vis, None
                obbs = obbs[valid]
                ctrs = ctrs[valid]

        n = len(obbs)
        if n < 2:
            return None, None, vis, None

        top_idx = [i for i in range(n) if ctrs[i, 1] < mid_y]
        bot_idx = [i for i in range(n) if ctrs[i, 1] >= mid_y]

        # 3D 反投影每個偵測
        base_pts   = [None] * n
        base_zs    = [None] * n
        cam_depths = [None] * n
        valid_ids  = []
        for i in range(n):
            px, py = map(float, ctrs[i, :2])
            d = self.median_depth(depth_frame, int(px), int(py), DEPTH_PATCH_RADIUS_PIXELS)
            if d <= 0:
                continue
            pc = self.backproject(px, py, d, intr)
            pb = (T[:3, :3] @ pc) + T[:3, 3]
            pb[2] += self.calibration_z_offset_m
            base_pts[i]   = pb
            base_zs[i]    = float(pb[2])
            cam_depths[i] = float(d)
            valid_ids.append(i)

        if not valid_ids:
            return None, None, vis, None

        # v_top = [i for i in top_idx if i in valid_ids]
        # v_bot = [i for i in bot_idx if i in valid_ids]
        # top_sorted = sorted(v_top, key=lambda i: float(ctrs[i, 0]))
        # bot_sorted = sorted(v_bot, key=lambda i: float(ctrs[i, 0]))

        # POS = 1  # 選第 2 顆（從左到右）

        # top_ok = len(top_sorted) > POS
        # bot_ok = len(bot_sorted) > POS

        # if not top_ok and not bot_ok:
        #     return None, None, vis, None

        # if top_ok and not bot_ok:
        #     sel_region, sel_sorted = 'TOP', top_sorted
        # elif bot_ok and not top_ok:
        #     sel_region, sel_sorted = 'BOT', bot_sorted
        # else:
        #     ti = top_sorted[POS]
        #     bi = bot_sorted[POS]
        #     if base_zs[ti] > base_zs[bi]:
        #         sel_region, sel_sorted = 'TOP', top_sorted
        #     else:
        #         sel_region, sel_sorted = 'BOT', bot_sorted

        # if len(sel_sorted) <= POS + 1:
        #     return None, None, vis, None

        # 只使用畫面中心線下方的牙齒作為下排
        bottom_detection_indices = [
            detection_index
            for detection_index in range(n)
            if ctrs[detection_index, 1] >= mid_y and detection_index in valid_ids
        ]

        bottom_sorted_indices = sorted(
            bottom_detection_indices,
            key=lambda detection_index: float(ctrs[detection_index, 0])
        )

        POS = 1  # 選下排第 2 顆（從左到右）

        # 需要至少有第 2 顆和第 3 顆，因為 tooth2 是翹取中心，tooth3 用來決定方向
        if len(bottom_sorted_indices) <= POS + 1:
            return None, None, vis, None

        sel_region = 'BOT'
        sel_sorted = bottom_sorted_indices

        tooth2_idx = sel_sorted[POS]
        tooth3_idx = sel_sorted[POS + 1]

        pick_pt  = base_pts[tooth2_idx]
        neig_pt  = base_pts[tooth3_idx]
        dep_m    = cam_depths[tooth2_idx]

        if pick_pt is None or neig_pt is None or dep_m is None:
            return None, None, vis, None

        # y_vec = pick_pt - neig_pt   # tooth3 → tooth2

        # tcp_ori, tcp_R4, ori_debug = self.compute_tcp_rpy_sxyz_from_yvec_and_tooth2_obb(
        #     y_axis_vector_base=y_vec,
        #     oriented_box_tooth2=obbs[tooth2_idx],
        #     depth_tooth2_meters=dep_m,
        #     intrinsics=intr,
        #     base_from_camera_transform=T,
        # )

        x_vec = pick_pt - neig_pt   # tooth3 → tooth2
        tcp_ori, tcp_R4, ori_debug = self.compute_tcp_rpy_sxyz_from_xvec(
            x_axis_vector_base=x_vec,
)
        if tcp_ori is None or tcp_R4 is None:
            return None, None, vis, None

        rx_deg, ry_deg, rz_deg = tcp_ori
        t2cx, t2cy = ctrs[tooth2_idx]
        t3cx, t3cy = ctrs[tooth3_idx]

        cv2.circle(vis, (int(t2cx), int(t2cy)), 7, (0, 255, 0), -1)
        cv2.circle(vis, (int(t3cx), int(t3cy)), 7, (255, 0, 0), -1)
        cv2.line(vis, (int(t3cx), int(t3cy)), (int(t2cx), int(t2cy)), (0, 255, 255), 2)
        cv2.drawMarker(vis, (int(round(t2cx)), int(round(t2cy))),
                       (0, 0, 255), cv2.MARKER_CROSS, 18, 2)

        tcp_x = tcp_R4[:3, 0].copy()
        tcp_y = tcp_R4[:3, 1].copy()
        tcp_z = tcp_R4[:3, 2].copy()
        self._draw_tcp_axes(vis, T, intr,
                            origin_base=pick_pt,
                            x_base=tcp_x, y_base=tcp_y, z_base=tcp_z)

        cv2.putText(vis, f'REGION={sel_region} | n={len(sel_sorted)} | center=tooth2',
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(vis,
                    f'P=({pick_pt[0]:.3f},{pick_pt[1]:.3f},{pick_pt[2]:.3f}) m',
                    (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(vis,
                    f'Euler sxyz: rx={rx_deg:.1f}, ry={ry_deg:.1f}, rz={rz_deg:.1f}',
                    (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        det_out = {
            'region': sel_region,
            'count': len(sel_sorted),
            'pick_point_base_meters': pick_pt,
            'tcp_orientation_sxyz_degrees': (rx_deg, ry_deg, rz_deg),
            'z_tcp_base': tcp_z,
            'x_tcp_base': tcp_x,
            'y_tcp_base': tcp_y,
        }
        return pick_pt, (rx_deg, ry_deg, rz_deg), vis, det_out

    def _localize_peel_pose_with_vision(self) -> Tuple[Dict[str, float], Dict[str, float]]:
        """
        累積 NEEDED_VISION_SAMPLES 幀有效樣本，回傳 (peel_pose_mm, lift_pose_mm)。
        """
        valid_count   = 0
        pick_pt_m     = None
        pick_ori_deg  = None
        tcp_z_base    = None
        last_vis      = None
        pt_list = []
        ori_list = []
        tcp_x_list = []
        tcp_y_list = []
        tcp_z_list = []

        self.cam_start()
        try:
            for _ in range(MAX_VISION_FRAMES):
                pt, ori, vis, det = self.capture_one_detection()

                if vis is not None:
                    last_vis = vis
                    if SHOW_VISION_WINDOW:
                        cv2.imshow(VISION_WINDOW_NAME, vis)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break

                if pt is not None and ori is not None:
                    valid_count += 1
                    rx, ry, rz = ori
                    pt_list.append(pt)
                    ori_list.append(ori)
                    self.get_logger().info(
                        f'[VISION] sample {valid_count}/{NEEDED_VISION_SAMPLES}: '
                        f'x={pt[0]:.4f} y={pt[1]:.4f} z={pt[2]:.4f} m | '
                        f'sxyz=({rx:.2f},{ry:.2f},{rz:.2f})°'
                    )
                    # pick_pt_m    = pt
                    # pick_ori_deg = ori
                    if det:
                        if 'x_tcp_base' in det:
                            tcp_x_list.append(det['x_tcp_base'])
                        if 'y_tcp_base' in det:
                            tcp_y_list.append(det['y_tcp_base'])
                        if 'z_tcp_base' in det:
                            tcp_z_list.append(det['z_tcp_base'])
                            tcp_z_base = det['z_tcp_base']
                    if valid_count >= NEEDED_VISION_SAMPLES:
                        break
                    time.sleep(VISION_SAMPLE_INTERVAL_S)
        finally:
            self.cam_stop()
            if SHOW_VISION_WINDOW:
                if last_vis is not None:
                    cv2.imshow('YOLO OBB - LAST', last_vis)
                    cv2.waitKey(100)
                cv2.destroyAllWindows()

        # if pick_pt_m is None or pick_ori_deg is None:
        if len(pt_list) < NEEDED_VISION_SAMPLES:
            raise RuntimeError('視覺定位失敗：有效樣本數不足')

        # rx, ry, rz = pick_ori_deg
        pick_pt_m = np.median(np.array(pt_list), axis=0)
        ori_array = np.array(ori_list)
        rx, ry, rz = np.median(ori_array, axis=0)
        if len(tcp_x_list) > 0 and len(tcp_y_list) > 0:
            tcp_x_base = np.mean(np.array(tcp_x_list), axis=0)
            tcp_x_base = tcp_x_base / max(np.linalg.norm(tcp_x_base), 1e-9)

            tcp_y_base = np.mean(np.array(tcp_y_list), axis=0)
            tcp_y_base = tcp_y_base / max(np.linalg.norm(tcp_y_base), 1e-9)
        else:
            raise RuntimeError('視覺定位失敗：TCP 軸向量不足')
        peel_pose = {
            'x': float(pick_pt_m[0] * 1000.0),
            'y': float(pick_pt_m[1] * 1000.0),
            'z': float(pick_pt_m[2] * 1000.0),
            'rx': float(rx), 'ry': float(ry), 'rz': float(rz),
            'tcp_x_base': tcp_x_base,
            'tcp_y_base': tcp_y_base,
        }

        # 計算抬升位姿（供日後使用）
        up = np.array([0., 0., 1.], float)
        if tcp_z_base is not None:
            nz = float(np.linalg.norm(tcp_z_base))
            if nz > 1e-9:
                up = -tcp_z_base / nz

        lift_pt_m = np.asarray(pick_pt_m, float) + 60.0 / 1000.0 * up
        lift_pose = {
            'x': float(lift_pt_m[0] * 1000.0),
            'y': float(lift_pt_m[1] * 1000.0),
            'z': float(lift_pt_m[2] * 1000.0),
            'rx': float(rx), 'ry': float(ry), 'rz': float(rz),
        }
        return peel_pose, lift_pose

    # =========================================================
    # Trajectory / Projection / Phase helpers（不變）
    # =========================================================

    def generate_trajectory(self):
        s = min(self.traj_speed * self.traj_time, self.traj_length)
        pos = self.traj_start + self.traj_dir * s
        vel = self.traj_dir * self.traj_speed
        return pos, vel

    def project_onto_traj(self, reference_pos, reference_vel, t_hat):
        predicted_pos = reference_pos + self.delay_steps * reference_vel * self.dt
        cur_pos = np.array([self.pose[0], self.pose[2]], dtype=float)
        v2p = predicted_pos - cur_pos
        dL1 = np.linalg.norm(v2p)
        dL2 = max(float(np.dot(v2p, t_hat)), 0.0)
        cos_t = float(np.clip(dL2 / dL1 if dL1 > 1e-9 else 0.0, -1.0, 1.0))
        corrected_data = [dL1, dL2, math.degrees(math.acos(cos_t))]
        return cur_pos + dL2 * t_hat, corrected_data

    
    def _traj_speed_scale(self, force_error):
        if force_error > 1.2:
            return 0.0
        if force_error > 0.8:
            return 0.4
        if force_error > 0.5:
            return 0.7
        if force_error > 0.4:
            return 0.9

        if force_error < -1.3:
            return 0.0
        if force_error < -1.0:
            return 0.4
        if force_error < -0.6:
            return 0.7
        if force_error < -0.3:
            return 0.9

        return 1.0

    def _phase_name(self):
        return {
            self.PHASE_APPROACH:         'APPROACH',
            self.PHASE_FORCE_REGULATION: 'FORCE_REGULATION',
            self.PHASE_FOLLOW_SURFACE:   'FOLLOW_SURFACE',
            self.PHASE_TRENCH_TRAVERSE:  'TRENCH_TRAVERSE',
            self.PHASE_DONE:             'DONE',
        }.get(self.phase, 'UNKNOWN')

    def _set_phase(self, new_phase: int):
        if self.phase != new_phase:
            self.phase = new_phase
            # 切換 phase 後，下一個 PVTPoint 以目前 feedback 為準，避免被上一個 phase 的 target 卡住。
            self.active_pvt_target_pose = None
            self.target_pose[:] = self.pose
            self.target_vel[:] = 0.0
            self.get_logger().info(f'=== Phase → {self._phase_name()} ===')

    def _reset_contact_state(self, cur_pos: np.ndarray):
        self.is_in_hybrid      = False
        self.force_established = False
        self.force_stable_time = 0.0
        self.traj_time         = 0.0
        self.traj_start        = cur_pos.copy()
        self.force_integral    = 0.0

    def _return_to_view_after_trench(self, backoff_target, line_script, leave_script):
        self.get_logger().info('[TRENCH] Exit PVT')

        self.send_script("PVTExit()")
        time.sleep(0.2)

        self.send_script("StopAndClearBuffer(0)")
        time.sleep(0.2)

        self.get_logger().info('[TRENCH] backoff')
        self.send_script(backoff_target)
        time.sleep(0.2)

        self.get_logger().info('[TRENCH] trench traverse')
        self.send_script(line_script)
        time.sleep(0.2)

        self.get_logger().info('[TRENCH] leave')
        self.send_script(leave_script)
        time.sleep(0.2)

        self.get_logger().info('[TRENCH] ChangeTCP tcp_obb_208')
        self.send_script('ChangeTCP("tcp_obb_208")')
        time.sleep(0.2)

        view_script = (
            'PTP("CPP",'
            f'{VIEW_POSE_CPP["x"]:.4f},'
            f'{VIEW_POSE_CPP["y"]:.4f},'
            f'{VIEW_POSE_CPP["z"]:.4f},'
            f'{VIEW_POSE_CPP["rx"]:.4f},'
            f'{VIEW_POSE_CPP["ry"]:.4f},'
            f'{VIEW_POSE_CPP["rz"]:.4f},'
            f'{VIEW_MOVE_SPEED},'
            f'{VIEW_MOVE_ACCELERATION_MS},'
            '0,true)'
        )

        self.get_logger().info('[TRENCH] send VIEW_POSE_CPP')
        self.send_script(view_script)

        # 這裡可以等 feedback 到位，但不要用 service blocking
        self._wait_arrival_visual(VIEW_POSE_CPP)

        self.get_logger().info('[TRENCH] 已回到觀察姿態')
        self._set_phase(self.PHASE_DONE)
        self.done = True

    # =========================================================
    # Control Loop（不變，visual_only 模式下不會被呼叫）
    # =========================================================

    def control_loop(self):
        if not self.has_pose:
            return
        if self.phase == self.PHASE_DONE:
            return

        current_time = time.time()
        dt_measured  = current_time - self.last_time
        self.last_time = current_time
        self.dt = max(dt_measured, 1.0 / self.ctrl_hz)

        cur_pos = np.array([self.pose[0], self.pose[2]], dtype=float)
        corrected_data = np.zeros(3)
        # trench_traj_speed = self.traj_speed * 5.0
        trench_traj_speed = self.trench_actual_speed_mm_s

        force_xz_norm = self.force_xz_norm
        fz_base       = self.fz_base
        n_hat         = self.n_hat.copy()
        t_hat         = self.t_hat.copy()
        force_error      = self.force_error
        force_error_rate = self.force_error_rate

        f_xz = np.array([self.force_base[0], self.force_base[2]], dtype=float)
        normal_force_raw = float(np.dot(f_xz, n_hat))
        normal_force     = max(normal_force_raw, 0.0)

        if force_xz_norm > self.max_force_limit:
            self.get_logger().error(f'Force limit: {force_xz_norm:.2f}N')
            self.send_script("PVTExit()")
            self.timer.cancel()
            return

        if self.pose[2] < self.min_z_position:
            self.get_logger().error(f'Z limit: {self.pose[2]:.2f}mm')
            self.send_script("PVTExit()")
            self.timer.cancel()
            return

        if self.phase == self.PHASE_APPROACH:
            self.is_in_hybrid = False
            # if force_xz_norm > self.force_threshold_on:
            #     self.is_in_hybrid = True
            #     self._reset_contact_state(cur_pos)
            #     self.force_stable_time = 0.0
            #     self._set_phase(self.PHASE_FORCE_REGULATION)
            if force_xz_norm > self.force_threshold_on:
                self.contact_confirm_time += self.dt
            else:
                self.contact_confirm_time = 0.0

            if self.contact_confirm_time >= self.contact_confirm_required:
                self.contact_confirm_time = 0.0
                self.is_in_hybrid = True
                self._reset_contact_state(cur_pos)
                self.force_stable_time = 0.0
                self._set_phase(self.PHASE_FORCE_REGULATION)

        elif self.phase in (self.PHASE_FORCE_REGULATION, self.PHASE_FOLLOW_SURFACE):
            self.is_in_hybrid = True
            # if force_xz_norm < self.force_threshold_off:
            #     self._reset_contact_state(cur_pos)
            #     self._set_phase(self.PHASE_APPROACH)
            if force_xz_norm < self.force_threshold_off:
                self.contact_lost_time += self.dt
            else:
                self.contact_lost_time = 0.0

            if self.contact_lost_time >= self.contact_lost_required:
                self.contact_lost_time = 0.0
                self._reset_contact_state(cur_pos)
                self._set_phase(self.PHASE_APPROACH)

        elif self.phase == self.PHASE_TRENCH_TRAVERSE:
            self.is_in_hybrid = False

        if self.is_in_hybrid:
            # self.Kf = 0.015
            self.Kf = self.fuzzy.compute_kf(force_error, force_error_rate)
            msg_fuzzy = Float64MultiArray()
            msg_fuzzy.data = [
                float(force_error), float(force_error_rate),
                float(self.Kf), float(force_xz_norm), float(self.reference_force),
            ]
            self.pub_fuzzy_params.publish(msg_fuzzy)
        else:
            self.Kf = self.Kf_base

        # 力量與安全判斷維持高頻，但 PVTPoint 不再每次 control_loop 都送；
        # 必須等上一個目標接近後，或超過合理等待時間，才產生下一個命令。
        # if not self.ready_to_send_next_pvt():
        #     return
        # if self.ready_to_send_next_pvt():
        #     self.send_pvt()

        force_stable  = False
        reference_pos = cur_pos.copy()
        traj_scale    = 0.0

        # if self.phase == self.PHASE_APPROACH:
        #     self.force_stable_time = 0.0
        #     self.force_established = False
        #     self.force_correction  = np.zeros(2)
        #     self.traj_time += self.dt
        #     new_reference_pos = cur_pos + np.array([self.approach_speed_mm_s * self.dt, 0.0])
        if self.phase == self.PHASE_APPROACH:
            # 未接觸前快速接近：
            # 例：TMFlow 5%、真實 10 mm/s -> command 200 mm/s，
            # pvt_duration=0.01 s -> 每次 PVTPoint 前進 2 mm。
            # previous_target_pos = np.array([self.target_pose[0], self.target_pose[2]], dtype=float)
            # approach_direction_xz = np.array([self.rotation_tool_to_base[1,0], self.rotation_tool_to_base[2,0]], dtype=float)
            # approach_direction_xz /= max(np.linalg.norm(approach_direction_xz), 1e-9)
            # self.approach_direction_xz = approach_direction_xz.copy()
            # new_reference_pos = previous_target_pos + approach_direction_xz * self.fast_approach_step_mm
            previous_target_pos_xyz = np.array(
                [self.target_pose[0], self.target_pose[1], self.target_pose[2]],
                dtype=float
            )

            approach_direction_xyz = np.array([
                self.rotation_tool_to_base[0, 1],
                self.rotation_tool_to_base[1, 1],
                0.0,
            ], dtype=float)

            approach_direction_xyz /= max(np.linalg.norm(approach_direction_xyz), 1e-9)

            new_reference_pos_xyz = previous_target_pos_xyz + approach_direction_xyz * self.fast_approach_step_mm

            target_pose_temp = self.pose.copy()
            target_pose_temp[0] = float(new_reference_pos_xyz[0])
            target_pose_temp[1] = float(new_reference_pos_xyz[1])
            target_pose_temp[2] = float(new_reference_pos_xyz[2])
            target_pose_temp = self.limit_command_lead(
                target_pose_temp,
                max_command_lead_mm=self.fast_max_command_lead_mm,
            )
            new_reference_pos = np.array([target_pose_temp[0], target_pose_temp[2]], dtype=float)

            self.target_vel[0] = float(approach_direction_xyz[0] * self.fast_command_approach_speed_mm_s)
            self.target_vel[1] = float(approach_direction_xyz[1] * self.fast_command_approach_speed_mm_s)
            self.target_vel[2] = 0.0
            self.traj_time += self.estimated_actual_segment_time()

        elif self.phase in (self.PHASE_FORCE_REGULATION, self.PHASE_FOLLOW_SURFACE):
            force_error_dot      = (force_error - self.prev_force_error) / self.dt
            self.prev_force_error = force_error

            if abs(force_error) <= self.force_deadband:
                self.force_stable_time += self.dt
            else:
                self.force_stable_time = 0.0
            force_stable = self.force_stable_time >= self.force_stable_required

            self.force_integral += self.Ki * force_error * self.dt
            self.force_integral  = float(np.clip(self.force_integral,
                                                  -self.force_integral_limit,
                                                   self.force_integral_limit))

            delta = (self.Kf * force_error
                     + self.force_integral)
            delta = float(np.clip(delta, -self.max_normal_step, self.max_normal_step))
            self.force_correction = -n_hat * delta

            if force_stable and not self.force_established:
                self.force_established = True
                self.traj_start = cur_pos.copy()
                self.traj_time  = 0.0
                traj_end = self.traj_start + self.traj_dir * self.traj_length
                self.traj_end_z = float(traj_end[1])
                self.get_logger().info(
                    f'Force established. traj_start=[{self.traj_start[0]:.2f},{self.traj_start[1]:.2f}]'
                )

            if self.phase == self.PHASE_FORCE_REGULATION:
                if force_stable:
                    self._set_phase(self.PHASE_FOLLOW_SURFACE)
                new_reference_pos = cur_pos + self.force_correction
            else:
                traj_scale = self._traj_speed_scale(force_error)
                # self.traj_time += self.dt * traj_scale
                actual_step_mm = self.follow_actual_speed_mm_s * self.current_min_pvt_interval_sec() * traj_scale
                # reference_pos, reference_vel = self.generate_trajectory()
                # projected_pos, corrected_data = self.project_onto_traj(reference_pos, reference_vel, t_hat)
                # new_reference_pos = projected_pos + self.force_correction
                reference_pos = cur_pos + t_hat * actual_step_mm
                new_reference_pos = reference_pos + self.force_correction

                if fz_base < self.trench_fz_threshold:
                    backoff_pos = np.array([cur_pos[0] - self.trench_backoff_step_mm, cur_pos[1]])
                    self.trench_traj_start = backoff_pos
                    self.trench_traj_time  = 0.0
                    self.is_in_hybrid      = False
                    self.force_correction  = np.zeros(2)
                    self._set_phase(self.PHASE_TRENCH_TRAVERSE)
                    self.pub_phase.publish(String(data=self._phase_name()))
                    self.trench_record_start_time = time.time()
                    self.get_logger().info(
                        f'Trench detected fz={fz_base:.2f}N → TRENCH_TRAVERSE'
                    )
                    trench_end_pos = backoff_pos + self.traj_dir * self.traj_length

                    target_x = float(trench_end_pos[0])
                    target_y = float(self.pose[1])
                    target_z = float(trench_end_pos[1])
                    target_rx = float(self.pose[3])
                    target_ry = float(self.pose[4])
                    target_rz = float(self.pose[5])

                    backoff_target = (
                        f'Line("CPP", '
                        f'{{{backoff_pos[0]:.3f},{self.pose[1]:.3f},{backoff_pos[1]:.3f},'
                        f'{self.pose[3]:.3f},{self.pose[4]:.3f},{self.pose[5]:.3f}}}, '
                        f'50, 100, 0, false)'
                    )

                    line_script = (
                        f'Line("CPP", '
                        f'{{{target_x:.3f},{target_y:.3f},{target_z:.3f},'
                        f'{target_rx:.3f},{target_ry:.3f},{target_rz:.3f}}}, '
                        f'100, 100, 0, false)'
                    )

                    leave_script = (
                        f'Line("CPP", '
                        f'{{{target_x - 15.0 :.3f},{target_y:.3f},{target_z:.3f},'
                        f'{target_rx:.3f},{target_ry:.3f},{target_rz:.3f}}}, '
                        f'100, 100, 0, false)'
                    )

                    self.timer.cancel()

                    threading.Thread(
                        target=self._return_to_view_after_trench,
                        args=(backoff_target, line_script, leave_script),
                        daemon=True
                    ).start()

                    return

                    # self.send_script("PVTExit()")
                    # time.sleep(0.1)
                    # # self.send_script("StopAndClearBuffer()")
                    # # time.sleep(0.1)
                    # self.send_script(backoff_target)
                    # time.sleep(0.1)
                    # self.send_script(line_script)
                    # time.sleep(0.1)
                    # self.send_script(leave_script)
                    # time.sleep(0.5)
                    # # self.get_logger().info('[VISUAL_TEST] 切換 TCP → tcp_obb_208')
                    # self._send_script_blocking('ChangeTCP("tcp_obb_208")')
                    # self.timer.cancel()
                    # time.sleep(0.1)
                    # self.get_logger().info('[VISUAL_TEST] 移動到觀測位姿...')
                    # view_script = (
                    #     'PTP("CPP",'
                    #     f'{VIEW_POSE_CPP["x"]:.4f},'
                    #     f'{VIEW_POSE_CPP["y"]:.4f},'
                    #     f'{VIEW_POSE_CPP["z"]:.4f},'
                    #     f'{VIEW_POSE_CPP["rx"]:.4f},'
                    #     f'{VIEW_POSE_CPP["ry"]:.4f},'
                    #     f'{VIEW_POSE_CPP["rz"]:.4f},'
                    #     f'{VIEW_MOVE_SPEED},'
                    #     f'{VIEW_MOVE_ACCELERATION_MS},'
                    #     '0,true)'
                    # )

                    # self.get_logger().info('[TRENCH] send VIEW_POSE_CPP')
                    # self.send_script(view_script)

                    # # 這裡可以等 feedback 到位，但不要用 service blocking
                    # self._wait_arrival_visual(VIEW_POSE_CPP)
                    # self.get_logger().info('已到達觀測位姿')
                    # self._set_phase(self.PHASE_DONE)
                    
                    return

        elif self.phase == self.PHASE_TRENCH_TRAVERSE:
            self.is_in_hybrid       = False
            self.force_correction   = np.zeros(2)
            new_reference_pos = cur_pos.copy()
        #     self.trench_traj_time  += self.dt
        #     # trench_traj_speed = self.traj_speed * 2.0
        #     # trench_traj_speed = self.trench_command_speed_mm_s
        #     displacement = self.traj_dir * trench_traj_speed * self.pvt_duration_sec
        #     new_reference_pos = cur_pos + displacement
        #     # displacement = self.traj_dir * trench_traj_speed * self.trench_traj_time
        #     # new_reference_pos = self.trench_traj_start + displacement

            # if self.traj_end_z is not None and cur_pos[1] >= self.traj_end_z:
            #     self._set_phase(self.PHASE_DONE)
            #     self.get_logger().info('Trajectory end reached. Stopping.')
            #     self.send_script("PVTExit()")
            #     self.timer.cancel()
            #     return
            if self.trench_record_start_time is None:
                self.trench_record_start_time = time.time()

            self.trench_traj_time = time.time() - self.trench_record_start_time

            if self.trench_traj_time >= self.trench_record_duration_sec:
                self._set_phase(self.PHASE_DONE)
                self.get_logger().info('TRENCH_TRAVERSE recording finished. Stopping.')
                self.send_script("PVTExit()")
                self.timer.cancel()
                return
        else:
            new_reference_pos = cur_pos.copy()

        position_error = new_reference_pos - cur_pos

        now = self.get_clock().now().to_msg()
        self.pub_state.publish(String(data='HYBRID_CONTROL' if self.is_in_hybrid else 'POSITION'))
        self.pub_phase.publish(String(data=self._phase_name()))

        def _pt(x, z):
            m = PointStamped()
            m.header.stamp = now
            m.header.frame_id = 'base'
            m.point.x, m.point.z = float(x), float(z)
            return m

        self.pub_desired_point.publish(_pt(reference_pos[0], reference_pos[1]))
        self.pub_corrected_point.publish(_pt(new_reference_pos[0], new_reference_pos[1]))

        msg_cd = Float64MultiArray()
        msg_cd.data = list(corrected_data)
        self.pub_corrected_data.publish(msg_cd)

        trench_flag = 1.0 if fz_base < self.trench_fz_threshold else 0.0
        msg_ctrl = Float64MultiArray()
        msg_ctrl.data = [
            float(self.phase), float(self.is_in_hybrid),
            float(force_xz_norm), float(normal_force), float(normal_force_raw),
            float(force_error), float(force_error_rate), float(self.Kf),
            float(n_hat[0]), float(n_hat[1]),
            float(t_hat[0]), float(t_hat[1]),
            float(self.force_correction[0]), float(self.force_correction[1]),
            float(position_error[0]), float(position_error[1]),
            float(trench_flag), float(fz_base),
            float(self.trench_traj_time), float(traj_scale),
            float(self.force_integral),
        ]
        self.pub_control_data.publish(msg_ctrl)

        target_pose_temp = self.pose.copy()

        if self.phase == self.PHASE_APPROACH:
            target_pose_temp[0] = float(new_reference_pos_xyz[0])
            target_pose_temp[1] = float(new_reference_pos_xyz[1])
            target_pose_temp[2] = float(new_reference_pos_xyz[2])
        else:
            target_pose_temp[0] = float(new_reference_pos[0])
            target_pose_temp[1] = float(self.pose[1])
            target_pose_temp[2] = float(new_reference_pos[1])

        target_pose_temp[3:] = self.pose[3:]

        # target_pose_temp = self.pose.copy()
        # target_pose_temp[0]  = float(new_reference_pos[0])
        # target_pose_temp[1]  = float(self.pose[1])
        # target_pose_temp[2]  = float(new_reference_pos[1])
        # target_pose_temp[3:] = self.pose[3:]

        if self.phase == self.PHASE_APPROACH:
            target_pose_temp = self.limit_command_lead_xy(
                target_pose_temp,
                max_command_lead_mm=self.fast_max_command_lead_mm,
            )
        else:
            target_pose_temp = self.limit_command_lead(
                target_pose_temp,
                max_command_lead_mm=self.slow_max_command_lead_mm,
            )

        self.target_pose[:] = target_pose_temp

        if self.phase == self.PHASE_APPROACH:
            # APPROACH 使用固定速度命令，不用 (target - feedback) / pvt_duration。
            # 50 Hz + QueueTag gating 下 feedback 會落後，若除以 0.001 s，
            # 0.3~0.4 mm 的 command lead 會被放大成 300~400 mm/s，造成接近不穩定。
            # approach_velocity_direction_xz = self.approach_direction_xz.copy()
            # approach_velocity_direction_xz /= max(np.linalg.norm(approach_velocity_direction_xz), 1e-9)
            # self.target_vel[0] = float(approach_velocity_direction_xz[0] * self.fast_command_approach_speed_mm_s)
            # self.target_vel[2] = float(approach_velocity_direction_xz[1] * self.fast_command_approach_speed_mm_s)
            self.target_vel[0] = float(approach_direction_xyz[0] * self.fast_command_approach_speed_mm_s)
            self.target_vel[1] = float(approach_direction_xyz[1] * self.fast_command_approach_speed_mm_s)
            self.target_vel[2] = 0.0
        elif self.phase == self.PHASE_FORCE_REGULATION:
            self.target_vel[0] = float((self.target_pose[0] - self.pose[0]) / max(self.pvt_duration_sec, 1e-6))
            self.target_vel[2] = float((self.target_pose[2] - self.pose[2]) / max(self.pvt_duration_sec, 1e-6))
        elif self.phase == self.PHASE_FOLLOW_SURFACE:
            actual_follow_dir = t_hat.copy()
            self.target_vel[0] = float(actual_follow_dir[0] * self.follow_command_speed_mm_s * traj_scale)
            self.target_vel[2] = float(actual_follow_dir[1] * self.follow_command_speed_mm_s * traj_scale)
        
        # elif self.phase == self.PHASE_FOLLOW_SURFACE and self.is_in_hybrid:
        #     self.target_vel[0] = float(self.traj_dir[0] * self.follow_command_speed_mm_s * traj_scale)
        #     self.target_vel[2] = float(self.traj_dir[1] * self.follow_command_speed_mm_s * traj_scale)
        # elif self.phase == self.PHASE_TRENCH_TRAVERSE:
        #     self.target_vel[0] = float(self.traj_dir[0] * trench_traj_speed)
        #     self.target_vel[2] = float(self.traj_dir[1] * trench_traj_speed)
        else:
            self.target_vel[0] = 0.0
            self.target_vel[2] = 0.0
        if self.phase != self.PHASE_APPROACH:
            self.target_vel[1] = 0.0
        self.target_vel[3:] = 0.0

        self.log_counter += 1
        if self.log_counter % 20 == 0:
            self.get_logger().info(
                f"{self._phase_name()} | {'HYBRID' if self.is_in_hybrid else 'POS'} | "
                f"t={self.traj_time:.2f}s | scale={traj_scale:.2f} | "
                f"F={force_xz_norm:.2f}N (err={force_error:.2f}) | "
                f"Kf={self.Kf:.5f} | I={self.force_integral:.5f} | "
                f"pos=[{cur_pos[0]:.2f},{cur_pos[1]:.2f}]"
            )

        # self.send_pvt()
        if self.ready_to_send_next_pvt():
            self.send_pvt()

        self.pub_target_point.publish(_pt(self.target_pose[0], self.target_pose[2]))

    # =========================================================
    # PVT feedback-gating helpers
    # =========================================================

    def estimated_actual_segment_time(self):
        return self.pvt_duration_sec / max(self.speed_override_ratio, 1e-6)

    def current_pvt_target_error_mm(self):
        if self.active_pvt_target_pose is None:
            return 0.0

        actual_xz = np.array([self.pose[0], self.pose[2]], dtype=float)
        target_xz = np.array(
            [self.active_pvt_target_pose[0], self.active_pvt_target_pose[2]],
            dtype=float,
        )
        return float(np.linalg.norm(target_xz - actual_xz))

    def current_buffer_limit(self):
        if self.phase in (self.PHASE_FORCE_REGULATION, self.PHASE_FOLLOW_SURFACE):
            return self.hybrid_pvt_buffer_limit
        return self.position_pvt_buffer_limit

    def clear_stale_queue_tags(self):
        if not self.sent_tags_in_buffer:
            return

        now = time.time()
        for tag_number, sent_time in list(self.sent_tags_in_buffer.items()):
            elapsed_time = now - sent_time
            if elapsed_time > self.queue_tag_timeout_sec:
                self.get_logger().warn(
                    f"[QueueTag] timeout release tag={tag_number}, "
                    f"elapsed={elapsed_time:.3f}s, "
                    f"buffer_before={len(self.sent_tags_in_buffer)}/{self.current_buffer_limit()}"
                )
                del self.sent_tags_in_buffer[tag_number]

    def ready_to_send_next_pvt(self):
        # 避免單一 QueueTag 沒回來時，buffer 永久卡住。
        self.clear_stale_queue_tags()

        if len(self.sent_tags_in_buffer) < self.current_buffer_limit():
            return True
        return False

    def current_min_pvt_interval_sec(self):
        # PVT 命令由 control_loop 送出；因此步距應依照 control_loop / PVT send rate 計算。
        # 例如 logic_rate_hz=50 時，每次 PVTPoint 對應約 0.02 s，而不是舊版固定 0.2 s。
        return 1.0 / max(self.ctrl_hz, 1e-6)


    def limit_command_lead(self, target_pose, max_command_lead_mm):
        limited_pose = target_pose.copy()

        actual_xz = np.array([self.pose[0], self.pose[2]], dtype=float)
        target_xz = np.array([limited_pose[0], limited_pose[2]], dtype=float)
        command_lead_vector = target_xz - actual_xz
        command_lead_norm = float(np.linalg.norm(command_lead_vector))

        if command_lead_norm > max_command_lead_mm:
            limited_xz = (
                actual_xz
                + command_lead_vector / max(command_lead_norm, 1e-9) * max_command_lead_mm
            )
            limited_pose[0] = float(limited_xz[0])
            limited_pose[2] = float(limited_xz[1])
            self.last_command_lead_mm = float(max_command_lead_mm)
        else:
            self.last_command_lead_mm = command_lead_norm

        return limited_pose

    def limit_command_lead_xy(self, target_pose, max_command_lead_mm):
        limited_pose = target_pose.copy()

        actual_xy = np.array([self.pose[0], self.pose[1]], dtype=float)
        target_xy = np.array([limited_pose[0], limited_pose[1]], dtype=float)

        command_lead_vector = target_xy - actual_xy
        command_lead_norm = float(np.linalg.norm(command_lead_vector))

        if command_lead_norm > max_command_lead_mm:
            limited_xy = (
                actual_xy
                + command_lead_vector / max(command_lead_norm, 1e-9)
                * max_command_lead_mm
            )
            limited_pose[0] = float(limited_xy[0])
            limited_pose[1] = float(limited_xy[1])
            self.last_command_lead_mm = float(max_command_lead_mm)
        else:
            self.last_command_lead_mm = command_lead_norm

        return limited_pose

    def remove_finished_futures(self):
        self.pending_script_futures = [
            future for future in self.pending_script_futures
            if not future.done()
        ]

    def handle_script_response(self, future, request_id: str):
        try:
            response = future.result()
            if not response.ok:
                self.get_logger().warn(f'SendScript {request_id}: response.ok = False')
        except Exception as exc:
            self.get_logger().error(f'SendScript {request_id} failed: {exc}')

    # =========================================================
    # TM Command（不變）
    # =========================================================

    def tmsta_tag_callback(self, completed_tag_number):
        """
        當手臂真正走完該點，驅動器會收到 $TMSTA,10,01,XX,true
        觸發此處將對應的標籤從 Buffer 紀錄中移除放行
        """
        if completed_tag_number in self.sent_tags_in_buffer:
            del self.sent_tags_in_buffer[completed_tag_number]
            # 移除後，下一次的 control_loop 進入 ready_to_send_next_pvt 就會立刻自動放行！

    def send_pvt(self):
        if self.phase in [self.PHASE_TRENCH_TRAVERSE, self.PHASE_DONE]:
            return False

        if len(self.sent_tags_in_buffer) >= self.current_buffer_limit():
            return False

        pvt_time = self.pvt_duration_sec
        position_text = "{" + ",".join(f"{value:.4f}" for value in self.target_pose) + "}"
        velocity_text = "{" + ",".join(f"{value:.4f}" for value in self.target_vel) + "}"

        tag_to_send = self.current_tag

        combined_script = (
            f"PVTPoint({position_text},{velocity_text},{pvt_time:.6f})\r\n"
            f"QueueTag({tag_to_send})"
        )

        script_sent = self.send_script(combined_script)

        if script_sent:
            self.sent_tags_in_buffer[tag_to_send] = time.time()

            if self.pvt_point_counter < 10 or self.pvt_point_counter % 25 == 0:
                self.get_logger().info(
                    f"[PVT_SEND] point={self.pvt_point_counter + 1}, "
                    f"phase={self._phase_name()}, tag={tag_to_send}, "
                    f"target_x={self.target_pose[0]:.3f}, "
                    f"target_z={self.target_pose[2]:.3f}, "
                    f"vel_x={self.target_vel[0]:.3f}, "
                    f"vel_z={self.target_vel[2]:.3f}, "
                    f"buffer={len(self.sent_tags_in_buffer)}/{self.current_buffer_limit()}"
                )

            self.current_tag += 1
            if self.current_tag > 15:
                self.current_tag = 1

            self.pvt_point_counter += 1
            self.active_pvt_target_pose = self.target_pose.copy()
            self.last_pvt_send_time = time.time()

        return script_sent

    def send_script(self, script: str):
        self.remove_finished_futures()

        if len(self.pending_script_futures) >= 5: # 20
            self.get_logger().warn(
                f'Skip SendScript: too many pending requests ({len(self.pending_script_futures)})'
            )
            return False

        self.script_sequence_number += 1
        safe_request_id = f'S{self.script_sequence_number:06d}'

        req = SendScript.Request()
        req.id = safe_request_id
        req.script = script

        future = self.send_script_client.call_async(req)
        future.add_done_callback(
            lambda done_future, request_id=safe_request_id:
            self.handle_script_response(done_future, request_id)
        )
        self.pending_script_futures.append(future)
        return True


# =========================================================
# main
# =========================================================

def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Visual localization + feedback-gated PVT control for TM5-900 at TMFlow 5%.'
    )
    parser.add_argument('--visual-only', action='store_true', help='Only run visual localization test.')
    parser.add_argument('--model-path', default=DEFAULT_MODEL_PATH)
    parser.add_argument('--speed-override-ratio', type=float, default=0.05)
    parser.add_argument('--pvt-duration-sec', type=float, default=0.01)
    parser.add_argument('--fast-actual-approach-speed-mm-s', type=float, default=2.0)
    parser.add_argument('--slow-actual-force-speed-mm-s', type=float, default=1.2)
    parser.add_argument('--follow-actual-speed-mm-s', type=float, default=0.8)
    parser.add_argument('--fast-max-command-lead-mm', type=float, default=2.0)
    parser.add_argument('--slow-max-command-lead-mm', type=float, default=0.25)
    parser.add_argument('--pvt-target-settle-error-mm', type=float, default=0.2)
    parser.add_argument('--logic-rate-hz', type=float, default=20.0)
    parser.add_argument('--pvt-buffer-limit', type=int, default=3)
    parser.add_argument('--hybrid-pvt-buffer-limit', type=int, default=1)
    parser.add_argument('--queue-tag-timeout-sec', type=float, default=0.30)
    return parser.parse_args()


def main():
    args = parse_arguments()
    rclpy.init()

    node = HybridContourFollowingNode(
        visual_only=args.visual_only,
        model_path=args.model_path,
        speed_override_ratio=args.speed_override_ratio,
        pvt_duration_sec=args.pvt_duration_sec,
        fast_actual_approach_speed_mm_s=args.fast_actual_approach_speed_mm_s,
        slow_actual_force_speed_mm_s=args.slow_actual_force_speed_mm_s,
        follow_actual_speed_mm_s=args.follow_actual_speed_mm_s,
        fast_max_command_lead_mm=args.fast_max_command_lead_mm,
        slow_max_command_lead_mm=args.slow_max_command_lead_mm,
        pvt_target_settle_error_mm=args.pvt_target_settle_error_mm,
        logic_rate_hz=args.logic_rate_hz,
        pvt_buffer_limit=args.pvt_buffer_limit,
        hybrid_pvt_buffer_limit=args.hybrid_pvt_buffer_limit,
        queue_tag_timeout_sec=args.queue_tag_timeout_sec,
    )

    try:
        # rclpy.spin(node)
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
            
    except KeyboardInterrupt:
        print('KeyboardInterrupt: sending PVTExit before shutdown...')
        try:
            if rclpy.ok():
                node.send_script('PVTExit()')
                node.send_script('StopAndClearBuffer(0)')
                time.sleep(0.1)
        except Exception as exc:
            print(f'Shutdown command failed: {exc}')

    finally:
        print('Shutting down...')
        node._stop_event.set()
        node._force_thread.join(timeout=1.0)

        try:
            node.cam_stop()
        except Exception:
            pass

        try:
            if SHOW_VISION_WINDOW:
                cv2.destroyAllWindows()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()