#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
將 test_pickup_slip_check.py 的視覺定位邏輯整合到 pickup.py。

流程：
  1. 移動到觀測位姿
  2. 使用 YOLO OBB + RealSense + TF 估測夾取點與 TCP 姿態
  3. 移動到視覺定位出的夾取位姿
  4. 進入力控夾持
  5. 沿視覺估測出的抬升方向逐步抬升
  6. 若抬升中 slip，PAUSE → 補夾 → 等 PVDF 穩定 → RESUME
"""

import argparse
import csv
import math
import os
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
import tf_transformations
import torch
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from tf2_ros import Buffer, TransformListener
from tm_msgs.msg import FeedbackState
from tm_msgs.srv import SendScript, SetEvent
from robotiq_gripper import RobotiqGripper

current_script_directory = Path(__file__).resolve().parent
ultralytics_repository_root = current_script_directory / "ultralytics"
local_ultralytics_init_file = ultralytics_repository_root / "ultralytics" / "__init__.py"
if local_ultralytics_init_file.exists():
    sys.path.insert(0, str(ultralytics_repository_root))

from ultralytics import YOLO
import open3d as o3d



def clamp_value(value: float, lower_bound: float, upper_bound: float) -> float:
    if value < lower_bound:
        return lower_bound
    if value > upper_bound:
        return upper_bound
    return value


# ── 力控參數 ──────────────────────────────────────────────────
FORCE_TARGET_GRAMS = 750.0
FORCE_DEADBAND_GRAMS = 20.0
FORCE_STABLE_SECONDS = 0.1
FORCE_CONTROL_GAIN = 0.001
MAX_GRIPPER_STEP_MM = 0.0225
FORCE_FILTER_ALPHA = 0.5
FORCE_CONTROL_HZ = 100.0
GRIPPER_COMMAND_HZ = 30.0

# ── 抬升參數 ──────────────────────────────────────────────────
LIFT_STEP_MILLIMETERS = 60.0
TOTAL_LIFT_DISTANCE_MILLIMETERS = 60.0
LIFT_MOVE_SPEED = 60
LIFT_MOVE_ACCELERATION_MS = 150
ARRIVAL_TOLERANCE_MILLIMETERS = 0.1

# ── 補夾 / 防彈跳 / PVDF ───────────────────────────────────────
TIGHTEN_DISTANCE_MILLIMETERS = 0.7
SLIP_BOUNCE_SECONDS = 0.4
PVDF_SETTLE_THRESHOLD = 0.001
PVDF_SETTLE_SECONDS = 0.1

# ── 夾爪參數 ──────────────────────────────────────────────────
GRIPPER_OPEN_MILLIMETERS = 20.0
GRIPPER_OPEN_SPEED = 255
GRIPPER_OPEN_FORCE = 100
GRIPPER_CONTROL_SPEED = 150
GRIPPER_CONTROL_FORCE = 200

# ── 初始運動 / 視角位姿 ───────────────────────────────────────
VIEW_POSE_CPP = {
    "x": -140.0,
    "y": -520.0,
    "z": 190.0,
    "rx": 180.0,
    "ry": 0.0,
    "rz": 90.0,
}
VIEW_MOVE_SPEED = 100
VIEW_MOVE_ACCELERATION_MS = 200

DROP_POSE = {
    "x": 185.0,
    "y": -675.0,
    "z": 110.0,
    "rx": 180.0,
    "ry": 0.0,
    "rz": 90.0,
}
DROP_MOVE_SPEED = 100
DROP_MOVE_ACCELERATION_MS = 200


# ── 視覺定位參數 ─────────────────────────────────────────────
DEFAULT_MODEL_PATH = '/home/jamesho5055/ws_moveit/src/tm5-900_moveit_config/scripts/pickup_visual/final_obb.pt'
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
NEEDED_VISION_SAMPLES = 10
MAX_VISION_FRAMES = 300
DEPTH_PATCH_RADIUS_PIXELS = 4
VISION_SAMPLE_INTERVAL_SECONDS = 0.3  # 0.3
SHOW_VISION_WINDOW = True
VISION_WINDOW_NAME = 'YOLO OBB (PICKUP + PCA Normal + TCP axes)'

# ── 標定 / 法向估計參數 ───────────────────────────────────────
CALIBRATION_Z_OFFSET_MILLIMETERS = -0.0 # -4.0
PCA_ROI_RADIUS_PIXELS = 8
PCA_MIN_RAW_POINTS = 120
PCA_DEPTH_BAND_MILLIMETERS = 4.0
PCA_NEIGHBOR_RADIUS_MILLIMETERS = 2.0
PCA_MIN_NEIGHBOR_POINTS = 80
PCA_USE_SOR = False
PCA_SOR_NEIGHBORS = 20
PCA_SOR_STD_RATIO = 2.0
PCA_PLANE_OUTLIER_MILLIMETERS = 1.0
PCA_PLANE_REFINE_ITERATIONS = 2
FORCE_TCP_NORMAL_DOWNWARD = True
AXIS_DRAW_LENGTH_MILLIMETERS = 30.0

# ── 記錄 ──────────────────────────────────────────────────────
LOG_HZ = 100.0
LOG_DIR = os.path.expanduser('~/logs')
LOG_HEADER = [
    'timestamp', 'state',
    'F_raw', 'F_filt', 'cmd_mm',
    'slip_flag', 'avg_power',
    'pose_x', 'pose_y', 'pose_z',
    'pose_rx', 'pose_ry', 'pose_rz',
    'tcp_speed_x', 'tcp_speed_y', 'tcp_speed_z',
    'tcp_speed_rx', 'tcp_speed_ry', 'tcp_speed_rz',
    'tcp_force_x', 'tcp_force_y', 'tcp_force_z',
    'tcp_force_rx', 'tcp_force_ry', 'tcp_force_rz',
    'joint_0', 'joint_1', 'joint_2',
    'joint_3', 'joint_4', 'joint_5',
]


class State:
    INIT = 'INIT'
    FORCE_CTRL = 'FORCE_CTRL'
    LIFTING = 'LIFTING'
    SLIP_RESPONSE = 'SLIP_RESPONSE'
    PVDF_SETTLING = 'PVDF_SETTLING'
    DROP = 'DROP'
    DONE = 'DONE'
    ERROR = 'ERROR'


class GraspAndLiftNode(Node):

    def __init__(self, gripper_port: str, model_path: str):
        super().__init__('grasp_and_lift_node')

        # ── TM client ───────────────────────────────────────
        self.tm_client = self.create_client(SendScript, 'send_script')
        self.set_event_client = self.create_client(SetEvent, 'set_event')
        self.get_logger().info('Waiting for TM services...')
        self.tm_client.wait_for_service()
        self.set_event_client.wait_for_service()
        self.get_logger().info('TM services ready.')

        # ── TF / 視覺模型 ────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.model_path = model_path
        self.model = YOLO(self.model_path)
        self.model_device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.model_device)
        self.camera_pipeline = None
        self.camera_aligner = None
        self.camera_started = False
        self.calibration_z_offset_meters = CALIBRATION_Z_OFFSET_MILLIMETERS / 1000.0

        # ── 夾爪 ─────────────────────────────────────────────
        self.gripper = RobotiqGripper(gripper_port)
        self.gripper._aCoef = -0.3744493392070485
        self.gripper._bCoef = 86.12334801762114
        self.gripper.openmm = 85
        self.gripper.closemm = 0
        self.gripper.goTomm(GRIPPER_OPEN_MILLIMETERS, GRIPPER_OPEN_SPEED, GRIPPER_OPEN_FORCE)
        self.gripper_closed_limit_millimeters = 0.0
        self.gripper_open_limit_millimeters = 85.0

        # ── 共享狀態 ─────────────────────────────────────────
        self.state_lock = threading.Lock()
        self.state = State.INIT
        self.raw_force_grams = 0.0
        self.filtered_force_grams = 0.0
        self.has_force_measurement = False
        self.gripper_command_millimeters = self.gripper.getPositionmm()
        self.force_stable_since_monotonic = None
        self.force_is_stable = False

        self.slip_active = False
        self.avg_power = 0.0
        self.slip_ignore_until_wall_time = 0.0
        self.pvdf_settle_since_monotonic = None

        self.current_tool_pose = [0.0] * 6
        self.current_tcp_speed = [0.0] * 6
        self.current_tcp_force = [0.0] * 6
        self.current_joint_positions = [0.0] * 6

        self.detected_pick_pose = None
        self.detected_lift_pose = None
        self.current_lift_base_pose = None
        self.pick_pose_acquired = False
        self.done = False

        # ── 訂閱 ─────────────────────────────────────────────
        self.create_subscription(Float32, '/esp32/force', self.cb_force, 10)
        self.create_subscription(Float32MultiArray, '/esp32/slip', self.cb_slip, 10)
        self.create_subscription(FeedbackState, '/feedback_states', self.cb_feedback, 10)

        # ── Logger ───────────────────────────────────────────
        os.makedirs(LOG_DIR, exist_ok=True)
        log_path = os.path.join(LOG_DIR, f'grasp_{int(time.time())}.csv')
        self.log_file = open(log_path, 'w', newline='', buffering=1)
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(LOG_HEADER)
        self.get_logger().info(f'Log: {log_path}')

        # ── 執行緒 ───────────────────────────────────────────
        threading.Thread(target=self._force_control_loop, daemon=True, name='force_ctrl').start()
        threading.Thread(target=self._gripper_command_loop, daemon=True, name='gripper_cmd').start()
        threading.Thread(target=self._log_loop, daemon=True, name='logger').start()
        threading.Thread(target=self._main_control_loop, daemon=True, name='main_ctrl').start()

        self.get_logger().info(
            f'GraspAndLiftNode ready. Target force: {FORCE_TARGET_GRAMS:.0f} g, model: {self.model_path}'
        )

    # ═══ ROS Callbacks ══════════════════════════════════════════

    def cb_force(self, message: Float32):
        measured_force_grams = max(0.0, float(message.data))
        with self.state_lock:
            self.raw_force_grams = measured_force_grams
            self.filtered_force_grams = (
                FORCE_FILTER_ALPHA * measured_force_grams
                + (1.0 - FORCE_FILTER_ALPHA) * self.filtered_force_grams
            )
            self.has_force_measurement = True

    def cb_slip(self, message: Float32MultiArray):
        if len(message.data) < 2:
            return

        slip_flag = bool(message.data[0])
        average_power = float(message.data[1])
        current_wall_time = time.time()

        with self.state_lock:
            self.avg_power = average_power
            if self.state in (State.INIT, State.FORCE_CTRL):
                self.slip_active = False
            elif current_wall_time < self.slip_ignore_until_wall_time:
                self.slip_active = False
            else:
                self.slip_active = slip_flag

    def cb_feedback(self, message: FeedbackState):
        tool_pose = message.tool_pose
        tcp_speed = message.tcp_speed
        tcp_force = message.tcp_force
        joint_positions = message.joint_pos

        with self.state_lock:
            if len(tool_pose) >= 6:
                self.current_tool_pose[:] = [
                    tool_pose[0] * 1000.0,
                    tool_pose[1] * 1000.0,
                    tool_pose[2] * 1000.0,
                    math.degrees(tool_pose[3]),
                    math.degrees(tool_pose[4]),
                    math.degrees(tool_pose[5]),
                ]
            if len(tcp_speed) >= 6:
                self.current_tcp_speed[:] = list(tcp_speed[:6])
            if len(tcp_force) >= 6:
                self.current_tcp_force[:] = list(tcp_force[:6])
            if len(joint_positions) >= 6:
                self.current_joint_positions[:] = list(joint_positions[:6])

    # ═══ Thread 1: 力控（100 Hz）══════════════════════════════

    def _force_control_loop(self):
        control_period_seconds = 1.0 / FORCE_CONTROL_HZ
        while True:
            loop_start_monotonic = time.monotonic()
            with self.state_lock:
                if self.done:
                    break
                current_state = self.state
                has_force_measurement = self.has_force_measurement
                filtered_force_grams = self.filtered_force_grams
                force_is_stable = self.force_is_stable
                current_gripper_command_millimeters = self.gripper_command_millimeters
                current_monotonic_time = time.monotonic()

            if current_state == State.FORCE_CTRL and has_force_measurement and not force_is_stable:
                force_error_grams = FORCE_TARGET_GRAMS - filtered_force_grams
                if abs(force_error_grams) <= FORCE_DEADBAND_GRAMS:
                    with self.state_lock:
                        if self.force_stable_since_monotonic is None:
                            self.force_stable_since_monotonic = current_monotonic_time
                        elif (current_monotonic_time - self.force_stable_since_monotonic) >= FORCE_STABLE_SECONDS:
                            self.force_is_stable = True
                            self.get_logger().info(
                                f'Force stable at {filtered_force_grams:.1f} g, '
                                f'gripper {current_gripper_command_millimeters:.2f} mm'
                            )
                else:
                    with self.state_lock:
                        self.force_stable_since_monotonic = None
                    effective_force_error = force_error_grams - math.copysign(FORCE_DEADBAND_GRAMS, force_error_grams)
                    gripper_delta_millimeters = -clamp_value(
                        FORCE_CONTROL_GAIN * effective_force_error,
                        -MAX_GRIPPER_STEP_MM,
                        MAX_GRIPPER_STEP_MM,
                    )
                    with self.state_lock:
                        self.gripper_command_millimeters = clamp_value(
                            current_gripper_command_millimeters + gripper_delta_millimeters,
                            self.gripper_closed_limit_millimeters,
                            self.gripper_open_limit_millimeters,
                        )

            elapsed_seconds = time.monotonic() - loop_start_monotonic
            time.sleep(max(0.0, control_period_seconds - elapsed_seconds))

    # ═══ Thread 2: 夾爪指令（30 Hz）══════════════════════════

    def _gripper_command_loop(self):
        command_period_seconds = 1.0 / GRIPPER_COMMAND_HZ
        last_command_millimeters = None
        while True:
            loop_start_monotonic = time.monotonic()
            with self.state_lock:
                if self.done:
                    break
                current_state = self.state
                target_gripper_command_millimeters = self.gripper_command_millimeters

            if current_state in (State.FORCE_CTRL, State.SLIP_RESPONSE):
                if (
                    last_command_millimeters is None
                    or abs(target_gripper_command_millimeters - last_command_millimeters) > 0.05
                ):
                    try:
                        self.gripper.goTomm(
                            target_gripper_command_millimeters,
                            GRIPPER_CONTROL_SPEED,
                            GRIPPER_CONTROL_FORCE,
                        )
                        last_command_millimeters = target_gripper_command_millimeters
                    except Exception as exc:
                        self.get_logger().warn(f'Gripper error: {exc}')

            elapsed_seconds = time.monotonic() - loop_start_monotonic
            time.sleep(max(0.0, command_period_seconds - elapsed_seconds))

    # ═══ Thread 3: 記錄（100 Hz）══════════════════════════════

    def _log_loop(self):
        log_period_seconds = 1.0 / LOG_HZ
        while True:
            loop_start_monotonic = time.monotonic()
            with self.state_lock:
                if self.done:
                    break
                log_row = [
                    f'{time.time():.6f}', self.state,
                    f'{self.raw_force_grams:.4f}', f'{self.filtered_force_grams:.4f}',
                    f'{self.gripper_command_millimeters:.4f}',
                    '1' if self.slip_active else '0', f'{self.avg_power:.8f}',
                    *[f'{value:.4f}' for value in self.current_tool_pose],
                    *[f'{value:.6f}' for value in self.current_tcp_speed],
                    *[f'{value:.6f}' for value in self.current_tcp_force],
                    *[f'{value:.6f}' for value in self.current_joint_positions],
                ]
            self.log_writer.writerow(log_row)
            elapsed_seconds = time.monotonic() - loop_start_monotonic
            time.sleep(max(0.0, log_period_seconds - elapsed_seconds))

    # ═══ Thread 4: 主控制狀態機 ════════════════════════════════

    def _main_control_loop(self):

        # Visual localization test
        # try:
        #     self._run_visual_only_sequence()
        #     with self.state_lock:
        #         self.state = State.DONE
        #         self.done = True
        #     self.get_logger().info('✅ Visual localization test complete → DONE')
        # except Exception as exc:
        #     self.get_logger().error(f'Control loop failed: {exc}')
        #     with self.state_lock:
        #         self.state = State.ERROR
        #         self.done = True

        try:
            self._run_init_sequence()

            with self.state_lock:
                self.state = State.FORCE_CTRL
            self.get_logger().info('[SM] → FORCE_CTRL')

            self.get_logger().info('[SM] FORCE_CTRL: waiting for force stable...')
            while True:
                with self.state_lock:
                    if self.done:
                        return
                    force_is_stable = self.force_is_stable
                if force_is_stable:
                    break
                time.sleep(0.05)

            with self.state_lock:
                self.state = State.LIFTING
            self.get_logger().info('[SM] → LIFTING')

            next_lift_target_pose = None

            while True:
                with self.state_lock:
                    if self.done:
                        return
                    current_state = self.state
                    average_power = self.avg_power
                    pvdf_settle_since_monotonic = self.pvdf_settle_since_monotonic
                    slip_ignore_until_wall_time = self.slip_ignore_until_wall_time
                    current_lift_base_pose = None if self.current_lift_base_pose is None else dict(self.current_lift_base_pose)
                    detected_lift_pose = None if self.detected_lift_pose is None else dict(self.detected_lift_pose)

                current_wall_time = time.time()
                current_monotonic_time = time.monotonic()

                if current_state == State.LIFTING:
                    if current_lift_base_pose is None or detected_lift_pose is None:
                        raise RuntimeError('Lift pose not initialized.')

                    if self._distance_between_positions_millimeters(current_lift_base_pose, detected_lift_pose) <= ARRIVAL_TOLERANCE_MILLIMETERS:
                        with self.state_lock:
                            self.state = State.DROP
                        self.get_logger().info('[SM] Lift complete → DROP')
                        self._run_drop_sequence()
                        with self.state_lock:
                            self.state = State.DONE
                            self.done = True
                        self.get_logger().info('✅ [SM] Drop complete → DONE')
                        return

                    if next_lift_target_pose is None:
                        next_lift_target_pose = self._build_next_lift_target_pose(
                            current_pose=current_lift_base_pose,
                            final_pose=detected_lift_pose,
                            step_distance_millimeters=LIFT_STEP_MILLIMETERS,
                        )
                        self._send_ptp(next_lift_target_pose)
                        self.get_logger().info(
                            '[LIFT] '
                            f'({current_lift_base_pose["x"]:.2f}, {current_lift_base_pose["y"]:.2f}, {current_lift_base_pose["z"]:.2f}) '
                            f'→ ({next_lift_target_pose["x"]:.2f}, {next_lift_target_pose["y"]:.2f}, {next_lift_target_pose["z"]:.2f}) mm'
                        )

                    wait_result = self._wait_arrival(next_lift_target_pose, watch_slip=True)

                    if wait_result == 'done':
                        with self.state_lock:
                            self.current_lift_base_pose = dict(next_lift_target_pose)
                        next_lift_target_pose = None
                    elif wait_result == 'slipped':
                        self.get_logger().warn('[SM] Slip detected mid-move → SLIP_RESPONSE')
                        with self.state_lock:
                            self.gripper_command_millimeters = clamp_value(
                                self.gripper_command_millimeters - TIGHTEN_DISTANCE_MILLIMETERS,
                                self.gripper_closed_limit_millimeters,
                                self.gripper_open_limit_millimeters,
                            )
                            self.state = State.SLIP_RESPONSE

                elif current_state == State.SLIP_RESPONSE:
                    time.sleep(0.15)
                    with self.state_lock:
                        self.slip_ignore_until_wall_time = time.time() + SLIP_BOUNCE_SECONDS
                        self.slip_active = False
                        self.pvdf_settle_since_monotonic = None
                        self.state = State.PVDF_SETTLING
                    self.get_logger().info(
                        f'[SM] → PVDF_SETTLING (bounce {SLIP_BOUNCE_SECONDS:.1f}s)'
                    )

                elif current_state == State.PVDF_SETTLING:
                    if current_wall_time < slip_ignore_until_wall_time:
                        time.sleep(0.05)
                        continue

                    if average_power < PVDF_SETTLE_THRESHOLD:
                        if pvdf_settle_since_monotonic is None:
                            with self.state_lock:
                                self.pvdf_settle_since_monotonic = current_monotonic_time
                        elif (current_monotonic_time - pvdf_settle_since_monotonic) >= PVDF_SETTLE_SECONDS:
                            self._send_set_event(SetEvent.Request.RESUME)
                            with self.state_lock:
                                self.pvdf_settle_since_monotonic = None
                                self.state = State.LIFTING
                            self.get_logger().info(
                                f'[SM] PVDF stable (power={average_power:.5f}) → RESUME → LIFTING'
                            )
                    else:
                        with self.state_lock:
                            self.pvdf_settle_since_monotonic = None

                    time.sleep(0.05)
        except Exception as exc:
            self.get_logger().error(f'Control loop failed: {exc}')
            with self.state_lock:
                self.state = State.ERROR
                self.done = True

    # ═══ INIT / 視覺定位 ════════════════════════════════════════

    def _run_init_sequence(self):
        self.get_logger().info('[SM] INIT: moving to view pose...')
        self.change_tcp('tcp_obb')
        self._send_ptp(VIEW_POSE_CPP, speed=VIEW_MOVE_SPEED, acc_ms=VIEW_MOVE_ACCELERATION_MS)
        self._wait_arrival(VIEW_POSE_CPP, tol=0.1)
        self._reactivate_gripper()
        self._open_gripper_to_default()

        self.get_logger().info('[SM] INIT: running visual localization...')
        visual_pick_pose, detected_lift_pose = self._localize_pick_pose_with_vision()
        self.get_logger().info(
            '[VISION] Pick pose: '
            f'({visual_pick_pose["x"]:.2f}, {visual_pick_pose["y"]:.2f}, {visual_pick_pose["z"]:.2f}, '
            f'{visual_pick_pose["rx"]:.2f}, {visual_pick_pose["ry"]:.2f}, {visual_pick_pose["rz"]:.2f})'
        )
        

        self.get_logger().info('[SM] INIT: moving above pick point...')
        move_above_pick_pose = dict(visual_pick_pose)
        move_above_pick_pose['z'] = VIEW_POSE_CPP['z']
        self._send_line(
            x_millimeters=move_above_pick_pose['x'],
            y_millimeters=move_above_pick_pose['y'],
            z_millimeters=move_above_pick_pose['z'],
            rx_degrees=move_above_pick_pose['rx'],
            ry_degrees=move_above_pick_pose['ry'],
            rz_degrees=move_above_pick_pose['rz'],
            velocity=100,
            acceleration=200,
        )
        self._wait_arrival(move_above_pick_pose, tol=0.1)

        self.get_logger().info('[SM] INIT: descending to pick pose...')
        self._send_ptp(visual_pick_pose, speed=30, acc_ms=200)
        self._wait_arrival(visual_pick_pose, tol=1.0)

        with self.state_lock:
            self.detected_pick_pose = dict(visual_pick_pose)
            self.detected_lift_pose = dict(detected_lift_pose)
            self.current_lift_base_pose = dict(visual_pick_pose)
            self.pick_pose_acquired = True

    def _run_visual_only_sequence(self):
        self.get_logger().info('[SM] VISUAL_ONLY: moving to view pose...')
        self._open_gripper_to_default()
        self.change_tcp('tcp_obb')
        self._send_ptp(VIEW_POSE_CPP, speed=VIEW_MOVE_SPEED, acc_ms=VIEW_MOVE_ACCELERATION_MS)
        self._wait_arrival(VIEW_POSE_CPP, tol=2.0)

        self.get_logger().info('[SM] VISUAL_ONLY: running visual localization...')
        visual_pick_pose, _ = self._localize_pick_pose_with_vision()

        self.get_logger().info(
            '[VISION] Pick pose: '
            f'({visual_pick_pose["x"]:.2f}, {visual_pick_pose["y"]:.2f}, {visual_pick_pose["z"]:.2f}, '
            f'{visual_pick_pose["rx"]:.2f}, {visual_pick_pose["ry"]:.2f}, {visual_pick_pose["rz"]:.2f})'
        )

        move_above_pick_pose = dict(visual_pick_pose)
        move_above_pick_pose['z'] = VIEW_POSE_CPP['z']   # 保持在上方高度

        self.get_logger().info('[SM] VISUAL_ONLY: moving above target...')
        self._send_line(
            x_millimeters=move_above_pick_pose['x'],
            y_millimeters=move_above_pick_pose['y'],
            z_millimeters=move_above_pick_pose['z'],
            rx_degrees=move_above_pick_pose['rx'],
            ry_degrees=move_above_pick_pose['ry'],
            rz_degrees=move_above_pick_pose['rz'],
            velocity=100,
            acceleration=100,
        )
        self._wait_arrival(move_above_pick_pose, tol=1.0)

        self.get_logger().info('[SM] VISUAL_ONLY: reached above target, stop here.')

        with self.state_lock:
            self.detected_pick_pose = dict(visual_pick_pose)
            self.current_lift_base_pose = dict(move_above_pick_pose)
            self.pick_pose_acquired = True

    def _run_drop_sequence(self):
        # 取得目前抬升高度，作為橫移過去放置點的安全高度
        with self.state_lock:
            lift_pose = dict(self.detected_lift_pose)

        # Step 1：水平移動到 DROP 上方（保持抬升高度）
        safe_above_drop = dict(DROP_POSE)
        safe_above_drop['z'] = lift_pose['z']
        self.get_logger().info(
            f'[DROP] Moving above drop point at z={safe_above_drop["z"]:.2f}mm...'
        )
        self._send_ptp(safe_above_drop, speed=DROP_MOVE_SPEED, acc_ms=DROP_MOVE_ACCELERATION_MS)
        self._wait_arrival(safe_above_drop, tol=1.0)

        # Step 2：下降到放置高度
        self.get_logger().info('[DROP] Descending to drop pose...')
        self._send_ptp(DROP_POSE, speed=DROP_MOVE_SPEED // 2, acc_ms=DROP_MOVE_ACCELERATION_MS)
        self._wait_arrival(DROP_POSE, tol=1.0)

        # Step 3：開爪放置
        self.get_logger().info('[DROP] Opening gripper...')
        self._open_gripper_to_default()
        time.sleep(0.5)

        # Step 4：抬升離開
        self.get_logger().info('[DROP] Retreating...')
        self._send_ptp(safe_above_drop, speed=DROP_MOVE_SPEED, acc_ms=DROP_MOVE_ACCELERATION_MS)
        self._wait_arrival(safe_above_drop, tol=1.0)

        # Step 5：返回 VIEW 姿勢
        self.get_logger().info('[DROP] Returning to view pose...')
        self._send_ptp(VIEW_POSE_CPP, speed=VIEW_MOVE_SPEED, acc_ms=VIEW_MOVE_ACCELERATION_MS)
        self._wait_arrival(VIEW_POSE_CPP, tol=1.0)

    def _localize_pick_pose_with_vision(self) -> Tuple[Dict[str, float], Dict[str, float]]:
        valid_sample_count = 0
        selected_tcp_z_axis_base = None
        last_visualization_frame = None

        pick_points = []        # ← 移到迴圈外
        pick_orientations = []  # ← 移到迴圈外

        self.cam_start()
        try:
            for _ in range(MAX_VISION_FRAMES):
                pick_point_meters, pick_orientation_degrees, visualization_frame, detection_output = self.capture_one_detection()

                if visualization_frame is not None:
                    last_visualization_frame = visualization_frame
                    if SHOW_VISION_WINDOW:
                        cv2.imshow(VISION_WINDOW_NAME, visualization_frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break

                if pick_point_meters is not None and pick_orientation_degrees is not None:
                    pick_points.append(pick_point_meters)
                    pick_orientations.append(pick_orientation_degrees)
                    valid_sample_count += 1

                    roll_degrees, pitch_degrees, yaw_degrees = pick_orientation_degrees
                    self.get_logger().info(
                        f'[VISION] sample {valid_sample_count}/{NEEDED_VISION_SAMPLES}: '
                        f'x={pick_point_meters[0]:.4f}, y={pick_point_meters[1]:.4f}, z={pick_point_meters[2]:.4f} m | '
                        f'sxyz=({roll_degrees:.2f},{pitch_degrees:.2f},{yaw_degrees:.2f})'
                    )

                    if detection_output is not None and 'z_tcp_base' in detection_output:
                        selected_tcp_z_axis_base = detection_output['z_tcp_base']

                    if valid_sample_count >= NEEDED_VISION_SAMPLES:
                        break
                    time.sleep(VISION_SAMPLE_INTERVAL_SECONDS)
        finally:
            self.cam_stop()
            if SHOW_VISION_WINDOW:
                if last_visualization_frame is not None:
                    cv2.imshow('YOLO OBB - LAST', last_visualization_frame)
                    cv2.waitKey(500)
                cv2.destroyAllWindows()

        if not pick_points:
            raise RuntimeError('Not enough valid visual samples for pickup.')

        # ← 取平均，不再只用最後一筆
        points_array = np.array(pick_points)
        orientations_array = np.array(pick_orientations)
        selected_pick_point_meters = np.median(points_array, axis=0)
        selected_pick_orientation_degrees = np.median(orientations_array, axis=0)


        x_tcp_offset_meters = 0.003
        y_tcp_offset_meters = 0.001
        z_tcp_offset_meters = 0.0015

        rx, ry, rz = selected_pick_orientation_degrees
        M = tf_transformations.euler_matrix(
            math.radians(rx),
            math.radians(ry),
            math.radians(rz),
            axes='sxyz'
        )

        tcp_x_unit = M[:3, 0]   # TCP x 軸
        tcp_y_unit = M[:3, 1]   # TCP y 軸
        tcp_z_unit = M[:3, 2]   # TCP z 軸（和 selected_tcp_z_axis_base 應一致）

        selected_pick_point_meters += (
            x_tcp_offset_meters * -tcp_x_unit
            + y_tcp_offset_meters * tcp_y_unit
            + z_tcp_offset_meters * -tcp_z_unit
        )
        
        upward_direction_vector = -tcp_z_unit
        # upward_direction_vector = np.array([0.0, 0.0, 1.0], dtype=float)

        # if selected_tcp_z_axis_base is not None:
        #     tcp_z_axis_base = np.asarray(selected_tcp_z_axis_base, dtype=float)
        #     tcp_z_axis_norm = float(np.linalg.norm(tcp_z_axis_base))
            
        #     if tcp_z_axis_norm > 1e-9:
        #         # tcp_z_unit = tcp_z_axis_base / tcp_z_axis_norm
        #         # selected_pick_point_meters += z_tcp_offset_meters * -tcp_z_unit
        #         upward_direction_vector = -tcp_z_axis_base / tcp_z_axis_norm
                

        pick_pose = {
            'x': float(selected_pick_point_meters[0] * 1000.0),
            'y': float(selected_pick_point_meters[1] * 1000.0),
            'z': float(selected_pick_point_meters[2] * 1000.0),
            'rx': float(selected_pick_orientation_degrees[0]),
            'ry': float(selected_pick_orientation_degrees[1]),
            'rz': float(selected_pick_orientation_degrees[2]),
        }

        

        lift_target_point_meters = np.asarray(selected_pick_point_meters, dtype=float) + (
            TOTAL_LIFT_DISTANCE_MILLIMETERS / 1000.0
        ) * upward_direction_vector

        lift_pose = {
            'x': float(lift_target_point_meters[0] * 1000.0),
            'y': float(lift_target_point_meters[1] * 1000.0),
            'z': float(lift_target_point_meters[2] * 1000.0),
            'rx': float(selected_pick_orientation_degrees[0]),
            'ry': float(selected_pick_orientation_degrees[1]),
            'rz': float(selected_pick_orientation_degrees[2]),
        }

        



        return pick_pose, lift_pose
    
    # ═══ 視覺 / 相機工具 ════════════════════════════════════════

    def cam_start(self):
        if self.camera_started:
            return
        self.camera_pipeline = rs.pipeline()
        camera_config = rs.config()
        camera_config.enable_stream(rs.stream.color, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.bgr8, CAMERA_FPS)
        camera_config.enable_stream(rs.stream.depth, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.z16, CAMERA_FPS)
        self.camera_pipeline.start(camera_config)
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
        self.camera_aligner = None

    def get_base_from_camera_transform(self):
        try:
            transform_message = self.tf_buffer.lookup_transform(
                'base',
                'camera_color_optical_frame',
                rclpy.time.Time(),
            )
            translation = transform_message.transform.translation
            rotation = transform_message.transform.rotation
            transform_matrix = tf_transformations.quaternion_matrix(
                [rotation.x, rotation.y, rotation.z, rotation.w]
            )
            transform_matrix[:3, 3] = [translation.x, translation.y, translation.z]
            return transform_matrix
        except Exception:
            return None

    def backproject(self, pixel_x: float, pixel_y: float, depth_meters: float, intrinsics):
        camera_x = (pixel_x - intrinsics.ppx) / intrinsics.fx * depth_meters
        camera_y = (pixel_y - intrinsics.ppy) / intrinsics.fy * depth_meters
        return np.array([camera_x, camera_y, depth_meters], dtype=float)

    def median_depth(self, depth_frame, center_x: int, center_y: int, patch_radius_pixels: int) -> float:
        frame_height = depth_frame.get_height()
        frame_width = depth_frame.get_width()
        x_min = max(center_x - patch_radius_pixels, 0)
        x_max = min(center_x + patch_radius_pixels, frame_width - 1)
        y_min = max(center_y - patch_radius_pixels, 0)
        y_max = min(center_y + patch_radius_pixels, frame_height - 1)

        valid_depth_values = []
        for pixel_y in range(y_min, y_max + 1):
            for pixel_x in range(x_min, x_max + 1):
                depth_meters = depth_frame.get_distance(int(pixel_x), int(pixel_y))
                if depth_meters > 0:
                    valid_depth_values.append(depth_meters)

        return float(np.median(valid_depth_values)) if valid_depth_values else 0.0

    def _normalize_vector(self, vector: np.ndarray, epsilon: float = 1e-12):
        vector_norm = float(np.linalg.norm(vector))
        if vector_norm < epsilon:
            return None
        return vector / vector_norm

    def _angle_degrees(self, vector_a: np.ndarray, vector_b: np.ndarray, epsilon: float = 1e-12) -> float:
        norm_a = np.linalg.norm(vector_a)
        norm_b = np.linalg.norm(vector_b)
        if norm_a < epsilon or norm_b < epsilon:
            return 180.0
        cosine_value = float(np.dot(vector_a, vector_b) / (norm_a * norm_b))
        cosine_value = max(-1.0, min(1.0, cosine_value))
        return math.degrees(math.acos(cosine_value))

    def _project_base_point_to_pixel(self, base_point: np.ndarray, base_from_camera_transform: np.ndarray, intrinsics):
        base_rotation = base_from_camera_transform[:3, :3]
        base_translation = base_from_camera_transform[:3, 3]
        camera_rotation = base_rotation.T
        camera_point = camera_rotation @ (np.asarray(base_point, dtype=float) - base_translation)
        camera_z = float(camera_point[2])
        if camera_z <= 1e-6:
            return None
        pixel_x = int(round(intrinsics.fx * (float(camera_point[0]) / camera_z) + intrinsics.ppx))
        pixel_y = int(round(intrinsics.fy * (float(camera_point[1]) / camera_z) + intrinsics.ppy))
        return pixel_x, pixel_y

    def _draw_tcp_axes(self, image, base_from_camera_transform, intrinsics, origin_base, x_axis_base, y_axis_base, z_axis_base):
        axis_origin_pixel = self._project_base_point_to_pixel(origin_base, base_from_camera_transform, intrinsics)
        if axis_origin_pixel is None:
            return

        axis_draw_length_meters = AXIS_DRAW_LENGTH_MILLIMETERS / 1000.0
        x_axis_pixel = self._project_base_point_to_pixel(
            origin_base + axis_draw_length_meters * x_axis_base,
            base_from_camera_transform,
            intrinsics,
        )
        y_axis_pixel = self._project_base_point_to_pixel(
            origin_base + axis_draw_length_meters * y_axis_base,
            base_from_camera_transform,
            intrinsics,
        )
        z_axis_pixel = self._project_base_point_to_pixel(
            origin_base + axis_draw_length_meters * z_axis_base,
            base_from_camera_transform,
            intrinsics,
        )

        if x_axis_pixel is not None:
            cv2.arrowedLine(image, axis_origin_pixel, x_axis_pixel, (0, 0, 255), 2, tipLength=0.2)
            cv2.putText(image, 'X', x_axis_pixel, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        if y_axis_pixel is not None:
            cv2.arrowedLine(image, axis_origin_pixel, y_axis_pixel, (0, 255, 0), 2, tipLength=0.2)
            cv2.putText(image, 'Y', y_axis_pixel, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if z_axis_pixel is not None:
            cv2.arrowedLine(image, axis_origin_pixel, z_axis_pixel, (255, 0, 0), 2, tipLength=0.2)
            cv2.putText(image, 'Z', z_axis_pixel, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        cv2.drawMarker(image, axis_origin_pixel, (255, 255, 255), cv2.MARKER_CROSS, 12, 2)

    def estimate_tangent_normal_pca(
        self,
        depth_frame,
        intrinsics,
        base_from_camera_transform: np.ndarray,
        pick_pixel_x: int,
        pick_pixel_y: int,
        pick_point_base: np.ndarray,
    ):
        frame_height = depth_frame.get_height()
        frame_width = depth_frame.get_width()
        roi_radius_pixels = int(PCA_ROI_RADIUS_PIXELS)

        x_min = max(int(pick_pixel_x) - roi_radius_pixels, 0)
        x_max = min(int(pick_pixel_x) + roi_radius_pixels, frame_width - 1)
        y_min = max(int(pick_pixel_y) - roi_radius_pixels, 0)
        y_max = min(int(pick_pixel_y) + roi_radius_pixels, frame_height - 1)

        local_depth_values = []
        for pixel_y in range(y_min, y_max + 1):
            for pixel_x in range(x_min, x_max + 1):
                depth_meters = depth_frame.get_distance(int(pixel_x), int(pixel_y))
                if depth_meters > 0:
                    local_depth_values.append(depth_meters)

        if len(local_depth_values) < int(PCA_MIN_RAW_POINTS):
            return False, None, {'reason': 'too_few_depth_points', 'n_raw': len(local_depth_values)}

        median_depth_meters = float(np.median(local_depth_values))
        allowed_depth_band_meters = PCA_DEPTH_BAND_MILLIMETERS / 1000.0
        neighbor_radius_meters = PCA_NEIGHBOR_RADIUS_MILLIMETERS / 1000.0
        pick_point_base = np.asarray(pick_point_base, dtype=float)

        candidate_points_base = []
        for pixel_y in range(y_min, y_max + 1):
            for pixel_x in range(x_min, x_max + 1):
                depth_meters = depth_frame.get_distance(int(pixel_x), int(pixel_y))
                if depth_meters <= 0:
                    continue
                if abs(depth_meters - median_depth_meters) > allowed_depth_band_meters:
                    continue
                point_camera = self.backproject(float(pixel_x), float(pixel_y), float(depth_meters), intrinsics)
                point_base = (base_from_camera_transform[:3, :3] @ point_camera) + base_from_camera_transform[:3, 3]
                point_base[2] += self.calibration_z_offset_meters
                if float(np.linalg.norm(point_base - pick_point_base)) > neighbor_radius_meters:
                    continue
                candidate_points_base.append(point_base)

        if len(candidate_points_base) < int(PCA_MIN_NEIGHBOR_POINTS):
            return False, None, {'reason': 'too_few_neighbor_points', 'n': len(candidate_points_base)}

        point_cloud_base = np.asarray(candidate_points_base, dtype=np.float64)

        if PCA_USE_SOR and o3d is not None:
            open3d_point_cloud = o3d.geometry.PointCloud()
            open3d_point_cloud.points = o3d.utility.Vector3dVector(point_cloud_base)
            open3d_point_cloud, _ = open3d_point_cloud.remove_statistical_outlier(
                nb_neighbors=int(PCA_SOR_NEIGHBORS),
                std_ratio=float(PCA_SOR_STD_RATIO),
            )
            point_cloud_base = np.asarray(open3d_point_cloud.points)
            if len(point_cloud_base) < int(PCA_MIN_NEIGHBOR_POINTS) * 0.5:
                return False, None, {'reason': 'too_few_after_SOR', 'n': int(len(point_cloud_base))}

        outlier_threshold_meters = PCA_PLANE_OUTLIER_MILLIMETERS / 1000.0
        refine_iterations = int(PCA_PLANE_REFINE_ITERATIONS)

        def fit_plane_with_pca(points_base: np.ndarray):
            cloud_center = points_base.mean(axis=0)
            centered_points = points_base - cloud_center
            covariance_matrix = (centered_points.T @ centered_points) / max(len(points_base) - 1, 1)
            eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)
            plane_normal = self._normalize_vector(eigenvectors[:, 0])
            return cloud_center, plane_normal, eigenvalues

        refined_point_cloud = point_cloud_base
        for _ in range(refine_iterations):
            plane_center, plane_normal, _ = fit_plane_with_pca(refined_point_cloud)
            if plane_normal is None:
                return False, None, {'reason': 'degenerate_pca_normal'}
            point_to_plane_distances = (refined_point_cloud - plane_center) @ plane_normal
            inlier_mask = np.abs(point_to_plane_distances) <= outlier_threshold_meters
            inlier_points = refined_point_cloud[inlier_mask]
            if len(inlier_points) < int(PCA_MIN_NEIGHBOR_POINTS) * 0.5:
                break
            refined_point_cloud = inlier_points

        plane_center, plane_normal, eigenvalues = fit_plane_with_pca(refined_point_cloud)
        if plane_normal is None:
            return False, None, {'reason': 'degenerate_pca_normal_final'}

        if FORCE_TCP_NORMAL_DOWNWARD:
            if float(np.dot(plane_normal, np.array([0.0, 0.0, -1.0], dtype=float))) < 0.0:
                plane_normal = -plane_normal

        tilt_degrees = self._angle_degrees(plane_normal, np.array([0.0, 0.0, -1.0], dtype=float))
        planar_score = float(eigenvalues[0] / max(eigenvalues[1] + eigenvalues[2], 1e-12))
        original_distances = (point_cloud_base - plane_center) @ plane_normal
        inlier_count = int(np.sum(np.abs(original_distances) <= outlier_threshold_meters))

        debug_info = {
            'reason': 'ok',
            'tilt_deg': float(tilt_degrees),
            'planar_score': float(planar_score),
            'n_in': int(inlier_count),
            'n_all': int(len(point_cloud_base)),
        }
        return True, plane_normal, debug_info

    def compute_tcp_rpy_sxyz_from_yvec_and_plane_normal(self, y_axis_vector_base: np.ndarray, plane_normal_base: np.ndarray):
        tcp_y_axis = self._normalize_vector(y_axis_vector_base)
        plane_normal = self._normalize_vector(plane_normal_base)
        if tcp_y_axis is None or plane_normal is None:
            return None, None

        tcp_x_axis = self._normalize_vector(np.cross(plane_normal, tcp_y_axis))
        if tcp_x_axis is None:
            return None, None

        tcp_z_axis = self._normalize_vector(np.cross(tcp_x_axis, tcp_y_axis))
        if tcp_z_axis is None:
            return None, None

        if np.dot(tcp_z_axis, np.array([0.0, 0.0, -1.0], dtype=float)) < 0.0:
            tcp_z_axis = -tcp_z_axis

        tcp_x_axis = self._normalize_vector(np.cross(tcp_y_axis, tcp_z_axis))
        if tcp_x_axis is None:
            return None, None

        rotation_matrix_homogeneous = np.eye(4, dtype=float)
        rotation_matrix_homogeneous[:3, 0] = tcp_x_axis
        rotation_matrix_homogeneous[:3, 1] = tcp_y_axis
        rotation_matrix_homogeneous[:3, 2] = tcp_z_axis

        roll_radians, pitch_radians, yaw_radians = tf_transformations.euler_from_matrix(
            rotation_matrix_homogeneous,
            axes='sxyz',
        )
        return (
            math.degrees(roll_radians),
            math.degrees(pitch_radians),
            math.degrees(yaw_radians),
        ), rotation_matrix_homogeneous

    def capture_one_detection(self):
        base_from_camera_transform = self.get_base_from_camera_transform()
        if base_from_camera_transform is None:
            return None, None, None, None

        try:
            frameset = self.camera_pipeline.wait_for_frames(timeout_ms=1000)
            aligned_frameset = self.camera_aligner.process(frameset)
            depth_frame = aligned_frameset.get_depth_frame()
            color_frame = aligned_frameset.get_color_frame()
            if not depth_frame or not color_frame:
                return None, None, None, None
            color_image = np.asanyarray(color_frame.get_data())
        except Exception:
            return None, None, None, None

        image_height, image_width = color_image.shape[:2]
        image_midline_y = image_height / 2.0
        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

        model_results = self.model.predict(color_image, verbose=False)
        if len(model_results) == 0:
            return None, None, color_image, None

        first_result = model_results[0]
        visualization_image = first_result.plot(labels=False, conf=False)

        if (
            getattr(first_result, 'obb', None) is None
            or getattr(first_result.obb, 'xywhr', None) is None
            or len(first_result.obb.xywhr) == 0
        ):
            return None, None, visualization_image, None

        oriented_boxes = first_result.obb.xywhr.detach().cpu().numpy()
        center_pixels = oriented_boxes[:, :2]

        if getattr(first_result.obb, 'cls', None) is not None:
            class_indices = first_result.obb.cls.detach().cpu().numpy().astype(int)
            tooth_class_index = None
            for class_index, class_name in self.model.names.items():
                if str(class_name).strip().lower() == 'tooth-s8ie':
                    tooth_class_index = int(class_index)
                    break
            if tooth_class_index is not None:
                valid_class_indices = np.where(class_indices == tooth_class_index)[0]
                if len(valid_class_indices) == 0:
                    return None, None, visualization_image, None
                oriented_boxes = oriented_boxes[valid_class_indices]
                center_pixels = center_pixels[valid_class_indices]

        detection_count = len(oriented_boxes)
        if detection_count < 2:
            return None, None, visualization_image, None

        top_half_indices = [index for index in range(detection_count) if center_pixels[index, 1] < image_midline_y]
        bottom_half_indices = [index for index in range(detection_count) if center_pixels[index, 1] >= image_midline_y]

        base_points_by_detection = [None] * detection_count
        base_z_values_by_detection = [None] * detection_count
        valid_depth_detection_indices = []

        for detection_index in range(detection_count):
            center_x_pixel, center_y_pixel = map(float, center_pixels[detection_index, :2])
            depth_meters = self.median_depth(
                depth_frame,
                int(center_x_pixel),
                int(center_y_pixel),
                DEPTH_PATCH_RADIUS_PIXELS,
            )
            if depth_meters <= 0:
                continue
            point_camera = self.backproject(center_x_pixel, center_y_pixel, depth_meters, color_intrinsics)
            point_base = (base_from_camera_transform[:3, :3] @ point_camera) + base_from_camera_transform[:3, 3]
            point_base[2] += self.calibration_z_offset_meters
            base_points_by_detection[detection_index] = point_base
            base_z_values_by_detection[detection_index] = float(point_base[2])
            valid_depth_detection_indices.append(detection_index)

        if not valid_depth_detection_indices:
            return None, None, visualization_image, None

        valid_top_half_indices = [index for index in top_half_indices if index in valid_depth_detection_indices]
        valid_bottom_half_indices = [index for index in bottom_half_indices if index in valid_depth_detection_indices]

        top_sorted_indices = sorted(valid_top_half_indices, key=lambda index: float(center_pixels[index, 0]))
        bottom_sorted_indices = sorted(valid_bottom_half_indices, key=lambda index: float(center_pixels[index, 0]))

        selected_tooth_position_left_to_right = 1
        top_has_target = len(top_sorted_indices) > selected_tooth_position_left_to_right
        bottom_has_target = len(bottom_sorted_indices) > selected_tooth_position_left_to_right

        if not top_has_target and not bottom_has_target:
            return None, None, visualization_image, None

        if top_has_target and not bottom_has_target:
            selected_region_name = 'TOP'
            selected_region_indices = top_sorted_indices
        elif bottom_has_target and not top_has_target:
            selected_region_name = 'BOT'
            selected_region_indices = bottom_sorted_indices
        else:
            selected_top_index = top_sorted_indices[selected_tooth_position_left_to_right]
            selected_bottom_index = bottom_sorted_indices[selected_tooth_position_left_to_right]
            selected_top_z = base_z_values_by_detection[selected_top_index]
            selected_bottom_z = base_z_values_by_detection[selected_bottom_index]
            if selected_top_z > selected_bottom_z:
                selected_region_name = 'TOP'
                selected_region_indices = top_sorted_indices
            else:
                selected_region_name = 'BOT'
                selected_region_indices = bottom_sorted_indices

        if len(selected_region_indices) <= selected_tooth_position_left_to_right:
            return None, None, visualization_image, None

        selected_detection_index = selected_region_indices[selected_tooth_position_left_to_right]
        left_neighbor_detection_index = selected_region_indices[selected_tooth_position_left_to_right - 1]

        selected_pick_point_base = base_points_by_detection[selected_detection_index]
        left_neighbor_point_base = base_points_by_detection[left_neighbor_detection_index]
        if selected_pick_point_base is None or left_neighbor_point_base is None:
            return None, None, visualization_image, None

        y_axis_vector_base = left_neighbor_point_base - selected_pick_point_base
        selected_center_x_pixel, selected_center_y_pixel = center_pixels[selected_detection_index]
        left_center_x_pixel, left_center_y_pixel = center_pixels[left_neighbor_detection_index]
        selected_pixel_x = int(round(selected_center_x_pixel))
        selected_pixel_y = int(round(selected_center_y_pixel))

        normal_estimation_ok, plane_normal_base, normal_debug_info = self.estimate_tangent_normal_pca(
            depth_frame=depth_frame,
            intrinsics=color_intrinsics,
            base_from_camera_transform=base_from_camera_transform,
            pick_pixel_x=selected_pixel_x,
            pick_pixel_y=selected_pixel_y,
            pick_point_base=selected_pick_point_base,
        )
        if not normal_estimation_ok or plane_normal_base is None:
            plane_normal_base = np.array([0.0, 0.0, 1.0], dtype=float)

        tcp_orientation_degrees, tcp_rotation_matrix = self.compute_tcp_rpy_sxyz_from_yvec_and_plane_normal(
            y_axis_vector_base,
            plane_normal_base,
        )
        if tcp_orientation_degrees is None or tcp_rotation_matrix is None:
            return None, None, visualization_image, None

        tcp_roll_degrees, tcp_pitch_degrees, tcp_yaw_degrees = tcp_orientation_degrees
        cv2.circle(visualization_image, (int(selected_center_x_pixel), int(selected_center_y_pixel)), 7, (0, 255, 0), -1)
        cv2.circle(visualization_image, (int(left_center_x_pixel), int(left_center_y_pixel)), 7, (255, 0, 0), -1)
        cv2.line(
            visualization_image,
            (int(selected_center_x_pixel), int(selected_center_y_pixel)),
            (int(left_center_x_pixel), int(left_center_y_pixel)),
            (0, 255, 255),
            2,
        )
        cv2.drawMarker(visualization_image, (selected_pixel_x, selected_pixel_y), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)

        tcp_x_axis_base = tcp_rotation_matrix[:3, 0].copy()
        tcp_y_axis_base = tcp_rotation_matrix[:3, 1].copy()
        tcp_z_axis_base = tcp_rotation_matrix[:3, 2].copy()
        self._draw_tcp_axes(
            visualization_image,
            base_from_camera_transform,
            color_intrinsics,
            origin_base=selected_pick_point_base,
            x_axis_base=tcp_x_axis_base,
            y_axis_base=tcp_y_axis_base,
            z_axis_base=tcp_z_axis_base,
        )

        overlay_line_1 = f'REGION={selected_region_name} | cnt={len(selected_region_indices)} | FIX sel=2nd (L->R)'
        overlay_line_2 = (
            f'P_pick(sel)=({selected_pick_point_base[0]:.3f},'
            f'{selected_pick_point_base[1]:.3f},{selected_pick_point_base[2]:.3f}) m'
        )
        overlay_line_3 = (
            f'Euler sxyz: rx={tcp_roll_degrees:.1f}, '
            f'ry={tcp_pitch_degrees:.1f}, rz={tcp_yaw_degrees:.1f}'
        )
        if normal_debug_info is not None and normal_debug_info.get('reason') == 'ok':
            overlay_line_4 = (
                f'PCA OK | tilt={normal_debug_info["tilt_deg"]:.1f} | '
                f'score={normal_debug_info["planar_score"]:.2e} | n_in={normal_debug_info["n_in"]}'
            )
        else:
            overlay_line_4 = (
                'PCA FAIL -> fallback -Z | '
                f'{None if normal_debug_info is None else normal_debug_info.get("reason")}'
            )

        cv2.putText(visualization_image, overlay_line_1, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 0), 2)
        cv2.putText(visualization_image, overlay_line_2, (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(visualization_image, overlay_line_3, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(visualization_image, overlay_line_4, (10, 94), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        detection_output = {
            'region': selected_region_name,
            'count': len(selected_region_indices),
            'selected_index': selected_detection_index,
            'left_neighbor_index': left_neighbor_detection_index,
            'pick_point_base_meters': selected_pick_point_base,
            'tcp_orientation_sxyz_degrees': (
                tcp_roll_degrees,
                tcp_pitch_degrees,
                tcp_yaw_degrees,
            ),
            'pca_ok': bool(normal_estimation_ok),
            'z_tcp_base': tcp_z_axis_base,
            'pca_debug': normal_debug_info,
        }
        return selected_pick_point_base, (tcp_roll_degrees, tcp_pitch_degrees, tcp_yaw_degrees), visualization_image, detection_output

    # ═══ 運動工具 ═══════════════════════════════════════════════

    def send_script(self, script: str) -> bool:
        request = SendScript.Request()
        request.id = 'pickup'
        request.script = script
        future = self.tm_client.call_async(request)
        while not future.done():
            time.sleep(0.005)
        return True

    def _send_ptp(self, target_pose: Dict[str, float], speed: int = LIFT_MOVE_SPEED, acc_ms: int = LIFT_MOVE_ACCELERATION_MS):
        script = (
            'PTP("CPP",'
            f'{target_pose["x"]:.4f},{target_pose["y"]:.4f},{target_pose["z"]:.4f},'
            f'{target_pose["rx"]:.4f},{target_pose["ry"]:.4f},{target_pose["rz"]:.4f},'
            f'{speed},{acc_ms},0,true)'
        )
        self.send_script(script)

    def _send_line(
        self,
        x_millimeters: float,
        y_millimeters: float,
        z_millimeters: float,
        rx_degrees: float,
        ry_degrees: float,
        rz_degrees: float,
        velocity: int = 100,
        acceleration: int = 100,
        blend: int = 0,
        precise_disable: bool = False,
    ):
        target_pose_string = (
            f'{{{x_millimeters:.3f},{y_millimeters:.3f},{z_millimeters:.3f},'
            f'{rx_degrees:.3f},{ry_degrees:.3f},{rz_degrees:.3f}}}'
        )
        script = (
            f'Line("CPP", {target_pose_string}, {velocity}, {acceleration}, '
            f'{blend}, {str(precise_disable).lower()})'
        )
        self.send_script(script)

    def change_tcp(self, tcp_name: str):
        self.send_script(f'ChangeTCP("{tcp_name}")')

    def _wait_arrival(self, target_pose: Dict[str, float], watch_slip: bool = False, tol: float = ARRIVAL_TOLERANCE_MILLIMETERS) -> str:
        target_x = target_pose['x']
        target_y = target_pose['y']
        target_z = target_pose['z']

        while True:
            with self.state_lock:
                current_x = self.current_tool_pose[0]
                current_y = self.current_tool_pose[1]
                current_z = self.current_tool_pose[2]
                slip_detected = self.slip_active if watch_slip else False

            if slip_detected:
                self._send_set_event(SetEvent.Request.PAUSE)
                return 'slipped'

            if math.sqrt((current_x - target_x) ** 2 + (current_y - target_y) ** 2 + (current_z - target_z) ** 2) < tol:
                return 'done'

            time.sleep(0.02)

    def _send_set_event(self, function_code: int, arg0: int = 0, arg1: int = 0):
        request = SetEvent.Request()
        request.func = function_code
        request.arg0 = arg0
        request.arg1 = arg1
        future = self.set_event_client.call_async(request)
        while not future.done():
            time.sleep(0.005)

    def _reactivate_gripper(self):
        self.gripper.reactivate()

    def _open_gripper_to_default(self):
        self.gripper.goTomm(GRIPPER_OPEN_MILLIMETERS, GRIPPER_OPEN_SPEED, GRIPPER_OPEN_FORCE)

    def _build_next_lift_target_pose(
        self,
        current_pose: Dict[str, float],
        final_pose: Dict[str, float],
        step_distance_millimeters: float,
    ) -> Dict[str, float]:
        current_position = np.array([current_pose['x'], current_pose['y'], current_pose['z']], dtype=float)
        final_position = np.array([final_pose['x'], final_pose['y'], final_pose['z']], dtype=float)
        delta_vector = final_position - current_position
        remaining_distance = float(np.linalg.norm(delta_vector))

        if remaining_distance <= step_distance_millimeters or remaining_distance <= 1e-9:
            return dict(final_pose)

        direction_vector = delta_vector / remaining_distance
        next_position = current_position + direction_vector * step_distance_millimeters
        return {
            'x': float(next_position[0]),
            'y': float(next_position[1]),
            'z': float(next_position[2]),
            'rx': float(final_pose['rx']),
            'ry': float(final_pose['ry']),
            'rz': float(final_pose['rz']),
        }

    def _distance_between_positions_millimeters(self, pose_a: Dict[str, float], pose_b: Dict[str, float]) -> float:
        delta_x = pose_a['x'] - pose_b['x']
        delta_y = pose_a['y'] - pose_b['y']
        delta_z = pose_a['z'] - pose_b['z']
        return math.sqrt(delta_x ** 2 + delta_y ** 2 + delta_z ** 2)

    def close_log(self):
        try:
            self.log_file.flush()
            self.log_file.close()
        except Exception:
            pass


def main():
    argument_parser = argparse.ArgumentParser()
    argument_parser.add_argument('--gripper_port', default='/dev/ttyUSB0')
    argument_parser.add_argument('--model_path', default=DEFAULT_MODEL_PATH)
    arguments = argument_parser.parse_args()

    rclpy.init()
    node = GraspAndLiftNode(arguments.gripper_port, arguments.model_path)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down...')
        try:
            node.cam_stop()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.close_log()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()