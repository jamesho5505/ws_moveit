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
FAST_CLOSE_FORCE_THRESHOLD_GRAMS = 100.0
FORCE_CONTROL_FAST_GAIN = 0.003
MAX_GRIPPER_FAST_STEP_MM = 0.085
FORCE_TARGET_GRAMS = 1000.0
FORCE_DEADBAND_GRAMS = 50.0
FORCE_STABLE_SECONDS = 0.3
FORCE_CONTROL_GAIN = 0.00005
MAX_GRIPPER_STEP_MM = 0.1
FORCE_FILTER_ALPHA = 1.0
FORCE_CONTROL_HZ = 100.0
GRIPPER_COMMAND_HZ = 30.0

# ── Slip + force-drop confirmation ─────────────────────────────
FORCE_DROP_SLIP_THRESHOLD_GRAMS = 50.0
FORCE_DROP_SLIP_RATIO = 0.08
FORCE_DROP_CONFIRM_SECONDS = 0.0


# ── 抬升參數 ──────────────────────────────────────────────────
LIFT_STEP_MILLIMETERS = 60.0
TOTAL_LIFT_DISTANCE_MILLIMETERS = 60.0
LIFT_MOVE_SPEED = 50
LIFT_MOVE_ACCELERATION_MS = 1000
ARRIVAL_TOLERANCE_MILLIMETERS = 0.1

# ── 補夾 / 防彈跳 / PVDF ───────────────────────────────────────
TIGHTEN_DISTANCE_MILLIMETERS = 2.0
SLIP_BOUNCE_SECONDS = 0.6
# PVDF_SETTLE_THRESHOLD = 0.001
# PVDF_SETTLE_SECONDS = 0.1
INITIAL_LIFT_SLIP_IGNORE_SECONDS = 0.08

SLIP_RESPONSE_MIN_WAIT_SECONDS = 0.30
SLIP_RESPONSE_MAX_WAIT_SECONDS = 0.80
PVDF_RESUME_THRESHOLD = 0.025

# ── 夾爪參數 ──────────────────────────────────────────────────
GRIPPER_OPEN_MILLIMETERS = 22.0
GRIPPER_OPEN_SPEED = 255
GRIPPER_OPEN_FORCE = 100
GRIPPER_CLOSED_MILLIMETERS = 0.0
GRIPPER_CLOSED_SPEED = 150
GRIPPER_CLOSED_FORCE = 200
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
DROP_POSE_CPP = {
    "x": -140.0,
    "y": -700.0,
    "z": 110.0,
    "rx": 180.0,
    "ry": 0.0,
    "rz": 90.0,
}
VIEW_MOVE_SPEED = 250
VIEW_MOVE_ACCELERATION_MS = 10

PRE_CONTACT_CLEARANCE_MILLIMETERS = 12.0
MOVE_ABOVE_PICK_VELOCITY = 200
MOVE_ABOVE_PICK_ACCELERATION = 15
FAST_DESCEND_SPEED = 120
FAST_DESCEND_ACCELERATION_MS = 15
SLOW_CONTACT_DESCEND_VELOCITY = 100
SLOW_CONTACT_DESCEND_ACCELERATION = 80

# ── 視覺定位參數 ─────────────────────────────────────────────
DEFAULT_MODEL_PATH = '/home/jamesho5055/ws_moveit/src/tm5-900_moveit_config/scripts/pickup_visual/final_obb.pt'
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
NEEDED_VISION_SAMPLES = 8
MAX_VISION_FRAMES = 300
DEPTH_PATCH_RADIUS_PIXELS = 4
VISION_SAMPLE_INTERVAL_SECONDS = 0.1  # 0.3
SHOW_VISION_WINDOW = True
VISION_WINDOW_NAME = 'YOLO OBB (PICKUP no PCA + TCP axes)'

# ── 標定 / 法向估計參數 ───────────────────────────────────────
CALIBRATION_Z_OFFSET_MILLIMETERS = 0.0 
PCA_ROI_RADIUS_PIXELS = 10
PCA_MIN_RAW_POINTS = 120
PCA_DEPTH_BAND_MILLIMETERS = 8.0
PCA_NEIGHBOR_RADIUS_MILLIMETERS = 3.0
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
        self.min_z_position = 107.0

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
        self.slip_response_has_been_used = False

        self.lift_reference_force_grams = None
        self.force_drop_confirm_since_monotonic = None

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
                    # gripper_delta_millimeters = -clamp_value(
                    #     FORCE_CONTROL_GAIN * effective_force_error,
                    #     -MAX_GRIPPER_STEP_MM,
                    #     MAX_GRIPPER_STEP_MM,
                    # )
                    if filtered_force_grams < FAST_CLOSE_FORCE_THRESHOLD_GRAMS:
                        selected_force_control_gain = FORCE_CONTROL_FAST_GAIN
                        selected_max_gripper_step_millimeters = MAX_GRIPPER_FAST_STEP_MM
                    else:
                        selected_force_control_gain = FORCE_CONTROL_GAIN
                        selected_max_gripper_step_millimeters = MAX_GRIPPER_STEP_MM

                    gripper_delta_millimeters = -clamp_value(
                        selected_force_control_gain * effective_force_error,
                        -selected_max_gripper_step_millimeters,
                        selected_max_gripper_step_millimeters,
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
                    or abs(target_gripper_command_millimeters - last_command_millimeters) > 0.01
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
                self.slip_active = False
                self.pvdf_settle_since_monotonic = None
                self.slip_ignore_until_wall_time = time.time() + INITIAL_LIFT_SLIP_IGNORE_SECONDS
                self.lift_reference_force_grams = self.filtered_force_grams
                self.force_drop_confirm_since_monotonic = None
                self.state = State.LIFTING
            self.get_logger().info('[SM] → LIFTING')

            next_lift_target_pose = None

            while True:
                with self.state_lock:
                    if self.done:
                        return
                    current_state = self.state
                    # average_power = self.avg_power
                    # pvdf_settle_since_monotonic = self.pvdf_settle_since_monotonic
                    # slip_ignore_until_wall_time = self.slip_ignore_until_wall_time
                    current_lift_base_pose = None if self.current_lift_base_pose is None else dict(self.current_lift_base_pose)
                    detected_lift_pose = None if self.detected_lift_pose is None else dict(self.detected_lift_pose)

                # current_wall_time = time.time()
                # current_monotonic_time = time.monotonic()

                if current_state == State.LIFTING:
                    if current_lift_base_pose is None or detected_lift_pose is None:
                        raise RuntimeError('Lift pose not initialized.')

                    if self._distance_between_positions_millimeters(current_lift_base_pose, detected_lift_pose) <= ARRIVAL_TOLERANCE_MILLIMETERS:
                        self.get_logger().info('✅ [SM] Lift complete → start DROP sequence')

                        self._drop_and_return_to_view(current_lift_base_pose)

                        with self.state_lock:
                            self.state = State.DONE
                            self.done = True

                        self.get_logger().info('✅ [SM] DROP + RETURN complete → DONE')
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

                    with self.state_lock:
                        should_watch_slip = not self.slip_response_has_been_used

                    wait_result = self._wait_arrival(
                        next_lift_target_pose,
                        watch_slip=should_watch_slip,
                    )

                    # wait_result = self._wait_arrival(next_lift_target_pose, watch_slip=True)

                    if wait_result == 'done':
                        with self.state_lock:
                            self.current_lift_base_pose = dict(next_lift_target_pose)
                        next_lift_target_pose = None
                    elif wait_result == 'slipped':
                        self.get_logger().warn('[SM] Slip detected mid-move → SLIP_RESPONSE')
                        with self.state_lock:
                            self.slip_response_has_been_used = True
                            self.gripper_command_millimeters = clamp_value(
                                self.gripper_command_millimeters - TIGHTEN_DISTANCE_MILLIMETERS,
                                self.gripper_closed_limit_millimeters,
                                self.gripper_open_limit_millimeters,
                            )
                            self.state = State.SLIP_RESPONSE

                elif current_state == State.SLIP_RESPONSE:
                    # 等夾爪補夾命令實際送出與動作一小段時間
                    time.sleep(0.15)
                    slip_response_start_time = time.monotonic()

                    while True:
                        elapsed_slip_response_seconds = time.monotonic() - slip_response_start_time

                        with self.state_lock:
                            current_average_power = self.avg_power
                            current_slip_active = self.slip_active

                        minimum_wait_done = elapsed_slip_response_seconds >= SLIP_RESPONSE_MIN_WAIT_SECONDS
                        pvdf_is_quiet = (
                            current_average_power < PVDF_RESUME_THRESHOLD
                            and not current_slip_active
                        )
                        timeout_reached = elapsed_slip_response_seconds >= SLIP_RESPONSE_MAX_WAIT_SECONDS

                        if (minimum_wait_done and pvdf_is_quiet) or timeout_reached:
                            break

                        time.sleep(0.01)

                    # 不等 PVDF 穩定，只保留 slip 防彈跳時間
                    with self.state_lock:
                        self.slip_ignore_until_wall_time = time.time() + SLIP_BOUNCE_SECONDS
                        self.slip_active = False
                        self.pvdf_settle_since_monotonic = None

                    # 因為 _wait_arrival() 裡 slip 時有 PAUSE，所以這裡必須 RESUME
                    self._send_set_event(SetEvent.Request.RESUME)

                    with self.state_lock:
                        self.state = State.LIFTING

                    self.get_logger().info(
                        f'[SM] Slip response complete → RESUME → LIFTING '
                        f'(ignore slip {SLIP_BOUNCE_SECONDS:.1f}s)'
                    )

        except Exception as exc:
            self.get_logger().error(f'Control loop failed: {exc}')
            with self.state_lock:
                self.state = State.ERROR
                self.done = True

    # ═══ INIT / 視覺定位 ════════════════════════════════════════

    def _run_init_sequence(self):
        self.get_logger().info('[SM] INIT: moving to view pose...')
        self.change_tcp('tcp_obb_208')
        self._send_ptp(VIEW_POSE_CPP, speed=VIEW_MOVE_SPEED, acc_ms=VIEW_MOVE_ACCELERATION_MS)
        self._wait_arrival(VIEW_POSE_CPP, tol=0.1)
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
        move_above_pick_pose['rx'] = 180.0
        move_above_pick_pose['ry'] = 15.0


        self._send_line(
            x_millimeters=move_above_pick_pose['x'],
            y_millimeters=move_above_pick_pose['y'],
            z_millimeters=move_above_pick_pose['z'],
            rx_degrees=move_above_pick_pose['rx'],
            ry_degrees=move_above_pick_pose['ry'],
            rz_degrees=move_above_pick_pose['rz'],
            velocity=200,
            acceleration=10,
        )
        self._wait_arrival(move_above_pick_pose, tol=0.1)

        # self.get_logger().info('[SM] INIT: descending to pick pose...')
        # self._send_ptp(visual_pick_pose, speed=40, acc_ms=200)
        # self._wait_arrival(visual_pick_pose, tol=0.1)

        self.get_logger().info('[SM] INIT: fast descending to pre-contact pose...')
        if visual_pick_pose['z'] < self.min_z_position:
            self.get_logger().warn(
                f'Visual pick pose z={visual_pick_pose["z"]:.2f} mm is below minimum {self.min_z_position} mm, '
                f'adjusting to minimum height for safety.'
            )
            visual_pick_pose['z'] = self.min_z_position

        pre_contact_pose = dict(visual_pick_pose)
        pre_contact_pose['z'] += PRE_CONTACT_CLEARANCE_MILLIMETERS
        pre_contact_pose['rx'] = 180.0
        pre_contact_pose['ry'] = 15.0

        self._send_ptp(
            pre_contact_pose,
            speed=FAST_DESCEND_SPEED,
            acc_ms=FAST_DESCEND_ACCELERATION_MS,
        )
        self._wait_arrival(pre_contact_pose, tol=0.1)

        self.get_logger().info('[SM] INIT: slow final approach to pick pose...')

        visual_pick_pose['x'] += -0.5
        visual_pick_pose['z'] += 1.0  # 加一點高度保險，避免直接撞到物體
        # visual_pick_pose['z'] = 108.5  # 加一點高度保險，避免直接撞到物體
        visual_pick_pose['rx'] = 180.0
        visual_pick_pose['ry'] = 15.0
        detected_lift_pose['rx'] = 180.0
        detected_lift_pose['ry'] = 15.0

        self._send_line(
            x_millimeters=visual_pick_pose['x'],
            y_millimeters=visual_pick_pose['y'],
            z_millimeters=visual_pick_pose['z'],
            rx_degrees=visual_pick_pose['rx'],
            ry_degrees=visual_pick_pose['ry'],
            rz_degrees=visual_pick_pose['rz'],
            velocity=SLOW_CONTACT_DESCEND_VELOCITY,
            acceleration=SLOW_CONTACT_DESCEND_ACCELERATION,
        )
        self._wait_arrival(visual_pick_pose, tol=0.1)

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
        self._wait_arrival(VIEW_POSE_CPP, tol=0.1)

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
            velocity=200,
            acceleration=100,
        )
        self._wait_arrival(move_above_pick_pose, tol=0.1)

        self.get_logger().info('[SM] VISUAL_ONLY: reached above target, stop here.')

        with self.state_lock:
            self.detected_pick_pose = dict(visual_pick_pose)
            self.current_lift_base_pose = dict(move_above_pick_pose)
            self.pick_pose_acquired = True

    def _localize_pick_pose_with_vision(self) -> Tuple[Dict[str, float], Dict[str, float]]:
        valid_sample_count = 0
        selected_pick_point_meters = None
        selected_pick_orientation_degrees = None
        selected_tcp_z_axis_base = None
        last_visualization_frame = None

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
                    valid_sample_count += 1
                    roll_degrees, pitch_degrees, yaw_degrees = pick_orientation_degrees
                    self.get_logger().info(
                        f'[VISION] sample {valid_sample_count}/{NEEDED_VISION_SAMPLES}: '
                        f'x={pick_point_meters[0]:.4f}, y={pick_point_meters[1]:.4f}, z={pick_point_meters[2]:.4f} m | '
                        f'sxyz=({roll_degrees:.2f},{pitch_degrees:.2f},{yaw_degrees:.2f})'
                    )
                    selected_pick_point_meters = pick_point_meters
                    selected_pick_orientation_degrees = pick_orientation_degrees
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

        if selected_pick_point_meters is None or selected_pick_orientation_degrees is None:
            raise RuntimeError('Not enough valid visual samples for pickup.')

        pick_pose = {
            'x': float(selected_pick_point_meters[0] * 1000.0),
            'y': float(selected_pick_point_meters[1] * 1000.0),
            'z': float(selected_pick_point_meters[2] * 1000.0),
            'rx': float(selected_pick_orientation_degrees[0]),
            'ry': float(selected_pick_orientation_degrees[1]),
            'rz': float(selected_pick_orientation_degrees[2]),
        }

        upward_direction_vector = np.array([0.0, 0.0, 1.0], dtype=float)
        if selected_tcp_z_axis_base is not None:
            tcp_z_axis_base = np.asarray(selected_tcp_z_axis_base, dtype=float)
            tcp_z_axis_norm = float(np.linalg.norm(tcp_z_axis_base))
            if tcp_z_axis_norm > 1e-9:
                upward_direction_vector = -tcp_z_axis_base / tcp_z_axis_norm

        lift_target_point_meters = np.asarray(selected_pick_point_meters, dtype=float) + (
            TOTAL_LIFT_DISTANCE_MILLIMETERS / 1000.0
        ) * upward_direction_vector

        lift_pose = {
            'x': float(lift_target_point_meters[0] * 1000.0),
            'y': float(lift_target_point_meters[1] * 1000.0) - 20.0,
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

    def _obb_axis_vectors_base(self, oriented_box_one: np.ndarray, depth_meters: float, intrinsics, base_from_camera_transform: np.ndarray):
        """
        Convert the two 2D OBB axes of tooth 2 into approximate 3D base-frame direction vectors.
        The conversion uses two back-projected points at the same depth, so it does not use PCA.

        oriented_box_one = (cx, cy, width, height, theta)
        returns: [(axis_name, axis_vector_base), ...]
        """
        center_x_pixel, center_y_pixel, box_width, box_height, theta = map(float, oriented_box_one[:5])
        if depth_meters <= 0:
            return []

        step_along_theta = max(8.0, 0.5 * max(abs(box_width), abs(box_height)))
        step_along_theta_plus_90 = max(8.0, 0.5 * min(abs(box_width), abs(box_height)))

        candidate_pixel_axes = [
            ('obb_axis_theta', np.array([math.cos(theta), math.sin(theta)], dtype=float), step_along_theta),
            ('obb_axis_theta_plus_90', np.array([-math.sin(theta), math.cos(theta)], dtype=float), step_along_theta_plus_90),
        ]

        base_rotation_from_camera = base_from_camera_transform[:3, :3]
        candidate_base_axes = []

        for axis_name, pixel_direction, pixel_step in candidate_pixel_axes:
            plus_point_camera = self.backproject(
                center_x_pixel + pixel_step * pixel_direction[0],
                center_y_pixel + pixel_step * pixel_direction[1],
                depth_meters,
                intrinsics,
            )
            minus_point_camera = self.backproject(
                center_x_pixel - pixel_step * pixel_direction[0],
                center_y_pixel - pixel_step * pixel_direction[1],
                depth_meters,
                intrinsics,
            )
            axis_base = base_rotation_from_camera @ (plus_point_camera - minus_point_camera)
            axis_base = self._normalize_vector(axis_base)
            if axis_base is not None:
                candidate_base_axes.append((axis_name, axis_base))

        return candidate_base_axes

    def compute_tcp_rpy_sxyz_from_yvec_and_tooth2_obb(
        self,
        y_axis_vector_base: np.ndarray,
        oriented_box_tooth2: np.ndarray,
        depth_tooth2_meters: float,
        intrinsics,
        base_from_camera_transform: np.ndarray,
    ):
        """
        New pickup-pose rule:
          - pickup center remains at tooth 2;
          - y_tcp is fixed by the vector tooth 3 -> tooth 2;
          - x_tcp is taken from the tooth-2 OBB axis and projected perpendicular to y_tcp;
          - z_tcp = x_tcp × y_tcp and is forced toward base -Z.

        No PCA tangent plane is used here. Do not clamp RX/RY/RZ after this function,
        because changing a single Euler angle would change the real TCP axes and break y_tcp.
        """
        tcp_y_axis = self._normalize_vector(np.asarray(y_axis_vector_base, dtype=float))
        if tcp_y_axis is None:
            return None, None, {'reason': 'bad_y_axis'}

        downward_axis_base = np.array([0.0, 0.0, -1.0], dtype=float)
        candidate_x_axes = []

        for axis_name, raw_obb_axis_base in self._obb_axis_vectors_base(
            oriented_box_tooth2,
            depth_tooth2_meters,
            intrinsics,
            base_from_camera_transform,
        ):
            projected_x_axis = raw_obb_axis_base - float(np.dot(raw_obb_axis_base, tcp_y_axis)) * tcp_y_axis
            projected_x_axis = self._normalize_vector(projected_x_axis)
            if projected_x_axis is None:
                continue

            perpendicular_quality = abs(float(np.dot(raw_obb_axis_base, tcp_y_axis)))
            candidate_x_axes.append((perpendicular_quality, axis_name, projected_x_axis, raw_obb_axis_base))

        if candidate_x_axes:
            candidate_x_axes.sort(key=lambda item: item[0])
            _, selected_axis_name, tcp_x_axis, raw_obb_axis_base = candidate_x_axes[0]
            fallback_used = False
        else:
            tcp_x_axis = np.cross(tcp_y_axis, downward_axis_base)
            tcp_x_axis = self._normalize_vector(tcp_x_axis)
            if tcp_x_axis is None:
                tcp_x_axis = np.cross(tcp_y_axis, np.array([1.0, 0.0, 0.0], dtype=float))
                tcp_x_axis = self._normalize_vector(tcp_x_axis)
            if tcp_x_axis is None:
                return None, None, {'reason': 'cannot_build_x_axis'}
            selected_axis_name = 'fallback_cross_y_down'
            raw_obb_axis_base = None
            fallback_used = True

        tcp_z_axis = np.cross(tcp_x_axis, tcp_y_axis)
        tcp_z_axis = self._normalize_vector(tcp_z_axis)
        if tcp_z_axis is None:
            return None, None, {'reason': 'cannot_build_z_axis'}

        # Force z_tcp downward without changing y_tcp: flip x and z together only.
        if float(np.dot(tcp_z_axis, downward_axis_base)) < 0.0:
            tcp_x_axis = -tcp_x_axis
            tcp_z_axis = -tcp_z_axis

        # Rebuild x_tcp for numerical orthogonality while preserving y_tcp and z_tcp.
        tcp_x_axis = np.cross(tcp_y_axis, tcp_z_axis)
        tcp_x_axis = self._normalize_vector(tcp_x_axis)
        if tcp_x_axis is None:
            return None, None, {'reason': 'cannot_rebuild_x_axis'}

        rotation_matrix_homogeneous = np.eye(4, dtype=float)
        rotation_matrix_homogeneous[:3, 0] = tcp_x_axis
        rotation_matrix_homogeneous[:3, 1] = tcp_y_axis
        rotation_matrix_homogeneous[:3, 2] = tcp_z_axis

        roll_radians, pitch_radians, yaw_radians = tf_transformations.euler_from_matrix(
            rotation_matrix_homogeneous,
            axes='sxyz',
        )

        debug_info = {
            'reason': 'ok',
            'axis_name': selected_axis_name,
            'fallback_used': bool(fallback_used),
            'dot_xy': float(np.dot(tcp_x_axis, tcp_y_axis)),
            'dot_z_down': float(np.dot(tcp_z_axis, downward_axis_base)),
            'raw_obb_axis_base': raw_obb_axis_base,
        }

        return (
            math.degrees(roll_radians),
            math.degrees(pitch_radians),
            math.degrees(yaw_radians),
        ), rotation_matrix_homogeneous, debug_info

    def compute_tcp_rpy_sxyz_from_yvec(
        self,
        y_axis_vector_base: np.ndarray,
    ):
        """
        OBB-independent pickup pose rule:
        - y_tcp is fixed by the vector tooth3 -> tooth2
        - z_tcp is forced as close as possible to base -Z
        - x_tcp = y_tcp × z_tcp

        This function does not use the OBB orientation angle.
        """
        tcp_y_axis = self._normalize_vector(np.asarray(y_axis_vector_base, dtype=float))
        if tcp_y_axis is None:
            return None, None, {'reason': 'bad_y_axis'}

        downward_axis_base = np.array([0.0, 0.0, -1.0], dtype=float)

        # 將 base -Z 投影到垂直 y_tcp 的平面，避免 z_tcp 與 y_tcp 不垂直
        tcp_z_axis = downward_axis_base - float(np.dot(downward_axis_base, tcp_y_axis)) * tcp_y_axis
        tcp_z_axis = self._normalize_vector(tcp_z_axis)

        if tcp_z_axis is None:
            # 如果 y_tcp 幾乎平行 base -Z，改用 base X 當備援
            fallback_axis_base = np.array([1.0, 0.0, 0.0], dtype=float)
            tcp_z_axis = fallback_axis_base - float(np.dot(fallback_axis_base, tcp_y_axis)) * tcp_y_axis
            tcp_z_axis = self._normalize_vector(tcp_z_axis)

        if tcp_z_axis is None:
            return None, None, {'reason': 'cannot_build_z_axis'}

        tcp_x_axis = np.cross(tcp_y_axis, tcp_z_axis)
        tcp_x_axis = self._normalize_vector(tcp_x_axis)

        if tcp_x_axis is None:
            return None, None, {'reason': 'cannot_build_x_axis'}

        # 重新計算 z，確保三軸正交
        tcp_z_axis = np.cross(tcp_x_axis, tcp_y_axis)
        tcp_z_axis = self._normalize_vector(tcp_z_axis)

        if tcp_z_axis is None:
            return None, None, {'reason': 'cannot_rebuild_z_axis'}

        # 確保 z_tcp 朝向 base -Z
        if float(np.dot(tcp_z_axis, downward_axis_base)) < 0.0:
            tcp_x_axis = -tcp_x_axis
            tcp_z_axis = -tcp_z_axis

        rotation_matrix_homogeneous = np.eye(4, dtype=float)
        rotation_matrix_homogeneous[:3, 0] = tcp_x_axis
        rotation_matrix_homogeneous[:3, 1] = tcp_y_axis
        rotation_matrix_homogeneous[:3, 2] = tcp_z_axis

        roll_radians, pitch_radians, yaw_radians = tf_transformations.euler_from_matrix(
            rotation_matrix_homogeneous,
            axes='sxyz',
        )

        debug_info = {
            'reason': 'ok',
            'axis_rule': 'y_tcp = tooth3_to_tooth2, z_tcp = projected_base_down',
            'dot_y_down': float(np.dot(tcp_y_axis, downward_axis_base)),
            'dot_z_down': float(np.dot(tcp_z_axis, downward_axis_base)),
            'dot_xy': float(np.dot(tcp_x_axis, tcp_y_axis)),
            'dot_xz': float(np.dot(tcp_x_axis, tcp_z_axis)),
            'dot_yz': float(np.dot(tcp_y_axis, tcp_z_axis)),
        }

        return (
            math.degrees(roll_radians),
            math.degrees(pitch_radians),
            math.degrees(yaw_radians),
        ), rotation_matrix_homogeneous, debug_info

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
            pb[2] += self.calibration_z_offset_meters
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

        y_vec = pick_pt - neig_pt   # tooth3 → tooth2
        # y_vec = neig_pt - pick_pt   # tooth2 → tooth3

        # tcp_ori, tcp_R4, ori_debug = self.compute_tcp_rpy_sxyz_from_yvec_and_tooth2_obb(
        #     y_axis_vector_base=y_vec,
        #     oriented_box_tooth2=obbs[tooth2_idx],
        #     depth_tooth2_meters=dep_m,
        #     intrinsics=intr,
        #     base_from_camera_transform=T,
        # )
        tcp_ori, tcp_R4, ori_debug = self.compute_tcp_rpy_sxyz_from_yvec(
            y_axis_vector_base=y_vec,
        )


        # x_vec = pick_pt - neig_pt   # tooth3 → tooth2
        # tcp_ori, tcp_R4, ori_debug = self.compute_tcp_rpy_sxyz_from_xvec(
        #     x_axis_vector_base=x_vec,
# )
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
        self._draw_tcp_axes(
            vis,
            T,
            intr,
            origin_base=pick_pt,
            x_axis_base=tcp_x,
            y_axis_base=tcp_y,
            z_axis_base=tcp_z,
        )

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

    # ═══ 運動工具 ═══════════════════════════════════════════════

    def _is_force_drop_confirmed_during_lift(self) -> bool:
        current_monotonic_time = time.monotonic()

        with self.state_lock:
            lift_reference_force_grams = self.lift_reference_force_grams
            current_filtered_force_grams = self.filtered_force_grams
            force_drop_confirm_since_monotonic = self.force_drop_confirm_since_monotonic

        if lift_reference_force_grams is None:
            return False

        force_drop_threshold_grams = max(
            FORCE_DROP_SLIP_THRESHOLD_GRAMS,
            lift_reference_force_grams * FORCE_DROP_SLIP_RATIO,
        )

        force_drop_grams = lift_reference_force_grams - current_filtered_force_grams
        force_drop_condition = force_drop_grams >= force_drop_threshold_grams

        if force_drop_condition:
            if force_drop_confirm_since_monotonic is None:
                with self.state_lock:
                    self.force_drop_confirm_since_monotonic = current_monotonic_time
                return False

            force_drop_duration_seconds = current_monotonic_time - force_drop_confirm_since_monotonic
            return force_drop_duration_seconds >= FORCE_DROP_CONFIRM_SECONDS

        with self.state_lock:
            self.force_drop_confirm_since_monotonic = None

        return False

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
                # slip_detected = self.slip_active if watch_slip else False
                pvdf_slip_detected = self.slip_active if watch_slip else False
                current_filtered_force_grams = self.filtered_force_grams
                lift_reference_force_grams = self.lift_reference_force_grams

            # if slip_detected:
            #     self._send_set_event(SetEvent.Request.PAUSE)
            #     return 'slipped'
            force_drop_confirmed = (
                self._is_force_drop_confirmed_during_lift()
                if watch_slip else False
            )

            confirmed_slip_with_force_drop = pvdf_slip_detected and force_drop_confirmed

            if confirmed_slip_with_force_drop:
                self.get_logger().warn(
                    '[SLIP_CONFIRM] PVDF slip + force drop detected: '
                    f'F_ref={lift_reference_force_grams:.1f} g, '
                    f'F_now={current_filtered_force_grams:.1f} g'
                )
                self._send_set_event(SetEvent.Request.PAUSE)
                return 'slipped'

            if math.sqrt((current_x - target_x) ** 2 + (current_y - target_y) ** 2 + (current_z - target_z) ** 2) < tol:
                return 'done'

            time.sleep(0.005)

    def _send_set_event(self, function_code: int, arg0: int = 0, arg1: int = 0):
        request = SetEvent.Request()
        request.func = function_code
        request.arg0 = arg0
        request.arg1 = arg1
        future = self.set_event_client.call_async(request)
        while not future.done():
            time.sleep(0.005)

    def _open_gripper_to_default(self):
        self.gripper.goTomm(GRIPPER_OPEN_MILLIMETERS, GRIPPER_OPEN_SPEED, GRIPPER_OPEN_FORCE)

    def _closed_gripper_to_default(self):
        self.gripper.goTomm(GRIPPER_CLOSED_MILLIMETERS, GRIPPER_CLOSED_SPEED, GRIPPER_CLOSED_FORCE)

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

    def _drop_and_return_to_view(self, current_pose: Dict[str, float]):
        drop_above_pose = {
            'x': float(DROP_POSE_CPP['x']),
            'y': float(DROP_POSE_CPP['y']),
            'z': float(current_pose['z']),   # giữ độ cao lift hiện tại
            'rx': float(DROP_POSE_CPP['rx']),
            'ry': float(DROP_POSE_CPP['ry']),
            'rz': float(DROP_POSE_CPP['rz']),
        }

        drop_pose = dict(DROP_POSE_CPP)

        return_above_drop_pose = {
            'x': float(DROP_POSE_CPP['x']),
            'y': float(DROP_POSE_CPP['y']),
            'z': float(current_pose['z']),
            'rx': float(DROP_POSE_CPP['rx']),
            'ry': float(DROP_POSE_CPP['ry']),
            'rz': float(DROP_POSE_CPP['rz']),
        }

        self.get_logger().info('[DROP] Move above drop point...')
        self._send_line(
            x_millimeters=drop_above_pose['x'],
            y_millimeters=drop_above_pose['y'],
            z_millimeters=drop_above_pose['z'],
            rx_degrees=drop_above_pose['rx'],
            ry_degrees=drop_above_pose['ry'],
            rz_degrees=drop_above_pose['rz'],
            velocity=200,
            acceleration=10,
        )
        self._wait_arrival(drop_above_pose, tol=0.1)

        self.get_logger().info('[DROP] Descend to drop pose...')
        self._send_ptp(drop_pose, speed=120, acc_ms=50)
        self._wait_arrival(drop_pose, tol=0.1)

        self.get_logger().info('[DROP] Open gripper...')
        self._open_gripper_to_default()
        time.sleep(0.5)

        self.get_logger().info('[DROP] Lift back up...')
        self._send_ptp(return_above_drop_pose, speed=200, acc_ms=50)
        self._wait_arrival(return_above_drop_pose, tol=0.1)

        self.get_logger().info('[RETURN] Move back to view pose...')
        self._send_ptp(VIEW_POSE_CPP, speed=VIEW_MOVE_SPEED, acc_ms=VIEW_MOVE_ACCELERATION_MS)
        self._wait_arrival(VIEW_POSE_CPP, tol=0.1)

        self.get_logger().info('[RETURN] Close gripper to default and returning to view...')
        self._closed_gripper_to_default()
        time.sleep(0.3)
    
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