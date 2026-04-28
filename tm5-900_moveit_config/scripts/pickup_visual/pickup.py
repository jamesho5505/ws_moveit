#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
import tf_transformations

import numpy as np
import cv2
import pyrealsense2 as rs
import torch
from ultralytics import YOLO

from tm_msgs.srv import SendScript
from tm_msgs.msg import FeedbackState

import time
import math

# ==== Gripper (Robotiq) ====
import minimalmodbus
import ROS2_gripper as rq

# ==== Open3D (optional SOR) ====
import open3d as o3d


class YOLOPickAndMove(Node):
    def __init__(self):
        super().__init__('yolo_pick_and_move_obb')

        # === TF2 ===
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # === YOLO OBB ===
        model_path = '/home/jamesho5055/ws_moveit/src/tm5-900_moveit_config/scripts/pickup_visual/final_obb.pt'
        self.model = YOLO(model_path)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)

        # === Camera (deferred start) ===
        self.pipe = None
        self.align = None
        self.cam_started = False

        # === TM send_script ===
        self.script_client = self.create_client(SendScript, 'send_script')
        while not self.script_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for send_script...')

        # === Feedback ===
        self.latest_fb = None
        self.create_subscription(FeedbackState, 'feedback_states', self._fb_cb, 10)

        # === View Pose ===
        # (x,y,z,rx,ry,rz,acc,vel,coord,blend)
        self.VIEW_CPP = (-140, -520, 190, 180, 0, 90, 300, 300, 0, False)

        # === Sampling ===
        self.NEEDED_SAMPLES = 5
        self.MAX_FRAMES = 300
        self.PATCH_K = 4

        # === tolerances ===
        self.POS_TOL_MM = 4.0
        self.ANG_TOL_DEG = 4.0
        self.CONSEC_OK = 2

        # === Grasp params ===
        self.LIFT_AFTER_GRASP_MM = 100

        # === Z-offset for calibration ===
        self.Z_OFFSET_MM = -2.0
        self.Z_OFFSET_M = self.Z_OFFSET_MM / 1000.0

        # === Drop pose (mm/deg) ===
        self.DROP_POSE = (185, -675, 110, 180, 0, 90)

        # =======================================================
        # NORMAL (PCA tangent plane - FreeCAD-like)
        # =======================================================
        self.ROI_R_PX = 10
        self.MIN_PTS_RAW = 120
        self.Z_BAND_MM = 8.0

        self.NEIGHBOR_RADIUS_MM = 3.0
        self.MIN_PTS_NEIGHBOR = 80

        self.USE_SOR = False
        self.SOR_NB = 20
        self.SOR_STD = 2.0

        self.PLANE_OUTLIER_MM = 1.0
        self.PLANE_REFINE_ITERS = 2

        self.FORCE_NORMAL_DOWN = True

        # =======================================================
        # GUI overlay
        # =======================================================
        self.SHOW_AXES = True
        self.AXIS_LEN_MM = 30.0
        self.FONT_SCALE = 0.55

        # =======================================================
        # Gripper / retry config
        # =======================================================
        self.GRIPPER_SPEED = 80
        self.GRIPPER_FORCE = 80
        self.GRIPPER_FORCE_STEP = 15
        self.GRIPPER_FORCE_MAX = 255   # keep in mode 2 if desired
        self.GRIPPER_CLOSE_POS = 240
        self.GRIPPER_OPEN_POS = 155

        self.GRASP_SETTLE_SEC = 0.7
        self.MAX_REPICK_TRIES = 3

        # Slip decision parameters
        # Secondary heuristic: if gPO at lift grows too much compared to after-close
        self.SLIP_GPO_DELTA = 25
        self.LIFT_CHECK_POLLS = 5
        self.LIFT_CHECK_DT = 0.08

        # gOBJ states
        self.GOBJ_MOVING = 0
        self.GOBJ_OPEN_CONTACT = 1
        self.GOBJ_CLOSE_CONTACT = 2
        self.GOBJ_AT_POS_NO_OBJ = 3

        # === Gripper ===
        self.gripper = None
        self.inst = None
        self._init_gripper()

        self.get_logger().info(
            "OBB Model Ready (Pickup: FIXED 2nd tooth; Orientation: y from neighbor, z from PCA tangent plane; Euler sxyz)."
        )

    # =======================================================
    # GRIPPER
    # =======================================================
    def _init_gripper(self):
        try:
            self.inst = minimalmodbus.Instrument('/dev/ttyUSB0', 9)
            self.inst.serial.baudrate = 115200
            self.inst.serial.timeout = 0.2
            self.inst.debug = False

            self.gripper = rq.RobotiqGripper('/dev/ttyUSB0', 9)
            self.get_logger().info("Gripper READY")
        except Exception as e:
            self.get_logger().error(f"Gripper init failed: {e}")
            self.gripper = None
            self.inst = None

    def gripper_open(self, pos=None):
        if pos is None:
            pos = self.GRIPPER_OPEN_POS
        if self.gripper:
            try:
                self.gripper.goTo(pos, self.GRIPPER_SPEED, self.GRIPPER_FORCE)
            except Exception as e:
                self.get_logger().warn(f"gripper_open failed: {e}")

    def gripper_close(self, pos=None, force=None):
        if pos is None:
            pos = self.GRIPPER_CLOSE_POS
        if force is None:
            force = self.GRIPPER_FORCE
        if self.gripper:
            try:
                self.gripper.goTo(pos, self.GRIPPER_SPEED, force)
            except Exception as e:
                self.get_logger().warn(f"gripper_close failed: {e}")

    def _read_gripper_status_bytes(self):
        """
        Robotiq input register first address = 2000.
        Need bytes 0..5:
          byte0 = status (contains gOBJ, gSTA, gGTO, gACT)
          byte4 = gPO
          byte5 = gCU
        """
        if self.inst is None:
            return None

        try:
            regs = self.inst.read_registers(2000, 3, functioncode=4)
        except Exception:
            try:
                regs = self.inst.read_registers(2000, 3, functioncode=3)
            except Exception as e:
                self.get_logger().warn(f"Read gripper status failed: {e}")
                return None

        data = []
        for reg in regs:
            low = reg & 0xFF
            high = (reg >> 8) & 0xFF
            data.extend([high, low])

        if len(data) < 6:
            return None
        return data

    def get_gripper_status(self):
        data = self._read_gripper_status_bytes()
        if data is None:
            return None

        status_byte = data[0]

        gOBJ = (status_byte >> 6) & 0x03  # bits 7..6
        gSTA = (status_byte >> 4) & 0x03  # bits 5..4
        gGTO = (status_byte >> 3) & 0x01  # bit 3
        gACT = (status_byte >> 0) & 0x01  # bit 0

        gPO = data[4]
        gCU = data[5]

        return {
            "gOBJ": int(gOBJ),
            "gSTA": int(gSTA),
            "gGTO": int(gGTO),
            "gACT": int(gACT),
            "gPO": int(gPO),
            "gCU": int(gCU),
        }

    def slipped_at_lift(self, st_close, poll_n=None, poll_dt=None):
        """
        Manual-consistent logic:
          - if after close we had gOBJ = 0x02 (contact while closing)
          - and at lift it becomes gOBJ = 0x03
            => object lost / dropped

        Secondary rule:
          - if gPO grows significantly toward full close compared to after-close
            => likely slip

        We poll multiple times after reaching lift, not just a single sample.
        """
        if poll_n is None:
            poll_n = self.LIFT_CHECK_POLLS
        if poll_dt is None:
            poll_dt = self.LIFT_CHECK_DT

        if st_close is None:
            self.get_logger().warn("No reference status after close; cannot evaluate slip reliably.")
            return False

        gobj_close = st_close["gOBJ"]
        gpo_close = st_close["gPO"]

        for _ in range(poll_n):
            st = self.get_gripper_status()
            if st is None:
                time.sleep(poll_dt)
                continue

            gobj = st["gOBJ"]
            gpo = st["gPO"]
            gcu = st["gCU"]

            self.get_logger().info(
                f"Check at lift: gOBJ={gobj}, gPO={gpo}, gCU={gcu} | "
                f"close_ref: gOBJ={gobj_close}, gPO={gpo_close}"
            )

            # Primary rule from manual behavior
            if gobj_close == self.GOBJ_CLOSE_CONTACT and gobj == self.GOBJ_AT_POS_NO_OBJ:
                self.get_logger().warn("Slip detected by gOBJ transition 0x02 -> 0x03")
                return True

            # Secondary support rule using position increase
            if gpo >= min(255, gpo_close + self.SLIP_GPO_DELTA):
                self.get_logger().warn(
                    f"Slip suspected by gPO increase: {gpo_close} -> {gpo}"
                )
                return True

            time.sleep(poll_dt)

        return False

    # =======================================================
    # FEEDBACK
    # =======================================================
    def _fb_cb(self, msg):
        self.latest_fb = msg

    def _get_tcp_mmdeg(self):
        fb = self.latest_fb
        if fb is None or len(fb.tool_pose) < 6:
            return None
        x, y, z, rx, ry, rz = fb.tool_pose[:6]
        return (
            x * 1000, y * 1000, z * 1000,
            rx * 180 / np.pi, ry * 180 / np.pi, rz * 180 / np.pi
        )

    def wait_until_reached(self, target, timeout_s=120):
        t0 = time.time()
        ok_count = 0
        while time.time() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.02)
            cur = self._get_tcp_mmdeg()
            if cur is None:
                continue
            pos_err = max(abs(cur[i] - target[i]) for i in range(3))
            ang_err = max(abs(cur[i + 3] - target[i + 3]) for i in range(3))
            if pos_err <= self.POS_TOL_MM and ang_err <= self.ANG_TOL_DEG:
                ok_count += 1
            else:
                ok_count = 0
            if ok_count >= self.CONSEC_OK:
                return True
        return False

    # =======================================================
    # CAMERA
    # =======================================================
    def cam_start(self):
        if self.cam_started:
            return
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipe.start(cfg)
        self.align = rs.align(rs.stream.color)
        self.cam_started = True

    def cam_stop(self):
        if not self.cam_started:
            return
        try:
            self.pipe.stop()
        except Exception:
            pass
        self.cam_started = False

    # =======================================================
    # TF, DEPTH
    # =======================================================
    def get_T_base_from_camera(self):
        try:
            tfmsg = self.tf_buffer.lookup_transform(
                'base', 'camera_color_optical_frame', rclpy.time.Time()
            )
            t = tfmsg.transform.translation
            q = tfmsg.transform.rotation
            T = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
            T[:3, 3] = [t.x, t.y, t.z]
            return T
        except Exception:
            return None

    def backproject(self, u, v, Z, intr):
        X = (u - intr.ppx) / intr.fx * Z
        Y = (v - intr.ppy) / intr.fy * Z
        return np.array([X, Y, Z], dtype=float)

    def median_depth(self, depth_frame, cx, cy, k):
        h, w = depth_frame.get_height(), depth_frame.get_width()
        x1, x2 = max(cx - k, 0), min(cx + k, w - 1)
        y1, y2 = max(cy - k, 0), min(cy + k, h - 1)
        vals = []
        for vv in range(y1, y2 + 1):
            for uu in range(x1, x2 + 1):
                z = depth_frame.get_distance(int(uu), int(vv))
                if z > 0:
                    vals.append(z)
        return float(np.median(vals)) if vals else 0.0

    def _normalize(self, v, eps=1e-12):
        n = float(np.linalg.norm(v))
        if n < eps:
            return None
        return v / n

    def _angle_deg(self, a, b, eps=1e-12):
        na = np.linalg.norm(a)
        nb = np.linalg.norm(b)
        if na < eps or nb < eps:
            return 180.0
        c = float(np.dot(a, b) / (na * nb))
        c = max(-1.0, min(1.0, c))
        return math.degrees(math.acos(c))

    # =======================================================
    # Project 3D (base) -> pixel
    # =======================================================
    def _project_base_to_pixel(self, P_base, T_base_cam, intr):
        Rbc = T_base_cam[:3, :3]
        tbc = T_base_cam[:3, 3]
        Rcb = Rbc.T
        P_cam = Rcb @ (np.asarray(P_base, dtype=float) - tbc)
        Z = float(P_cam[2])
        if Z <= 1e-6:
            return None
        u = int(round(intr.fx * (float(P_cam[0]) / Z) + intr.ppx))
        v = int(round(intr.fy * (float(P_cam[1]) / Z) + intr.ppy))
        return (u, v)

    def _draw_tcp_axes(self, img, T_base_cam, intr, origin_base, x_base, y_base, z_base):
        if not self.SHOW_AXES:
            return
        L = float(self.AXIS_LEN_MM) / 1000.0
        p0 = self._project_base_to_pixel(origin_base, T_base_cam, intr)
        if p0 is None:
            return
        px = self._project_base_to_pixel(origin_base + L * x_base, T_base_cam, intr)
        py = self._project_base_to_pixel(origin_base + L * y_base, T_base_cam, intr)
        pz = self._project_base_to_pixel(origin_base + L * z_base, T_base_cam, intr)

        if px is not None:
            cv2.arrowedLine(img, p0, px, (0, 0, 255), 2, tipLength=0.2)
            cv2.putText(img, "X", px, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        if py is not None:
            cv2.arrowedLine(img, p0, py, (0, 255, 0), 2, tipLength=0.2)
            cv2.putText(img, "Y", py, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if pz is not None:
            cv2.arrowedLine(img, p0, pz, (255, 0, 0), 2, tipLength=0.2)
            cv2.putText(img, "Z", pz, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        cv2.drawMarker(img, p0, (255, 255, 255), cv2.MARKER_CROSS, 12, 2)

    # =======================================================
    # PCA Tangent plane normal
    # =======================================================
    def estimate_tangent_normal_pca(self, depth_frame, intr, T_base_cam, u0, v0, P_pick_base):
        h, w = depth_frame.get_height(), depth_frame.get_width()
        Rpx = int(self.ROI_R_PX)

        u1 = max(int(u0) - Rpx, 0)
        u2 = min(int(u0) + Rpx, w - 1)
        v1 = max(int(v0) - Rpx, 0)
        v2 = min(int(v0) + Rpx, h - 1)

        zs = []
        for vv in range(v1, v2 + 1):
            for uu in range(u1, u2 + 1):
                z = depth_frame.get_distance(int(uu), int(vv))
                if z > 0:
                    zs.append(z)
        if len(zs) < int(self.MIN_PTS_RAW):
            return False, None, {"reason": "too_few_depth_points", "n_raw": len(zs)}

        z_med = float(np.median(zs))
        z_band = float(self.Z_BAND_MM) / 1000.0
        neighbor_r = float(self.NEIGHBOR_RADIUS_MM) / 1000.0

        P_pick_base = np.asarray(P_pick_base, dtype=float)
        pts = []

        for vv in range(v1, v2 + 1):
            for uu in range(u1, u2 + 1):
                z = depth_frame.get_distance(int(uu), int(vv))
                if z <= 0:
                    continue
                if abs(z - z_med) > z_band:
                    continue
                P_cam = self.backproject(float(uu), float(vv), float(z), intr)
                P_base = (T_base_cam[:3, :3] @ P_cam) + T_base_cam[:3, 3]
                P_base[2] += self.Z_OFFSET_M
                if float(np.linalg.norm(P_base - P_pick_base)) > neighbor_r:
                    continue
                pts.append(P_base)

        if len(pts) < int(self.MIN_PTS_NEIGHBOR):
            return False, None, {"reason": "too_few_neighbor_points", "n": len(pts)}

        X = np.asarray(pts, dtype=np.float64)

        if bool(self.USE_SOR):
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(X)
            pcd, _ = pcd.remove_statistical_outlier(
                nb_neighbors=int(self.SOR_NB),
                std_ratio=float(self.SOR_STD)
            )
            X = np.asarray(pcd.points)
            if len(X) < int(self.MIN_PTS_NEIGHBOR) * 0.5:
                return False, None, {"reason": "too_few_after_SOR", "n": int(len(X))}

        out_th = float(self.PLANE_OUTLIER_MM) / 1000.0
        iters = int(self.PLANE_REFINE_ITERS)

        def fit_plane_pca(pts_):
            c = pts_.mean(axis=0)
            Y = pts_ - c
            C = (Y.T @ Y) / max(len(pts_) - 1, 1)
            evals, evecs = np.linalg.eigh(C)
            n = self._normalize(evecs[:, 0])
            return c, n, evals

        Xcur = X
        for _ in range(iters):
            c0, n0, evals0 = fit_plane_pca(Xcur)
            if n0 is None:
                return False, None, {"reason": "degenerate_pca_normal"}
            d = (Xcur - c0) @ n0
            mask = np.abs(d) <= out_th
            Xn = Xcur[mask]
            if len(Xn) < int(self.MIN_PTS_NEIGHBOR) * 0.5:
                break
            if len(Xn) == len(Xcur):
                Xcur = Xn
                break
            Xcur = Xn

        c, n, evals = fit_plane_pca(Xcur)
        if n is None:
            return False, None, {"reason": "degenerate_pca_normal_final"}

        if bool(self.FORCE_NORMAL_DOWN):
            if float(np.dot(n, np.array([0.0, 0.0, -1.0]))) < 0.0:
                n = -n

        tilt = self._angle_deg(n, np.array([0.0, 0.0, -1.0], dtype=float))
        planar_score = float(evals[0] / max(evals[1] + evals[2], 1e-12))

        d_all = (X - c) @ n
        n_in = int(np.sum(np.abs(d_all) <= out_th))

        dbg = {
            "reason": "ok",
            "tilt_deg": float(tilt),
            "planar_score": float(planar_score),
            "n_in": int(n_in),
            "n_all": int(len(X)),
        }
        return True, n, dbg

    # =======================================================
    # ORIENTATION
    # =======================================================
    def compute_tcp_rpy_sxyz_from_yvec_and_znorm(self, y_vec_base, z_tcp_base):
        y_tcp = self._normalize(y_vec_base)
        z_tcp = self._normalize(z_tcp_base)
        if y_tcp is None or z_tcp is None:
            return None, None

        y_tcp = y_tcp - np.dot(y_tcp, z_tcp) * z_tcp
        y_tcp = self._normalize(y_tcp)
        if y_tcp is None:
            return None, None

        x_tcp = np.cross(y_tcp, z_tcp)
        x_tcp = self._normalize(x_tcp)
        if x_tcp is None:
            return None, None

        y_tcp = np.cross(z_tcp, x_tcp)
        y_tcp = self._normalize(y_tcp)
        if y_tcp is None:
            return None, None

        M = np.eye(4, dtype=float)
        M[:3, 0] = x_tcp
        M[:3, 1] = y_tcp
        M[:3, 2] = z_tcp

        roll, pitch, yaw = tf_transformations.euler_from_matrix(M, axes='sxyz')
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw)), M

    # =======================================================
    # DETECTION
    # =======================================================
    def capture_one_detection(self):
        T = self.get_T_base_from_camera()
        if T is None:
            return None, None, None, None

        try:
            frames = self.pipe.wait_for_frames(timeout_ms=1000)
            aligned = self.align.process(frames)
            depth = aligned.get_depth_frame()
            color_f = aligned.get_color_frame()
            if not depth or not color_f:
                return None, None, None, None
            color = np.asanyarray(color_f.get_data())
        except Exception:
            return None, None, None, None

        h, w = color.shape[:2]
        mid_y = h / 2.0
        intr = color_f.profile.as_video_stream_profile().intrinsics

        res = self.model.predict(color, verbose=False)
        if len(res) == 0:
            return None, None, color, None

        r = res[0]
        vis = r.plot(labels=False, conf=False)

        if getattr(r, "obb", None) is None or getattr(r.obb, "xywhr", None) is None or len(r.obb.xywhr) == 0:
            return None, None, vis, None

        xywhr = r.obb.xywhr.detach().cpu().numpy()
        centers_px = xywhr[:, :2]

        if getattr(r.obb, "cls", None) is not None:
            cls = r.obb.cls.detach().cpu().numpy().astype(int)
            tooth_id = None
            for k, v in self.model.names.items():
                if str(v).strip().lower() == "tooth-s8ie":
                    tooth_id = int(k)
                    break
            if tooth_id is not None:
                keep = np.where(cls == tooth_id)[0]
                if len(keep) == 0:
                    return None, None, vis, None
                xywhr = xywhr[keep]
                centers_px = centers_px[keep]

        n_det = len(xywhr)
        if n_det < 2:
            return None, None, vis, None

        top_all = [i for i in range(n_det) if centers_px[i, 1] < mid_y]
        bot_all = [i for i in range(n_det) if centers_px[i, 1] >= mid_y]

        P_base_list = [None] * n_det
        Z_base = [None] * n_det
        valid = []
        for i in range(n_det):
            cx_i, cy_i = map(float, centers_px[i, :2])
            zc = self.median_depth(depth, int(cx_i), int(cy_i), self.PATCH_K)
            if zc <= 0:
                continue
            P_cam = self.backproject(cx_i, cy_i, zc, intr)
            P_base = (T[:3, :3] @ P_cam) + T[:3, 3]
            P_base[2] += self.Z_OFFSET_M
            P_base_list[i] = P_base
            Z_base[i] = float(P_base[2])
            valid.append(i)

        if not valid:
            return None, None, vis, None

        i_maxZ = max(valid, key=lambda ii: Z_base[ii])
        region_is_top = centers_px[i_maxZ, 1] < mid_y
        region_name = "TOP" if region_is_top else "BOT"
        region_all = top_all if region_is_top else bot_all

        region_valid = [i for i in region_all if i in valid]
        if len(region_valid) < 2:
            return None, None, vis, None

        region_sorted = sorted(region_valid, key=lambda ii: float(centers_px[ii, 0]), reverse=False)
        cnt = len(region_sorted)

        sel_pos = 1
        if cnt <= sel_pos:
            return None, None, vis, None

        idx_sel = region_sorted[sel_pos]
        idx_left = region_sorted[sel_pos - 1]

        P_sel = P_base_list[idx_sel]
        P_left = P_base_list[idx_left]
        if P_sel is None or P_left is None:
            return None, None, vis, None

        P_pick = P_sel.copy()
        y_vec = (P_left - P_sel)

        cx_s, cy_s = centers_px[idx_sel]
        cx_l, cy_l = centers_px[idx_left]

        u0 = int(round(cx_s))
        v0 = int(round(cy_s))

        ok_n, z_tcp, dbg_n = self.estimate_tangent_normal_pca(
            depth_frame=depth,
            intr=intr,
            T_base_cam=T,
            u0=u0,
            v0=v0,
            P_pick_base=P_pick
        )

        if not ok_n or z_tcp is None:
            z_tcp = np.array([0.0, 0.0, -1.0], dtype=float)

        rpy_deg, M = self.compute_tcp_rpy_sxyz_from_yvec_and_znorm(y_vec, z_tcp)
        if rpy_deg is None or M is None:
            return None, None, vis, None

        rx, ry, rz = rpy_deg

        cv2.circle(vis, (int(cx_s), int(cy_s)), 7, (0, 255, 0), -1)
        cv2.circle(vis, (int(cx_l), int(cy_l)), 7, (255, 0, 0), -1)
        cv2.line(vis, (int(cx_s), int(cy_s)), (int(cx_l), int(cy_l)), (0, 255, 255), 2)
        cv2.drawMarker(vis, (u0, v0), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)

        x_tcp = M[:3, 0].copy()
        y_tcp = M[:3, 1].copy()
        z_tcp_u = M[:3, 2].copy()
        self._draw_tcp_axes(vis, T, intr, origin_base=P_pick, x_base=x_tcp, y_base=y_tcp, z_base=z_tcp_u)

        txt1 = f"REGION={region_name} | cnt={cnt} | FIX sel=2nd (L->R)"
        txt2 = f"P_pick(sel)=({P_pick[0]:.3f},{P_pick[1]:.3f},{P_pick[2]:.3f}) m"
        txt3 = f"Euler sxyz: rx={rx:.1f}, ry={ry:.1f}, rz={rz:.1f}"

        if dbg_n is not None and dbg_n.get("reason") == "ok":
            txt4 = f"PCA OK | tilt={dbg_n['tilt_deg']:.1f} | score={dbg_n['planar_score']:.2e} | n_in={dbg_n['n_in']}"
        else:
            txt4 = f"PCA FAIL -> fallback -Z | {None if dbg_n is None else dbg_n.get('reason')}"

        cv2.putText(vis, txt1, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 0), 2)
        cv2.putText(vis, txt2, (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(vis, txt3, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(vis, txt4, (10, 94), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        out = {
            "region": region_name,
            "count": cnt,
            "sel_pos_L2R": sel_pos,
            "idx_sel": idx_sel,
            "idx_left": idx_left,
            "P_pick_base_m": P_pick,
            "rxryrz_sxyz_deg": (rx, ry, rz),
            "pca_ok": bool(ok_n),
            "z_tcp_base": z_tcp_u,
            "pca_dbg": dbg_n,
        }

        return P_pick, (rx, ry, rz), vis, out

    # =======================================================
    # PICKUP WITH RETRY
    # =======================================================
    def pickup_with_slip_retry(self, x_mm, y_mm, z_mm, rx, ry, rz,
                               x_lift_mm, y_lift_mm, z_lift_mm):
        """
        Logic:
          - move to pickup
          - close gripper
          - store reference status right after close
          - move to lift
          - after reaching lift, poll several times
          - if object lost -> open, go back, increase force, retry
        """
        force_now = self.GRIPPER_FORCE

        for attempt in range(1, self.MAX_REPICK_TRIES + 1):
            self.get_logger().info(
                f"=== PICK ATTEMPT {attempt}/{self.MAX_REPICK_TRIES} | force={force_now} ==="
            )

            # 1) xuống pickup pose
            self.ptp_cpp(x_mm, y_mm, z_mm, rx, ry, rz, 40, 50)
            if not self.wait_until_reached((x_mm, y_mm, z_mm, rx, ry, rz)):
                self.get_logger().warn("Cannot reach pickup pose.")
                continue

            # 2) close gripper
            self.gripper_close(force=force_now)
            time.sleep(self.GRASP_SETTLE_SEC)

            st_close = self.get_gripper_status()
            if st_close is not None:
                self.get_logger().info(
                    f"After close: gOBJ={st_close['gOBJ']} gPO={st_close['gPO']} gCU={st_close['gCU']}"
                )
            else:
                self.get_logger().warn("Could not read gripper status after close.")

            # 3) move lên lift
            self.ptp_cpp(x_lift_mm, y_lift_mm, z_lift_mm, rx, ry, rz, 30, 30)
            reached = self.wait_until_reached((x_lift_mm, y_lift_mm, z_lift_mm, rx, ry, rz))

            if not reached:
                self.get_logger().warn("Failed to reach lift pose.")
                self.gripper_open()
                time.sleep(0.3)
                force_now = min(force_now + self.GRIPPER_FORCE_STEP, self.GRIPPER_FORCE_MAX)
                continue

            # 4) check object lost after lift
            if self.slipped_at_lift(st_close):
                self.get_logger().warn("Object lost at lift -> open, back to pickup, retry.")
                self.gripper_open()
                time.sleep(0.3)

                self.ptp_cpp(x_mm, y_mm, z_mm, rx, ry, rz, 30, 30)
                self.wait_until_reached((x_mm, y_mm, z_mm, rx, ry, rz))

                force_now = min(force_now + self.GRIPPER_FORCE_STEP, self.GRIPPER_FORCE_MAX)
                continue

            self.get_logger().info("Pickup success: object still held at lift.")
            self.GRIPPER_FORCE = force_now
            return True

        return False

    # =======================================================
    # MAIN LOGIC
    # =======================================================
    def run(self):
        self.change_tcp("tcp_obb")

        # go to view
        x_v, y_v, z_v, rx_v, ry_v, rz_v, acc_v, vel_v, coord_v, blend_v = self.VIEW_CPP
        self.ptp_cpp(x_v, y_v, z_v, rx_v, ry_v, rz_v, acc_v, vel_v, coord_v, blend_v)
        self.wait_until_reached((x_v, y_v, z_v, rx_v, ry_v, rz_v))
        self.gripper_open()

        # camera start
        self.cam_start()

        valid_count = 0
        P_pick = None
        rpy_pick = None
        last_vis = None
        last_out = None
        z_tcp_pick = None

        for _ in range(self.MAX_FRAMES):
            rclpy.spin_once(self, timeout_sec=0.01)
            P, rpy, vis, out = self.capture_one_detection()
            if out is not None and ("z_tcp_base" in out):
                z_tcp_pick = out["z_tcp_base"]

            if vis is not None:
                last_vis = vis
                last_out = out
                cv2.imshow("YOLO OBB (PICKUP + PCA Normal + TCP axes)", vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            if P is not None and rpy is not None:
                valid_count += 1
                rx, ry, rz = rpy
                self.get_logger().info(
                    f"sample {valid_count}/{self.NEEDED_SAMPLES}: "
                    f"x={P[0]:.4f}, y={P[1]:.4f}, z={P[2]:.4f} (m) | "
                    f"sxyz(rx,ry,rz)=({rx:.2f},{ry:.2f},{rz:.2f})"
                )
                if valid_count >= self.NEEDED_SAMPLES:
                    P_pick = P
                    rpy_pick = rpy
                    z_tcp_pick = out["z_tcp_base"] if (out is not None and "z_tcp_base" in out) else z_tcp_pick
                    break
                time.sleep(0.3)

        if P_pick is None or rpy_pick is None:
            self.get_logger().error("Not enough valid samples for pickup.")
            self.cam_stop()
            cv2.destroyAllWindows()
            return

        self.cam_stop()
        if last_vis is not None:
            cv2.imshow("YOLO OBB - LAST", last_vis)
            cv2.waitKey(500)
        cv2.destroyAllWindows()

        # ==== MOVE PICK ====
        x_mm, y_mm, z_mm = (P_pick * 1000).tolist()
        rx, ry, rz = rpy_pick

        # 1) go XY (keep view z)
        self.line_cpp(x_mm, y_mm, z_v, rx, ry, rz, 100, 100)
        self.wait_until_reached((x_mm, y_mm, z_v, rx, ry, rz))

        # 2) compute lift point along normal direction
        lift_m = self.LIFT_AFTER_GRASP_MM / 1000.0

        if z_tcp_pick is not None:
            z_tcp_pick = np.asarray(z_tcp_pick, dtype=float)
            nrm = float(np.linalg.norm(z_tcp_pick))
            if nrm > 1e-9:
                z_tcp_u = z_tcp_pick / nrm
                v_up = -z_tcp_u
            else:
                v_up = np.array([0.0, 0.0, 1.0], dtype=float)
        else:
            v_up = np.array([0.0, 0.0, 1.0], dtype=float)

        P_lift = np.asarray(P_pick, dtype=float) + lift_m * v_up
        x_lift_mm, y_lift_mm, z_lift_mm = (P_lift * 1000.0).tolist()

        # 3) pickup with retry
        ok_pick = self.pickup_with_slip_retry(
            x_mm=x_mm,
            y_mm=y_mm,
            z_mm=z_mm,
            rx=rx,
            ry=ry,
            rz=rz,
            x_lift_mm=x_lift_mm,
            y_lift_mm=y_lift_mm,
            z_lift_mm=z_lift_mm
        )

        if not ok_pick:
            self.get_logger().error("Pickup failed after retries.")
            self.ptp_cpp(x_v, y_v, z_v, rx_v, ry_v, rz_v, acc_v, vel_v, coord_v, blend_v)
            self.wait_until_reached((x_v, y_v, z_v, rx_v, ry_v, rz_v))
            return

        # ==== DROP ====
        x_d, y_d, z_d, rx_d, ry_d, rz_d = self.DROP_POSE

        # move ngang tới drop, giữ z_lift_mm
        self.line_cpp(x_d, y_d, z_lift_mm, rx_d, ry_d, rz_d, 100, 100)
        self.wait_until_reached((x_d, y_d, z_lift_mm, rx_d, ry_d, rz_d))

        # xuống z_d
        self.ptp_cpp(x_d, y_d, z_d, rx_d, ry_d, rz_d, 50, 50)
        self.wait_until_reached((x_d, y_d, z_d, rx_d, ry_d, rz_d))

        # thả
        self.gripper_open()
        time.sleep(0.5)

        # ==== RETURN TO VIEW ====
        self.ptp_cpp(x_d, y_d, z_lift_mm, rx_d, ry_d, rz_d, 100, 100)
        self.wait_until_reached((x_d, y_d, z_lift_mm, rx_d, ry_d, rz_d))

        self.ptp_cpp(x_v, y_v, z_v, rx_v, ry_v, rz_v, acc_v, vel_v, coord_v, blend_v)
        self.wait_until_reached((x_v, y_v, z_v, rx_v, ry_v, rz_v))
        self.gripper_close()

        if last_out is not None:
            self.get_logger().info(
                f"DONE. Region={last_out.get('region')} cnt={last_out.get('count')} "
                f"FIX sel=2nd idx_sel={last_out.get('idx_sel')} idx_left={last_out.get('idx_left')} "
                f"pca_ok={last_out.get('pca_ok')} "
                f"sxyz(rx,ry,rz)=({rx:.2f},{ry:.2f},{rz:.2f})"
            )
        else:
            self.get_logger().info("DONE PICK, DROP, RETURN VIEW")

    # =======================================================
    # SEND SCRIPT
    # =======================================================
    def send_script(self, script):
        req = SendScript.Request()
        req.id = "demo"
        req.script = script
        future = self.script_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().ok:
            return True
        return False

    def ptp_cpp(self, x, y, z, rx, ry, rz, acc=100, vel=100, coord=0, blend=False):
        cmd = (
            f'PTP("CPP",{x:.3f},{y:.3f},{z:.3f},'
            f'{rx:.3f},{ry:.3f},{rz:.3f},'
            f'{acc},{vel},{coord},{str(blend).lower()})'
        )
        self.get_logger().info(cmd)
        return self.send_script(cmd)

    def line_cpp(self, x, y, z, rx, ry, rz, vel=100, acc=100, blend=0, precise_disable=False):
        target = f"{{{x:.3f},{y:.3f},{z:.3f},{rx:.3f},{ry:.3f},{rz:.3f}}}"
        cmd = f'Line("CPP", {target}, {vel}, {acc}, {blend}, {str(precise_disable).lower()})'
        self.get_logger().info(cmd)
        return self.send_script(cmd)

    def change_tcp(self, name):
        cmd = f'ChangeTCP("{name}")'
        self.send_script(cmd)


def main():
    rclpy.init()
    node = YOLOPickAndMove()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.cam_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()