#!/usr/bin/env python3

# kw_tcp2 130 -300 120 180 0 180
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from tm_msgs.msg import FeedbackState
from tm_msgs.srv import SendScript
import time


class CircleContourAveragingVerifier(Node):
    def __init__(self):
        super().__init__("circle_contour_averaging_verifier")

        # ===============================
        # Hand–Eye: camera_optical → flange
        # ===============================
        self.T_flange_camera_optical= np.array([
            [-0.99926981,  0.03763377, -0.00659936,  0.0342429 ],
            [-0.03773388, -0.99916346,  0.01576506, -0.08719725],
            [-0.00600054,  0.01600256,  0.99985395,  0.06636586],
            [ 0.,          0.,          0.,          1.]
        ])

        # Camera
        self.camera_matrix = None
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_depth_image = None

        # Robot
        self.latest_flange_pose = None
        self.latest_tcp_pose = None

        # Capture control
        self.capture_count = 10
        self.current_capture = 0
        self.is_capturing = False
        self.T_base_flange_fixed = None
        self.all_contours_base = []
        self.mean_contour = None


        # Parameters
        self.num_resample_points = 36
        self.display_indices = [0, 9, 18, 27]  # 0°,90°,180°,270°

        # ROS
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info",
                                 self.cb_info, 10)
        self.create_subscription(Image, "/camera/camera/color/image_raw",
                                 self.cb_image, 10)
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw",
                                 self.cb_depth, 10)
        self.create_subscription(FeedbackState, "/feedback_states",
                                 self.cb_feedback, 10)

        cv2.namedWindow("Contour Averaging", cv2.WINDOW_NORMAL)

        self.get_logger().info("=" * 70)
        self.get_logger().info("Circle contour averaging verifier (10-frame)")
        self.get_logger().info("SPACE: capture 10 frames")
        self.get_logger().info("=" * 70)

        self.send_script_client = self.create_client(SendScript, 'send_script')
        while not self.send_script_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 send_script 服務...')

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------

    def cb_info(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("✓ Camera intrinsics loaded")

    def cb_feedback(self, msg):
        self.latest_flange_pose = np.array(msg.tool0_pose, dtype=float)
        self.latest_tcp_pose = np.array(msg.tool_pose, dtype=float)

    def cb_depth(self, msg):
        # 將原始資料轉為 numpy
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.latest_depth_image = depth_image

        # --- 視覺化優化 ---
        # 1. 將深度限制在 0-2 公尺之間 (2000mm)，超過的會變黑色
        max_dist = 2000 
        depth_display = np.where((depth_image > 0) & (depth_image < max_dist), depth_image, 0)
        
        # 2. 轉換為 8-bit 影像以便顯示 (縮放比例約 255/2000)
        depth_vis = cv2.convertScaleAbs(depth_display, alpha=0.12) 
        
        # 3. 套用彩色濾鏡 (Jet 模式：近處紅/橘，遠處藍)
        depth_colormap = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        # 4. 如果已經偵測到圓心，在深度圖上畫個十字標記，確認該位置是否有深度值
        if hasattr(self, 'circle_cx') and self.circle_cx is not None:
            cv2.drawMarker(depth_colormap, (self.circle_cx, self.circle_cy), 
                        (255, 255, 255), cv2.MARKER_CROSS, 20, 2)

        cv2.imshow("RealSense Depth Monitor", depth_colormap)
        cv2.waitKey(1)


    def cb_image(self, msg):
        if self.camera_matrix is None or self.latest_flange_pose is None:
            return
        
        if self.latest_depth_image is not None:
            depth_mm = self.latest_depth_image.astype(np.float32)

            # 你實際工作距離（依你桌面，大約 300~600mm）
            min_mm = 300.0
            max_mm = 600.0

            # 無效值 mask
            valid_mask = depth_mm > 0

            depth_vis = np.zeros_like(depth_mm, dtype=np.uint8)

            depth_clipped = np.clip(depth_mm, min_mm, max_mm)
            depth_norm = (depth_clipped - min_mm) / (max_mm - min_mm)
            depth_norm = (depth_norm * 255).astype(np.uint8)

            depth_vis[valid_mask] = depth_norm[valid_mask]

            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            cv2.imshow("Depth Debug", depth_vis)


        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        display = image.copy()

        contour_pixels = self.detect_circle_contour(image)
        if contour_pixels is None:
            return
        (self.circle_cx, self.circle_cy), radius = cv2.minEnclosingCircle(contour_pixels)
        self.circle_cx = int(self.circle_cx)
        self.circle_cy = int(self.circle_cy)

        # 顯示圓心
        cv2.drawMarker(
            display,
            (self.circle_cx, self.circle_cy),
            (0, 255, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=30,
            thickness=2
        )

        cv2.putText(
            display,
            "Center",
            (self.circle_cx + 10, self.circle_cy - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )



        if contour_pixels is None:
            cv2.imshow("Contour Averaging", display)
            cv2.waitKey(1)
            return

        resampled_pixels = self.resample_contour(contour_pixels,
                                                  self.num_resample_points)

        # 顯示選定的代表點（像素）
        for idx in self.display_indices:
            u, v = resampled_pixels[idx]
            cv2.circle(display, (int(u), int(v)), 6, (0, 0, 255), -1)
            cv2.putText(display, f"{idx}", (int(u) + 8, int(v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 1)

        cv2.imshow("Contour Averaging", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):
            self.start_capture(resampled_pixels)
        elif key == ord('c'):
            if hasattr(self, "circle_center_base"):
                self.get_logger().info("Move to circle center")
                self.move_robot(self.circle_center_base)
        elif key == ord('m'):
            idx = 0  # or 9, 18, 27
            target_point = self.mean_contour[idx]  # shape = (3,)
            self.get_logger().info(f"Moving robot to point index {idx}: {target_point}")
            self.move_robot(target_point)
        elif key == ord('q'):
            rclpy.shutdown()
       

        if self.is_capturing:
            self.process_capture(resampled_pixels)

    # --------------------------------------------------
    # Capture logic
    # --------------------------------------------------

    def start_capture(self, resampled_pixels):
        self.get_logger().info("▶ Start capturing 10 frames")
        self.is_capturing = True
        self.current_capture = 0
        self.all_contours_base.clear()
        self.T_base_flange_fixed = self.pose6d_to_T(self.latest_flange_pose)

    def process_capture(self, resampled_pixels):
        contour_base = []
        depth = self.get_depth_at_center(self.circle_cx, self.circle_cy)
        if depth is None:
            self.get_logger().warn("Center depth invalid, skip frame")
            return

        for (u, v) in resampled_pixels:
            if depth is None:
                continue  # 深度不可靠就跳過這個點
            p_camera = self.pixel_to_camera_optical(u, v, depth)
            p_flange = self.T_flange_camera_optical @ p_camera
            p_base = self.T_base_flange_fixed @ p_flange
            contour_base.append(p_base[:3])

        if len(contour_base) < self.num_resample_points * 0.5:
            self.get_logger().warn("Too few valid depth points, skip frame")
            return
        self.all_contours_base.append(np.array(contour_base))
        self.current_capture += 1
        time.sleep(0.05)  # 避免 pipeline jitter

        self.get_logger().info(f"Captured {self.current_capture}/{self.capture_count}")

        if self.current_capture >= self.capture_count:
            self.finish_capture()




    # def finish_capture(self):
    #     self.is_capturing = False
    #     contours = np.stack(self.all_contours_base, axis=0)
    #     mean_contour = contours.mean(axis=0)
    #     self.mean_contour = mean_contour

    #     self.get_logger().info("=" * 60)
    #     self.get_logger().info("Averaged contour base points:")
    #     for idx in self.display_indices:
    #         p = mean_contour[idx]
    #         self.get_logger().info(
    #             f"  idx {idx:2d}: [{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}]"
    #         )
    #     self.get_logger().info("=" * 60)

    def finish_capture(self):
        self.is_capturing = False

        # contours shape: (N_frames, N_points, 3)
        contours_base = np.stack(self.all_contours_base, axis=0)

        # === (1) 輪廓平均 ===
        mean_contour_base = contours_base.mean(axis=0)   # (N_points, 3)
        self.mean_contour = mean_contour_base

        # === (2) 幾何圓心（3D）===
        circle_center_base = mean_contour_base.mean(axis=0)
        self.circle_center_base = circle_center_base
        self.circle_center_base[2] = 50.0

        self.get_logger().info("=" * 60)
        self.get_logger().info("Averaged contour points (base frame):")
        for idx in self.display_indices:
            p = mean_contour_base[idx]
            self.get_logger().info(
                f"  idx {idx:2d}: [{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}]"
            )

        self.get_logger().info(
            f"Circle center (base): "
            f"[{circle_center_base[0]:.4f}, "
            f"{circle_center_base[1]:.4f}, "
            f"{circle_center_base[2]:.4f}]"
        )
        self.get_logger().info("=" * 60)

        # === (3) 建立圓形軌跡 ===
        self.circle_trajectory_base = self.build_circle_trajectory(
            mean_contour_base,
            close_loop=True
        )

    def build_circle_trajectory(self, contour_points_base, close_loop=True):
        """
        contour_points_base: (N, 3) in base frame
        return: list of 3D points (trajectory)
        """

        trajectory_points = []

        for point_base in contour_points_base:
            trajectory_points.append(point_base)

        if close_loop:
            trajectory_points.append(contour_points_base[0])

        return trajectory_points



    # --------------------------------------------------
    # Geometry utilities
    # --------------------------------------------------

    def detect_circle_contour(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(cv2.medianBlur(gray, 7), 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        if not contours:
            return None
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 800:
            return None
        return c.reshape(-1, 2)

    def resample_contour(self, contour, n):
        idx = np.linspace(0, len(contour) - 1, n).astype(int)
        return contour[idx]

    def pixel_to_camera_optical(self, u, v, depth):
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        return np.array([x, y, depth, 1.0])

    def pose6d_to_T(self, pose):
        T = np.eye(4)
        T[:3, 3] = pose[:3]
        T[:3, :3] = R.from_euler("zyx", pose[3:], degrees=False).as_matrix()
        return T

    def get_depth_from_ring(self, u, v, inner_r=5, outer_r=20):
        """
        在 (u,v) 周圍取一圈 depth，回傳 median depth（公尺）
        """
        if self.latest_depth_image is None:
            return None

        h, w = self.latest_depth_image.shape
        depths = []

        for angle in np.linspace(0, 2*np.pi, 36):
            for r in range(inner_r, outer_r, 2):
                x = int(u + r * np.cos(angle))
                y = int(v + r * np.sin(angle))
                if 0 <= x < w and 0 <= y < h:
                    d = self.latest_depth_image[y, x]
                    if d > 0:
                        depths.append(d)

        if len(depths) < 10:
            return None

        depth_mm = np.median(depths)
        return depth_mm / 1000.0  # 轉成公尺
    
    def get_depth_at_center(self, cx, cy, half_size=20):
        if self.latest_depth_image is None:
            return None

        h, w = self.latest_depth_image.shape
        depths = []

        for dx in range(-half_size, half_size):
            for dy in range(-half_size, half_size):
                x = cx + dx
                y = cy + dy
                if 0 <= x < w and 0 <= y < h:
                    d = self.latest_depth_image[y, x]
                    if d > 0:
                        depths.append(d)

        if len(depths) < 50:
            return None

        return np.median(depths) / 1000.0

    
    def send_script_command(self, script_text: str):
        request = SendScript.Request()
        request.id = str(int(time.time() * 1000) % 100000)
        request.script = script_text
        self.send_script_client.call_async(request)
    
    def move_robot(self, target_pose):
        current_time = time.time()
        x_mm = target_pose[0] * 1000
        y_mm = target_pose[1] * 1000
        # z_mm = target_pose[2] * 1000
        z_mm = 50.0  # 固定高度 3 cm

        # 保持當前姿態 (rad → deg)
        rx_deg = 180
        ry_deg = 0
        rz_deg = -90.0

        # 正規化到 [-180, 180]
        def norm(a):
            while a > 180: a -= 360
            while a < -180: a += 360
            return a
        
        # rx_deg = norm(rx_deg) * np.pi / 180
        # ry_deg = norm(ry_deg) * np.pi / 180
        # rz_deg = norm(rz_deg) * np.pi / 180

        script = f'PTP("CPP",{x_mm:.1f},{y_mm:.1f},{z_mm:.1f},{rx_deg:.2f},{ry_deg:.2f},{rz_deg:.2f},10,150,0,false)'
        
        self.send_script_command(script)
        self.get_logger().info(f"Moving robot to {target_pose} at {current_time}")


def main():
    rclpy.init()
    node = CircleContourAveragingVerifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
