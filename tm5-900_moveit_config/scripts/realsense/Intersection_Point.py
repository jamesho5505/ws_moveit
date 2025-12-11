#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

class IntersectionPoint(Node):
    def __init__(self):
        super().__init__('intersection_point')
        self.bridge = CvBridge()

        # --- topics / params ---
        self.color_topic = self.declare_parameter(
            'color_topic', '/camera/camera/color/image_raw').value
        self.depth_topic = self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw').value
        self.info_topic = self.declare_parameter(
            'camera_info_topic', '/camera/camera/color/camera_info').value
        self.pattern_size = (4, 4)  # 5x5 squares => 4x4 inner corners
        self.square_size_mm = 26.5
        self.stabilization_frames = int(
            self.declare_parameter('stabilization_frames', 10).value)

        # --- fixed transform (T_base<-cam) ---
        # Example calibration (update with your measured matrix)
        self.T_base_cam = np.array([
            # [1.00613151, -0.01859431, -0.00712982,  0.09039068],
            # [-0.01389467, -1.00147827,  0.00311466,  0.43842151],
            # [0.00218016,  -0.00799811, -0.99691601,  0.42706775],
            # [0., 0., 0., 1.]


            # [ 0.99913178, -0.02174434, -0.00641554,  0.09032781],
            # [-0.01426083, -1.00084698,  0.00243736,  0.43862181],
            # [-0.00001124, -0.00557938, -0.99934064,  0.42780577],
            # [ 0.,          0.,          0.,          1.        ]

            # [ 1.00641604, -0.0197055,  -0.00690774,  0.09032285],
            # [-0.01510883, -1.0010946,   0.00252583,  0.43853788],
            # [-0.00538898, -0.00476441, -1.00282228,  0.42859171],
            # [ 0.,          0.,          0.,         1.        ]

            # [ 1.00704726, -0.01848521, -0.00672967,  0.09026132],
            # [-0.02012308, -1.00551677,  0.00292352,  0.43834038],
            # [-0.01357108, -0.01585009, -1.01013796,  0.43069252],
            # [ 0.,        0.,          0.,          1.        ]

            [ 1.00704726, -0.01848521, -0.00672967,  0.09026132],
            [-0.02012308, -1.00551677,  0.00292352, 0.43834038],
            [-0.01325381, -0.01659153, -1.00963331,  0.43008273],
            [ 0.,          0.,          0.,          1.,        ]


        ], dtype=np.float64)

        # --- camera info ---
        self.fx = self.fy = self.cx = self.cy = None
        self.K = None
        self.D = None
        self.depth_img = None
        self.depth_enc = None

        # --- state ---
        self.frame_counter = 0
        self.stabilized_frame = None
        self.stabilized_depth = None
        self.processing_done = False

        # --- subs ---
        self.create_subscription(CameraInfo, self.info_topic, self.cb_info, 10)
        self.create_subscription(Image, self.color_topic, self.cb_color, 10)
        self.create_subscription(Image, self.depth_topic, self.cb_depth, 10)

        # --- windows ---
        cv.namedWindow('Frame 1 - Stabilization', cv.WINDOW_AUTOSIZE)
        cv.namedWindow('Frame 2 - Selected Points', cv.WINDOW_AUTOSIZE)
        cv.namedWindow('Frame 3 - Results', cv.WINDOW_AUTOSIZE)

        self.get_logger().info(f"Initialized. Waiting {self.stabilization_frames} frames...")

    # -------------------------------------------------
    def cb_info(self, msg: CameraInfo):
        self.K = np.array(msg.k).reshape(3, 3)
        self.D = np.array(msg.d)
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]

    def cb_depth(self, msg: Image):
        self.depth_enc = msg.encoding
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # -------------------------------------------------
    def cb_color(self, msg: Image):
        if self.fx is None or self.depth_img is None:
            return
        if self.processing_done:
            key = cv.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                rclpy.shutdown()
            return

        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.frame_counter += 1

        # stabilization display
        stab = bgr.copy()
        cv.putText(stab, f"Stabilizing... {self.frame_counter}/{self.stabilization_frames}",
                   (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv.imshow('Frame 1 - Stabilization', stab)
        cv.waitKey(1)

        if self.frame_counter < self.stabilization_frames:
            return

        self.stabilized_frame = bgr.copy()
        self.stabilized_depth = self.depth_img.copy()
        self.get_logger().info("Stabilized. Processing single frame...")
        self.process_single_frame()
        self.processing_done = True

    # -------------------------------------------------
    def process_single_frame(self):
        bgr = self.stabilized_frame
        depth = self.stabilized_depth
        gray = cv.cvtColor(bgr, cv.COLOR_BGR2GRAY)

        ok, corners = cv.findChessboardCornersSB(gray, self.pattern_size)
        if not ok:
            self.get_logger().error("Checker not found.")
            return

        corners = cv.cornerSubPix(
            gray, corners, (5, 5), (-1, -1),
            (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_COUNT, 30, 0.001))
        corners = corners.reshape(-1, 2)

        # chọn 4 góc trung tâm
        ids = [5, 6, 9, 10]
        pts = corners[ids]

        # --- hiển thị các góc ---
        img_sel = bgr.copy()
        for i, (u, v) in zip(ids, pts):
            cv.circle(img_sel, (int(round(u)), int(round(v))), 1, (0, 255, 0), -1)
            cv.putText(img_sel, f"{i}", (int(u) + 5, int(v) - 5),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv.imshow('Frame 2 - Selected Points', img_sel)

        # --- tính tọa độ ---
        results = []
        for idx, (u, v) in zip(ids, pts):
            z = self.get_depth_mm(int(v), int(u))
            if z is None:
                continue
            Xc = self.pixel_to_cam(u, v, z)
            Xb = self.cam_to_base(Xc)
            results.append((idx, (u, v), z, Xc, Xb))

        # --- hiển thị kết quả ---
        info = np.zeros((400, 900, 3), dtype=np.uint8)
        y = 30
        cv.putText(info, "=== CENTRAL 4 CORNERS ===",
                   (10, y), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        y += 35
        cv.putText(info, "1) Camera Coordinates (m):",
                   (10, y), cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1)
        y += 25
        for idx, _, _, Xc, _ in results:
            cv.putText(info, f"Corner {idx:2d}: ({Xc[0]:.6f}, {Xc[1]:.6f}, {Xc[2]:.6f})",
                       (10, y), cv.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
            y += 20
        y += 10
        cv.putText(info, "2) Base Coordinates (m):",
                   (10, y), cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1)
        y += 25
        for idx, _, _, _, Xb in results:
            cv.putText(info, f"Corner {idx:2d}: ({Xb[0]:.6f}, {Xb[1]:.6f}, {Xb[2]:.6f})",
                       (10, y), cv.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
            y += 20

        cv.imshow('Frame 3 - Results', info)
        cv.waitKey(1)

        self.get_logger().info("==== RESULTS ====")
        for idx, (u, v), z, Xc, Xb in results:
            self.get_logger().info(
                f"id{idx}: pix=({u:.3f},{v:.3f}) depth={z:.1f}mm "
                f"cam=({Xc[0]:.6f},{Xc[1]:.6f},{Xc[2]:.6f})m "
                f"base=({Xb[0]:.6f},{Xb[1]:.6f},{Xb[2]:.6f})m")

    # -------------------------------------------------
    def get_depth_mm(self, r, c):
        if self.depth_img is None:
            return None
        d = self.depth_img[r, c]
        if '16UC1' in str(self.depth_enc):
            return float(d)
        if '32FC1' in str(self.depth_enc):
            return float(d) * 1000.0
        return float(d)

    def pixel_to_cam(self, u, v, z_mm):
        x = (u - self.cx) * z_mm / self.fx / 1000.0
        y = (v - self.cy) * z_mm / self.fy / 1000.0
        z = z_mm / 1000.0
        return np.array([x, y, z], dtype=np.float64)

    def cam_to_base(self, Xc):
        """Dùng ma trận cố định T_base<-cam"""
        Xh = np.array([Xc[0], Xc[1], Xc[2], 1.0], dtype=np.float64)
        Xb = self.T_base_cam @ Xh
        return Xb[:3]

# -------------------------------------------------
def main():
    rclpy.init()
    node = IntersectionPoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
