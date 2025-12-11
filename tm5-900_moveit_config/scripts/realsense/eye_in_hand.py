# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import csv
# from tm_msgs.msg import FeedbackState
# from std_srvs.srv import Trigger

# class SimplifiedHandEyeCollector(Node):
#     def __init__(self):
#         super().__init__("simplified_handeye_collector")

#         # Chessboard
#         self.cols = 7
#         self.rows = 9
#         self.square_size = 0.010
        
#         # 固定記錄的角點編號
#         self.target_corner_id = 0  # 左上角第一個點
        
#         # Camera
#         self.camera_matrix = None
#         self.dist_coeffs = None
#         self.bridge = CvBridge()
#         self.latest_color = None

#         # Robot state
#         self.latest_robot_pose = None
        
#         # 暫存當前拍攝的相機座標
#         self.pending_camera_coords = None
        
#         # CSV 輸出
#         self.csv_path = "handeye_calibration_data.csv"
#         with open(self.csv_path, "w", newline="") as f:
#             writer = csv.writer(f)
#             writer.writerow(["sample_id", "Xc", "Yc", "Zc", "Xr", "Yr", "Zr"])
        
#         self.sample_id = 0

#         # Subscribers
#         self.create_subscription(
#             CameraInfo, "/camera/camera/color/camera_info", self.cb_info, 10
#         )
#         self.create_subscription(
#             Image, "/camera/camera/color/image_raw", self.cb_color, 10
#         )
#         self.create_subscription(
#             FeedbackState, "/feedback_states", self.cb_feedback, 10
#         )

#         # Services
#         self.create_service(Trigger, 'capture_camera', self.srv_capture_camera)
#         self.create_service(Trigger, 'record_touch', self.srv_record_touch)

#         cv2.namedWindow("camera_view", cv2.WINDOW_NORMAL)

#         self.get_logger().info("=== Simplified Hand-Eye Calibration (Correct Method) ===")
#         self.get_logger().info("Method: Multiple captures, one corner each time")
#         self.get_logger().info("")
#         self.get_logger().info("Setup:")
#         self.get_logger().info("  1. FIXED chessboard on table")
#         self.get_logger().info("  2. Camera on robot arm")
#         self.get_logger().info(f"  3. Always recording corner #{self.target_corner_id}")
#         self.get_logger().info("")
#         self.get_logger().info("Workflow (repeat 20-30 times):")
#         self.get_logger().info("  1. Move robot to a NEW position")
#         self.get_logger().info("  2. Press SPACE → Capture corner #0 camera coords")
#         self.get_logger().info("  3. Move TCP to touch corner #0")
#         self.get_logger().info("  4. Press 'r' → Record TCP position")
#         self.get_logger().info("  5. Repeat from step 1")
#         self.get_logger().info("")
#         self.get_logger().info("Goal: 20-30 samples with varied robot poses")
#         self.get_logger().info("")

#     def cb_info(self, msg):
#         if self.camera_matrix is None:
#             self.camera_matrix = np.array(msg.k).reshape(3, 3)
#             self.dist_coeffs = np.array(msg.d)
#             self.get_logger().info("✓ Camera intrinsics loaded")

#     def cb_feedback(self, msg: FeedbackState):
#         self.latest_robot_pose = msg.tool_pose

#     def cb_color(self, msg):
#         self.latest_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
#         display_img = self.latest_color.copy()
        
#         # 顯示狀態
#         if self.pending_camera_coords is None:
#             text = "Step 1: Press SPACE to capture"
#             color = (0, 255, 0)
#         else:
#             text = f"Step 2: Touch corner #{self.target_corner_id}, press 'r'"
#             color = (0, 0, 255)
        
#         cv2.putText(display_img, text, (10, 30), 
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
#         cv2.putText(display_img, f"Samples: {self.sample_id}", (10, 70),
#                     cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
        
#         if self.latest_robot_pose is not None:
#             robot_text = f"TCP: X={self.latest_robot_pose[0]:.5f} Y={self.latest_robot_pose[1]:.5f} Z={self.latest_robot_pose[2]:.5f}"
#             cv2.putText(display_img, robot_text, (10, 110),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
#         cv2.imshow("camera_view", display_img)
        
#         key = cv2.waitKey(10) & 0xFF

#         if key == ord(' '):
#             self.capture_camera_coordinates()
#         elif key == ord('r'):
#             self.record_touch_position()
#         elif key == ord('q'):
#             self.get_logger().info("Exiting...")
#             self.get_logger().info(f"Total samples: {self.sample_id}")
#             if self.sample_id >= 20:
#                 self.get_logger().info("✓ Enough samples collected!")
#             else:
#                 self.get_logger().warn(f"⚠ Only {self.sample_id} samples. Recommend 20-30.")
#             rclpy.shutdown()

#     def srv_capture_camera(self, request, response):
#         success = self.capture_camera_coordinates()
#         response.success = success
#         return response

#     def srv_record_touch(self, request, response):
#         success = self.record_touch_position()
#         response.success = success
#         return response

#     def capture_camera_coordinates(self):
#         """步驟 1：拍攝並計算目標角點的相機座標"""
#         if self.latest_color is None or self.camera_matrix is None:
#             self.get_logger().warn("Camera not ready")
#             return False

#         if self.pending_camera_coords is not None:
#             self.get_logger().warn("Already captured! Press 'r' first or move to new pose")
#             return False

#         gray = cv2.cvtColor(self.latest_color, cv2.COLOR_BGR2GRAY)
#         found, corners = cv2.findChessboardCornersSB(
#             gray, (self.cols, self.rows),
#             flags=cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY
#         )

#         if not found:
#             self.get_logger().warn("⚠ Chessboard NOT found!")
#             return False

#         # 建立 3D 模型
#         objp = np.zeros((self.cols * self.rows, 3), np.float32)
#         objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
#         objp *= self.square_size

#         # PnP：計算棋盤格到相機的位姿
#         ok, rvec, tvec = cv2.solvePnP(
#             objp, corners, self.camera_matrix, self.dist_coeffs
#         )
#         if not ok:
#             self.get_logger().warn("⚠ PnP failed!")
#             return False

#         R, _ = cv2.Rodrigues(rvec)
#         t = tvec.reshape(3, 1)

#         # 只計算目標角點的相機座標
#         p_obj = objp[self.target_corner_id].reshape(3, 1)
#         p_cam = R @ p_obj + t
#         Xc, Yc, Zc = p_cam.flatten()

#         # 暫存
#         self.pending_camera_coords = (Xc, Yc, Zc)

#         # 視覺化：高亮目標角點
#         img_show = self.latest_color.copy()
#         cv2.drawChessboardCorners(img_show, (self.cols, self.rows), corners, True)
        
#         # 高亮目標角點
#         corner_pixel = corners[self.target_corner_id].ravel()
#         px, py = int(corner_pixel[0]), int(corner_pixel[1])
#         cv2.circle(img_show, (px, py), 20, (0, 0, 255), 4)
#         cv2.putText(img_show, f"Touch corner #{self.target_corner_id}", 
#                     (px + 25, py), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
#         cv2.imshow("camera_view", img_show)
#         cv2.waitKey(2000)

#         self.get_logger().info(f"✓ Camera: X={Xc:.4f} Y={Yc:.4f} Z={Zc:.4f}")
#         self.get_logger().info("→ Now touch the RED corner and press 'r'")
        
#         return True

#     def record_touch_position(self):
#         """步驟 2：記錄 TCP 觸碰位置"""
#         if self.pending_camera_coords is None:
#             self.get_logger().warn("⚠ Capture camera coordinates first! (Press SPACE)")
#             return False

#         if self.latest_robot_pose is None:
#             self.get_logger().error("⚠ No robot feedback!")
#             return False

#         # 相機座標（剛才拍攝時計算的）
#         Xc, Yc, Zc = self.pending_camera_coords
        
#         # Robot 座標（當前 TCP 位置）
#         Xr, Yr, Zr = self.latest_robot_pose[:3]

#         # 寫入 CSV
#         with open(self.csv_path, "a", newline="") as f:
#             writer = csv.writer(f)
#             writer.writerow([self.sample_id, Xc, Yc, Zc, Xr, Yr, Zr])

#         self.get_logger().info(f"✓ Sample #{self.sample_id} recorded")
#         self.get_logger().info(f"  Camera: ({Xc:.4f}, {Yc:.4f}, {Zc:.4f})")
#         self.get_logger().info(f"  Robot:  ({Xr:.4f}, {Yr:.4f}, {Zr:.4f})")
#         self.get_logger().info("")
#         self.get_logger().info("→ Move robot to NEW position and press SPACE")
#         self.get_logger().info("")
        
#         self.sample_id += 1
#         self.pending_camera_coords = None  # 重置
        
#         return True


# def main():
#     rclpy.init()
#     node = SimplifiedHandEyeCollector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from tm_msgs.msg import FeedbackState
from std_srvs.srv import Trigger
import math

class EyeInHandCalibration(Node):
    def __init__(self):
        super().__init__("eye_in_hand_calibration")

        # Chessboard
        self.cols = 7
        self.rows = 9
        self.square_size = 0.010
        
        # Camera
        self.camera_matrix = None
        self.dist_coeffs = None
        self.bridge = CvBridge()
        self.latest_color = None

        # Robot state
        self.latest_robot_pose = None  # [X, Y, Z, Rx, Ry, Rz]
        
        # 數據存儲
        self.R_gripper2base_list = []  # Flange → Base 的旋轉
        self.t_gripper2base_list = []  # Flange → Base 的平移
        self.R_target2cam_list = []    # 棋盤格 → 相機的旋轉
        self.t_target2cam_list = []    # 棋盤格 → 相機的平移
        
        self.sample_id = 0

        # Subscribers
        self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", self.cb_info, 10
        )
        self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.cb_color, 10
        )
        self.create_subscription(
            FeedbackState, "/feedback_states", self.cb_feedback, 10
        )

        # Services
        self.create_service(Trigger, 'capture_pose', self.srv_capture_pose)

        cv2.namedWindow("camera_view", cv2.WINDOW_NORMAL)

        self.get_logger().info("=" * 70)
        self.get_logger().info("Eye-in-Hand Calibration (Standard Method)")
        self.get_logger().info("=" * 70)
        self.get_logger().info("Setup:")
        self.get_logger().info("  - Camera mounted on robot arm (flange)")
        self.get_logger().info("  - Chessboard FIXED on table")
        self.get_logger().info("  - NO touching required!")
        self.get_logger().info("")
        self.get_logger().info("Instructions:")
        self.get_logger().info("  1. Fix chessboard on table (DO NOT MOVE IT)")
        self.get_logger().info("  2. Move robot to different positions/angles")
        self.get_logger().info("  3. Press SPACE to capture each pose")
        self.get_logger().info("  4. Collect 20-30 samples")
        self.get_logger().info("  5. Press 'q' to compute calibration")
        self.get_logger().info("")
        self.get_logger().info("Tips:")
        self.get_logger().info("  - Vary X, Y, Z positions")
        self.get_logger().info("  - Vary Rx, Ry, Rz angles")
        self.get_logger().info("  - Keep chessboard fully visible")
        self.get_logger().info("=" * 70)
        self.get_logger().info("")

    def cb_info(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("✓ Camera intrinsics loaded")

    def cb_feedback(self, msg: FeedbackState):
        # 使用 tool0_pose（= flange）
        self.latest_robot_pose = msg.tool0_pose  # [X, Y, Z, Rx, Ry, Rz]

    def cb_color(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        display_img = self.latest_color.copy()
        
        cv2.putText(display_img, "Press SPACE to capture pose", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(display_img, f"Samples: {self.sample_id}/20-30", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
        
        if self.latest_robot_pose is not None:
            robot_text = f"Flange: X={self.latest_robot_pose[0]:.3f} Y={self.latest_robot_pose[1]:.3f} Z={self.latest_robot_pose[2]:.3f}"
            cv2.putText(display_img, robot_text, (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        cv2.imshow("camera_view", display_img)
        
        key = cv2.waitKey(10) & 0xFF

        if key == ord(' '):
            self.capture_pose()
        elif key == ord('q'):
            self.finish_calibration()
            rclpy.shutdown()

    def srv_capture_pose(self, request, response):
        success = self.capture_pose()
        response.success = success
        response.message = f"Sample {self.sample_id-1} captured" if success else "Failed"
        return response

    def capture_pose(self):
        """捕捉一組位姿數據（不需要觸碰）"""
        if self.latest_color is None or self.camera_matrix is None:
            self.get_logger().warn("Camera not ready")
            return False

        if self.latest_robot_pose is None:
            self.get_logger().warn("Robot feedback not ready")
            return False

        # 檢測棋盤格
        gray = cv2.cvtColor(self.latest_color, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(
            gray, (self.cols, self.rows),
            flags=cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY
        )

        if not found:
            self.get_logger().warn("⚠ Chessboard NOT found!")
            return False

        # 建立 3D 棋盤格模型
        objp = np.zeros((self.cols * self.rows, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        objp *= self.square_size

        # PnP：計算棋盤格相對相機的位姿
        ok, rvec, tvec = cv2.solvePnP(
            objp, corners, self.camera_matrix, self.dist_coeffs
        )
        if not ok:
            self.get_logger().warn("⚠ PnP failed!")
            return False

        R_target2cam, _ = cv2.Rodrigues(rvec)
        t_target2cam = tvec.reshape(3, 1)

        # 從 robot feedback 取得 Flange 位姿
        X, Y, Z, Rx, Ry, Rz = self.latest_robot_pose

        # 將 RPY 轉換為旋轉矩陣
        R_gripper2base = self.rpy_to_rotation_matrix(Rx, Ry, Rz)
        t_gripper2base = np.array([[X], [Y], [Z]])

        # 儲存數據
        self.R_gripper2base_list.append(R_gripper2base)
        self.t_gripper2base_list.append(t_gripper2base)
        self.R_target2cam_list.append(R_target2cam)
        self.t_target2cam_list.append(t_target2cam)

        # 視覺化
        img_show = self.latest_color.copy()
        cv2.drawChessboardCorners(img_show, (self.cols, self.rows), corners, True)
        cv2.imshow("camera_view", img_show)
        cv2.waitKey(1000)

        self.get_logger().info(f"✓ Sample #{self.sample_id} captured")
        self.sample_id += 1
        
        return True

    def finish_calibration(self):
        """計算手眼標定"""
        if len(self.R_gripper2base_list) < 5:
            self.get_logger().error(f"Not enough samples! Got {len(self.R_gripper2base_list)}, need at least 5")
            return

        self.get_logger().info("")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Computing calibration with {len(self.R_gripper2base_list)} samples...")
        self.get_logger().info("=" * 70)

        # 使用 OpenCV 的 calibrateHandEye
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            self.R_gripper2base_list,
            self.t_gripper2base_list,
            self.R_target2cam_list,
            self.t_target2cam_list,
            method=cv2.CALIB_HAND_EYE_TSAI
        )

        # 組成 4x4 轉換矩陣 (Camera → Flange)
        T_cam2gripper = np.eye(4)
        T_cam2gripper[:3, :3] = R_cam2gripper
        T_cam2gripper[:3, 3] = t_cam2gripper.flatten()

        # 計算 Flange → Camera（用於 URDF）
        T_gripper2cam = self.invT(T_cam2gripper)

        np.set_printoptions(suppress=True, precision=9)
        
        print("\n" + "="*70)
        print("Calibration Result")
        print("="*70)
        print("\nT_flange2cam (Flange → Camera):")
        print(T_gripper2cam)
        
        xyz = T_gripper2cam[:3, 3]
        rpy = self.rot_to_rpy_xyz(T_gripper2cam[:3, :3])
        
        print("\n" + "="*70)
        print("URDF <origin> tag:")
        print("="*70)
        print(f'<origin xyz="{xyz[0]:.9f} {xyz[1]:.9f} {xyz[2]:.9f}"')
        print(f'        rpy="{rpy[0]:.9f} {rpy[1]:.9f} {rpy[2]:.9f}" />')
        
        print("\n" + "="*70)
        print("Translation:")
        print("="*70)
        print(f"  X: {xyz[0]:+.6f} m ({xyz[0]*1000:+.2f} mm)")
        print(f"  Y: {xyz[1]:+.6f} m ({xyz[1]*1000:+.2f} mm)")
        print(f"  Z: {xyz[2]:+.6f} m ({xyz[2]*1000:+.2f} mm)")
        print(f"  Distance: {np.linalg.norm(xyz):.3f} m ({np.linalg.norm(xyz)*1000:.1f} mm)")
        
        print("\n" + "="*70)
        print("Rotation:")
        print("="*70)
        print(f"  Roll:  {math.degrees(rpy[0]):+.3f}°")
        print(f"  Pitch: {math.degrees(rpy[1]):+.3f}°")
        print(f"  Yaw:   {math.degrees(rpy[2]):+.3f}°")
        print("="*70)
        
        # 保存
        np.save("hand_eye_calibration.npy", T_gripper2cam)
        print("\n✓ Saved to 'hand_eye_calibration.npy'")

    @staticmethod
    def invT(T: np.ndarray) -> np.ndarray:
        R, t = T[:3, :3], T[:3, 3]
        Ti = np.eye(4)
        Ti[:3, :3] = R.T
        Ti[:3, 3] = -R.T @ t
        return Ti

    @staticmethod
    def rpy_to_rotation_matrix(rx, ry, rz):
        """RPY 角度轉旋轉矩陣（單位：弧度）"""
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])
        Ry = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])
        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])
        return Rz @ Ry @ Rx

    @staticmethod
    def rot_to_rpy_xyz(R: np.ndarray):
        sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
        if sy < 1e-9:
            roll  = math.atan2(-R[1,2], R[1,1])
            pitch = math.atan2(-R[2,0], sy)
            yaw = 0.0
        else:
            roll  = math.atan2(R[2,1], R[2,2])
            pitch = math.atan2(-R[2,0], sy)
            yaw   = math.atan2(R[1,0], R[0,0])
        return np.array([roll, pitch, yaw], dtype=float)


def main():
    rclpy.init()
    node = EyeInHandCalibration()
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

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import csv
# from tm_msgs.msg import FeedbackState
# from std_srvs.srv import Trigger

# class SimplifiedHandEyeCollector(Node):
#     def __init__(self):
#         super().__init__("simplified_handeye_collector")

#         # Chessboard
#         self.cols = 7
#         self.rows = 9
#         self.square_size = 0.010
        
#         # Camera
#         self.camera_matrix = None
#         self.dist_coeffs = None
#         self.bridge = CvBridge()
#         self.latest_color = None

#         # Robot state
#         self.latest_robot_pose = None
        
#         # 儲存拍攝時的棋盤格角點相機座標
#         self.camera_corners = None  # 63x3 array
#         self.camera_captured = False
        
#         # CSV 輸出
#         self.csv_path = "handeye_simple_calibration.csv"
#         with open(self.csv_path, "w", newline="") as f:
#             writer = csv.writer(f)
#             writer.writerow(["corner_id", "Xc", "Yc", "Zc", "Xr", "Yr", "Zr"])
        
#         self.touch_count = 0

#         # Subscribers
#         self.create_subscription(
#             CameraInfo, "/camera/camera/color/camera_info", self.cb_info, 10
#         )
#         self.create_subscription(
#             Image, "/camera/camera/color/image_raw", self.cb_color, 10
#         )
#         self.create_subscription(
#             FeedbackState, "/feedback_states", self.cb_feedback, 10
#         )

#         # Services
#         self.create_service(Trigger, 'capture_camera', self.srv_capture_camera)
#         self.create_service(Trigger, 'record_touch', self.srv_record_touch)

#         cv2.namedWindow("camera_view", cv2.WINDOW_NORMAL)

#         self.get_logger().info("=== Simplified Hand-Eye Calibration ===")
#         self.get_logger().info("Method: One-time capture + 63 touch points")
#         self.get_logger().info("")
#         self.get_logger().info("Setup:")
#         self.get_logger().info("  1. FIXED chessboard on table")
#         self.get_logger().info("  2. Camera on robot arm")
#         self.get_logger().info("")
#         self.get_logger().info("Workflow:")
#         self.get_logger().info("  Step 1: Position robot to see chessboard")
#         self.get_logger().info("  Step 2: Press SPACE to capture all 63 corners")
#         self.get_logger().info("  Step 3: Touch each corner (0-62) with TCP")
#         self.get_logger().info("  Step 4: Press 'r' after each touch")
#         self.get_logger().info("  Step 5: Repeat until all 63 corners touched")
#         self.get_logger().info("")
#         self.get_logger().info("Controls:")
#         self.get_logger().info("  SPACE: Capture camera coordinates")
#         self.get_logger().info("  r: Record current TCP position")
#         self.get_logger().info("  q: Quit")
#         self.get_logger().info("")

#     def cb_info(self, msg):
#         if self.camera_matrix is None:
#             self.camera_matrix = np.array(msg.k).reshape(3, 3)
#             self.dist_coeffs = np.array(msg.d)
#             self.get_logger().info("✓ Camera intrinsics loaded")

#     def cb_feedback(self, msg: FeedbackState):
#         self.latest_robot_pose = msg.tool_pose

#     def cb_color(self, msg):
#         self.latest_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
#         display_img = self.latest_color.copy()
        
#         # 顯示狀態
#         if not self.camera_captured:
#             text = "Step 1: Press SPACE to capture corners"
#             color = (0, 255, 0)
#         else:
#             text = f"Step 2: Touch corner #{self.touch_count}, press 'r'"
#             color = (0, 0, 255)
        
#         cv2.putText(display_img, text, (10, 30), 
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
#         cv2.putText(display_img, f"Touched: {self.touch_count}/63", (10, 70),
#                     cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
        
#         if self.latest_robot_pose is not None:
#             robot_text = f"TCP: X={self.latest_robot_pose[0]:.3f} Y={self.latest_robot_pose[1]:.3f} Z={self.latest_robot_pose[2]:.3f}"
#             cv2.putText(display_img, robot_text, (10, 110),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
#         cv2.imshow("camera_view", display_img)
        
#         key = cv2.waitKey(10) & 0xFF

#         if key == ord(' '):
#             self.capture_camera_coordinates()
#         elif key == ord('r'):
#             self.record_touch_position()
#         elif key == ord('q'):
#             self.get_logger().info("Exiting...")
#             self.get_logger().info(f"Total touches recorded: {self.touch_count}/63")
#             rclpy.shutdown()

#     def srv_capture_camera(self, request, response):
#         success = self.capture_camera_coordinates()
#         response.success = success
#         return response

#     def srv_record_touch(self, request, response):
#         success = self.record_touch_position()
#         response.success = success
#         return response

#     def capture_camera_coordinates(self):
#         """步驟 1：拍攝並計算所有 63 個角點的相機座標"""
#         if self.latest_color is None or self.camera_matrix is None:
#             self.get_logger().warn("Camera not ready")
#             return False

#         if self.camera_captured:
#             self.get_logger().warn("Already captured! Starting new calibration...")
#             self.camera_captured = False
#             self.touch_count = 0

#         gray = cv2.cvtColor(self.latest_color, cv2.COLOR_BGR2GRAY)
#         found, corners = cv2.findChessboardCornersSB(
#             gray, (self.cols, self.rows),
#             flags=cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY
#         )

#         if not found:
#             self.get_logger().warn("⚠ Chessboard NOT found!")
#             return False

#         # 建立 3D 模型
#         objp = np.zeros((self.cols * self.rows, 3), np.float32)
#         objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
#         objp *= self.square_size

#         # PnP：計算所有角點在相機座標系的位置
#         ok, rvec, tvec = cv2.solvePnP(
#             objp, corners, self.camera_matrix, self.dist_coeffs
#         )
#         if not ok:
#             self.get_logger().warn("⚠ PnP failed!")
#             return False

#         R, _ = cv2.Rodrigues(rvec)
#         t = tvec.reshape(3, 1)

#         # 計算所有 63 個角點的相機座標
#         self.camera_corners = []
#         for i in range(len(objp)):
#             p_obj = objp[i].reshape(3, 1)
#             p_cam = R @ p_obj + t
#             self.camera_corners.append(p_cam.flatten())
        
#         self.camera_corners = np.array(self.camera_corners)  # 63x3
#         self.camera_captured = True

#         # 視覺化：標註所有角點
#         img_show = self.latest_color.copy()
#         cv2.drawChessboardCorners(img_show, (self.cols, self.rows), corners, True)
        
#         # 標註角點編號
#         for i, corner in enumerate(corners):
#             px, py = corner.ravel()
#             px, py = int(px), int(py)
#             cv2.putText(img_show, str(i), (px+5, py), 
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
#         # 高亮下一個要觸碰的角點
#         if self.touch_count < len(corners):
#             next_corner = corners[self.touch_count].ravel()
#             px, py = int(next_corner[0]), int(next_corner[1])
#             cv2.circle(img_show, (px, py), 15, (0, 0, 255), 3)
        
#         cv2.imshow("camera_view", img_show)
#         cv2.waitKey(3000)

#         self.get_logger().info(f"✓ Captured all 63 corners in camera frame")
#         self.get_logger().info(f"Now touch corner #0 (top-left) and press 'r'")
#         self.get_logger().info("")
        
#         return True

#     def record_touch_position(self):
#         """步驟 2：記錄 TCP 觸碰位置"""
#         if not self.camera_captured:
#             self.get_logger().warn("⚠ Capture camera coordinates first! (Press SPACE)")
#             return False

#         if self.touch_count >= 63:
#             self.get_logger().warn("⚠ All 63 corners already recorded!")
#             return False

#         if self.latest_robot_pose is None:
#             self.get_logger().error("⚠ No robot feedback!")
#             return False

#         # 當前觸碰的角點編號
#         corner_id = self.touch_count
        
#         # 相機座標（之前拍攝時計算的）
#         Xc, Yc, Zc = self.camera_corners[corner_id]
        
#         # Robot 座標（當前 TCP 位置）
#         Xr, Yr, Zr = self.latest_robot_pose[:3]

#         # 寫入 CSV
#         with open(self.csv_path, "a", newline="") as f:
#             writer = csv.writer(f)
#             writer.writerow([corner_id, Xc, Yc, Zc, Xr, Yr, Zr])

#         self.get_logger().info(f"✓ Corner #{corner_id} recorded")
#         self.get_logger().info(f"  Camera: ({Xc:.4f}, {Yc:.4f}, {Zc:.4f})")
#         self.get_logger().info(f"  Robot:  ({Xr:.4f}, {Yr:.4f}, {Zr:.4f})")
        
#         self.touch_count += 1

#         if self.touch_count < 63:
#             self.get_logger().info(f"Next: Touch corner #{self.touch_count} and press 'r'")
#         else:
#             self.get_logger().info("")
#             self.get_logger().info("=" * 50)
#             self.get_logger().info("✓ All 63 corners recorded!")
#             self.get_logger().info(f"Data saved to: {self.csv_path}")
#             self.get_logger().info("=" * 50)
        
#         self.get_logger().info("")
        
#         return True


# def main():
#     rclpy.init()
#     node = SimplifiedHandEyeCollector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import csv
# from tm_msgs.msg import FeedbackState  # 使用 feedback_states topic

# class HandEyeDataCollector(Node):
#     def __init__(self):
#         super().__init__("handeye_data_collector")

#         # Chessboard
#         self.cols = 7
#         self.rows = 9
#         self.square_size = 0.010
        
#         self.target_corner_id = 0
        
#         # Camera
#         self.camera_matrix = None
#         self.dist_coeffs = None
#         self.bridge = CvBridge()
#         self.latest_color = None

#         # Robot state
#         self.latest_robot_pose = None
#         self.robot_feedback_received = False  
        
#         # CSV 輸出
#         self.csv_path = "handeye_calibration_data.csv"
#         with open(self.csv_path, "w", newline="") as f:
#             writer = csv.writer(f)
#             writer.writerow(["sample_id", "Xc", "Yc", "Zc", "Xr", "Yr", "Zr"])
        
#         self.sample_id = 0

#         # Subscribers
#         self.create_subscription(
#             CameraInfo, "/camera/camera/color/camera_info", self.cb_info, 10
#         )
#         self.create_subscription(
#             Image, "/camera/camera/color/image_raw", self.cb_color, 10
#         )
#         # ===== 訂閱 feedback_states =====
#         self.create_subscription(
#             FeedbackState, "/feedback_states", self.cb_feedback, 10
#         )

#         cv2.namedWindow("camera_view", cv2.WINDOW_NORMAL)

#         self.get_logger().info("=== Hand-Eye Calibration Data Collector ===")
#         self.get_logger().info(f"Target corner: #{self.target_corner_id}")
#         self.get_logger().info("Using feedback_states for robot position")
#         self.get_logger().info("")
#         self.get_logger().info("Instructions:")
#         self.get_logger().info("1. Move chessboard to a new position")
#         self.get_logger().info("2. Press SPACE → Capture camera coordinates")
#         self.get_logger().info("3. Move TCP to touch the RED corner")
#         self.get_logger().info("4. Press 'r' → Record robot coordinates")
#         self.get_logger().info("5. Repeat 20-30 times")
#         self.get_logger().info("Press 'q' to quit")

#         self.pending_camera_coords = None

#     def cb_info(self, msg):
#         if self.camera_matrix is None:
#             self.camera_matrix = np.array(msg.k).reshape(3, 3)
#             self.dist_coeffs = np.array(msg.d)
#             self.get_logger().info("Camera intrinsics loaded.")

#     def cb_feedback(self, msg: FeedbackState):
#         """接收機器人狀態"""
#         self.latest_robot_pose = msg.tool_pose
        
#         # 只在第一次顯示訊息
#         if not self.robot_feedback_received:
#             self.robot_feedback_received = True
#             self.get_logger().info(f"✓ Robot feedback connected (X={self.latest_robot_pose[0]:.3f}, "
#                                    f"Y={self.latest_robot_pose[1]:.3f}, Z={self.latest_robot_pose[2]:.3f})")
#             self.get_logger().info("")
#             self.get_logger().info("Ready to collect data! Press SPACE to start.")
#             self.get_logger().info("")

#     def cb_color(self, msg):
#         self.latest_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
#         display_img = self.latest_color.copy()
        
#         # 顯示提示
#         if self.pending_camera_coords is None:
#             text = "Press SPACE to capture"
#             color = (0, 255, 0)
#         else:
#             text = "Touch RED corner, then press 'r'"
#             color = (0, 0, 255)
        
#         cv2.putText(display_img, text, (10, 30), 
#                     cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
#         cv2.putText(display_img, f"Samples: {self.sample_id}", (10, 70),
#                     cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
        
#         # 顯示當前機器人位置
#         if self.latest_robot_pose is not None:
#             robot_text = f"Robot: X={self.latest_robot_pose[0]:.3f} Y={self.latest_robot_pose[1]:.3f} Z={self.latest_robot_pose[2]:.3f}"
#             cv2.putText(display_img, robot_text, (10, 110),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
#         cv2.imshow("camera_view", display_img)
        
#         # ===== 關鍵修改：增加 waitKey 的等待時間並確保視窗焦點 =====
#         # 將 waitKey(1) 改成 waitKey(10)，給更多時間捕捉按鍵
#         key = cv2.waitKey(10) & 0xFF

#         if key == ord(' '):
#             self.get_logger().info("[DEBUG] SPACE key detected!")
#             self.capture_camera_coordinates()
#         elif key == ord('r'):
#             self.get_logger().info("[DEBUG] 'r' key detected!")
#             self.record_robot_coordinates()
#         elif key == ord('q'):
#             self.get_logger().info("[DEBUG] 'q' key detected!")
#             self.get_logger().info("Exiting...")
#             self.get_logger().info(f"Total samples collected: {self.sample_id}")
#             rclpy.shutdown()
#         elif key != 255:  # 255 表示沒有按鍵
#             self.get_logger().info(f"[DEBUG] Unknown key pressed: {key}")

#     def capture_camera_coordinates(self):
#         """步驟 1：拍攝棋盤格，計算目標角點的相機座標"""
#         if self.latest_color is None or self.camera_matrix is None:
#             self.get_logger().warn("Not ready.")
#             return

#         if self.pending_camera_coords is not None:
#             self.get_logger().warn("Already captured! Press 'r' to record robot coords first.")
#             return

#         gray = cv2.cvtColor(self.latest_color, cv2.COLOR_BGR2GRAY)
#         found, corners = cv2.findChessboardCornersSB(
#             gray, (self.cols, self.rows),
#             flags=cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY
#         )

#         if not found:
#             self.get_logger().warn("Chessboard NOT found!")
#             return

#         objp = np.zeros((self.cols * self.rows, 3), np.float32)
#         objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
#         objp *= self.square_size

#         ok, rvec, tvec = cv2.solvePnP(
#             objp, corners, self.camera_matrix, self.dist_coeffs
#         )
#         if not ok:
#             self.get_logger().warn("solvePnP failed!")
#             return

#         R, _ = cv2.Rodrigues(rvec)
#         t = tvec.reshape(3, 1)

#         p_obj = objp[self.target_corner_id].reshape(3, 1)
#         p_cam = R @ p_obj + t
#         Xc, Yc, Zc = p_cam.flatten()

#         self.pending_camera_coords = (Xc, Yc, Zc)

#         # 視覺化
#         img_show = self.latest_color.copy()
#         cv2.drawChessboardCorners(img_show, (self.cols, self.rows), corners, True)
        
#         px, py = corners[self.target_corner_id].ravel()
#         px, py = int(px), int(py)
#         cv2.circle(img_show, (px, py), 20, (0, 0, 255), 4)
#         cv2.putText(img_show, f"Touch corner #{self.target_corner_id}", 
#                     (px + 25, py), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
#         cv2.imshow("camera_view", img_show)
#         cv2.waitKey(2000)

#         self.get_logger().info(f"✓ Camera coords: ({Xc:.6f}, {Yc:.6f}, {Zc:.6f})")
#         self.get_logger().info("Now touch the RED corner and press 'r'")

#     def record_robot_coordinates(self):
#         """步驟 2：記錄機器人 TCP 位置"""
#         if self.pending_camera_coords is None:
#             self.get_logger().warn("Capture camera coordinates first! (Press SPACE)")
#             return

#         if self.latest_robot_pose is None:
#             self.get_logger().error("No robot feedback received yet!")
#             return

#         # 從 feedback_states 取得位置
#         Xr, Yr, Zr = self.latest_robot_pose[:3]
#         Xc, Yc, Zc = self.pending_camera_coords

#         # 寫入 CSV
#         with open(self.csv_path, "a", newline="") as f:
#             writer = csv.writer(f)
#             writer.writerow([self.sample_id, Xc, Yc, Zc, Xr, Yr, Zr])

#         self.get_logger().info(f"✓ Sample {self.sample_id} recorded!")
#         self.get_logger().info(f"  Camera: ({Xc:.6f}, {Yc:.6f}, {Zc:.6f})")
#         self.get_logger().info(f"  Robot:  ({Xr:.6f}, {Yr:.6f}, {Zr:.6f})")
#         self.get_logger().info("")
        
#         # 重置
#         self.sample_id += 1
#         self.pending_camera_coords = None


# def main():
#     rclpy.init()
#     node = HandEyeDataCollector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()


# #!/usr/bin/env python3
# import math
# import numpy as np
# import cv2

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge

# import tf2_ros
# from geometry_msgs.msg import TransformStamped


# class HandEyeCollectorNode(Node):
#     def __init__(self):
#         super().__init__("handeye_collector_node")

#         # 參數（可以在 launch 裡 override）
#         self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
#         self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
#         self.declare_parameter("base_frame", "base")
#         self.declare_parameter("ee_frame", "flange")
#         self.declare_parameter("chessboard_cols", 9)  # 內角點列數 (x方向)
#         self.declare_parameter("chessboard_rows", 7)  # 內角點行數 (y方向)
#         self.declare_parameter("square_size", 0.010)  # 10 mm

#         self.color_topic = self.get_parameter("color_topic").get_parameter_value().string_value
#         self.camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
#         self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
#         self.ee_frame = self.get_parameter("ee_frame").get_parameter_value().string_value
#         self.chessboard_cols = int(self.get_parameter("chessboard_cols").value)
#         self.chessboard_rows = int(self.get_parameter("chessboard_rows").value)
#         self.square_size = float(self.get_parameter("square_size").value)

#         self.bridge = CvBridge()

#         # 相機內參
#         self.camera_matrix = None
#         self.dist_coeffs = None

#         # 最新影像
#         self.latest_color_image = None

#         # TF buffer
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         # 資料容器
#         self.R_gripper2base_list = []
#         self.t_gripper2base_list = []
#         self.R_target2cam_list = []
#         self.t_target2cam_list = []

#         # ROS 訂閱
#         self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
#         self.create_subscription(Image, self.color_topic, self.color_image_callback, 10)

#         cv2.namedWindow("handeye_view", cv2.WINDOW_NORMAL)

#         self.get_logger().info("HandEyeCollectorNode initialized.")
#         self.get_logger().info(f"Chessboard: {self.chessboard_cols} x {self.chessboard_rows}, "
#                                f"square = {self.square_size} m")
#         self.get_logger().info("Press SPACE to capture one pose, 'q' to quit and save.")

#     # ---------------- Camera callbacks ----------------
#     def camera_info_callback(self, msg: CameraInfo):
#         if self.camera_matrix is None:
#             K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
#             D = np.array(msg.d, dtype=np.float64)
#             self.camera_matrix = K
#             self.dist_coeffs = D
#             self.get_logger().info(f"Camera intrinsics loaded. K={K}, fx={K[0,0]:.1f}, fy={K[1,1]:.1f}")

#     def color_image_callback(self, msg: Image):
#         color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         self.latest_color_image = color_image
#         cv2.imshow("handeye_view", color_image)
#         key = cv2.waitKey(1) & 0xFF

#         if key == ord(' '):  # SPACE
#             self.capture_one_sample()
#         elif key == ord('q'):
#             self.finish_and_save()
#             rclpy.shutdown()

#     # ---------------- Chessboard detection + PnP ----------------
#     def detect_chessboard_and_pnp(self, color_image: np.ndarray):
#         if self.camera_matrix is None:
#             self.get_logger().warn("Camera intrinsics not ready.")
#             return None, None

#         gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
#         pattern_size = (self.chessboard_cols, self.chessboard_rows)

#         found, corners = cv2.findChessboardCorners(
#             gray_image,
#             pattern_size,
#             None
#             # flags=cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY
#         )

#         if not found:
#             self.get_logger().warn("Chessboard not found.")
#             return None, None

#         # refine
#         criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
#         corners_refined = cv2.cornerSubPix(
#             gray_image, corners, (11, 11), (-1, -1), criteria
#         )
#         corners_refined = corners_refined.reshape(-1, 2)

#         # 畫出棋盤角點
#         vis = color_image.copy()
#         cv2.drawChessboardCorners(vis, pattern_size, corners_refined, found)
#         cv2.imshow("handeye_view", vis)
#         cv2.waitKey(1)

#         # 建立棋盤在 target frame (P) 的 3D 點座標 (Z=0 平面)
#         object_points = np.zeros((self.chessboard_cols * self.chessboard_rows, 3), np.float64)
#         grid_x, grid_y = np.meshgrid(
#             np.arange(self.chessboard_cols),
#             np.arange(self.chessboard_rows)
#         )
#         object_points[:, 0] = grid_x.flatten() * self.square_size
#         object_points[:, 1] = grid_y.flatten() * self.square_size
#         object_points[:, 2] = 0.0

#         # solvePnP：board(target P) in camera C
#         success, rotation_vector, translation_vector = cv2.solvePnP(
#             object_points,
#             corners_refined,
#             self.camera_matrix,
#             self.dist_coeffs,
#             flags=cv2.SOLVEPNP_ITERATIVE
#         )
#         if not success:
#             self.get_logger().warn("solvePnP failed.")
#             return None, None

#         rotation_matrix, _ = cv2.Rodrigues(rotation_vector)  # ^C R_P
#         translation_vector = translation_vector.reshape(3, 1)  # ^C t_P

#         # 回傳 ^C R_P, ^C t_P
#         return rotation_matrix, translation_vector

#     # ---------------- TF lookup ----------------
#     def lookup_base_to_ee(self):
#         try:
#             transform: TransformStamped = self.tf_buffer.lookup_transform(
#                 self.base_frame,
#                 self.ee_frame,
#                 rclpy.time.Time()
#             )
#         except Exception as exc:
#             self.get_logger().warn(f"TF lookup failed: {exc}")
#             return None, None

#         tx = transform.transform.translation.x
#         ty = transform.transform.translation.y
#         tz = transform.transform.translation.z

#         qx = transform.transform.rotation.x
#         qy = transform.transform.rotation.y
#         qz = transform.transform.rotation.z
#         qw = transform.transform.rotation.w

#         translation_vector = np.array([[tx], [ty], [tz]], dtype=np.float64)
#         rotation_matrix = self.quaternion_to_rotation_matrix(qx, qy, qz, qw)

#         return rotation_matrix, translation_vector  # ^A R_F, ^A t_F

#     @staticmethod
#     def quaternion_to_rotation_matrix(qx, qy, qz, qw):
#         """Convert quaternion to 3x3 rotation matrix."""
#         norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
#         if norm < 1e-12:
#             return np.eye(3, dtype=np.float64)
#         qx /= norm
#         qy /= norm
#         qz /= norm
#         qw /= norm

#         rotation_matrix = np.array([
#             [1.0 - 2.0 * (qy*qy + qz*qz),     2.0 * (qx*qy - qz*qw),         2.0 * (qx*qz + qy*qw)],
#             [2.0 * (qx*qy + qz*qw),           1.0 - 2.0 * (qx*qx + qz*qz),   2.0 * (qy*qz - qx*qw)],
#             [2.0 * (qx*qz - qy*qw),           2.0 * (qy*qz + qx*qw),         1.0 - 2.0 * (qx*qx + qy*qy)],
#         ], dtype=np.float64)

#         return rotation_matrix

#     # ---------------- Capture one sample ----------------
#     def capture_one_sample(self):
#         if self.latest_color_image is None:
#             self.get_logger().warn("No color image yet.")
#             return

#         # 1) 取得 ^C T_P
#         rotation_camera_target, translation_camera_target = self.detect_chessboard_and_pnp(
#             self.latest_color_image
#         )
#         if rotation_camera_target is None:
#             return

#         # 2) 取得 ^A T_F
#         rotation_base_gripper, translation_base_gripper = self.lookup_base_to_ee()
#         if rotation_base_gripper is None:
#             return

#         # 3) 組成 calibrateHandEye 所需資料
#         # gripper2base: ^A R_F, ^A t_F
#         self.R_gripper2base_list.append(rotation_base_gripper)
#         self.t_gripper2base_list.append(translation_base_gripper)

#         # target2cam: ^C R_P^T, -^C R_P^T * ^C t_P
#         rotation_target_to_camera = rotation_camera_target.T
#         translation_target_to_camera = -rotation_target_to_camera @ translation_camera_target

#         self.R_target2cam_list.append(rotation_target_to_camera)
#         self.t_target2cam_list.append(translation_target_to_camera)

#         self.get_logger().info(f"Captured sample #{len(self.R_gripper2base_list)}")

#     # ---------------- Save data ----------------
#     def finish_and_save(self):
#         cv2.destroyAllWindows()
#         if len(self.R_gripper2base_list) < 5:
#             self.get_logger().warn("Not enough samples collected.")
#         else:
#             np.savez(
#                 "handeye_data.npz",
#                 R_gripper2base=np.array(self.R_gripper2base_list),
#                 t_gripper2base=np.array(self.t_gripper2base_list),
#                 R_target2cam=np.array(self.R_target2cam_list),
#                 t_target2cam=np.array(self.t_target2cam_list),
#             )
#             self.get_logger().info(f"Saved {len(self.R_gripper2base_list)} samples to handeye_data.npz")


# def main():
#     rclpy.init()
#     node = HandEyeCollectorNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.finish_and_save()
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import tf2_ros
# from geometry_msgs.msg import TransformStamped
# import csv
# import time


# class CheckerboardCollector(Node):
#     def __init__(self):
#         super().__init__('checkerboard_step1')

#         # ---------- Checkerboard setting ----------
#         # Your board: 8×10 squares → 7×9 inner corners
#         self.pattern_size = (9, 7)   # (columns, rows)
#         self.square_size_mm = 10.0   # 10 mm each grid

#         # ---------- Camera topics ----------
#         self.color_topic = "/camera/camera/color/image_raw"
#         self.depth_topic = "/camera/camera/depth/image_rect_raw"
#         self.info_topic  = "/camera/camera/color/camera_info"

#         self.bridge = CvBridge()

#         # Camera intrinsics
#         self.fx = self.fy = self.cx = self.cy = None
#         self.depth_img = None
#         self.color_img = None
#         self.depth_enc = None

#         # TF
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         # Subscriptions
#         self.create_subscription(CameraInfo, self.info_topic, self.cb_info, 10)
#         self.create_subscription(Image, self.color_topic, self.cb_color, 10)
#         self.create_subscription(Image, self.depth_topic, self.cb_depth, 10)

#         # Output CSV
#         self.csv_file = "checkerboard_calibration_pairs.csv"
#         with open(self.csv_file, "w", newline="") as f:
#             writer = csv.writer(f)
#             writer.writerow(["XC", "YC", "ZC", "XA", "YA", "ZA"])

#         cv2.namedWindow("Checkerboard", cv2.WINDOW_NORMAL)

#         self.get_logger().info("Checkerboard Collector started.")
#         self.get_logger().info("Board: inner corners 7×9, grid size 10 mm.")
#         self.get_logger().info("Move robot → Press SPACE to capture one frame.")

#     # ---------------- Callbacks ----------------
#     def cb_info(self, msg):
#         K = msg.k
#         self.fx = K[0]
#         self.fy = K[4]
#         self.cx = K[2]
#         self.cy = K[5]

#     def cb_color(self, msg):
#         self.color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         cv2.imshow("Checkerboard", self.color_img)
#         cv2.waitKey(1)

#     def cb_depth(self, msg):
#         self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         self.depth_enc = msg.encoding

#     # ------------------------------------------------------
#     def process_checkerboard(self):
#         if self.color_img is None or self.depth_img is None or self.fx is None:
#             self.get_logger().warn("Waiting for camera info or image.")
#             return

#         gray = cv2.cvtColor(self.color_img, cv2.COLOR_BGR2GRAY)

#         ok, corners = cv2.findChessboardCornersSB(gray, self.pattern_size)
#         if not ok:
#             self.get_logger().warn("Checkerboard NOT found.")
#             return

#         corners = corners.reshape(-1, 2)

#         # Show detected corners
#         vis = self.color_img.copy()
#         cv2.drawChessboardCorners(vis, self.pattern_size, corners, True)
#         cv2.imshow("Checkerboard", vis)
#         cv2.waitKey(1)

#         # Get robot base position
#         Xb = self.get_robot_position()
#         if Xb is None:
#             self.get_logger().warn("Cannot get TF base->tool0.")
#             return

#         # For each corner → depth → camera xyz
#         count_saved = 0
#         with open(self.csv_file, "a", newline="") as f:
#             writer = csv.writer(f)

#             for (u, v) in corners:
#                 z_mm = self.get_depth_mm(int(v), int(u))
#                 if z_mm is None:
#                     continue

#                 Xc = self.pixel_to_cam(u, v, z_mm)

#                 writer.writerow([Xc[0], Xc[1], Xc[2],
#                                  Xb[0], Xb[1], Xb[2]])
#                 count_saved += 1

#         self.get_logger().info(f"Saved {count_saved} pairs to CSV.")

#     # ------------------------------------------------------
#     def get_depth_mm(self, r, c):
#         if self.depth_img is None:
#             return None

#         d = self.depth_img[r, c]

#         if "16UC1" in str(self.depth_enc):
#             return float(d)

#         if "32FC1" in str(self.depth_enc):
#             return float(d) * 1000.0

#         return None

#     # Pixel → Camera
#     def pixel_to_cam(self, u, v, z_mm):
#         x = (u - self.cx) * z_mm / self.fx / 1000.0
#         y = (v - self.cy) * z_mm / self.fy / 1000.0
#         z = z_mm / 1000.0
#         return np.array([x, y, z], dtype=float)

#     # Read TF -> base→tool0
#     def get_robot_position(self):
#         try:
#             t: TransformStamped = self.tf_buffer.lookup_transform(
#                 "base", "flange", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
#             )
#         except:
#             return None

#         return np.array([
#             t.transform.translation.x,
#             t.transform.translation.y,
#             t.transform.translation.z
#         ], dtype=float)


# def main():
#     rclpy.init()
#     node = CheckerboardCollector()

#     try:
#         while rclpy.ok():
#             rclpy.spin_once(node, timeout_sec=0.01)
#             key = cv2.waitKey(1) & 0xFF

#             # Press SPACE → capture one frame of data
#             if key == 32:  # space bar
#                 node.get_logger().info("Capturing checkerboard...")
#                 node.process_checkerboard()

#             if key == ord('q'):
#                 break

#     except KeyboardInterrupt:
#         pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()
