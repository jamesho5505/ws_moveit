#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from tm_msgs.msg import FeedbackState
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation as R
import csv

class HandEyeCalibrator(Node):
    def __init__(self):
        super().__init__("handeye_calibrator")

        self.csv_path = "handeye_calibration.csv"
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            
            writer.writerow([
            "corner_id",
            "chess_x", "chess_y", "chess_z",
            "cam_rvec_x", "cam_rvec_y", "cam_rvec_z",
            "cam_tvec_x", "cam_tvec_y", "cam_tvec_z",
            "capture_tcp_x", "capture_tcp_y", "capture_tcp_z",
            "capture_tcp_rx", "capture_tcp_ry", "capture_tcp_rz",
            "capture_flange_x", "capture_flange_y", "capture_flange_z",
            "capture_flange_rx", "capture_flange_ry", "capture_flange_rz",
            "capture_joint_1", "capture_joint_2", "capture_joint_3",
            "capture_joint_4", "capture_joint_5", "capture_joint_6",
            "touch_tcp_x", "touch_tcp_y", "touch_tcp_z",
            "touch_tcp_rx", "touch_tcp_ry", "touch_tcp_rz",
            "touch_flange_x", "touch_flange_y", "touch_flange_z",
            "touch_flange_rx", "touch_flange_ry", "touch_flange_rz",
            "touch_joint_1", "touch_joint_2", "touch_joint_3",
            "touch_joint_4", "touch_joint_5", "touch_joint_6",
        ])

        # Chessboard
        self.cols = 4
        self.rows = 4
        self.square_size = 0.030
        
        # Camera
        self.camera_matrix = None
        self.dist_coeffs = None
        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_corners = None

        # Robot state
        self.capture_robot_tcp_pose = None
        self.capture_robot_flange_pose = None
        self.touch_robot_tcp_pose = None
        self.touch_robot_flange_pose = None
        self.touch_joint_pos = None
        self.capture_joint_pos = None
        
        # 存儲轉換矩陣
        self.T_base_flange_list = []
        self.T_camera_chess_list = []
        
        # ★★★ 關鍵修正：存儲點對應 ★★★
        self.P_chess_list = []  # 角點在棋盤座標系
        self.P_base_list = []   # 角點在 base 座標系
        
        # self.target_corner_ids = [0, 3, 12, 15]
        self.target_corner_ids = [6, 7, 10, 11]
        self.pose_count = 0
        self.target_poses = len(self.target_corner_ids)
        
        # Chessboard 3D model
        self.objp = np.zeros((self.cols * self.rows, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        self.objp *= self.square_size

        # Subscribers
        self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", self.cb_info, 10)
        self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.cb_color, 10)
        self.create_subscription(
            FeedbackState, "/feedback_states", self.cb_feedback, 10)

        cv2.namedWindow("camera_view", cv2.WINDOW_NORMAL)

        self.get_logger().info("=" * 70)
        self.get_logger().info("Hand-Eye Calibration (修正版 - SVD 求解)")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"將觸碰角點: {self.target_corner_ids}")
        self.get_logger().info("=" * 70)

    def cb_info(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("✓ 相機內參已載入")

    def cb_feedback(self, msg: FeedbackState):
        self.latest_tcp_pose = np.array(msg.tool_pose, dtype=float)
        self.latest_flange_pose = np.array(msg.tool0_pose, dtype=float)
        self.latest_joint_pos = np.array(msg.joint_pos, dtype=float)



    def cb_color(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        display_img = self.latest_color.copy()
        
        if self.latest_corners is not None:
            cv2.drawChessboardCorners(display_img, (self.cols, self.rows), 
                                     self.latest_corners, True)
            
            for i, corner in enumerate(self.latest_corners):
                px, py = corner.ravel()
                px, py = int(px), int(py)
                cv2.putText(display_img, str(i), (px+5, py), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            
            if self.pose_count < self.target_poses and len(self.T_camera_chess_list) > len(self.P_base_list):
                target_corner_id = self.target_corner_ids[self.pose_count]
                px, py = map(int, self.latest_corners[target_corner_id].ravel())
                cv2.circle(display_img, (px, py), 30, (0, 0, 255), 4)
                cv2.putText(display_img, f"Touch #{target_corner_id}", (px-50, py-40), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        status_text = f"Poses: {self.pose_count}/{self.target_poses}"
        cv2.putText(display_img, status_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        if self.pose_count < self.target_poses:
            if len(self.T_camera_chess_list) == self.pose_count:
                text = "SPACE: capture chessboard"
                color = (0, 255, 0)
            else:
                target_id = self.target_corner_ids[self.pose_count]
                text = f"Touch corner #{target_id}, press 'r'"
                color = (0, 0, 255)
        else:
            text = "Press 'c' to compute"
            color = (255, 0, 0)
        
        cv2.putText(display_img, text, (10, 70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        cv2.imshow("camera_view", display_img)
        
        key = cv2.waitKey(10) & 0xFF
        if key == ord(' '):
            self.capture_chessboard()
        elif key == ord('r'):
            self.record_corner()
        elif key == ord('c'):
            self.compute_calibration()
        elif key == ord('q'):
            rclpy.shutdown()

    def pose6d_to_T(self, pose):
        T = np.eye(4)
        T[:3, 3] = pose[:3]
        T[:3, :3] = R.from_euler('zyx', pose[3:], degrees=False).as_matrix()
        return T

    def capture_chessboard(self):
        if self.latest_tcp_pose is None:
            self.get_logger().warn("⚠ No robot feedback yet")
            return False

        if self.latest_color is None or self.camera_matrix is None:
            return False

        gray = cv2.cvtColor(self.latest_color, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(
            gray, (self.cols, self.rows),
            flags=cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY)

        if not found:
            self.get_logger().warn("⚠ 未檢測到棋盤格！")
            return False

        self.latest_corners = corners

        ok, rvec, tvec = cv2.solvePnP(
            self.objp, corners, self.camera_matrix, self.dist_coeffs)
        
        if not ok:
            return False

        self.last_rvec = rvec.flatten()
        self.last_tvec = tvec.flatten()

        self.pose_at_capture_tcp = self.latest_tcp_pose.copy()
        self.pose_at_capture_flange = self.latest_flange_pose.copy()
        self.pose_at_capture_joints = self.latest_joint_pos.copy()



        R_mat, _ = cv2.Rodrigues(rvec)
        T_camera_chess = np.eye(4)
        T_camera_chess[:3, :3] = R_mat
        T_camera_chess[:3, 3] = tvec.flatten()

        T_base_flange = self.pose6d_to_T(self.latest_flange_pose)

        self.T_camera_chess_list.append(T_camera_chess)
        self.T_base_flange_list.append(T_base_flange)

        target_corner_id = self.target_corner_ids[self.pose_count]
        self.get_logger().info(f"\n✓ 姿態 {self.pose_count + 1} 已拍照")
        self.get_logger().info(f"  → 觸碰角點 #{target_corner_id}，按 'r'")
        
        return True

    def record_corner(self):
        if self.latest_tcp_pose is None:
            self.get_logger().warn("⚠ No robot feedback yet")
            return False
        """★★★ 修正版：只記錄點對應 ★★★"""
        self.pose_at_touch_tcp = self.latest_tcp_pose.copy()
        self.pose_at_touch_flange = self.latest_flange_pose.copy()
        self.pose_at_touch_joints = self.latest_joint_pos.copy()

        if len(self.P_base_list) >= len(self.T_camera_chess_list):
            self.get_logger().warn("⚠ 請先拍照")
            return False

        tcp_pos = self.latest_tcp_pose[:3]
        target_corner_id = self.target_corner_ids[self.pose_count]
        
        # ★★★ 關鍵：只存點，不建 T_base_chess ★★★
        self.P_chess_list.append(self.objp[target_corner_id])
        self.P_base_list.append(tcp_pos)
        
        self.pose_count += 1

        self.get_logger().info(f"\n✓ 角點 #{target_corner_id} 已記錄")
        self.get_logger().info(f"  TCP: [{tcp_pos[0]:.4f}, {tcp_pos[1]:.4f}, {tcp_pos[2]:.4f}]")
        
        if self.pose_count < self.target_poses:
            self.get_logger().info(f"\n→ 下一個姿態，按 SPACE")
        else:
            self.get_logger().info("\n✓ 數據收集完成！按 'c' 計算")

        with open(self.csv_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                target_corner_id,

                # chessboard point
                self.objp[target_corner_id][0],
                self.objp[target_corner_id][1],
                self.objp[target_corner_id][2],

                # camera -> chess (PnP)
                self.last_rvec[0], self.last_rvec[1], self.last_rvec[2],
                self.last_tvec[0], self.last_tvec[1], self.last_tvec[2],

                # TCP pose
                self.pose_at_capture_tcp[0],
                self.pose_at_capture_tcp[1],
                self.pose_at_capture_tcp[2],
                self.pose_at_capture_tcp[3],
                self.pose_at_capture_tcp[4],
                self.pose_at_capture_tcp[5],

                # Flange pose
                self.pose_at_capture_flange[0],
                self.pose_at_capture_flange[1],
                self.pose_at_capture_flange[2],
                self.pose_at_capture_flange[3],
                self.pose_at_capture_flange[4],
                self.pose_at_capture_flange[5],

                # Joint positions
                *self.pose_at_capture_joints[:6],

                # Touch TCP pose
                self.pose_at_touch_tcp[0],
                self.pose_at_touch_tcp[1],
                self.pose_at_touch_tcp[2],
                self.pose_at_touch_tcp[3],
                self.pose_at_touch_tcp[4],
                self.pose_at_touch_tcp[5],

                # Touch Flange pose
                self.pose_at_touch_flange[0],
                self.pose_at_touch_flange[1],
                self.pose_at_touch_flange[2],
                self.pose_at_touch_flange[3],
                self.pose_at_touch_flange[4],
                self.pose_at_touch_flange[5],

                # Touch Joint positions
                *self.pose_at_touch_joints[:6],

            ])
        
        return True

    def solve_rigid(self, P, Q):
        """SVD 求解剛體轉換"""
        Pc = P - P.mean(axis=0)
        Qc = Q - Q.mean(axis=0)
        H = Pc.T @ Qc
        U, _, Vt = np.linalg.svd(H)
        Rm = Vt.T @ U.T
        if np.linalg.det(Rm) < 0:
            Vt[2, :] *= -1
            Rm = Vt.T @ U.T
        t = Q.mean(axis=0) - Rm @ P.mean(axis=0)
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3] = t
        return T

    def invT(self, T):
        R_mat, t = T[:3, :3], T[:3, 3]
        Ti = np.eye(4)
        Ti[:3, :3] = R_mat.T
        Ti[:3, 3] = -R_mat.T @ t
        return Ti

    def compute_calibration(self):
        """★★★ 修正版：先求唯一的 T_base_chess ★★★"""
        if len(self.P_base_list) < self.target_poses:
            self.get_logger().warn(f"⚠ 數據不足！")
            return False

        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("計算中...")
        self.get_logger().info("=" * 70)

        # ★★★ 關鍵修正：用所有點一次求解 T_base_chess ★★★
        P_chess = np.array(self.P_chess_list)
        P_base = np.array(self.P_base_list)
        
        T_base_chess = self.solve_rigid(P_chess, P_base)
        
        self.get_logger().info("\n唯一的 T_base_chess (棋盤格在 base 的姿態):")
        self.get_logger().info("旋轉矩陣:")
        for row in T_base_chess[:3, :3]:
            self.get_logger().info(f"  {row}")
        self.get_logger().info(f"平移: {T_base_chess[:3, 3]}")
        
        # 驗證重投影誤差
        errors = []
        for i in range(len(P_chess)):
            p_pred = (T_base_chess @ np.append(P_chess[i], 1))[:3]
            error = np.linalg.norm(p_pred - P_base[i])
            errors.append(error)
            self.get_logger().info(f"  角點 #{self.target_corner_ids[i]}: 誤差 = {error*1000:.2f} mm")
        
        rms_error = np.sqrt(np.mean(np.array(errors)**2))
        self.get_logger().info(f"\nT_base_chess 重投影 RMS: {rms_error*1000:.2f} mm")
        
        if rms_error * 1000 > 3.0:
            self.get_logger().warn("⚠️  重投影誤差較大，可能觸碰不準確！")

        # 對每個姿態計算 T_flange_camera
        T_flange_camera_list = []
        
        for i in range(len(self.T_base_flange_list)):
            T_base_flange = self.T_base_flange_list[i]
            T_camera_chess = self.T_camera_chess_list[i]
            
            # ★★★ 使用唯一的 T_base_chess ★★★
            T_flange_camera = self.invT(T_base_flange) @ T_base_chess @ self.invT(T_camera_chess)
            T_flange_camera_list.append(T_flange_camera)
            
            t = T_flange_camera[:3, 3]
            self.get_logger().info(f"\n姿態 {i+1}: 平移 = [{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}]")

        # 平均
        translations = np.array([T[:3, 3] for T in T_flange_camera_list])
        t_avg = translations.mean(axis=0)
        
        from scipy.spatial.transform import Rotation as Rot
        rotations = [Rot.from_matrix(T[:3, :3]) for T in T_flange_camera_list]
        quats = np.array([r.as_quat() for r in rotations])
        quat_avg = quats.mean(axis=0)
        quat_avg /= np.linalg.norm(quat_avg)
        R_avg = Rot.from_quat(quat_avg).as_matrix()
        
        T_flange_camera_optical = np.eye(4)
        T_flange_camera_optical[:3, :3] = R_avg
        T_flange_camera_optical[:3, 3] = t_avg

        # optical → camera_link
        R_optical_to_link = np.array([
            [ 0,  0,  1],
            [-1,  0,  0],
            [ 0, -1,  0]
        ])
        T_optical_to_link = np.eye(4)
        T_optical_to_link[:3, :3] = R_optical_to_link
        T_optical_to_link[:3, 3] = [0, 0.015, 0]

        T_flange_camera_link = T_flange_camera_optical @ T_optical_to_link
        print("T_flange_camera_link:", T_flange_camera_link)
        print("T_flange_camera_optical:", T_flange_camera_optical)
        print("T_optical_to_link:", T_optical_to_link)


        t = T_flange_camera_link[:3, 3]
        rpy = Rot.from_matrix(T_flange_camera_link[:3, :3]).as_euler('xyz', degrees=False)

        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("最終結果:")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f'<origin xyz="{t[0]:.6f} {t[1]:.6f} {t[2]:.6f}" rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/>')
                
        t_std = translations.std(axis=0)
        self.get_logger().info(f"\n一致性: [{t_std[0]*1000:.2f}, {t_std[1]*1000:.2f}, {t_std[2]*1000:.2f}] mm")
        
        if np.max(t_std) * 1000 < 1.0:
            self.get_logger().info("✅ 優秀 (< 1mm)")
        elif np.max(t_std) * 1000 < 3.0:
            self.get_logger().info("✅ 良好 (< 3mm)")
        else:
            self.get_logger().warn(f"⚠️  較差 ({np.max(t_std)*1000:.2f}mm)")
        
        self.get_logger().info("=" * 70)

        return True


def main():
    rclpy.init()
    node = HandEyeCalibrator()
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
# from tf2_ros import TransformBroadcaster

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
#         self.capture_robot_flange_pose = None
        
#         # 儲存拍攝時的棋盤格角點相機座標
#         self.camera_corners = None  # 63x3 array
#         self.camera_captured = False
#         self.rvec = None
#         self.tvec = None


#         self.base_flange_pose_at_capture = None  # base→flange at capture moment

#         # CSV 輸出
#         self.csv_path = "handeye_simple_calibration.csv"
#         with open(self.csv_path, "w", newline="") as f:
#             writer = csv.writer(f)
            
#             writer.writerow([
#                 "corner_id",
#                 "Xc", "Yc", "Zc",
#                 "Xr", "Yr", "Zr",
#                 "flange_cap_x", "flange_cap_y", "flange_cap_z",
#                 "flange_cap_rx", "flange_cap_ry", "flange_cap_rz"
#             ])


        
#         # 只需要觸碰四個角點
#         self.required_corner_ids = [0, 6, 28, 34, 56, 62]  # 四個角
#         self.touch_step = 0  # 當前要觸碰第幾個


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
#         self.get_logger().info(f"Method: One-time capture + {len(self.required_corner_ids)} touch points")
#         self.get_logger().info("")
#         self.get_logger().info("Setup:")
#         self.get_logger().info("  1. FIXED chessboard on table")
#         self.get_logger().info("  2. Camera on robot arm")
#         self.get_logger().info("")
#         self.get_logger().info("Workflow:")
#         self.get_logger().info("  Step 1: Position robot to see chessboard")
#         self.get_logger().info(f"  Step 2: Press SPACE to capture all {self.cols*self.rows} corners")
#         self.get_logger().info(f"  Step 3: Touch {len(self.required_corner_ids)} corners with TCP ({self.required_corner_ids})")
#         self.get_logger().info("  Step 4: Press 'r' after each touch")
#         self.get_logger().info(f"  Step 5: Repeat until all {len(self.required_corner_ids)} corners touched")
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
#         self.capture_robot_flange_pose = msg.tool0_pose

#     def cb_color(self, msg):
#         self.latest_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
#         display_img = self.latest_color.copy()
        
#         # 顯示狀態
#         if not self.camera_captured:
#             text = "Step 1: Press SPACE to capture corners"
#             color = (0, 255, 0)
#         else:
#             next_corner_id = self.required_corner_ids[self.touch_step] if self.touch_step < len(self.required_corner_ids) else 'None'
#             text = f"Step 2: Touch corner #{next_corner_id} and press 'r'"
#             color = (0, 0, 255)
#         cv2.putText(display_img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

#         cv2.putText(display_img,
#                     f"Touched: {self.touch_step}/{len(self.required_corner_ids)}",
#                     (10, 70),
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
#             self.get_logger().info(f"Total touches recorded: {self.touch_step}/{len(self.required_corner_ids)}")
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
#         """步驟 1：拍攝並計算所有角點的相機座標"""
#         if self.latest_color is None or self.camera_matrix is None:
#             self.get_logger().warn("Camera not ready")
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

#         # PnP：計算所有角點在相機座標系的位置
#         ok, rvec, tvec = cv2.solvePnP(
#             objp, corners, self.camera_matrix, self.dist_coeffs
#         )
#         if not ok:
#             self.get_logger().warn("⚠ PnP failed!")
#             return False

#         R, _ = cv2.Rodrigues(rvec)
#         t = tvec.reshape(3, 1)
#         self.rvec = rvec.copy()
#         self.tvec = tvec.copy()

#         # ===== 儲存 solvePnP 結果 =====
#         np.savez(
#             "pnp_capture.npz",
#             rvec=self.rvec,
#             tvec=self.tvec,
#             flange_pose=self.base_flange_pose_at_capture
#         )

#         self.get_logger().info("✓ Saved rvec/tvec and flange pose to pnp_capture.npz")



#         # 計算所有 63 個角點的相機座標
#         self.camera_corners = []
#         for i in range(len(objp)):
#             p_obj = objp[i].reshape(3, 1)
#             p_cam = R @ p_obj + t
#             self.camera_corners.append(p_cam.flatten())
        
#         self.camera_corners = np.array(self.camera_corners)
#         self.camera_captured = True

#         # ★★★★★ 同步記錄 Base→Flange 拍照姿勢 ★★★★★
#         if self.capture_robot_flange_pose is None:
#             self.get_logger().error("No robot pose! Cannot save capture pose.")
#             return False

#         self.base_flange_pose_at_capture = np.array(self.capture_robot_flange_pose[:6], dtype=float)

#         self.get_logger().info(f"✓ Saved Flange pose at capture: {self.base_flange_pose_at_capture}")


#         # 視覺化：標註所有角點
#         img_show = self.latest_color.copy()
#         cv2.drawChessboardCorners(img_show, (self.cols, self.rows), corners, True)
        
#         # 標註角點編號
#         for i, corner in enumerate(corners):
#             px, py = corner.ravel()
#             px, py = int(px), int(py)
#             cv2.putText(img_show, str(i), (px+5, py), 
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
#         # 高亮所有需要觸碰的角點
#         if self.camera_captured:
#             for corner_id in self.required_corner_ids:
#                 px, py = map(int, corners[corner_id].ravel())
#                 cv2.circle(img_show, (px, py), 20, (0, 0, 255), 3)

        
#         cv2.imshow("camera_view", img_show)
#         cv2.waitKey(2000)

#         self.get_logger().info(f"✓ Captured all {len(corners)} corners in camera frame")
#         self.get_logger().info(f"Now touch corner #0 (top-left) and press 'r'")
#         self.get_logger().info("")
        
#         return True

#     def record_touch_position(self):
#         if not self.camera_captured:
#             self.get_logger().warn("⚠ Capture camera coordinates first! (Press SPACE)")
#             return False

#         if self.touch_step >= len(self.required_corner_ids):
#             self.get_logger().warn("⚠ All required corners already recorded!")
#             return False

#         if self.capture_robot_flange_pose is None:
#             self.get_logger().error("⚠ No robot feedback!")
#             return False

#         # 目前要觸碰的角點 ID
#         corner_id = self.required_corner_ids[self.touch_step]

#         # 相機座標
#         Xc, Yc, Zc = self.camera_corners[corner_id]

#         # 機器人 TCP 座標
#         Xr, Yr, Zr = self.latest_robot_pose[:3]

#         # 寫入 CSV
#         with open(self.csv_path, "a", newline="") as f:
#             writer = csv.writer(f)
#             # unpack capture pose
#             flange_cap_x, flange_cap_y, flange_cap_z, flange_cap_rx, flange_cap_ry, flange_cap_rz = self.base_flange_pose_at_capture

#             writer.writerow([
#                 corner_id, Xc, Yc, Zc, Xr, Yr, Zr,
#                 flange_cap_x, flange_cap_y, flange_cap_z,
#                 flange_cap_rx, flange_cap_ry, flange_cap_rz
#             ])

#         self.get_logger().info(f"✓ Corner #{corner_id} recorded")
#         self.get_logger().info(f"  Camera: ({Xc:.4f}, {Yc:.4f}, {Zc:.4f})")
#         self.get_logger().info(f"  Robot:  ({Xr:.4f}, {Yr:.4f}, {Zr:.4f})")

#         self.touch_step += 1

#         if self.touch_step < len(self.required_corner_ids):
#             next_id = self.required_corner_ids[self.touch_step]
#             self.get_logger().info(f"Next: Touch corner #{next_id} and press 'r'")
#         else:
#             self.get_logger().info("")
#             self.get_logger().info("=" * 50)
#             self.get_logger().info("✓ All required corners recorded!")
#             self.get_logger().info(f"Data saved to: {self.csv_path}")
#             self.get_logger().info("=" * 50)

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
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == "__main__":
#     main()


