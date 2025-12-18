#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from tm_msgs.msg import FeedbackState

class SimpleCalibrationVerifier(Node):
    def __init__(self):
        super().__init__('simple_verifier')
        
        # ★★★ 你的標定矩陣 ★★★
        # self.T_flange_camera_optical = np.array([
        #     [-9.99420242e-01,  3.40429613e-02, -5.06696784e-04,  2.52975073e-02],
        #     [-3.40443073e-02, -9.99416001e-01,  2.93985984e-03, -8.53863223e-02],
        #     [-4.06319338e-04,  2.95540558e-03,  9.99995550e-01,  6.43833999e-02],
        #     [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
        # ])
        # self.T_flange_camera_optical = np.array([
        #     [-0.99934372,  0.0341722,   0.01201646,  0.02955424],
        #     [-0.03437293, -0.99926592, -0.01691517, -0.08029949],
        #     [ 0.01142961, -0.01731711,  0.99978472,  0.0642658 ],
        #     [ 0.,          0.,          0.,          1.,        ]
        # ])

        self.T_flange_camera_optical= np.array([
            [-0.99942486,  0.03322565,  0.00678327,  0.03121642],
            [-0.03325844, -0.99943536, -0.00477955, -0.08367524],
            [ 0.00662064, -0.00500241,  0.99996557,  0.0654927 ],
            [ 0.,          0.,          0.,          1.]
        ])
        
        # Chessboard
        self.cols = 4
        self.rows = 4
        self.square_size = 0.030
        
        # Camera
        self.camera_matrix = None
        self.dist_coeffs = None
        self.bridge = CvBridge()
        self.latest_image = None
        
        # Robot
        self.latest_tcp_pose = None
        self.latest_flange_pose = None
        
        # Chessboard model
        self.objp = np.zeros((self.cols * self.rows, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        self.objp *= self.square_size
        
        # 驗證數據
        self.captured_corners = None
        self.T_base_flange_at_capture = None  # 拍照時的 base→flange
        self.T_camera_chess = None  # 相機→棋盤格 (solvePnP)
        self.predicted_positions = None  # 預測的角點位置 (base 座標系)
        self.current_corner_id = 0
        self.verification_data = []
        
        # 要驗證的角點
        self.corner_ids_to_verify = [0, 3, 12, 15]
        
        # Subscribers
        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.cb_info, 10)
        self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.cb_image, 10)
        self.create_subscription(
            FeedbackState, '/feedback_states', self.cb_feedback, 10)
        
        cv2.namedWindow("Verification", cv2.WINDOW_NORMAL)
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("Hand-Eye Calibration 驗證 (使用標定矩陣)")
        self.get_logger().info("=" * 70)
        self.get_logger().info("流程:")
        self.get_logger().info("  1. 將棋盤格放在桌上")
        self.get_logger().info("  2. 按 SPACE 拍照並計算角點預測位置")
        self.get_logger().info(f"  3. 依次移動 TCP 到角點 {self.corner_ids_to_verify}")
        self.get_logger().info("  4. 每個角點按 'r' 記錄實際 TCP 位置")
        self.get_logger().info("")
        self.get_logger().info("使用標定矩陣:")
        self.get_logger().info("  T_flange_camera_optical (4x4)")
        self.get_logger().info("=" * 70)

    def cb_info(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("✓ 相機內參已載入")

    def cb_feedback(self, msg: FeedbackState):
        self.latest_tcp_pose = np.array(msg.tool_pose[:3], dtype=float)
        self.latest_flange_pose = np.array(msg.tool0_pose, dtype=float)

    def cb_image(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        display_img = self.latest_image.copy()
        
        # 顯示狀態
        if self.predicted_positions is None:
            status = "Press SPACE to capture"
            color = (0, 255, 0)
        elif self.current_corner_id < len(self.corner_ids_to_verify):
            corner_id = self.corner_ids_to_verify[self.current_corner_id]
            status = f"Move TCP to corner #{corner_id}, press 'r'"
            color = (0, 0, 255)
            
            # 高亮當前角點
            if self.captured_corners is not None:
                px, py = map(int, self.captured_corners[corner_id].ravel())
                cv2.circle(display_img, (px, py), 30, (0, 0, 255), 4)
                cv2.putText(display_img, f"#{corner_id}", (px-20, py-40),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:
            status = "Verification complete!"
            color = (0, 255, 0)
        
        cv2.putText(display_img, status, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        
        cv2.putText(display_img, f"Progress: {self.current_corner_id}/{len(self.corner_ids_to_verify)}",
                   (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        
        # 顯示 TCP 位置
        if self.latest_tcp_pose is not None:
            tcp_text = f"TCP: [{self.latest_tcp_pose[0]:.4f}, {self.latest_tcp_pose[1]:.4f}, {self.latest_tcp_pose[2]:.4f}]"
            cv2.putText(display_img, tcp_text, (10, 110),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 顯示預測位置
        if self.predicted_positions is not None and self.current_corner_id < len(self.corner_ids_to_verify):
            corner_id = self.corner_ids_to_verify[self.current_corner_id]
            pred_pos = self.predicted_positions[corner_id]
            pred_text = f"Predicted: [{pred_pos[0]:.4f}, {pred_pos[1]:.4f}, {pred_pos[2]:.4f}]"
            cv2.putText(display_img, pred_text, (10, 140),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # 繪製角點
        if self.captured_corners is not None:
            cv2.drawChessboardCorners(display_img, (self.cols, self.rows),
                                     self.captured_corners, True)
            
            for i, corner in enumerate(self.captured_corners):
                px, py = corner.ravel()
                px, py = int(px), int(py)
                cv2.putText(display_img, str(i), (px+5, py),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        cv2.imshow("Verification", display_img)
        
        key = cv2.waitKey(10) & 0xFF
        if key == ord(' '):
            self.capture_and_predict()
        elif key == ord('r'):
            self.record_tcp()
        elif key == ord('q'):
            rclpy.shutdown()

    def pose6d_to_T(self, pose):
        """6D pose → 4x4 matrix"""
        T = np.eye(4)
        T[:3, 3] = pose[:3]
        T[:3, :3] = R.from_euler('zyx', pose[3:], degrees=False).as_matrix()
        return T

    def capture_and_predict(self):
        """拍照並使用標定矩陣預測角點位置"""
        if self.latest_image is None or self.camera_matrix is None:
            self.get_logger().warn("⚠ 相機未就緒")
            return
        
        if self.latest_flange_pose is None:
            self.get_logger().warn("⚠ 無機器人數據")
            return
        
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(
            gray, (self.cols, self.rows),
            flags=cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY)
        
        if not found:
            self.get_logger().warn("⚠ 未檢測到棋盤格！")
            return
        
        self.captured_corners = corners
        
        # PnP: 計算 T_camera_chess
        ok, rvec, tvec = cv2.solvePnP(
            self.objp, corners, self.camera_matrix, self.dist_coeffs)
        
        if not ok:
            self.get_logger().warn("⚠ PnP 失敗！")
            return
        
        R_mat, _ = cv2.Rodrigues(rvec)
        self.T_camera_chess = np.eye(4)
        self.T_camera_chess[:3, :3] = R_mat
        self.T_camera_chess[:3, 3] = tvec.flatten()
        
        # 記錄拍照時的 base→flange
        self.T_base_flange_at_capture = self.pose6d_to_T(self.latest_flange_pose)
        
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("拍照成功！")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"T_camera_chess (solvePnP):")
        for row in self.T_camera_chess[:3]:
            self.get_logger().info(f"  {row}")
        
        # ★★★ 關鍵：使用標定矩陣計算所有角點的預測位置 ★★★
        # 公式: p_base = T_base_flange @ T_flange_camera_optical @ T_camera_chess @ p_chess
        
        self.predicted_positions = []
        for i in range(len(self.objp)):
            # 角點在棋盤座標系
            p_chess = np.append(self.objp[i], 1)  # [x, y, z, 1]
            
            # 逐步轉換
            p_camera = self.T_camera_chess @ p_chess  # 棋盤 → 相機
            p_flange = self.T_flange_camera_optical @ p_camera  # 相機 → flange
            p_base = self.T_base_flange_at_capture @ p_flange  # flange → base
            
            self.predicted_positions.append(p_base[:3])
        
        self.predicted_positions = np.array(self.predicted_positions)
        
        self.get_logger().info(f"\n✓ 已計算所有 {len(self.objp)} 個角點的預測位置")
        self.get_logger().info(f"將驗證角點: {self.corner_ids_to_verify}")
        
        # 顯示第一個角點
        corner_id = self.corner_ids_to_verify[0]
        pred = self.predicted_positions[corner_id]
        self.get_logger().info(f"\n第一個角點 #{corner_id}:")
        self.get_logger().info(f"  預測位置 (base): [{pred[0]:.4f}, {pred[1]:.4f}, {pred[2]:.4f}]")
        self.get_logger().info(f"  移動 TCP 到此位置，按 'r' 記錄")
        
        self.current_corner_id = 0
        self.verification_data = []

    def record_tcp(self):
        """記錄 TCP 位置並計算誤差"""
        if self.predicted_positions is None:
            self.get_logger().warn("⚠ 請先拍照 (按 SPACE)")
            return
        
        if self.current_corner_id >= len(self.corner_ids_to_verify):
            self.get_logger().warn("⚠ 已完成所有驗證")
            return
        
        if self.latest_tcp_pose is None:
            self.get_logger().warn("⚠ 無 TCP 數據")
            return
        
        corner_id = self.corner_ids_to_verify[self.current_corner_id]
        predicted = self.predicted_positions[corner_id]
        actual = self.latest_tcp_pose
        
        error_vec = predicted - actual
        error = np.linalg.norm(error_vec)
        
        self.verification_data.append({
            'corner_id': corner_id,
            'predicted': predicted,
            'actual': actual,
            'error_vec': error_vec,
            'error': error
        })
        
        self.get_logger().info(f"\n✓ 角點 #{corner_id} 已記錄")
        self.get_logger().info(f"  預測: [{predicted[0]:.4f}, {predicted[1]:.4f}, {predicted[2]:.4f}]")
        self.get_logger().info(f"  實際: [{actual[0]:.4f}, {actual[1]:.4f}, {actual[2]:.4f}]")
        self.get_logger().info(f"  誤差向量: [{error_vec[0]*1000:.2f}, {error_vec[1]*1000:.2f}, {error_vec[2]*1000:.2f}] mm")
        self.get_logger().info(f"  誤差: {error*1000:.2f} mm")
        
        self.current_corner_id += 1
        
        if self.current_corner_id < len(self.corner_ids_to_verify):
            next_corner_id = self.corner_ids_to_verify[self.current_corner_id]
            next_pred = self.predicted_positions[next_corner_id]
            self.get_logger().info(f"\n下一個角點 #{next_corner_id}:")
            self.get_logger().info(f"  預測位置: [{next_pred[0]:.4f}, {next_pred[1]:.4f}, {next_pred[2]:.4f}]")
        else:
            self.show_summary()

    def show_summary(self):
        """顯示驗證結果"""
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("驗證完成！")
        self.get_logger().info("=" * 70)
        
        errors = np.array([d['error'] for d in self.verification_data])
        error_vecs = np.array([d['error_vec'] for d in self.verification_data])
        
        self.get_logger().info("\n詳細結果:")
        for d in self.verification_data:
            self.get_logger().info(f"  角點 #{d['corner_id']:2d}: 誤差 = {d['error']*1000:6.2f} mm")
        
        rms_error = np.sqrt(np.mean(errors**2))
        max_error = np.max(errors)
        mean_error = np.mean(errors)
        mean_error_vec = error_vecs.mean(axis=0)
        std_error_vec = error_vecs.std(axis=0)
        errors_xy = np.linalg.norm(error_vecs[:, :2], axis=1)
        rms_error_xy = np.sqrt(np.mean(errors_xy**2))
        mean_error_xy = np.mean(errors_xy)
        
        self.get_logger().info(f"\n統計 (3D):")
        self.get_logger().info(f"  RMS 誤差:  {rms_error*1000:.2f} mm")
        self.get_logger().info(f"  最大誤差:  {max_error*1000:.2f} mm")
        self.get_logger().info(f"  平均誤差:  {mean_error*1000:.2f} mm")
        
        self.get_logger().info(f"\n統計 (XY 平面):")
        self.get_logger().info(f"  RMS 誤差:  {rms_error_xy*1000:.2f} mm")
        self.get_logger().info(f"  最大誤差:  {np.max(errors_xy)*1000:.2f} mm")
        self.get_logger().info(f"  平均誤差:  {mean_error_xy*1000:.2f} mm")
        
        self.get_logger().info(f"\n誤差向量:")
        self.get_logger().info(f"  平均: [{mean_error_vec[0]*1000:.2f}, {mean_error_vec[1]*1000:.2f}, {mean_error_vec[2]*1000:.2f}] mm")
        self.get_logger().info(f"  標準差: [{std_error_vec[0]*1000:.2f}, {std_error_vec[1]*1000:.2f}, {std_error_vec[2]*1000:.2f}] mm")
        
        if rms_error * 1000 < 1.0:
            self.get_logger().info("\n✅ 優秀！(< 1mm)")
        elif rms_error * 1000 < 3.0:
            self.get_logger().info("\n✅ 良好！(< 3mm)")
        elif rms_error * 1000 < 5.0:
            self.get_logger().info("\n⚠️  可接受 (< 5mm)")
        else:
            self.get_logger().warn("\n❌ 誤差較大 (> 5mm)")
            if np.abs(mean_error_vec[0]) > 0.01:
                self.get_logger().warn(f"  X 軸系統性偏差: {mean_error_vec[0]*1000:.1f} mm")
            if np.abs(mean_error_vec[2]) > 0.01:
                self.get_logger().warn(f"  Z 軸系統性偏差: {mean_error_vec[2]*1000:.1f} mm")
        
        self.get_logger().info("=" * 70)
        
        # 保存結果
        self.save_results()

    def save_results(self):
        """保存驗證結果"""
        import csv
        
        filename = "verification_with_matrix.csv"
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'corner_id',
                'predicted_x', 'predicted_y', 'predicted_z',
                'actual_x', 'actual_y', 'actual_z',
                'error_x', 'error_y', 'error_z',
                'error_mm'
            ])
            
            for d in self.verification_data:
                writer.writerow([
                    d['corner_id'],
                    d['predicted'][0], d['predicted'][1], d['predicted'][2],
                    d['actual'][0], d['actual'][1], d['actual'][2],
                    d['error_vec'][0], d['error_vec'][1], d['error_vec'][2],
                    d['error'] * 1000
                ])
        
        self.get_logger().info(f"\n✓ 結果已保存到: {filename}")

def main():
    rclpy.init()
    node = SimpleCalibrationVerifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()