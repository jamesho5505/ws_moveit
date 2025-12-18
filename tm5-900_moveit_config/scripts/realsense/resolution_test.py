#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from tm_msgs.msg import FeedbackState

class CameraResolutionTester(Node):
    def __init__(self):
        super().__init__('resolution_tester')
        
        # Chessboard
        self.cols = 4
        self.rows = 4
        
        # Camera
        self.camera_matrix = None
        self.dist_coeffs = None
        self.bridge = CvBridge()
        self.latest_image = None
        
        # Robot
        self.latest_tcp_pose = None
        
        # Test data
        self.reference_corner_pixel = None  # 參考位置的像素座標
        self.reference_tcp_pose = None      # 參考位置的 TCP 座標
        self.test_results = []
        self.target_corner_id = 0  # 要追蹤的角點 (預設左上角)
        self.Z_camera_to_chess = 0.0
        
        # Subscribers
        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.cb_info, 10)
        self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.cb_image, 10)
        self.create_subscription(
            FeedbackState, '/feedback_states', self.cb_feedback, 10)
        
        cv2.namedWindow("Resolution Test", cv2.WINDOW_NORMAL)
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("相機解析度測試")
        self.get_logger().info("=" * 70)
        self.get_logger().info("測試流程:")
        self.get_logger().info("  1. 將棋盤格放在桌上，調整相機高度")
        self.get_logger().info("  2. 按 'r' 記錄參考位置 (初始位置)")
        self.get_logger().info("  3. 移動手臂 X 方向 +1mm，按 't' 測試")
        self.get_logger().info("  4. 移動手臂 X 方向 +2mm，按 't' 測試")
        self.get_logger().info("  5. 重複多次後按 's' 顯示統計")
        self.get_logger().info("  6. 可選: Y 方向重複測試")
        self.get_logger().info("")
        self.get_logger().info("按鍵:")
        self.get_logger().info("  'r' - 設定參考位置")
        self.get_logger().info("  't' - 測試當前位置")
        self.get_logger().info("  's' - 顯示統計結果")
        self.get_logger().info("  'c' - 清除數據重新開始")
        self.get_logger().info("  'q' - 退出")
        self.get_logger().info("=" * 70)

    def cb_info(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
            cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
            self.get_logger().info("✓ 相機內參已載入")
            self.get_logger().info(f"  fx = {fx:.2f}, fy = {fy:.2f}")
            self.get_logger().info(f"  cx = {cx:.2f}, cy = {cy:.2f}")

    def cb_feedback(self, msg: FeedbackState):
        self.latest_tcp_pose = np.array(msg.tool_pose[:3], dtype=float)

    def cb_image(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        display_img = self.latest_image.copy()
        
        # 檢測棋盤格
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(
            gray, (self.cols, self.rows),
            flags=cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY)
        
        if found:
            cv2.drawChessboardCorners(display_img, (self.cols, self.rows), corners, True)
            
            # 標註目標角點
            target_pixel = corners[self.target_corner_id].ravel()
            px, py = int(target_pixel[0]), int(target_pixel[1])
            cv2.circle(display_img, (px, py), 15, (0, 0, 255), 3)
            cv2.putText(display_img, f"Target #{self.target_corner_id}", 
                       (px-50, py-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # 如果有參考位置，顯示當前偏移
            if self.reference_corner_pixel is not None:
                ref_px, ref_py = self.reference_corner_pixel
                cv2.circle(display_img, (int(ref_px), int(ref_py)), 10, (0, 255, 0), 2)
                cv2.line(display_img, (int(ref_px), int(ref_py)), (px, py), (255, 0, 0), 2)
                
                pixel_offset = target_pixel - self.reference_corner_pixel
                tcp_offset = self.latest_tcp_pose - self.reference_tcp_pose if self.latest_tcp_pose is not None else np.zeros(3)
                
                cv2.putText(display_img, f"Pixel offset: [{pixel_offset[0]:.1f}, {pixel_offset[1]:.1f}]",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.putText(display_img, f"TCP offset: [{tcp_offset[0]*1000:.2f}, {tcp_offset[1]*1000:.2f}] mm",
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        else:
            cv2.putText(display_img, "No chessboard detected!", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # 顯示狀態
        if self.reference_corner_pixel is None:
            status = "Press 'r' to set reference"
            color = (0, 255, 0)
        else:
            status = f"Reference set. Move and press 't' to test ({len(self.test_results)} tests)"
            color = (0, 255, 255)
        
        cv2.putText(display_img, status, (10, display_img.shape[0] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # TCP 位置
        if self.latest_tcp_pose is not None:
            tcp_text = f"TCP: [{self.latest_tcp_pose[0]:.4f}, {self.latest_tcp_pose[1]:.4f}, {self.latest_tcp_pose[2]:.4f}]"
            cv2.putText(display_img, tcp_text, (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow("Resolution Test", display_img)
        
        key = cv2.waitKey(10) & 0xFF
        if key == ord('r'):
            self.set_reference(corners if found else None)
        elif key == ord('t'):
            self.test_current_position(corners if found else None)
        elif key == ord('s'):
            self.show_statistics()
        elif key == ord('c'):
            self.clear_data()
        elif key == ord('q'):
            rclpy.shutdown()

    def set_reference(self, corners):
        """設定參考位置"""
        if corners is None:
            self.get_logger().warn("⚠ 未檢測到棋盤格！")
            return
        
        if self.latest_tcp_pose is None:
            self.get_logger().warn("⚠ 無 TCP 數據！")
            return
        
        self.reference_corner_pixel = corners[self.target_corner_id].ravel().copy()
        self.reference_tcp_pose = self.latest_tcp_pose.copy()
        
        # ★★★ 新增：計算相機到棋盤的距離 ★★★
        # 建立棋盤格 3D 模型
        square_size = 0.030  # 30mm
        objp = np.zeros((self.cols * self.rows, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        objp *= square_size
        
        # 使用 solvePnP 計算相機到棋盤的位姿
        ok, rvec, tvec = cv2.solvePnP(
            objp, corners, self.camera_matrix, self.dist_coeffs)
        
        if ok:
            self.Z_camera_to_chess = tvec[2, 0]  # 相機到棋盤的 Z 距離
            
            # 理論解析度（基於實際相機距離）
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            theoretical_res_x = self.Z_camera_to_chess / fx * 1000
            theoretical_res_y = self.Z_camera_to_chess / fy * 1000
            
            self.get_logger().info("\n" + "=" * 70)
            self.get_logger().info("✓ 參考位置已設定")
            self.get_logger().info(f"  像素座標: [{self.reference_corner_pixel[0]:.2f}, {self.reference_corner_pixel[1]:.2f}]")
            self.get_logger().info(f"  TCP 座標: [{self.reference_tcp_pose[0]:.4f}, {self.reference_tcp_pose[1]:.4f}, {self.reference_tcp_pose[2]:.4f}]")
            self.get_logger().info("")
            self.get_logger().info("📷 相機到棋盤距離 (solvePnP):")
            self.get_logger().info(f"  Z = {self.Z_camera_to_chess:.4f} m = {self.Z_camera_to_chess*1000:.1f} mm")
            self.get_logger().info("")
            self.get_logger().info("理論解析度 (基於實際相機距離):")
            self.get_logger().info(f"  X 方向: {theoretical_res_x:.4f} mm/pixel")
            self.get_logger().info(f"  Y 方向: {theoretical_res_y:.4f} mm/pixel")
            self.get_logger().info("=" * 70)
        else:
            self.get_logger().info("\n" + "=" * 70)
            self.get_logger().info("✓ 參考位置已設定")
            self.get_logger().info(f"  像素座標: [{self.reference_corner_pixel[0]:.2f}, {self.reference_corner_pixel[1]:.2f}]")
            self.get_logger().info(f"  TCP 座標: [{self.reference_tcp_pose[0]:.4f}, {self.reference_tcp_pose[1]:.4f}, {self.reference_tcp_pose[2]:.4f}]")
            self.get_logger().warn("  ⚠ solvePnP 計算失敗")
            self.get_logger().info("=" * 70)

    def test_current_position(self, corners):
        """測試當前位置"""
        if self.reference_corner_pixel is None:
            self.get_logger().warn("⚠ 請先設定參考位置 (按 'r')")
            return
        
        if corners is None:
            self.get_logger().warn("⚠ 未檢測到棋盤格！")
            return
        
        if self.latest_tcp_pose is None:
            self.get_logger().warn("⚠ 無 TCP 數據！")
            return
        
        current_pixel = corners[self.target_corner_id].ravel()
        pixel_offset = current_pixel - self.reference_corner_pixel
        tcp_offset = self.latest_tcp_pose - self.reference_tcp_pose
        
        # 計算解析度 (mm/pixel)
        pixel_distance = np.linalg.norm(pixel_offset)
        tcp_distance = np.linalg.norm(tcp_offset[:2]) * 1000  # 只考慮 XY，轉成 mm
        
        if pixel_distance < 0.5:
            self.get_logger().warn("⚠ 像素位移太小，請移動更多 (建議至少 1mm)")
            return
        
        resolution = tcp_distance / pixel_distance
        
        # 分別計算 X 和 Y 方向
        if abs(tcp_offset[0] * 1000) > 1.0: # X 位移大於 1mm
            resolution_x = abs(tcp_offset[0] * 1000) / abs(pixel_offset[0])
        else:
            resolution_x = None # 設為 None，統計時會自動排除

        if abs(tcp_offset[1] * 1000) > 1.0: # Y 位移大於 1mm
            resolution_y = abs(tcp_offset[1] * 1000) / abs(pixel_offset[1])
        else:
            resolution_y = None
        
        result = {
            'pixel_offset': pixel_offset,
            'tcp_offset_mm': tcp_offset * 1000,
            'pixel_distance': pixel_distance,
            'tcp_distance_mm': tcp_distance,
            'resolution': resolution,
            'resolution_x': resolution_x,
            'resolution_y': resolution_y
        }
        
        self.test_results.append(result)
        
        self.get_logger().info("\n" + "-" * 70)
        self.get_logger().info(f"✓ 測試 #{len(self.test_results)} 完成")
        self.get_logger().info(f"  像素位移: [{pixel_offset[0]:6.2f}, {pixel_offset[1]:6.2f}] px")
        self.get_logger().info(f"  TCP 位移:  [{tcp_offset[0]*1000:6.2f}, {tcp_offset[1]*1000:6.2f}, {tcp_offset[2]*1000:6.2f}] mm")
        self.get_logger().info(f"  像素距離: {pixel_distance:.2f} px")
        self.get_logger().info(f"  TCP 距離:  {tcp_distance:.2f} mm (XY)")
        self.get_logger().info(f"  整體解析度: {resolution:.4f} mm/pixel")
        if resolution_x:
            self.get_logger().info(f"  X 方向解析度: {resolution_x:.4f} mm/pixel")
        if resolution_y:
            self.get_logger().info(f"  Y 方向解析度: {resolution_y:.4f} mm/pixel")
        self.get_logger().info("-" * 70)

    def show_statistics(self):
        """顯示統計結果"""
        if len(self.test_results) == 0:
            self.get_logger().warn("⚠ 沒有測試數據！")
            return
        
        resolutions = np.array([r['resolution'] for r in self.test_results])
        resolutions_x = np.array([r['resolution_x'] for r in self.test_results if r['resolution_x'] is not None])
        resolutions_y = np.array([r['resolution_y'] for r in self.test_results if r['resolution_y'] is not None])
        
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("統計結果")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"測試次數: {len(self.test_results)}")
        self.get_logger().info("")
        
        self.get_logger().info("整體解析度 (XY):")
        self.get_logger().info(f"  平均: {resolutions.mean():.4f} mm/pixel")
        self.get_logger().info(f"  標準差: {resolutions.std():.4f} mm/pixel")
        self.get_logger().info(f"  範圍: [{resolutions.min():.4f}, {resolutions.max():.4f}] mm/pixel")
        
        if len(resolutions_x) > 0:
            self.get_logger().info("")
            self.get_logger().info("X 方向解析度:")
            self.get_logger().info(f"  平均: {resolutions_x.mean():.4f} mm/pixel")
            self.get_logger().info(f"  標準差: {resolutions_x.std():.4f} mm/pixel")
        
        if len(resolutions_y) > 0:
            self.get_logger().info("")
            self.get_logger().info("Y 方向解析度:")
            self.get_logger().info(f"  平均: {resolutions_y.mean():.4f} mm/pixel")
            self.get_logger().info(f"  標準差: {resolutions_y.std():.4f} mm/pixel")
        
        # 理論解析度 (從相機內參計算)
        if self.camera_matrix is not None and self.reference_tcp_pose is not None:
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            Z = self.Z_camera_to_chess  # 工作高度
            
            theoretical_res_x = Z / fx * 1000  # mm/pixel
            theoretical_res_y = Z / fy * 1000
            
            self.get_logger().info("")
            self.get_logger().info("理論解析度 (基於內參):")
            self.get_logger().info(f"  工作高度 Z = {Z:.4f} m = {Z*1000:.1f} mm")
            self.get_logger().info(f"  X 方向: {theoretical_res_x:.4f} mm/pixel")
            self.get_logger().info(f"  Y 方向: {theoretical_res_y:.4f} mm/pixel")
            
            if len(resolutions_x) > 0:
                error_x = abs(resolutions_x.mean() - theoretical_res_x) / theoretical_res_x * 100
                self.get_logger().info(f"  X 方向誤差: {error_x:.2f}%")
            if len(resolutions_y) > 0:
                error_y = abs(resolutions_y.mean() - theoretical_res_y) / theoretical_res_y * 100
                self.get_logger().info(f"  Y 方向誤差: {error_y:.2f}%")
        
        self.get_logger().info("=" * 70)
        
        # 保存結果
        self.save_results()

    def save_results(self):
        """保存測試結果"""
        import csv
        
        filename = "camera_resolution_test.csv"
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'test_id',
                'pixel_offset_x', 'pixel_offset_y',
                'tcp_offset_x_mm', 'tcp_offset_y_mm', 'tcp_offset_z_mm',
                'pixel_distance', 'tcp_distance_mm',
                'resolution_overall', 'resolution_x', 'resolution_y'
            ])
            
            for i, r in enumerate(self.test_results, 1):
                writer.writerow([
                    i,
                    r['pixel_offset'][0], r['pixel_offset'][1],
                    r['tcp_offset_mm'][0], r['tcp_offset_mm'][1], r['tcp_offset_mm'][2],
                    r['pixel_distance'], r['tcp_distance_mm'],
                    r['resolution'], r['resolution_x'], r['resolution_y']
                ])
        
        self.get_logger().info(f"\n✓ 結果已保存到: {filename}")

    def clear_data(self):
        """清除數據"""
        self.reference_corner_pixel = None
        self.reference_tcp_pose = None
        self.test_results = []
        self.get_logger().info("\n✓ 數據已清除，可重新開始")

def main():
    rclpy.init()
    node = CameraResolutionTester()
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