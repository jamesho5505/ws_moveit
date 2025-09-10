#!/usr/bin/env python3
import rclpy, sys, signal, csv
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# --- 圖表設定 ---
HISTORY_LEN = 500  # 圖表上保留的歷史數據點數量
YLIM_FORCE = (0, 3000)  # 左 Y 軸範圍（單位：克），請根據您的感測器範圍調整
YLIM_JOINT = (-0.1, 0.9)  # 右 Y 軸範圍（單位：弧度），對應 Robotiq 夾爪的 0.0~0.8 rad

# --- 夾爪關節名稱 ---
# 請根據您的 URDF 檔案確認，通常是 'robotiq_85_left_knuckle_joint'
GRIPPER_JOINT_NAME = 'robotiq_85_left_knuckle_joint'

class FSRPlotter(Node):
    """
    訂閱 /arduino/force 和 /joint_states 主題並即時繪製數據的節點。
    """
    def __init__(self):
        super().__init__('fsr_plotter_node')
        self.force_subscription = self.create_subscription(
            Float32MultiArray,
            '/arduino/force',
            self.force_callback,
            10)
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)
        self.ftarget_subscription = self.create_subscription(
            Float32,
            '/gripper/target_force',
            self.ftarget_callback,
            10)
        self.gripper_joint_index = -1
        self.output_filename = "gripper_data.csv"
        
        # 用於儲存數據的 Deque
        self.history = {
            'Force1': deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN),
            'Force2': deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN),
            'Average': deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN),
            'F_target': deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN),
            'GripperPos': deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        }

        self.force_labels = ['Force1', 'Force2', 'Average', 'F_target']
        self.force_colors = ['tab:blue', 'tab:green', 'tab:red', 'black']
        self.lines = []

        # 設定 Matplotlib 圖表
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.fig.canvas.manager.set_window_title("FSR Live Force Viewer")
        
        # 設定左 Y 軸 (力量)
        self.ax.set_title("Live Force and Gripper Position")
        self.ax.set_xlabel("Time (samples)")
        self.ax.set_ylabel("Force (grams)", color='tab:red')
        self.ax.set_xlim(0, HISTORY_LEN - 1)
        self.ax.set_ylim(*YLIM_FORCE)
        self.ax.tick_params(axis='y', labelcolor='tab:red')
        self.ax.grid(True)

        for label, color in zip(self.force_labels, self.force_colors):
            linestyle = '--' if label == 'F_target' else '-'
            linewidth = 2.0 if label == 'F_target' else 1.5
            line, = self.ax.plot(self.history[label], color=color, label=label, 
                                 linestyle=linestyle, linewidth=linewidth)
            self.lines.append(line)

        # 設定右 Y 軸 (夾爪位置)
        self.ax2 = self.ax.twinx()
        self.ax2.set_ylabel('Gripper Position (rad)', color='tab:purple')
        self.ax2.set_ylim(*YLIM_JOINT)
        self.ax2.tick_params(axis='y', labelcolor='tab:purple')
        line, = self.ax2.plot(self.history['GripperPos'], color='tab:purple', linestyle='--', label='Gripper Position')
        self.lines.append(line)

        # 整合圖例
        self.fig.legend(loc="upper right", bbox_to_anchor=(1,1), bbox_transform=self.ax.transAxes)
        self.fig.tight_layout()

    def force_callback(self, msg: Float32MultiArray):
        """當收到新的力量數據時，更新 deque。"""
        # To keep data synchronized, we should append to other deques as well.
        # Here, we just append the last known value.
        if len(msg.data) >= 3:
            self.history['Force1'].append(msg.data[0])
            self.history['Force2'].append(msg.data[1])
            self.history['Average'].append(msg.data[2])
            # Keep other deques in sync
            if self.history['F_target']: self.history['F_target'].append(self.history['F_target'][-1])
            if self.history['GripperPos']: self.history['GripperPos'].append(self.history['GripperPos'][-1])

    def ftarget_callback(self, msg: Float32):
        """當收到新的目標力道數據時，更新 deque。"""
        self.history['F_target'].append(msg.data)
        if self.history['Average']: self.history['Average'].append(self.history['Average'][-1])
        if self.history['Force1']: self.history['Force1'].append(self.history['Force1'][-1])
        if self.history['Force2']: self.history['Force2'].append(self.history['Force2'][-1])
        if self.history['GripperPos']: self.history['GripperPos'].append(self.history['GripperPos'][-1])

    def joint_callback(self, msg: JointState):
        """當收到新的關節狀態時，更新夾爪位置。"""
        # 第一次收到訊息時，找到夾爪關節的索引
        if self.gripper_joint_index == -1:
            try:
                self.gripper_joint_index = msg.name.index(GRIPPER_JOINT_NAME)
                self.get_logger().info(f"成功找到夾爪關節 '{GRIPPER_JOINT_NAME}' 於索引 {self.gripper_joint_index}")
            except ValueError:
                self.get_logger().error(f"在 /joint_states 中找不到關節 '{GRIPPER_JOINT_NAME}'！請檢查 URDF 和關節名稱。")
                self.gripper_joint_index = -2 # 標記為已嘗試但失敗

        if self.gripper_joint_index >= 0:
            self.history['GripperPos'].append(msg.position[self.gripper_joint_index])
            # Keep other deques in sync
            if self.history['F_target']: self.history['F_target'].append(self.history['F_target'][-1])
            if self.history['Average']: self.history['Average'].append(self.history['Average'][-1])
            if self.history['Force1']: self.history['Force1'].append(self.history['Force1'][-1])
            if self.history['Force2']: self.history['Force2'].append(self.history['Force2'][-1])

    def update_plot(self, frame):
        """動畫更新函式，由 FuncAnimation 呼叫。"""
        # 處理掛起的 ROS 訊息
        rclpy.spin_once(self, timeout_sec=0)
        
        # 更新圖表上的線條數據
        all_labels = self.force_labels + ['GripperPos']
        for line, label in zip(self.lines, all_labels):
            line.set_ydata(self.history[label])
        return self.lines

    def save_to_csv(self, filename):
        """將 history 中的數據儲存到 CSV 檔案。"""
        self.get_logger().info(f"正在將數據儲存至 {filename}...")
        try:
            # Convert deques to lists for saving
            data_to_save = {key: list(value) for key, value in self.history.items()}

            # Find the minimum length to avoid index errors if they are not perfectly synced on exit
            num_points = min(len(v) for v in data_to_save.values())
            if num_points == 0:
                self.get_logger().warn("No data points to save.")
                return

            header = ['Sample', 'Force1', 'Force2', 'Average', 'GripperPos', 'F_target']
            
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(header)
                for i in range(num_points):
                    row = [i, data_to_save['Force1'][i], data_to_save['Force2'][i], data_to_save['Average'][i], data_to_save['GripperPos'][i], data_to_save['F_target'][i]]
                    writer.writerow(row)
            self.get_logger().info(f"成功儲存 {num_points} 筆數據。")
        except Exception as e:
            self.get_logger().error(f"儲存 CSV 失敗: {e}")

def main():
    rclpy.init()
    node = FSRPlotter()

    # 設定動畫，每 50ms 更新一次
    ani = FuncAnimation(node.fig, node.update_plot, interval=50, blit=True)

    # 設定乾淨地關閉程式
    def quit_handler(sig, frame):
        print("\n偵測到 Ctrl-C，正在關閉節點...")
        node.save_to_csv(node.output_filename)
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, quit_handler)

    # 顯示圖表 (這是一個阻塞操作)
    plt.show()

    # 當圖表視窗被關閉後，清理資源
    node.save_to_csv(node.output_filename)
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
