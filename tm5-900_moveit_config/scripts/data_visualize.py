#!/usr/bin/env python3
import rclpy, sys, signal, csv
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# --- 圖表設定 ---
HISTORY_LEN = 500  # 圖表上保留的歷史數據點數量 (僅用於顯示)
YLIM_FORCE = (0, 3000)
YLIM_JOINT = (-0.1, 0.9)

# --- 夾爪關節名稱 ---
GRIPPER_JOINT_NAME = 'robotiq_85_left_knuckle_joint'

class FSRPlotter(Node):
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
        
        # 使用 List 來儲存所有數據
        self.history = {
            'Force1': [],
            'Force2': [],
            'Average': [],
            'F_target': [],
            'GripperPos': []
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
        self.ax.set_ylim(*YLIM_FORCE)
        self.ax.tick_params(axis='y', labelcolor='tab:red')
        self.ax.grid(True)

        for label, color in zip(self.force_labels, self.force_colors):
            linestyle = '--' if label == 'F_target' else '-'
            linewidth = 2.0 if label == 'F_target' else 1.5
            line, = self.ax.plot([], [], color=color, label=label, 
                                 linestyle=linestyle, linewidth=linewidth)
            self.lines.append(line)

        # 設定右 Y 軸 (夾爪位置)
        self.ax2 = self.ax.twinx()
        self.ax2.set_ylabel('Gripper Position (rad)', color='tab:purple')
        self.ax2.set_ylim(*YLIM_JOINT)
        self.ax2.tick_params(axis='y', labelcolor='tab:purple')
        line, = self.ax2.plot([], [], color='tab:purple', linestyle='--', label='Gripper Position')
        self.lines.append(line)

        # 整合圖例
        self.fig.legend(loc="upper right", bbox_to_anchor=(1,1), bbox_transform=self.ax.transAxes)
        self.fig.tight_layout()

    def force_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self.history['Force1'].append(msg.data[0])
            self.history['Force2'].append(msg.data[1])
            self.history['Average'].append(msg.data[2])
            
            # 確保所有列表長度同步
            target_len = len(self.history['Average'])
            while len(self.history['F_target']) < target_len:
                self.history['F_target'].append(self.history['F_target'][-1] if self.history['F_target'] else 0.0)
            while len(self.history['GripperPos']) < target_len:
                self.history['GripperPos'].append(self.history['GripperPos'][-1] if self.history['GripperPos'] else 0.0)

    def ftarget_callback(self, msg: Float32):
        self.history['F_target'].append(msg.data)
        
        # 確保所有列表長度同步
        target_len = len(self.history['F_target'])
        while len(self.history['Average']) < target_len:
            self.history['Average'].append(self.history['Average'][-1] if self.history['Average'] else 0.0)
        while len(self.history['Force1']) < target_len:
            self.history['Force1'].append(self.history['Force1'][-1] if self.history['Force1'] else 0.0)
        while len(self.history['Force2']) < target_len:
            self.history['Force2'].append(self.history['Force2'][-1] if self.history['Force2'] else 0.0)
        while len(self.history['GripperPos']) < target_len:
            self.history['GripperPos'].append(self.history['GripperPos'][-1] if self.history['GripperPos'] else 0.0)

    def joint_callback(self, msg: JointState):
        if self.gripper_joint_index == -1:
            try:
                self.gripper_joint_index = msg.name.index(GRIPPER_JOINT_NAME)
                self.get_logger().info(f"成功找到夾爪關節 '{GRIPPER_JOINT_NAME}' 於索引 {self.gripper_joint_index}")
            except ValueError:
                self.get_logger().error(f"在 /joint_states 中找不到關節 '{GRIPPER_JOINT_NAME}'！請檢查 URDF 和關節名稱。")
                self.gripper_joint_index = -2
        
        if self.gripper_joint_index >= 0:
            self.history['GripperPos'].append(msg.position[self.gripper_joint_index])
            
            # 確保所有列表長度同步
            target_len = len(self.history['GripperPos'])
            while len(self.history['Average']) < target_len:
                self.history['Average'].append(self.history['Average'][-1] if self.history['Average'] else 0.0)
            while len(self.history['Force1']) < target_len:
                self.history['Force1'].append(self.history['Force1'][-1] if self.history['Force1'] else 0.0)
            while len(self.history['Force2']) < target_len:
                self.history['Force2'].append(self.history['Force2'][-1] if self.history['Force2'] else 0.0)
            while len(self.history['F_target']) < target_len:
                self.history['F_target'].append(self.history['F_target'][-1] if self.history['F_target'] else 0.0)

    def update_plot(self, frame):
        rclpy.spin_once(self, timeout_sec=0)
        
        num_points = len(self.history['Average'])
        if num_points > 0:
            x_data = range(num_points)
            
            # 動態調整 x 軸顯示範圍
            if num_points > HISTORY_LEN:
                self.ax.set_xlim(num_points - HISTORY_LEN, num_points)
            else:
                self.ax.set_xlim(0, HISTORY_LEN)

            all_labels = self.force_labels + ['GripperPos']
            for line, label in zip(self.lines, all_labels):
                line.set_data(x_data, self.history[label])
        
        return self.lines

    def save_to_csv(self, filename):
        self.get_logger().info(f"正在將數據儲存至 {filename}...")
        try:
            data_to_save = self.history
            num_points = len(data_to_save['Average'])
            if num_points == 0:
                self.get_logger().warn("No data points to save.")
                return

            header = ['Sample', 'Force1', 'Force2', 'Average', 'GripperPos', 'F_target']
            
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(header)
                for i in range(num_points):
                    row = [
                        i,
                        data_to_save['Force1'][i],
                        data_to_save['Force2'][i],
                        data_to_save['Average'][i],
                        data_to_save['GripperPos'][i],
                        data_to_save['F_target'][i]
                    ]
                    writer.writerow(row)
            self.get_logger().info(f"成功儲存 {num_points} 筆數據。")
        except Exception as e:
            self.get_logger().error(f"儲存 CSV 失敗: {e}")

def main():
    rclpy.init()
    node = FSRPlotter()

    ani = FuncAnimation(node.fig, node.update_plot, interval=50, blit=False)

    def quit_handler(sig, frame):
        print("\n偵測到 Ctrl-C，正在關閉節點...")
        node.save_to_csv(node.output_filename)
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, quit_handler)

    plt.show()

    node.save_to_csv(node.output_filename)
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()