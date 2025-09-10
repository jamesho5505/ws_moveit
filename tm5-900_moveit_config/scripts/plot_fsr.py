#!/usr/bin/env python3
import rclpy, sys, signal
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# --- 圖表設定 ---
HISTORY_LEN = 500         # 圖表上保留的歷史數據點數量
YLIM = (0, 3000)          # Y 軸範圍（單位：克），請根據您的感測器範圍調整

class FSRPlotter(Node):
    """
    訂閱 /arduino/force 主題並即時繪製數據的節點。
    """
    def __init__(self):
        super().__init__('fsr_plotter_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/arduino/force',
            self.force_callback,
            10)
        
        # 用於儲存數據的 Deque
        self.history = {
            'Force1': deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN),
            'Force2': deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN),
            'Average': deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        }
        self.labels = ['Force1', 'Force2', 'Average']
        self.colors = ['tab:blue', 'tab:green', 'tab:red']
        self.lines = []

        # 設定 Matplotlib 圖表
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.fig.canvas.manager.set_window_title("FSR Live Force Viewer")
        
        for label, color in zip(self.labels, self.colors):
            line, = self.ax.plot(self.history[label], color=color, label=label)
            self.lines.append(line)
            
        self.ax.set_title("Live Force Sensor Data")
        self.ax.set_xlabel("Time (samples)")
        self.ax.set_ylabel("Force (grams)")
        self.ax.set_xlim(0, HISTORY_LEN - 1)
        self.ax.set_ylim(*YLIM)
        self.ax.legend()
        self.ax.grid(True)
        self.fig.tight_layout()

    def force_callback(self, msg: Float32MultiArray):
        """當收到新的力量數據時，更新 deque。"""
        if len(msg.data) >= 3:
            self.history['Force1'].append(msg.data[0])
            self.history['Force2'].append(msg.data[1])
            self.history['Average'].append(msg.data[2])

    def update_plot(self, frame):
        """動畫更新函式，由 FuncAnimation 呼叫。"""
        # 處理掛起的 ROS 訊息
        rclpy.spin_once(self, timeout_sec=0)
        
        # 更新圖表上的線條數據
        for line, label in zip(self.lines, self.labels):
            line.set_ydata(self.history[label])
        return self.lines

def main():
    rclpy.init()
    node = FSRPlotter()

    # 設定動畫，每 50ms 更新一次
    ani = FuncAnimation(node.fig, node.update_plot, interval=50, blit=True)

    # 設定乾淨地關閉程式
    def quit_handler(sig, frame):
        print("\n偵測到 Ctrl-C，正在關閉節點...")
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, quit_handler)

    # 顯示圖表 (這是一個阻塞操作)
    plt.show()

    # 當圖表視窗被關閉後，清理資源
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
