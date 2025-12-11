#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import minimalmodbus as mm
import serial
import time
import csv
from math import exp, pi
from scipy.signal import butter, lfilter, lfilter_zi
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
import tf_transformations

class PoseListener(Node):
    def __init__(self):
        super().__init__('tm_pose_listener')
        self.sub = self.create_subscription(PoseStamped, '/tool_pose', self.pose_callback, 10)
        self.R_b2s = np.eye(3)
        self.R = np.eye(3)
        # 建立一個 Publisher 來發布 WrenchStamped 訊息
        self.wrench_publisher = self.create_publisher(
            WrenchStamped, '/robotiq_force_torque_sensor_broadcaster/wrench', 10)

    def pose_callback(self, msg):
        q = msg.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        # euler_angles = tf_transformations.euler_from_quaternion([x, y, z, w])
        # rx = euler_angles[0]
        # ry = euler_angles[1]
        # rz = euler_angles[2]
        # roatation matrix
        # self.R_b2s = np.array([
        #     [np.cos(rz)*np.cos(ry), np.cos(rz)*np.sin(ry)*np.sin(rx)-np.sin(rz)*np.cos(rx), np.cos(rz)*np.sin(ry)*np.cos(rx)+np.sin(rz)*np.sin(rx)],
        #     [np.sin(rz)*np.cos(ry),  np.sin(rz)*np.sin(ry)*np.sin(rx)+np.cos(rz)*np.cos(rx),  np.sin(rz)*np.sin(ry)*np.cos(rx)-np.cos(rz)*np.sin(rx)],
        #     [-np.sin(ry),            np.cos(ry)*np.sin(rx),                                   np.cos(ry)*np.cos(rx)]
        # ])

        # quaternion → rotation matrix
        self.R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
        ])

    # def get_rotation_from_b2s(self):
    #     return self.R_b2s
    
    def get_rotation(self):
        return self.R
    
# ================= 通訊設定 =================
PORTNAME = "/dev/ttyUSB0"
SLAVEADDRESS = 9
BAUDRATE = 19200
BYTESIZE = 8
PARITY = "N"
STOPBITS = 1
TIMEOUT = 0.02
DURATION = 40.0   # 每個姿態收集時間 (秒)，可以縮短以加快流程
SAMPLE_RATE = 100 # 目標取樣率 (Hz)，迴圈會盡力達到這個速度

# ================= 感測器初始化 =================
ser = serial.Serial(port=PORTNAME, baudrate=BAUDRATE, bytesize=BYTESIZE,
                    parity=PARITY, stopbits=STOPBITS, timeout=TIMEOUT)
ser.write(b'\xff' * 50)
ser.close()

mm.BAUDRATE = BAUDRATE
mm.BYTESIZE = BYTESIZE
mm.PARITY = PARITY
mm.STOPBITS = STOPBITS
mm.TIMEOUT = TIMEOUT
ft300 = mm.Instrument(PORTNAME, slaveaddress=SLAVEADDRESS)

# ================= 轉換函式 =================
def forceConverter(v):
    b = bin(v)[2:].zfill(16)
    val = int(b, 2)
    if b[0] == "1":
        val -= 1 << 16
    return val / 100.0  # N

def torqueConverter(v):
    b = bin(v)[2:].zfill(16)
    val = int(b, 2)
    if b[0] == "1":
        val -= 1 << 16
    return val / 1000.0  # N·m

# ================= 濾波函式 =================
def butterworth_filter(x, z, b, a):
    y, zf = lfilter(b, a, [x], zi=z)
    return y[0], zf



# ================= 主流程 =================


# 一開始做一次歸零（取得電子偏置）
print("=== 感測器歸零中，請保持靜止 ===")
registers = ft300.read_registers(180, 6)
fx0, fy0, fz0 = [forceConverter(x) for x in registers[:3]]
tx0, ty0, tz0 = [torqueConverter(x) for x in registers[3:]]
print(f"零點偏置: Fx0={fx0:.3f}, Fy0={fy0:.3f}, Fz0={fz0:.3f} N, Tx0={tx0:.3f}, Ty0={ty0:.3f}, Tz0={tz0:.3f} N.m")
print("=== 歸零完成，準備開始量測 ===")
def main():
    rclpy.init()
    pose_listener = PoseListener()
    print("=== 準備開始力/力矩資料收集 ===")
    
    # EMA filter 參數
    # f_c = 2  # cutoff frequency
    # f_s = 100
    # alpha = 1 - exp(-2 * pi * f_c / f_s) # 越小越平滑但反應越慢

    # ============= Butterworth filter =============
    order = 2      # 濾波階數，可調 2~4
    f_c = 2        # 截止頻率 Hz
    f_s = 32       # 取樣頻率 Hz (根據實際量測約為 31-32 Hz)
    b, a = butter(order, f_c / (f_s / 2), btype='low')
    # ================= 濾波初始值 =================
    zi_force = [lfilter_zi(b, a) * 0 for _ in range(3)]   # Fx, Fy, Fz
    zi_torque = [lfilter_zi(b, a) * 0 for _ in range(3)]  # Tx, Ty, Tz

    fx_f = fy_f = fz_f = 0.0
    tx_f = ty_f = tz_f = 0.0
    
    try:
        for i in range(1, 10):
            print(f"\n=== 準備開始第 {i} 姿態量測 ({DURATION} 秒) ===")
            input("請移動到姿態後按 Enter 開始收集...")

            t0 = time.time()
            records = []
            R_list = []

            while time.time() - t0 < DURATION:
                rclpy.spin_once(pose_listener, timeout_sec=0.0)
                registers = ft300.read_registers(180, 6)
                fx = forceConverter(registers[0])
                fy = forceConverter(registers[1])
                fz = forceConverter(registers[2])
                tx = torqueConverter(registers[3])
                ty = torqueConverter(registers[4])
                tz = torqueConverter(registers[5])

                

                # fx = forceConverter(registers[0]) - fx0
                # fy = forceConverter(registers[1]) - fy0
                # fz = forceConverter(registers[2]) - fz0
                # tx = torqueConverter(registers[3]) - tx0
                # ty = torqueConverter(registers[4]) - ty0
                # tz = torqueConverter(registers[5]) - tz0

                # EMA filter
                # fx_f = (1 - alpha) * fx_f + alpha * fx
                # fy_f = (1 - alpha) * fy_f + alpha * fy
                # fz_f = (1 - alpha) * fz_f + alpha * fz
                # tx_f = (1 - alpha) * tx_f + alpha * tx
                # ty_f = (1 - alpha) * ty_f + alpha * ty
                # tz_f = (1 - alpha) * tz_f + alpha * tz
                
                # Butterworth filter
                fx_f, zi_force[0] = butterworth_filter(fx, zi_force[0], b, a)
                fy_f, zi_force[1] = butterworth_filter(fy, zi_force[1], b, a)
                fz_f, zi_force[2] = butterworth_filter(fz, zi_force[2], b, a)
                tx_f, zi_torque[0] = butterworth_filter(tx, zi_torque[0], b, a)
                ty_f, zi_torque[1] = butterworth_filter(ty, zi_torque[1], b, a)
                tz_f, zi_torque[2] = butterworth_filter(tz, zi_torque[2], b, a)
                # --- 發布原始力矩到 Topic ---
                wrench_msg = WrenchStamped()
                wrench_msg.header.stamp = pose_listener.get_clock().now().to_msg()
                # 根據您的 TF tree，這裡的 frame_id 可能是 'tool0' 或 'flange'
                wrench_msg.header.frame_id = "tool0" 
                wrench_msg.wrench.force.x = fx_f
                wrench_msg.wrench.force.y = fy_f
                wrench_msg.wrench.force.z = fz_f
                wrench_msg.wrench.torque.x = tx_f
                wrench_msg.wrench.torque.y = ty_f
                wrench_msg.wrench.torque.z = tz_f
                pose_listener.wrench_publisher.publish(wrench_msg)
                # --------------------------
                R_now = pose_listener.get_rotation()
                R_list.append(R_now)
                t = time.time() - t0
                # 在高速迴圈中打印會嚴重影響效能，建議只在調試時啟用或降低打印頻率
                if len(records) % 20 == 0: # 每 20 筆資料打印一次
                    print(f"\rCollecting... [Pose {i}] time={t:.2f}s", end="")

                records.append([time.time()-t0, fx_f, fy_f, fz_f, tx_f, ty_f, tz_f])
                time.sleep(1.0 / SAMPLE_RATE)

            R_avg = np.mean(np.array(R_list), axis=0)
            np.save(f"R{i}_matrix.npy", R_avg)
            filename = f"R{i}.csv"
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["t", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz"])
                writer.writerows(records)
            print(f"姿態 {i} 已完成並儲存：{filename}")

        print("\n=== 所有 9 組姿態資料收集完成 ===")
    except KeyboardInterrupt:
        pass
    finally:
        pose_listener.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

# #!/usr/bin/env python3
# import rosbag2_py
# import csv
# from rclpy.serialization import deserialize_message
# from geometry_msgs.msg import WrenchStamped
# # 10 17 18 20 22
# for i in range(1, 4):
#     bag_path = f"/home/jamesho5055/ws_moveit/F{i}_0"  
#     topic_name = "/robotiq/robotiq_force_torque_sensor_broadcaster/wrench"
#     out_file = f"/home/jamesho5055/ws_moveit/F{i}_0.csv"

#     reader = rosbag2_py.SequentialReader()
#     storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
#     converter_options = rosbag2_py.ConverterOptions('', '')
#     reader.open(storage_options, converter_options)

#     with open(out_file, 'w', newline='') as f:
#         writer = csv.writer(f)
#         writer.writerow(['time', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz'])

#         while reader.has_next():
#             (topic, data, t) = reader.read_next()
#             if topic == topic_name:
#                 msg = deserialize_message(data, WrenchStamped)
#                 t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#                 w = msg.wrench
#                 writer.writerow([
#                     f"{t:.6f}",
#                     w.force.x, w.force.y, w.force.z,
#                     w.torque.x, w.torque.y, w.torque.z
#                 ])

# print(f"Saved: {out_file}")
