#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import minimalmodbus as mm
import serial
import time
import csv
import numpy as np

# ================= 通訊設定 =================
PORTNAME = "/dev/ttyUSB0"
SLAVEADDRESS = 9
BAUDRATE = 19200
BYTESIZE = 8
PARITY = "N"
STOPBITS = 1
TIMEOUT = 0.02
DURATION = 10.0   # 每個姿態收集時間 (秒)
SAMPLE_RATE = 100 # 約 0.01 秒取樣

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

# ================= 主流程 =================
alpha = 0.3
fx_f = fy_f = fz_f = 0.0
tx_f = ty_f = tz_f = 0.0

# 一開始做一次歸零（取得電子偏置）
print("=== 感測器歸零中，請保持靜止 ===")
registers = ft300.read_registers(180, 6)
fx0, fy0, fz0 = [forceConverter(x) for x in registers[:3]]
tx0, ty0, tz0 = [torqueConverter(x) for x in registers[3:]]
print(f"零點偏置: Fx0={fx0:.3f}, Fy0={fy0:.3f}, Fz0={fz0:.3f} N")
print("=== 歸零完成，準備開始量測 ===")

for i in range(1, 25):
    print(f"\n=== 準備開始第 {i} 姿態量測 (10 秒) ===")
    input("請移動到姿態後按 Enter 開始收集...")

    t0 = time.time()
    records = []

    while time.time() - t0 < DURATION:
        registers = ft300.read_registers(180, 6)
        fx = forceConverter(registers[0]) - fx0
        fy = forceConverter(registers[1]) - fy0
        fz = forceConverter(registers[2]) - fz0
        tx = torqueConverter(registers[3]) - tx0
        ty = torqueConverter(registers[4]) - ty0
        tz = torqueConverter(registers[5]) - tz0

        fx_f = (1 - alpha) * fx_f + alpha * fx
        fy_f = (1 - alpha) * fy_f + alpha * fy
        fz_f = (1 - alpha) * fz_f + alpha * fz
        tx_f = (1 - alpha) * tx_f + alpha * tx
        ty_f = (1 - alpha) * ty_f + alpha * ty
        tz_f = (1 - alpha) * tz_f + alpha * tz

        records.append([time.time()-t0, fx_f, fy_f, fz_f, tx_f, ty_f, tz_f])
        time.sleep(1.0 / SAMPLE_RATE)

    filename = f"R{i}.csv"
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["t", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz"])
        writer.writerows(records)
    print(f"姿態 {i} 已完成並儲存：{filename}")

print("\n=== 所有 24 組姿態資料收集完成 ===")

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
