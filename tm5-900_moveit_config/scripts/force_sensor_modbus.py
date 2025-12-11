#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2021-01-04

This is a simple example showing of to get measurement data of FT300 using python.

Hardward preparation:
---------------------
The ft300 have to be connected to the PC via USB and power with a 24V power supply.

Dependencies:
*************
MinimalModbus: https://pypi.org/project/MinimalModbus/

@author: Benoit CASTETS
"""
#Libraries importation
import minimalmodbus as mm
import time
from math import *
import serial
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import csv
from scipy.signal import butter, lfilter, lfilter_zi
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
import tf_transformations


class PoseListener(Node):
    def __init__(self):
        super().__init__('tm_pose_listener')
        self.sub = self.create_subscription(PoseStamped, '/tool_pose', self.pose_callback, 10)
        self.R = np.eye(3)
        self.wrench_publisher = self.create_publisher(
            WrenchStamped, '/robotiq_force_torque_sensor_broadcaster/wrench', 10)

    def pose_callback(self, msg):
        q = msg.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        # quaternion → rotation matrix
        self.R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
        ])

    def get_rotation(self):
        return self.R

######################
#Connection parameters
######################

#Communication setup
BAUDRATE=19200
BYTESIZE=8
PARITY="N"
STOPBITS=1
TIMEOUT=0.02

#Change portname according the port on which is connected the FT300

#For Ubuntu
############
#Name of the port (string) where is connected the gripper. Usually
#/dev/ttyUSB0 on Linux. It is necesary to allow permission to access
#this connection using the bash command sudo chmod 666 /dev/ttyUSB0

#For windows
############
#Check the name of the port using robotiq user interface. It should be something
#like: COM12

PORTNAME="/dev/ttyUSB0"

SLAVEADDRESS=9

############################
#Desactivate streaming mode
############################


#To stop the data stream, communication must be interrupted by sending a series of 0xff characters to the Sensor. Sending for about
#0.5s (50 times)will ensure that the Sensor stops the stream.

ser=serial.Serial(port=PORTNAME, baudrate=BAUDRATE, bytesize=BYTESIZE, parity=PARITY, stopbits=STOPBITS, timeout=TIMEOUT)

packet = bytearray()
sendCount=0
while sendCount<50:
  packet.append(0xff)
  sendCount=sendCount+1
ser.write(packet)
ser.close()

####################
#Setup minimalmodbus
####################

#Communication setup
mm.BAUDRATE=BAUDRATE
mm.BYTESIZE=BYTESIZE
mm.PARITY=PARITY
mm.STOPBITS=STOPBITS
mm.TIMEOUT=TIMEOUT

#Create FT300 object
ft300=mm.Instrument(PORTNAME, slaveaddress=SLAVEADDRESS)

#Uncomment to see binary messages for debug
#ft300.debug=True
#ft300.mode=mm.MODE_RTU


# EMA filter
# f_c = 2  # cutoff frequency
# f_s = 100
# alpha = 1 - exp(-2 * pi * f_c / f_s) # 越小越平滑但反應越慢


# Butterworth filter
order = 2      # 濾波階數，可調 2~4
f_c = 5        # 截止頻率 Hz
f_s = 100      # 取樣頻率 Hz
b, a = butter(order, f_c / (f_s / 2), btype='low')
  
fx_f = fy_f = fz_f = 0.0
tx_f = ty_f = tz_f = 0.0

# === 補償參數（從 gravitycompensation.py 複製）===
# Fb = np.array([-0.59453481, -0.38467052, -8.75033126])    # N
# Tau_b = np.array([-0.0367479, -0.04578258, 0.03606966])   # N·m
# m = 0.8780394813817137                                   # kg
# r = np.array([-0.00070755, -0.00017134, 0.0037738])      # m
# g = 9.80665
# R = np.array([[1,0,0],[0,-1,0],[0,0,-1]])
Fb = np.array([-7.98084963, 6.901707, -9.09057885])    # N
Tau_b = np.array([0.25941941, 0.23372976, -0.00650272])   # N·m
m = 1.07106                                  # kg
r = np.array([0.00164493, 0.00839269, -0.05392863])      # m
g = 9.81

# === 補償函式 ===
def compensate(F_meas, Tau_meas, R, Fb, Tau_b, m, r, g=9.81):
    gI = np.array([0,0,-g])
    gs = R.T @ gI
    Fg = m * gs
    Tg = np.cross(r, Fg)
    F_contact = F_meas - Fb - Fg
    T_contact = Tau_meas - Tau_b - Tg
    return F_contact, T_contact

# ================= 濾波函式 =================
def butterworth_filter(x, z, b, a):
    y, zf = lfilter(b, a, [x], zi=z)
    return y[0], zf

zi_force = [lfilter_zi(b, a) * 0 for _ in range(3)]   # Fx, Fy, Fz
zi_torque = [lfilter_zi(b, a) * 0 for _ in range(3)]  # Tx, Ty, Tz



####################
#Functions
####################

def forceConverter(forceRegisterValue):
  """Return the force corresponding to force register value.
  
  input:
    forceRegisterValue: Value of the force register
    
  output:
    force: force corresponding to force register value in N
  """
  force=0

  forceRegisterBin=bin(forceRegisterValue)[2:]
  forceRegisterBin="0"*(16-len(forceRegisterBin))+forceRegisterBin
  if forceRegisterBin[0]=="1":
    #negative force
    force=-1*(int("1111111111111111",2)-int(forceRegisterBin,2)+1)/100
  else:
    #positive force
    force=int(forceRegisterBin,2)/100
  return force

def torqueConverter(torqueRegisterValue):
  """Return the torque corresponding to torque register value.
  
  input:
    torqueRegisterValue: Value of the torque register
    
  output:
    torque: torque corresponding to force register value in N.m
  """
  torque=0

  torqueRegisterBin=bin(torqueRegisterValue)[2:]
  torqueRegisterBin="0"*(16-len(torqueRegisterBin))+torqueRegisterBin
  if torqueRegisterBin[0]=="1":
    #negative force
    torque=-1*(int("1111111111111111",2)-int(torqueRegisterBin,2)+1)/1000
  else:
    #positive force
    torque=int(torqueRegisterBin,2)/1000
  return torque


####################
#Main program
####################

if __name__ == '__main__':
  rclpy.init()
  pose_node = PoseListener()

#Get FT300 force and torque
  try:
    #Initialisation
  
    #Read registers where are saved force and torque values.
    registers=ft300.read_registers(180,6)

    #Save measured values at rest. Those values are use to make the zero of the sensor.
    fxZero=forceConverter(registers[0])
    fyZero=forceConverter(registers[1])
    fzZero=forceConverter(registers[2])
    txZero=torqueConverter(registers[3])
    tyZero=torqueConverter(registers[4])
    tzZero=torqueConverter(registers[5])

    # 緩衝設定
    max_points = 1000   # 顯示最近 N 筆資料
    fx_list, fy_list, fz_list = deque(maxlen=max_points), deque(maxlen=max_points), deque(maxlen=max_points)
    fx_f_list, fy_f_list, fz_f_list = deque(maxlen=max_points), deque(maxlen=max_points), deque(maxlen=max_points)
    tx_list, ty_list, tz_list = deque(maxlen=max_points), deque(maxlen=max_points), deque(maxlen=max_points)
    tx_f_list, ty_f_list, tz_f_list = deque(maxlen=max_points), deque(maxlen=max_points), deque(maxlen=max_points)
    t_list = deque(maxlen=max_points)
    t0 = time.time()

    # === 顏色設定（可改 HEX 或常用色碼）===
    colors = {
        'Fx': 'tab:red',
        'Fy': 'tab:green',
        'Fz': 'tab:blue',
        'Tx': 'tab:orange',
        'Ty': 'tab:purple',
        'Tz': 'tab:brown',
        'F_x_f': 'tab:pink',
        'F_y_f': 'tab:cyan',
        'F_z_f': 'tab:gray',
        'T_x_f': 'tab:olive',
        'T_y_f': 'tab:red',
        'T_z_f': 'tab:green',
    }

    plt.ion()  # 開啟互動模式
    fig, axs = plt.subplots(3, 2, figsize=(10,12))
    names = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz']
    lines = []
    # for ax in axs.flat:
    #     line, = ax.plot([], [])
    #     lines.append(line)
    # for ax, name in zip(axs.flat, names):
    #     ax.set_title(name)
    #     ax.set_xlim(0, max_points)
    #     ax.set_ylim(-3, 3)  # 根據你的量程調整
    #     ax.grid(True)
    # for ax, name in zip(axs.flat, names):
    #   # line, = ax.plot([], [], color=colors[name], label=name)
    #   line_raw, = ax.plot([], [], color=colors[name], label=f"{name} raw")
    #   line_filt, = ax.plot([], [], color=colors[f"{name[0]}_{name[1]}_f"], linestyle='--', label=f"{name} filtered")
    #   ax.set_title(name)
    #   ax.set_xlim(0, max_points)
    #   ax.set_ylim(-1, 1)  # 依你的量測範圍調整
    #   ax.grid(True)
    #   ax.legend(loc='upper right')
    #   lines.append((line_raw, line_filt)) 

    # csv_filename = "force_sensor_modbus_data_5hz.csv"
    # csv_fields = ['time', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz', 'F_x_f', 'F_y_f', 'F_z_f', 'T_x_f', 'T_y_f', 'T_z_f']
    # csv_file = open(csv_filename, mode='w', newline='')
    # csv_writer = csv.writer(csv_file)
    # csv_writer.writerow(csv_fields)
    
    
    #main loop
    while True:
      rclpy.spin_once(pose_node, timeout_sec=0.0)   # 更新 ROS2 訂閱
      #Read registers where are saved force and torque values.
      registers=ft300.read_registers(180,6)
      
      #Calculate measured value form register values
      # fx=round(forceConverter(registers[0])-fxZero,0)
      # fy=round(forceConverter(registers[1])-fyZero,0)
      # fz=round(forceConverter(registers[2])-fzZero,0)
      # tx=round(torqueConverter(registers[3])-txZero,2)
      # ty=round(torqueConverter(registers[4])-tyZero,2)
      # tz=round(torqueConverter(registers[5])-tzZero,2)
      # no offset substract
      fx=forceConverter(registers[0])
      fy=forceConverter(registers[1])
      fz=forceConverter(registers[2])
      tx=torqueConverter(registers[3])
      ty=torqueConverter(registers[4])
      tz=torqueConverter(registers[5])
      # substract zero offset manually
      # fx=forceConverter(registers[0])-fxZero
      # fy=forceConverter(registers[1])-fyZero
      # fz=forceConverter(registers[2])-fzZero
      # tx=torqueConverter(registers[3])-txZero
      # ty=torqueConverter(registers[4])-tyZero
      # tz=torqueConverter(registers[5])-tzZero
      # 2hz
      # fx=forceConverter(registers[0])-(-9.16)
      # fy=forceConverter(registers[1])-(10.070)
      # fz=forceConverter(registers[2])-(-10.060)
      # tx=torqueConverter(registers[3])-(0.461)
      # ty=torqueConverter(registers[4])-(0.344)
      # tz=torqueConverter(registers[5])-(-0.011)
      #5hz
      # fx=forceConverter(registers[0])-(-3.28)
      # fy=forceConverter(registers[1])-(6.190)
      # fz=forceConverter(registers[2])-(6.070)
      # tx=torqueConverter(registers[3])-(0.331)
      # ty=torqueConverter(registers[4])-(0.149)
      # tz=torqueConverter(registers[5])-(-0.093)
      #10hz 零點偏置: Fx0=-2.390, Fy0=3.320, Fz0=7.750 N, Tx0=0.282, Ty0=0.124, Tz0=-0.110 N.m
      # fx=forceConverter(registers[0])-(-2.390)
      # fy=forceConverter(registers[1])-(3.320)
      # fz=forceConverter(registers[2])-(7.750)
      # tx=torqueConverter(registers[3])-(0.282)
      # ty=torqueConverter(registers[4])-(0.124)
      # tz=torqueConverter(registers[5])-(-0.110)
      # butterworth filter 零點偏置: Fx0=-2.900, Fy0=5.270, Fz0=6.950 N, Tx0=0.308, Ty0=0.167, Tz0=-0.058 N.m
      # 2 order 5 hz
      # fx=forceConverter(registers[0])-(-2.900)
      # fy=forceConverter(registers[1])-(5.270)
      # fz=forceConverter(registers[2])-(6.950)
      # tx=torqueConverter(registers[3])-(0.308)
      # ty=torqueConverter(registers[4])-(0.167)
      # tz=torqueConverter(registers[5])-(-0.058)

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

      wrench_msg = WrenchStamped()
      wrench_msg.header.stamp = pose_node.get_clock().now().to_msg()
      # 根據您的 TF tree，這裡的 frame_id 可能是 'tool0' 或 'flange'
      wrench_msg.header.frame_id = "tool0" 
      wrench_msg.wrench.force.x = fx_f
      wrench_msg.wrench.force.y = fy_f
      wrench_msg.wrench.force.z = fz_f
      wrench_msg.wrench.torque.x = tx_f
      wrench_msg.wrench.torque.y = ty_f
      wrench_msg.wrench.torque.z = tz_f
      pose_node.wrench_publisher.publish(wrench_msg)

      
      R = pose_node.get_rotation()                  # 即時旋轉矩陣


      # F_meas = np.array([fx_f, fy_f, fz_f])
      # Tau_meas = np.array([tx_f, ty_f, tz_f])
      # F_comp, Tau_comp = compensate(F_meas, Tau_meas, R, Fb, Tau_b, m, r)
      # print(f"補償後力: {F_comp},  |F|={np.linalg.norm(F_comp):.3f} N")

      # fx_f /= 9.80665
      # fy_f /= 9.80665
      # fz_f /= 9.80665
      # print(f"time={time.time()-t0:.5f} fx={fx_f:.5f} N, fy={fy_f:.5f} N, fz={fz_f:.5f} N, tx={tx_f:.5f} N.m, ty={ty_f:.5f} N.m, tz={tz_f:.5f} N.m")
      # print(f"R={R}")
      # print(f"time={time.time()-t0:.5f} fx={F_comp[0]:.5f} N, fy={F_comp[1]:.5f} N, fz={F_comp[2]:.5f} N, tx={Tau_comp[0]:.5f} N.m, ty={Tau_comp[1]:.5f} N.m, tz={Tau_comp[2]:.5f} N.m, |F|={np.linalg.norm(F_comp):.3f} N")
      # print(f"fx={fx_f:.3f} kg, fy={fy_f:.3f} kg, fz={fz_f:.3f} kg, tx={tx_f:.3f} N.m, ty={ty_f:.3f} N.m, tz={tz_f:.3f} N.m")
      # t = time.time() - t0
      # t_list.append(t)
      # fx_list.append(fx); fy_list.append(fy); fz_list.append(fz)
      # tx_list.append(tx); ty_list.append(ty); tz_list.append(tz)

      # fx_f_list.append(fx_f); fy_f_list.append(fy_f); fz_f_list.append(fz_f)
      # tx_f_list.append(tx_f); ty_f_list.append(ty_f); tz_f_list.append(tz_f)

      # t = time.time() - t0
      # t_list.append(t)
      # fx_list.append(F_comp[0]); fy_list.append(F_comp[1]); fz_list.append(F_comp[2])
      # tx_list.append(Tau_comp[0]); ty_list.append(Tau_comp[1]); tz_list.append(Tau_comp[2])

      # 更新六條曲線
    #   data_sets = [fx_list, fy_list, fz_list, tx_list, ty_list, tz_list]
    #   data_sets = [
    #     fx_list, fx_f_list,
    #     fy_list, fy_f_list,
    #     fz_list, fz_f_list,
    #     tx_list, tx_f_list,
    #     ty_list, ty_f_list,
    #     tz_list, tz_f_list
    # ]

      # csv_writer.writerow([t, fx, fy, fz, tx, ty, tz, fx_f, fy_f, fz_f, tx_f, ty_f, tz_f])
      # csv_writer.writerow([t, F_comp[0], F_comp[1], F_comp[2], Tau_comp[0], Tau_comp[1], Tau_comp[2], fx_f, fy_f, fz_f, tx_f, ty_f, tz_f])
      # # for i, line in enumerate(lines):
      # #     line.set_data(range(len(data_sets[i])), list(data_sets[i]))
      # #     axs.flat[i].relim(); axs.flat[i].autoscale_view()
      # for i, (line_raw, line_filt) in enumerate(lines):
      #   # 每個 i 對應兩組資料
      #   line_raw.set_data(range(len(data_sets[i*2])), list(data_sets[i*2]))
      #   line_filt.set_data(range(len(data_sets[i*2+1])), list(data_sets[i*2+1]))
      #   axs.flat[i].relim()
      #   axs.flat[i].autoscale_view()


      # plt.pause(0.01)

      #Display result
      # print("***Press Ctrl+C to stop the program***")
      # print("fx=",fx,"N")
      # print("fy=",fy,"N")
      # print("fz=",fz,"N")
      # print("tx=",tx,"N.m")
      # print("ty=",ty,"N.m")
      # print("tz=",tz,"N.m")
      
      # time.sleep(0.01)
      
  except KeyboardInterrupt:
    print("Program ended")
  finally:
    # csv_file.close() 
    pose_node.destroy_node()
    rclpy.shutdown()