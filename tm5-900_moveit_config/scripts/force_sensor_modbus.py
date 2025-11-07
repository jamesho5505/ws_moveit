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
import time

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

alpha = 0.5   # 越小越平滑但反應越慢
fx_f = fy_f = fz_f = 0.0
tx_f = ty_f = tz_f = 0.0

# === 補償參數（從 gravitycompensation.py 複製）===
Fb = np.array([0.19119961, -0.50376002, -10.18068766])   # N
Tau_b = np.array([0.0631148, -0.00212744, 0.09212566])   # N·m
m = 1.04                                                  # kg
r = np.array([0.00330599, 0.01234476, 0.00201757])       # m
g = 9.80665
R = np.array([[1,0,0],[0,-1,0],[0,0,-1]])

# === 補償函式 ===
def compensate(F_meas, Tau_meas, R, Fb, Tau_b, m, r, g=9.80665):
    gI = np.array([0,0,-g])
    gs = R.T @ gI
    Fg = m * gs
    Tg = np.cross(r, Fg)
    F_contact = F_meas - Fb - Fg
    T_contact = Tau_meas - Tau_b - Tg
    return F_contact, T_contact



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
    tx_list, ty_list, tz_list = deque(maxlen=max_points), deque(maxlen=max_points), deque(maxlen=max_points)
    t_list = deque(maxlen=max_points)
    t0 = time.time()

    # === 顏色設定（可改 HEX 或常用色碼）===
    colors = {
        'Fx': 'tab:red',
        'Fy': 'tab:green',
        'Fz': 'tab:blue',
        'Tx': 'tab:orange',
        'Ty': 'tab:purple',
        'Tz': 'tab:brown'
    }

    plt.ion()  # 開啟互動模式
    fig, axs = plt.subplots(3, 2, figsize=(10,8))
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
    for ax, name in zip(axs.flat, names):
      line, = ax.plot([], [], color=colors[name], label=name)
      ax.set_title(name)
      ax.set_xlim(0, max_points)
      ax.set_ylim(-2, 2)  # 依你的量測範圍調整
      ax.grid(True)
      ax.legend(loc='upper right')
      lines.append(line)
    
    #main loop
    while True:
      #Read registers where are saved force and torque values.
      registers=ft300.read_registers(180,6)
      
      #Calculate measured value form register values
      # fx=round(forceConverter(registers[0])-fxZero,0)
      # fy=round(forceConverter(registers[1])-fyZero,0)
      # fz=round(forceConverter(registers[2])-fzZero,0)
      # tx=round(torqueConverter(registers[3])-txZero,2)
      # ty=round(torqueConverter(registers[4])-tyZero,2)
      # tz=round(torqueConverter(registers[5])-tzZero,2)
      fx=forceConverter(registers[0])-fxZero
      fy=forceConverter(registers[1])-fyZero
      fz=forceConverter(registers[2])-fzZero
      tx=torqueConverter(registers[3])-txZero
      ty=torqueConverter(registers[4])-tyZero
      tz=torqueConverter(registers[5])-tzZero
      
      fx_f = (1 - alpha) * fx_f + alpha * fx
      fy_f = (1 - alpha) * fy_f + alpha * fy
      fz_f = (1 - alpha) * fz_f + alpha * fz
      tx_f = (1 - alpha) * tx_f + alpha * tx
      ty_f = (1 - alpha) * ty_f + alpha * ty
      tz_f = (1 - alpha) * tz_f + alpha * tz

      F_meas = np.array([fx_f, fy_f, fz_f])
      Tau_meas = np.array([tx_f, ty_f, tz_f])
      F_comp, Tau_comp = compensate(F_meas, Tau_meas, R, Fb, Tau_b, m, r)
      print(f"補償後力: {F_comp},  |F|={np.linalg.norm(F_comp):.3f} N")

      # fx_f /= 9.80665
      # fy_f /= 9.80665
      # fz_f /= 9.80665
      # print(f"fx={fx_f:.5f} N, fy={fy_f:.5f} N, fz={fz_f:.5f} N, tx={tx_f:.5f} N.m, ty={ty_f:.5f} N.m, tz={tz_f:.5f} N.m")
      # print(f"fx={fx_f:.3f} kg, fy={fy_f:.3f} kg, fz={fz_f:.3f} kg, tx={tx_f:.3f} N.m, ty={ty_f:.3f} N.m, tz={tz_f:.3f} N.m")
      # t = time.time() - t0
      # t_list.append(t)
      # fx_list.append(fx_f); fy_list.append(fy_f); fz_list.append(fz_f)
      # tx_list.append(tx_f); ty_list.append(ty_f); tz_list.append(tz_f)

      t = time.time() - t0
      t_list.append(t)
      fx_list.append(F_comp[0]); fy_list.append(F_comp[1]); fz_list.append(F_comp[2])
      tx_list.append(Tau_comp[0]); ty_list.append(Tau_comp[1]); tz_list.append(Tau_comp[2])

      # 更新六條曲線
      data_sets = [fx_list, fy_list, fz_list, tx_list, ty_list, tz_list]
      for i, line in enumerate(lines):
          line.set_data(range(len(data_sets[i])), list(data_sets[i]))
          axs.flat[i].relim(); axs.flat[i].autoscale_view()

      plt.pause(0.01)

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
    pass