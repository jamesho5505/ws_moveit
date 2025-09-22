#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from robotiq_gripper import RobotiqGripper # 引用你的 Modbus 通訊驅動程式
import time, math, csv
import numpy as np

# 定義一個 lambda 函數，用於將數值限制在一個指定的範圍 (lo, hi) 內
CLAMP = lambda x, lo, hi: lo if x < lo else (hi if x > hi else x)

class Mode:
    APPROACH = 0   # 位置控制：靠近目標直到接觸
    FORCE    = 1   # 力控制：維持設定力量
    HOLD     = 2   # 位置控制：維持（可做釋放）


class RobotiqGripperNode(Node):
    def __init__(self):
        super().__init__('robotiq_gripper_node')

        # 1. 宣告參數
        self.declare_parameter('portname', '/dev/ttyUSB0')
        self.declare_parameter('slaveaddress', 9)
        self.declare_parameter('baudrate', 115200)

        # --- PI 力控制器參數 ---
        self.declare_parameter('force_set_g', 300.0)                 # 目標夾持力 (克)
        self.declare_parameter('rise_time_s', 5)                   # 目標力道達到設定值的時間 (秒)
        self.declare_parameter('deadband_g', 20.0)                  # 力控制死區 (克)，在此範圍內不進行調整
        self.declare_parameter('k_p', 0.0001)                         # 比例增益 (P gain)
        self.declare_parameter('k_i', 0.0001) 
        self.declare_parameter('k_d', 0.00001) 
        self.declare_parameter("dx_mm_max", 0.3)
        self.declare_parameter("cmd_rate_hz", 120.0)
        

        # 夾爪速度/力度(0-255)，保持中等速度與力(避免振盪)
        self.declare_parameter('rq_speed', 5)
        self.declare_parameter('rq_force', 150)

        # --- FSR 力感測器訊號濾波參數 ---
        self.declare_parameter('filter_alpha', 0.9)        # 指數濾波 α
        self.declare_parameter('median_k', 3)  
        self.declare_parameter("force_idx", 2)  
        
        # --- 混合控制相關
        self.declare_parameter('approach_target_mm', 10.0)   # 接觸前收合到的目標寬度
        self.declare_parameter('approach_speed_mms', 40.0)   # 接觸前線速度 mm/s（內部轉成每周期步進）
        self.declare_parameter('contact_thresh_g', 100.0)     # 接觸門檻（力）
        self.declare_parameter('contact_dfdt_gps', 1000.0)     # 接觸門檻（dF/dt 尖峰）
        self.declare_parameter('contact_backoff_mm', 1.32)    # 接觸後微退後
        self.declare_parameter('hold', True)        # 達成目標力後是否保持夾持狀態

        # 2. 建立底層通訊物件並初始化夾爪
        port = self.get_parameter('portname').value
        self.gripper = RobotiqGripper(port) # 建立夾爪驅動物件        
        self.gripper.calibrate(10, 85) # 校準夾爪行程 (0mm - 85mm)
        self.gripper.goTomm(30, 255, 100)
        self.close_mm, self.open_mm = 10.0, 85.0  # 夾爪最小/最大開度 (mm)
        self.rq_speed = int(self.get_parameter('rq_speed').value)
        self.rq_force = int(self.get_parameter('rq_force').value) 

        # --- 狀態變數初始化 ---
        self.F_raw = 0.0          # 最新的原始力道讀數
        self.F_filt = 0.0         # 最新的濾波後力道讀數
        self.mode = Mode.APPROACH  # 初始模式為接近
        self.have_force = False   # 是否已收到力道讀數
        self._median_buf = []     # 中值濾波緩衝區
        self.int_e = 0.0          # 積分誤差
        self.cmd_mm = self.gripper.getPositionmm() # 目前夾爪位置 (mm)，初始為全開
        self.prev_time = time.time()
        self.contact_mm = 0.0  # 接觸時的夾爪位置 (mm)
        self.prev_F = 0.0
        self.dfdt = 0.0
        # self.dx_max   = float(self.get_parameter('dx_mm_max').value)
        self.dx_max = abs(self.gripper._aCoef)

        self.last_sent_bit = None
        self.min_send_dt   = 0.035   # 35 ms，對應你實測單次 Modbus ≈32 ms
        self.next_send_time = 0.0

        
        # --- 數據記錄 ---
        self.output_filename = "gripper_modbus_log.csv"
        self.history = {
            'time': [], 'current_mm': [], 'F_raw': [], 'F_filt': [], 'dfdt': [], 'mode': []
        }
        self.start_time = time.time()
        # 註冊 Ctrl+C 處理函式
 
        # 3. 建立 ROS2 publisher 和 subscriber
        # self.cmd_sub = self.create_subscription(
        #     Float64MultiArray,
        #     '/robotiq_gripper_controller/commands',
        #     self.gripper_cmd_cb,
        #     10
        # )
        # 訂閱來自 Arduino 的力感測器數據
        self.force_sub= self.create_subscription(
            Float32MultiArray,
            '/arduino/force',
            self.cb_force, 10
        )
        
        # 這裡可以再建立一個 publisher 來發布夾爪狀態
        self.state_pub = self.create_publisher(
            Float64MultiArray,
            '/robotiq_gripper_controller/state',
            10
        )

        # 4. 啟動定時器來定期讀取狀態
        dt = 1.0 / float(self.get_parameter('cmd_rate_hz').value)
        self.timer = self.create_timer(dt, self.timer_cb)

        self.get_logger().info('Robotiq Gripper Node is ready.')

    # def gripper_cmd_cb(self, msg: Float64MultiArray):
    #     if not msg.data:
    #         return
    #     # 假設 msg.data[0] 是目標位置（0-255）
    #     target_pos = msg.data[0]
    #     self.gripper.goTo(int(target_pos))

    def cb_force(self, msg: Float32MultiArray):
        """訂閱 /arduino/force 的回呼函式，用來更新力道讀數"""
        if not msg.data:
            return
        i = int(self.get_parameter('force_idx').value)
        # 安全地從陣列中讀取力道值
        if -len(msg.data) <= i < len(msg.data):
            self.F_raw = max(0.0, float(msg.data[i]) - 62.5) # 讀取原始力道值，並確保不為負
            self.have_force = True

    def _update_force_filter(self):
        """在 timer_cb 中統一處理力道濾波"""
        k = int(self.get_parameter('median_k').value)
        # --- 中值濾波 ---
        self._median_buf.append(self.F_raw)
        if len(self._median_buf) > k:
            self._median_buf.pop(0)
        f_med = sorted(self._median_buf)[len(self._median_buf)//2]
        # --- 指數移動平均濾波 (EMA) ---
        alpha = float(self.get_parameter('filter_alpha').value)
        # 如果是第一次收到數據，直接使用中值濾波結果；否則進行 EMA 濾波
        self.F_filt = alpha * f_med + (1 - alpha) * self.F_filt

    def detect_contact(self, dt: float) -> bool:
        if not self.have_force:
            return False
        f_th = float(self.get_parameter('contact_thresh_g').value)
        s_th = float(self.get_parameter('contact_dfdt_gps').value) # dF/dt threshold

        return (self.F_filt >= f_th) or (abs(self.dfdt) >= s_th)

    def pi_force_step(self, dt: float) -> float:
        target = float(self.get_parameter('force_set_g').value)
        deadband = float(self.get_parameter('deadband_g').value)
        e = target - self.F_filt
        e_eff = 0.0 if abs(e) < deadband else (e - math.copysign(deadband, e))
        k_p = float(self.get_parameter('k_p').value)
        k_i = float(self.get_parameter('k_i').value)
        k_d = float(self.get_parameter('k_d').value)
        p = k_p * e_eff
        i_next = self.int_e + e_eff * dt * k_i
        d = k_d * (self.dfdt)  # 注意這裡是 dF/dt，不是 dx/dt
        dx = -CLAMP(p + i_next + d, -self.dx_max, self.dx_max)
        # anti-windup
        if abs(p + i_next) > self.dx_max and math.copysign(1.0, p + i_next) == math.copysign(1.0, e_eff):
            pass
        else:
            self.int_e = i_next
        return dx

    def timer_cb(self):
        """定時器回呼函式，用來讀取夾爪狀態並進行 PI 力控制"""
        # pos, current = self.gripper.getPositionCurrent()
        # publish(pos, current)
        t0 = time.perf_counter_ns()
        now = time.time()
        dt = now - self.prev_time
        # dt = 1.0 / float(self.get_parameter('cmd_rate_hz').value) 
        if dt <= 0.0:
            return
        self.prev_time = now

        t_f0 = time.perf_counter_ns()
        # 1. 更新濾波後的力量值
        if self.have_force:
            self._update_force_filter()

        # 2. 計算 dF/dt (力量變化率)，確保其即時性
        self.dfdt = (self.F_filt - self.prev_F) / dt if dt > 1e-6 else 0.0
        self.prev_F = self.F_filt
        t_f1 = time.perf_counter_ns()

        # --- 數據記錄 (移至開頭) ---
        # 確保所有列表長度一致，使用上一個時間點的值
        self.history['time'].append(now - self.start_time)
        # self.history['current_mm'].append(self.gripper.getPositionmm())
        self.history['current_mm'].append(self.cmd_mm)
        self.history['F_raw'].append(self.F_raw)
        self.history['F_filt'].append(self.F_filt)
        self.history['dfdt'].append(self.dfdt)
        self.history['mode'].append(self.mode)
        
        t_c0 = time.perf_counter_ns()
        # 狀態機
        if self.mode == Mode.APPROACH:
            # 先位置收合到目標或直到接觸
            target_mm = float(self.get_parameter('approach_target_mm').value)
            v = float(self.get_parameter('approach_speed_mms').value)
            step = CLAMP((target_mm - self.cmd_mm), -self.dx_max, self.dx_max)
            # 用速度限制每周期位移
            vmax_step = math.copysign(min(abs(v * dt), self.dx_max), step)
            self.cmd_mm = CLAMP(self.cmd_mm + vmax_step, self.close_mm, self.open_mm)
            print("APPROACH", "Position:", self.cmd_mm, "Force:", self.F_filt, "dFdt:", self.dfdt)
            # 接觸偵測
            if self.detect_contact(dt):
                # 微退後，重置 PI 積分，進入力控
                self.contact_mm = self.gripper.getPositionmm() 
                backoff = float(self.get_parameter('contact_backoff_mm').value)
                self.cmd_mm = CLAMP(self.cmd_mm + backoff, self.close_mm, self.open_mm)
                self.int_e = 0.0
                self.mode = Mode.FORCE

        elif self.mode == Mode.FORCE:
            # 力控制
            dx = self.pi_force_step(dt) if self.have_force else 0.0
            self.cmd_mm = CLAMP(self.cmd_mm + dx, self.close_mm, self.open_mm)
            print("PI", "Position:", self.cmd_mm, "Force:", self.F_filt, "dFdt:", self.dfdt)
            # 可選：達標後持續夾持
            if bool(self.get_parameter('hold').value):
                target = float(self.get_parameter('force_set_g').value)
                if abs(self.F_filt - target) < float(self.get_parameter('deadband_g').value):
                    self.mode = Mode.HOLD

        elif self.mode == Mode.HOLD:
            print("HOLD", self.cmd_mm)
            pass # 在 HOLD 模式下，保持當前位置，不更新 cmd_mm
        
        t_c1 = time.perf_counter_ns()
        # 下發命令
        t_m0 = time.perf_counter_ns()
        self.gripper.goTomm(self.cmd_mm, self.rq_speed, self.rq_force)
        # target_bit = int(round(self.gripper._mmToBit(self.cmd_mm)))
        # now = time.time()
        # if (self.last_sent_bit is None) or (target_bit != self.last_sent_bit):
        #     if now >= self.next_send_time:
        #         # 直送 bit，避免多餘換算/等待
        #         self.gripper.goTo(target_bit, self.rq_speed, self.rq_force)
        #         self.last_sent_bit  = target_bit
        #         self.next_send_time = now + self.min_send_dt
        t_m1 = time.perf_counter_ns()
        # 狀態發布：cmd_mm, F_raw, F_filt, dfdt, mode
        msg = Float64MultiArray()
        msg.data = [self.cmd_mm, float(self.F_raw), float(self.F_filt), float(self.dfdt), float(self.mode)]
        t_p0 = time.perf_counter_ns()
        self.state_pub.publish(msg)
        t_p1 = time.perf_counter_ns()
        t1 = time.perf_counter_ns()
        filter_us = (t_f1 - t_f0) / 1000
        control_us = (t_c1 - t_c0) / 1000
        modbus_us = (t_m1 - t_m0) / 1000
        pub_us = (t_p1 - t_p0) / 1000
        total_us = (t1 - t0) / 1000
        if int(time.time()*2) % 40 == 0:
            self.get_logger().info(f"filter_us:{filter_us:.2f} control_us:{control_us:.2f} modbus_us:{modbus_us:.2f} pub_us:{pub_us:.2f} total_us:{total_us:.2f}")

    def save_to_csv(self):
        """將 history 字典中的數據儲存到 CSV 檔案"""
        self.get_logger().info(f"正在將數據儲存至 {self.output_filename}...")
        try:
            num_points = len(self.history['time'])
            if num_points == 0:
                print("警告: 沒有數據可以儲存。")
                return

            with open(self.output_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)

                # 1. 寫入所有 ROS 參數
                writer.writerow(['# Parameters'])
                # 取得所有已宣告的參數
                param_list = self.get_parameters_by_prefix('')
                for name, param in param_list.items():
                    writer.writerow([f'# {name}', param.value])
                
                # 2. 寫入一個空行作為分隔
                writer.writerow([])

                # 3. 寫入時間序列數據的標頭和內容
                header = list(self.history.keys())
                writer.writerow(header)
                for i in range(num_points):
                    row = [self.history[key][i] for key in header]
                    writer.writerow(row)
            print(f"成功儲存 {num_points} 筆數據至 {self.output_filename}。")
        except Exception as e:
            print(f"錯誤: 儲存 CSV 失敗: {e}")


def main():
    rclpy.init()
    node = RobotiqGripperNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt):
        pass
    finally:
        node.save_to_csv()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()