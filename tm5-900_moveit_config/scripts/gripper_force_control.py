#!/usr/bin/env python3
import math, time, rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from collections import deque
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

# 根據你的 URDF 或 ros2_control 設定，定義夾爪的關節名稱
GRIPPER_JOINT_NAME = 'robotiq_85_left_knuckle_joint'

class GripperApproachBackoffPI(Node):
    """
    一個實現三階段（接近-退讓-力控制）夾爪控制策略的 ROS2 節點。
    - APPROACH: 以恆定速度關閉夾爪，直到偵測到接觸。
    - BACKOFF: 接觸後稍微打開，為力控制做準備。
    - HOLD_PI: 使用 PI 控制器維持指定的夾持力。
    """
    def __init__(self):
        super().__init__('gripper_approach_backoff_pi')

        # ===== 參數 =====
        # --- 力訊號參數 ---
        self.declare_parameter('force_idx', 2)
        self.declare_parameter('force_alpha', 0.8)                   # 力道訊號的指數移動平均濾波器 alpha 值 (0-1)
        self.declare_parameter('contact_threshold_g', 250.0)         # 判斷接觸的力道閾值 (克)
        self.declare_parameter('dFdt_threshold_gps', 10000.0)            # 判斷接觸的力道變化率閾值 (克/秒)，設為 0 表示不使用

        # --- 幾何與行程參數 ---
        self.declare_parameter('width_min_mm', 0.0)
        self.declare_parameter('width_max_mm', 85.0)
        self.declare_parameter('joint_open_rad', 0.0)   # 夾爪完全打開時的關節角度 (rad)，對應 wmax
        self.declare_parameter('joint_close_rad', math.pi/4)  # 夾爪完全關閉時的關節角度 (rad)，對應 wmin
        self.declare_parameter('invert_width_mapping', False)

        # --- 接近與退讓參數 ---
        self.declare_parameter('start_width_mm', 60.0)
        self.declare_parameter('approach_rate_mm_s', 20.0)           # 接近物體時的關閉速度 (mm/s)
        self.declare_parameter('backoff_in', 'mm')                   # 退讓距離的單位 ('mm' 或 'rad')
        self.declare_parameter('safe_backoff_val', 0.50)             # 接觸後退讓的距離值

        # --- 目標夾持 (PI 控制器) 參數 ---
        self.declare_parameter('force_set_g', 200.0)                 # 目標夾持力 (克)
        self.declare_parameter('rise_time_s', 5)                   # 目標力道達到設定值的時間 (秒)
        self.declare_parameter('deadband_g', 50.0)                  # 力控制死區 (克)，在此範圍內不進行調整
        self.declare_parameter('k_p', 0.005)                         # 比例增益 (P gain)
        self.declare_parameter('k_i', 0.0005)                        # 積分增益 (I gain)
        self.declare_parameter('pi_rate_mm_s', 2.0)                  # PI 控制器輸出的最大速度限制 (mm/s)
        self.declare_parameter('i_clamp_mm', 6.0)                    # 積分項最大值 (mm)，用於 anti-windup

        # ===== 狀態 =====
        self.state = 'APPROACH'         # FSM 初始狀態
        self.width_cmd = None           # 目標夾爪寬度 (mm)，是整個控制器的核心輸出
        self.q_init = None              # 初始關節角度
        self._last_q = None             # 上一次送出的角度(rad)
        self._pending_goal = None       # 用於追蹤未完成的 action 目標
        self.w_init = None              # 初始夾爪寬度
        self.contact_width = None       # 接觸瞬間的寬度
        self.contact_joint = None       # 接觸瞬間的關節角度
        self.contact_force = None       # 接觸瞬間的力道
        self.contact_time = None        # 接觸瞬間的時間戳

        self.F_raw = float('nan')       # 原始力道讀數
        self.have_force = False         # 是否已收到力道訊號
        self.F_filt = 0.0               # 濾波後的力道
        self.dFdt = 0.0                 # 力道變化率
        self.I = 0.0                    # PI 控制器的積分項

        self.t_prev = time.monotonic()  # 上一次 step 執行的時間
        self.last_sent = 0.0            # 上一次發送指令的時間
        # self._last_w = None             # 上一次發送的寬度指令
        self._dbg_t = 0.0               # 上一次列印除錯訊息的時間
        self.F_med_buf = deque(maxlen=5)   # 用於中位數濾波的緩衝區
        self.Fset_curr = 0.0  # 用於斜坡控制的當前力道目標，從 0 開始
        self.pi_active = False          # PI 控制器是否啟動的遲滯旗標

        # ===== ROS 通訊 =====
        self.sub_js   = self.create_subscription(JointState, '/joint_states', self.cb_joint, 10)
        self.sub_force= self.create_subscription(Float32MultiArray, '/arduino/force', self.cb_force, 10)
        # ros2_control 的 ForwardCommandController 通常接收 Float64MultiArray 格式的指令
        self.ac = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')
        self.ac.wait_for_server()  # 等 action server
        self.pub_contact = self.create_publisher(Float64MultiArray, '/gripper/contact_info', 10)
        self.pub_ftarget = self.create_publisher(Float32, '/gripper/target_force', 10)
        self.create_timer(0.01, self.step)  # 建立 100 Hz 的計時器來驅動主控制迴圈

        self.get_logger().info('FSM: APPROACH -> BACKOFF -> HOLD_PI')

    def send_angle_action(self, q_rad, max_effort_n=40.0):
        if self._pending_goal and not self._pending_goal.done():
            return  # 上一個還在處理就略過
        goal = GripperCommand.Goal()
        goal.command.position = float(q_rad)
        goal.command.max_effort = float(max_effort_n)
        self._pending_goal = self.ac.send_goal_async(goal, feedback_callback=self._on_fb)
    
    def _on_fb(self, fb):
        # 可用來監看到達與卡滯（非硬即時）
        pass

    # ==== 幾何：線性對應（穩健、方向明確） ====
    def width_to_joint(self, w_mm: float) -> float:
        wmin = float(self.get_parameter('width_min_mm').value)
        wmax = float(self.get_parameter('width_max_mm').value)
        q_open  = float(self.get_parameter('joint_open_rad').value)
        q_close = float(self.get_parameter('joint_close_rad').value)
        inv = bool(self.get_parameter('invert_width_mapping').value)
        w = max(wmin, min(wmax, w_mm))
        a = 0.0 if wmax == wmin else (w - wmin) / (wmax - wmin)
        # 正常：w↑→q 朝 q_open；反轉：w↑→q 朝 q_close
        if not inv:
            return q_close*(1.0 - a) + q_open*a
        else:
            return q_open*(1.0 - a) + q_close*a

    def joint_to_width(self, q_rad: float) -> float:
        wmin = float(self.get_parameter('width_min_mm').value)
        wmax = float(self.get_parameter('width_max_mm').value)
        q_open  = float(self.get_parameter('joint_open_rad').value)
        q_close = float(self.get_parameter('joint_close_rad').value)
        inv = bool(self.get_parameter('invert_width_mapping').value)
        if abs(q_close - q_open) < 1e-9:
            return wmin
        a = (q_rad - q_open) / (q_close - q_open)
        a = max(0.0, min(1.0, a))
        # 與上面一致地反轉
        if not inv:
            return max(wmin, min(wmax, wmin + a*(wmax - wmin)))
        else:
            return max(wmin, min(wmax, wmax - a*(wmax - wmin)))


    # ==== 回呼 ====
    def cb_joint(self, msg: JointState):
        """訂閱 /joint_states 的回呼函式，用來更新夾爪目前狀態"""
        try:
            idx = msg.name.index(GRIPPER_JOINT_NAME)
        except ValueError:
            return
        q = float(msg.position[idx])
        w = self.joint_to_width(q)
        # 首次收到關節狀態時，初始化相關變數
        if self.q_init is None:
            self.q_init = q
            self.w_init = w
            self.get_logger().info(f'Init joint={q:.3f} rad, width={w:.2f} mm')
        # 如果指令尚未初始化，則將其設為目前寬度，以避免啟動時夾爪突然移動
        if self.width_cmd is None:
            self.width_cmd = w

    def cb_force(self, msg: Float32MultiArray):
        """訂閱 /arduino/force 的回呼函式，用來更新力道讀數"""
        if not msg.data:
            return
        i = int(self.get_parameter('force_idx').value)
        # 安全地從陣列中讀取力道值
        if -len(msg.data) <= i < len(msg.data):
            self.F_raw = float(msg.data[i])
            self.have_force = True

    # ==== 主迴圈 ====
    def step(self):
        """主控制迴圈，由計時器以 100 Hz 頻率呼叫"""
        now = time.monotonic()
        dt = max(1e-3, min(0.05, now - self.t_prev)) # 計算時間間隔 dt，並限制其範圍以增加穩定性
        self.t_prev = now
        if self.width_cmd is None:
            return

        # --- 力道訊號處理：濾波與微分 ---
        a = float(self.get_parameter('force_alpha').value)
        x = 0.0 if math.isnan(self.F_raw) else self.F_raw
        prev = self.F_filt
        self.F_filt = a*x + (1.0 - a)*self.F_filt  # 一階低通濾波 (IIR)
        self.F_med_buf.append(self.F_filt)         # 將濾波後的值存入緩衝區
        self.F_filt = sorted(self.F_med_buf)[len(self.F_med_buf)//2]   # 中位數濾波，可有效去除突波雜訊
        self.dFdt = (self.F_filt - prev) / dt      # 計算力道變化率

        wmin = float(self.get_parameter('width_min_mm').value)
        wmax = float(self.get_parameter('width_max_mm').value)


        # --- 狀態機 ---
        if self.state == 'APPROACH':
            rate = float(self.get_parameter('approach_rate_mm_s').value)
            self.width_cmd = max(wmin, self.width_cmd - rate*dt)  # 以恆定速率減小寬度 (關閉)
            # 接觸觸發條件：力道或力道變化率超過閾值
            trig = self.have_force and (
                self.F_filt >= float(self.get_parameter('contact_threshold_g').value) or
                abs(self.dFdt) >= float(self.get_parameter('dFdt_threshold_gps').value)
            )
            if trig:
                # 記錄接觸資訊
                self.contact_width = self.width_cmd
                self.contact_joint = self.width_to_joint(self.width_cmd)
                self.contact_force = self.F_filt
                self.contact_time  = now
                self.publish_contact()

                # 計算退讓目標位置
                mode = str(self.get_parameter('backoff_in').value).lower()  # 'mm' 或 'rad'
                val  = float(self.get_parameter('safe_backoff_val').value)
                if mode == 'rad':
                    # 假設關節角度減小是打開
                    q_target = self.contact_joint - val
                    w_target = self.joint_to_width(q_target)
                else:  # 'mm'
                    w_target = self.contact_width + val # 增加寬度以打開
                self.backoff_target = min(wmax, max(wmin, w_target)) # 限制在物理範圍內
                self.I = 0.0  # 重置積分項
                self.state = 'BACKOFF'

        elif self.state == 'BACKOFF':
            rate = float(self.get_parameter('approach_rate_mm_s').value)
            step = rate*dt
            # 檢查是否已到達或超過退讓目標
            if self.width_cmd + step >= self.backoff_target:
                self.width_cmd = self.backoff_target
                self.state = 'HOLD_PI'
                # 從當前的實際力道開始斜坡上升，而不是從0，這樣更平滑
                self.Fset_curr = self.F_filt 
            else:
                self.width_cmd += step # 以恆定速率增加寬度 (打開)

        elif self.state == 'HOLD_PI':
            Ftarget = float(self.get_parameter('force_set_g').value)
            Tr = float(self.get_parameter('rise_time_s').value)
            tau = max(1e-3, Tr / 2.2)
            alpha = 1.0 - math.exp(-dt / tau)   # 離散化係數
            alpha = max(0.0, min(1.0, alpha))  
            self.Fset_curr += alpha * (Ftarget - self.Fset_curr)
            Fset = self.Fset_curr

            # (A) 目標力道斜坡 (Setpoint Ramping): 讓目標力道平滑變化，避免系統突變
            # ramp = 100.0  # g/s
            # if self.Fset_curr < Ftarget:
            #     self.Fset_curr = min(Ftarget, self.Fset_curr + ramp*dt)
            # else:
            #     self.Fset_curr = max(Ftarget, self.Fset_curr - ramp*dt)
            # Fset = self.Fset_curr

            dead = float(self.get_parameter('deadband_g').value)
            # (B) 遲滯 (Hysteresis): 設定不同的啟動/關閉閾值，防止在死區邊緣震盪
            DB_on  = dead * 1.2
            DB_off = dead * 0.8

            e = Fset - self.F_filt
            # 根據遲滯邏輯更新 PI 控制器的激活狀態
            if not self.pi_active and abs(e) > DB_on:
                self.pi_active = True
            elif self.pi_active and abs(e) < DB_off:
                self.pi_active = False

            # 如果控制器未激活，有效誤差為0；否則，從誤差中減去死區
            e_eff = 0.0 if not self.pi_active else (e - math.copysign(dead, e))

            kp = float(self.get_parameter('k_p').value)
            ki = float(self.get_parameter('k_i').value)
            vmax = float(self.get_parameter('pi_rate_mm_s').value)
            i_clamp = float(self.get_parameter('i_clamp_mm').value)

            # PI 控制律：P項 + I項
            # 注意：P項的符號為負，因為力道過大(e<0)時，需要增加寬度(w_des增加)，反之亦然。
            v_p = -kp * e_eff
            w_des = self.width_cmd + v_p + self.I

            # (C) 速率限幅: 限制每次迭代寬度的最大變化量
            w_step = max(-vmax*dt, min(vmax*dt, w_des - self.width_cmd))
            w_des = self.width_cmd + w_step

            # (D) 邊界檢查與反飽和 (Anti-windup)
            at_min = (w_des <= wmin + 1e-3)
            at_max = (w_des >= wmax - 1e-3)

            # 特殊情況：如果夾爪已夾到最緊，但力道仍然過大，則強制打開以釋放壓力，並重置積分項
            if at_min and (self.F_filt > (Fset + dead*0.5)):
                w_des = min(wmax, self.width_cmd + max(1.0, vmax)*dt)
                self.I = 0.0  # 重置積分項，這是 anti-windup 的關鍵部分

            # 只有在未達到邊界且控制器激活時，才累積積分項
            if not (at_min or at_max) and self.pi_active:
                self.I += -ki * e_eff * dt
                self.I = max(-i_clamp, min(i_clamp, self.I)) # 限制積分項在預設範圍內

            # 更新最終的寬度指令，並確保其在物理範圍內
            self.width_cmd = max(wmin, min(wmax, w_des))

        # --- 發佈指令 ---
        # 為了減少通訊負載，只有在指令變化足夠大或距離上次發送超過一定時間時才發送
        if self.q_init is not None:
            q_cmd = self.width_to_joint(self.width_cmd)
            if (self._last_q is None) or (abs(q_cmd - self._last_q) >= 0.01) or (now - self.last_sent >= 0.05):
                self.send_angle_action(q_cmd, max_effort_n=40.0)
                self._last_q = q_cmd
                self.last_sent = now
        
        # 發佈當前的目標力道，無論在哪個狀態
        ft_msg = Float32(); ft_msg.data = self.Fset_curr
        self.pub_ftarget.publish(ft_msg)

        # --- 除錯資訊輸出 ---
        if now - self._dbg_t > 0.2:
            self._dbg_t = now
            self.get_logger().info(f"{self.state} | w={self.width_cmd:.2f}mm, F={self.F_filt:.1f}g, dFdt={self.dFdt:.1f}g/s")

    def publish_contact(self):
        """發布接觸資訊"""
        arr = Float64MultiArray()
        # [contact_width_mm, contact_force_g, contact_time_s]
        arr.data = [self.contact_width or 0.0, self.contact_force or 0.0, self.contact_time or 0.0]
        self.pub_contact.publish(arr)

def main():
    rclpy.init()
    node = GripperApproachBackoffPI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("程式被使用者中斷")
        pass
    finally:
        node.destroy_node()
        # 確保 rclpy 乾淨地關閉
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
