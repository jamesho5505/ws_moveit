#!/usr/bin/env python3
import math, time, rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from collections import deque

GRIPPER_JOINT_NAME = 'robotiq_85_left_knuckle_joint'

class GripperApproachBackoffPI(Node):
    def __init__(self):
        super().__init__('gripper_approach_backoff_pi')

        # ===== 參數 =====
        # 力訊號
        self.declare_parameter('force_idx', 2)           
        self.declare_parameter('force_alpha', 0.8)       
        self.declare_parameter('contact_threshold_g', 250.0)
        self.declare_parameter('dFdt_threshold_gps', 0.0)  

        # 幾何與行程
        self.declare_parameter('width_min_mm', 0.0)
        self.declare_parameter('width_max_mm', 85.0)
        self.declare_parameter('joint_open_rad', 0.0)    
        self.declare_parameter('joint_close_rad', 0.8)   

        # 接近與退讓
        self.declare_parameter('start_width_mm', 60.0)
        self.declare_parameter('approach_rate_mm_s', 20.0)
        self.declare_parameter('backoff_in', 'mm')       
        self.declare_parameter('safe_backoff_val', 15.0)  

        # 目標夾持（PI）- 修正參數
        self.declare_parameter('force_set_g', 600.0)
        self.declare_parameter('deadband_g', 50.0)        # 減小死區
        self.declare_parameter('k_p', 0.005)              # 增加比例增益
        self.declare_parameter('k_i', 0.001)              # 增加積分增益
        self.declare_parameter('pi_rate_mm_s', 5.0)       # 增加PI反應速度
        self.declare_parameter('i_clamp_mm', 5.0)         # 減小積分限幅

        # ===== 狀態 =====
        self.state = 'APPROACH'
        self.width_cmd = None          
        self.q_init = None             
        self.w_init = None             
        self.contact_width = None
        self.contact_joint = None
        self.contact_force = None
        self.contact_time = None

        self.F_raw = float('nan')
        self.have_force = False
        self.F_filt = 0.0
        self.dFdt = 0.0
        self.I = 0.0

        self.t_prev = time.monotonic()
        self.last_sent = 0.0
        self._last_w = None
        self._dbg_t = 0.0
        self.F_med_buf = deque(maxlen=5)   
        self.Fset_curr = float(self.get_parameter('force_set_g').value)  
        self.pi_active = False  

        # ===== ROS 通訊 =====
        self.sub_js   = self.create_subscription(JointState, '/joint_states', self.cb_joint, 10)
        self.sub_force= self.create_subscription(Float32MultiArray, '/arduino/force', self.cb_force, 10)
        self.pub_arr  = self.create_publisher(Float64MultiArray, '/robotiq_gripper_controller/commands', 10)
        self.pub_contact = self.create_publisher(Float64MultiArray, '/gripper/contact_info', 10)
        self.create_timer(0.01, self.step)  # 100 Hz

        self.get_logger().info('FSM: APPROACH -> BACKOFF -> HOLD_PI (修正版)')

    # ==== 幾何：線性對應 ====
    def width_to_joint(self, w_mm: float) -> float:
        wmin = float(self.get_parameter('width_min_mm').value)
        wmax = float(self.get_parameter('width_max_mm').value)
        q_open  = float(self.get_parameter('joint_open_rad').value)
        q_close = float(self.get_parameter('joint_close_rad').value)
        a = 0.0 if wmax == wmin else (w_mm - wmin)/(wmax - wmin)  
        return q_close*(1.0 - a) + q_open*a

    def joint_to_width(self, q_rad: float) -> float:
        wmin = float(self.get_parameter('width_min_mm').value)
        wmax = float(self.get_parameter('width_max_mm').value)
        q_open  = float(self.get_parameter('joint_open_rad').value)
        q_close = float(self.get_parameter('joint_close_rad').value)
        if abs(q_close - q_open) < 1e-9:
            return wmin
        a = (q_rad - q_close) / (q_open - q_close)
        return max(wmin, min(wmax, wmin + a*(wmax - wmin)))

    # ==== 回呼 ====
    def cb_joint(self, msg: JointState):
        try:
            idx = msg.name.index(GRIPPER_JOINT_NAME)
        except ValueError:
            return
        q = float(msg.position[idx])
        w = self.joint_to_width(q)
        if self.q_init is None:
            self.q_init = q
            self.w_init = w
            self.get_logger().info(f'Init joint={q:.3f} rad, width={w:.2f} mm')
        if self.width_cmd is None:
            self.width_cmd = w

    def cb_force(self, msg: Float32MultiArray):
        if not msg.data:
            return
        i = int(self.get_parameter('force_idx').value)
        if -len(msg.data) <= i < len(msg.data):
            self.F_raw = float(msg.data[i])
            self.have_force = True

    # ==== 主迴圈 ====
    def step(self):
        now = time.monotonic()
        dt = max(1e-3, min(0.05, now - self.t_prev))
        self.t_prev = now
        if self.width_cmd is None:
            return

        # 力濾波與微分
        a = float(self.get_parameter('force_alpha').value)
        x = 0.0 if math.isnan(self.F_raw) else self.F_raw
        prev = self.F_filt
        self.F_filt = a*x + (1.0 - a)*self.F_filt
        self.F_med_buf.append(self.F_filt)
        self.F_filt = sorted(self.F_med_buf)[len(self.F_med_buf)//2]   
        self.dFdt = (self.F_filt - prev) / dt

        wmin = float(self.get_parameter('width_min_mm').value)
        wmax = float(self.get_parameter('width_max_mm').value)

        if self.state == 'APPROACH':
            rate = float(self.get_parameter('approach_rate_mm_s').value)
            self.width_cmd = max(wmin, self.width_cmd - rate*dt)  
            trig = self.have_force and (
                self.F_filt >= float(self.get_parameter('contact_threshold_g').value) or
                abs(self.dFdt) >= float(self.get_parameter('dFdt_threshold_gps').value)
            )
            if trig:
                self.contact_width = self.width_cmd
                self.contact_joint = self.width_to_joint(self.width_cmd)
                self.contact_force = self.F_filt
                self.contact_time  = now
                self.publish_contact()

                # 設定退讓目標
                mode = str(self.get_parameter('backoff_in').value).lower()
                val  = float(self.get_parameter('safe_backoff_val').value)
                if mode == 'rad':
                    q_target = self.contact_joint - val   
                    w_target = self.joint_to_width(q_target)
                else:  
                    w_target = self.contact_width + val
                self.backoff_target = min(wmax, max(wmin, w_target))
                self.I = 0.0  # 重置積分項
                self.state = 'BACKOFF'
                self.get_logger().info(f'觸碰檢測！寬度={self.contact_width:.2f}mm, 力道={self.contact_force:.1f}g, 退讓至{self.backoff_target:.2f}mm')

        elif self.state == 'BACKOFF':
            rate = float(self.get_parameter('approach_rate_mm_s').value)
            step = rate*dt
            if self.width_cmd + step >= self.backoff_target:
                self.width_cmd = self.backoff_target
                self.state = 'HOLD_PI'
                self.pi_active = False  # 重置PI狀態
                self.I = 0.0           # 重置積分項
                self.get_logger().info(f'開始PI力控制，目標力道={self.get_parameter("force_set_g").value:.0f}g')
            else:
                self.width_cmd += step

        elif self.state == 'HOLD_PI':
            # 修正的PI控制器
            force_target = float(self.get_parameter('force_set_g').value)
            deadband = float(self.get_parameter('deadband_g').value)
            kp = float(self.get_parameter('k_p').value)
            ki = float(self.get_parameter('k_i').value)
            max_rate = float(self.get_parameter('pi_rate_mm_s').value)
            i_clamp = float(self.get_parameter('i_clamp_mm').value)
            
            # 計算力誤差
            force_error = force_target - self.F_filt
            
            # 簡化的死區處理
            if abs(force_error) > deadband:
                self.pi_active = True
                error_compensated = force_error - math.copysign(deadband, force_error)
            else:
                error_compensated = 0.0
                # 保持PI激活狀態，避免震盪
            
            # PI控制計算
            if self.pi_active:
                # 比例項 
                p_term = kp * error_compensated
                
                # 積分項（有反飽和）
                self.I += ki * error_compensated * dt
                self.I = max(-i_clamp, min(i_clamp, self.I))
                
                # 計算位置變化
                position_change = p_term + self.I
                
                # 速率限制
                max_change = max_rate * dt
                position_change = max(-max_change, min(max_change, position_change))
                
                # 更新目標寬度
                new_width = self.width_cmd + position_change
                self.width_cmd = max(wmin, min(wmax, new_width))

        # 發佈指令
        if (self._last_w is None) or (abs(self.width_cmd - self._last_w) >= 0.05) or (now - self.last_sent >= 0.05):
            q_cmd = self.width_to_joint(self.width_cmd)
            m = Float64MultiArray(); m.data = [q_cmd]
            self.pub_arr.publish(m)
            self._last_w = self.width_cmd
            self.last_sent = now

        # 詳細除錯輸出
        if now - self._dbg_t > 0.5:  # 每0.5秒輸出
            self._dbg_t = now
            if self.state == 'HOLD_PI':
                force_error = float(self.get_parameter('force_set_g').value) - self.F_filt
                self.get_logger().info(
                    f"{self.state} | w={self.width_cmd:.2f}mm | "
                    f"F={self.F_filt:.1f}g (目標:{self.get_parameter('force_set_g').value:.0f}g) | "
                    f"誤差={force_error:.1f}g | PI激活={self.pi_active} | I={self.I:.3f}"
                )
            else:
                self.get_logger().info(f"{self.state} | w={self.width_cmd:.2f}mm, F={self.F_filt:.1f}g")

    def publish_contact(self):
        arr = Float64MultiArray()
        arr.data = [self.contact_width or 0.0, self.contact_force or 0.0, self.contact_time or 0.0]
        self.pub_contact.publish(arr)

def main():
    rclpy.init()
    node = GripperApproachBackoffPI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("程式終止")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()