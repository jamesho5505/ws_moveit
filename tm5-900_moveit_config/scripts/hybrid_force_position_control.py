#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import TwistStamped

class HybridImpedance(Node):
    def __init__(self):
        super().__init__('hybrid_impedance')

        # 參數：力(克)→寬度(mm) PI；導納增益
        self.declare_parameter('force_set_g', 500.0)
        self.declare_parameter('k_p_w', 0.0008)     # mm/g
        self.declare_parameter('k_i_w', 0.0002)     # mm/(g*s)
        self.declare_parameter('deadband_g', 10.0)
        self.declare_parameter('width_min_mm', 0.0)
        self.declare_parameter('width_max_mm', 85.0)
        self.declare_parameter('width_rate_mm_s', 20.0)
        # 導納（y向側移與yaw），單位調到合理速度
        self.declare_parameter('M_y', 50.0)         # 大→更鈍
        self.declare_parameter('B_y', 30.0)
        self.declare_parameter('K_y', 0.0)
        self.declare_parameter('alpha_y', 0.0005)   # m/s per g
        self.declare_parameter('M_yaw', 2.0)
        self.declare_parameter('B_yaw', 1.0)
        self.declare_parameter('K_yaw', 0.0)
        self.declare_parameter('alpha_yaw', 0.00002)# rad/s per g
        self.declare_parameter('vel_lim_y', 0.01)   # m/s
        self.declare_parameter('vel_lim_yaw', 0.2)  # rad/s

        # 介面
        self.sub = self.create_subscription(Float32MultiArray, '/arduino/force', self.cb_force, 10)
        self.pub_w = self.create_publisher(Float32, '/gripper/target_width', 10)
        self.pub_twist = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        # 狀態
        self.F_L = self.F_R = math.nan
        self.width_cmd = 60.0
        self.I_w = 0.0
        self.y = 0.0; self.vy = 0.0
        self.yaw = 0.0; self.wz = 0.0
        self.t_prev = time.monotonic()

        self.timer = self.create_timer(0.01, self.step)  # 100 Hz

    def cb_force(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.F_L, self.F_R = float(msg.data[0]), float(msg.data[1])

    def step(self):
        t = time.monotonic()
        dt = max(1e-3, min(0.05, t - self.t_prev))
        self.t_prev = t
        if math.isnan(self.F_L) or math.isnan(self.F_R):
            return

        # ===== 內圈：平均力 PI 調寬度 =====
        F_set = float(self.get_parameter('force_set_g').value)
        dead = float(self.get_parameter('deadband_g').value)
        F_avg = 0.5*(self.F_L + self.F_R)
        e = F_set - F_avg
        if abs(e) <= dead:
            e_eff = 0.0
        else:
            e_eff = e - math.copysign(dead, e)

        kp = float(self.get_parameter('k_p_w').value)
        ki = float(self.get_parameter('k_i_w').value)
        p = -kp * e_eff
        self.I_w += -ki * e_eff * dt
        width_des = self.width_cmd + p + self.I_w

        # 速限與邊界
        rate = float(self.get_parameter('width_rate_mm_s').value)
        max_step = rate * dt
        step = max(-max_step, min(max_step, width_des - self.width_cmd))
        width_des = self.width_cmd + step

        wmin = float(self.get_parameter('width_min_mm').value)
        wmax = float(self.get_parameter('width_max_mm').value)
        width_des = max(wmin, min(wmax, width_des))
        self.width_cmd = width_des

        m = Float32(); m.data = float(self.width_cmd); self.pub_w.publish(m)

        # ===== 外圈：導納（左右平衡）=====
        F_delta = self.F_L - self.F_R

        # y 向
        My = float(self.get_parameter('M_y').value)
        By = float(self.get_parameter('B_y').value)
        Ky = float(self.get_parameter('K_y').value)
        ay = float(self.get_parameter('alpha_y').value)
        self.vy += (ay*F_delta - By*self.vy - Ky*self.y) * (dt / My)
        self.y  += self.vy * dt

        # yaw
        Mm = float(self.get_parameter('M_yaw').value)
        Bm = float(self.get_parameter('B_yaw').value)
        Km = float(self.get_parameter('K_yaw').value)
        am = float(self.get_parameter('alpha_yaw').value)
        self.wz += (am*F_delta - Bm*self.wz - Km*self.yaw) * (dt / Mm)
        self.yaw += self.wz * dt

        # 速限
        vy_lim = float(self.get_parameter('vel_lim_y').value)
        wz_lim = float(self.get_parameter('vel_lim_yaw').value)
        self.vy = max(-vy_lim, min(vy_lim, self.vy))
        self.wz = max(-wz_lim, min(wz_lim, self.wz))

        # 發送 twist（MoveIt Servo）
        tw = TwistStamped()
        tw.header.stamp = self.get_clock().now().to_msg()
        tw.header.frame_id = 'tool0'  # 依你的 Servo 設定調整
        tw.twist.linear.y  = float(self.vy)
        tw.twist.angular.z = float(self.wz)
        self.pub_twist.publish(tw)

def main():
    rclpy.init(); n = HybridImpedance()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
