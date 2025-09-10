#!/usr/bin/env python3
import math, time, rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from collections import deque

GRIPPER_JOINT_NAME = 'robotiq_85_left_knuckle_joint'

class GripperApproachBackoffPI(Node):
    def __init__(self):
        super().__init__('gripper_approach_backoff_pi')

        # ===== 參數 =====
        self.declare_parameter('force_idx', 2)
        self.declare_parameter('force_alpha', 0.8)
        self.declare_parameter('contact_threshold_g', 250.0)
        self.declare_parameter('dFdt_threshold_gps', 0.0)

        self.declare_parameter('width_min_mm', 0.0)
        self.declare_parameter('width_max_mm', 85.0)
        # 修正方向：打開=0.8 rad, 關閉=0.0 rad
        self.declare_parameter('joint_open_rad', 0.8)
        self.declare_parameter('joint_close_rad', 0.0)

        self.declare_parameter('start_width_mm', 60.0)
        self.declare_parameter('approach_rate_mm_s', 20.0)
        self.declare_parameter('backoff_in', 'mm')
        self.declare_parameter('safe_backoff_val', 5.0)

        self.declare_parameter('force_set_g', 800.0)
        self.declare_parameter('deadband_g', 200.0)
        self.declare_parameter('k_p', 0.0005)
        self.declare_parameter('k_i', 0.0005)
        self.declare_parameter('pi_rate_mm_s', 2.0)
        self.declare_parameter('i_clamp_mm', 6.0)

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

        self.sub_js   = self.create_subscription(JointState, '/joint_states', self.cb_joint, 10)
        self.sub_force= self.create_subscription(Float32MultiArray, '/arduino/force', self.cb_force, 10)
        self.pub_f64  = self.create_publisher(Float64, '/robotiq_gripper_controller/command', 10)
        self.pub_contact = self.create_publisher(Float64MultiArray, '/gripper/contact_info', 10)
        self.create_timer(0.01, self.step)

        self.get_logger().info('FSM: APPROACH -> BACKOFF -> HOLD_PI')

    def width_to_joint(self, w_mm: float) -> float:
        wmin = float(self.get_parameter('width_min_mm').value)
        wmax = float(self.get_parameter('width_max_mm').value)
        q_open  = float(self.get_parameter('joint_open_rad').value)
        q_close = float(self.get_parameter('joint_close_rad').value)
        a = 0.0 if wmax == wmin else (w_mm - wmin)/(wmax - wmin)
        return q_open*(1.0 - a) + q_close*a

    def joint_to_width(self, q_rad: float) -> float:
        wmin = float(self.get_parameter('width_min_mm').value)
        wmax = float(self.get_parameter('width_max_mm').value)
        q_open  = float(self.get_parameter('joint_open_rad').value)
        q_close = float(self.get_parameter('joint_close_rad').value)
        if abs(q_close - q_open) < 1e-9:
            return wmin
        a = (q_rad - q_close) / (q_close - q_open)
        return max(wmin, min(wmax, wmin + a*(wmax - wmin)))

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

    def step(self):
        now = time.monotonic()
        dt = max(1e-3, min(0.05, now - self.t_prev))
        self.t_prev = now
        if self.width_cmd is None:
            return

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

                mode = str(self.get_parameter('backoff_in').value).lower()
                val  = float(self.get_parameter('safe_backoff_val').value)
                if mode == 'rad':
                    q_target = self.contact_joint - val
                    w_target = self.joint_to_width(q_target)
                else:
                    w_target = self.contact_width + val
                self.backoff_target = min(wmax, max(wmin, w_target))
                self.I = 0.0
                self.state = 'BACKOFF'

        elif self.state == 'BACKOFF':
            rate = float(self.get_parameter('approach_rate_mm_s').value)
            step = rate*dt
            if self.width_cmd + step >= self.backoff_target:
                self.width_cmd = self.backoff_target
                self.state = 'HOLD_PI'
            else:
                self.width_cmd += step

        elif self.state == 'HOLD_PI':
            Ftarget = float(self.get_parameter('force_set_g').value)
            ramp = 100.0
            if self.Fset_curr < Ftarget:
                self.Fset_curr = min(Ftarget, self.Fset_curr + ramp*dt)
            else:
                self.Fset_curr = max(Ftarget, self.Fset_curr - ramp*dt)
            Fset = self.Fset_curr

            dead = float(self.get_parameter('deadband_g').value)
            DB_on  = dead * 1.2
            DB_off = dead * 0.8

            e = Fset - self.F_filt
            if not self.pi_active and abs(e) > DB_on:
                self.pi_active = True
            elif self.pi_active and abs(e) < DB_off:
                self.pi_active = False

            e_eff = 0.0 if not self.pi_active else (e - math.copysign(dead, e))

            kp = float(self.get_parameter('k_p').value)
            ki = float(self.get_parameter('k_i').value)
            vmax = float(self.get_parameter('pi_rate_mm_s').value)
            i_clamp = float(self.get_parameter('i_clamp_mm').value)

            v_p = -kp * e_eff
            w_des = self.width_cmd + v_p + self.I

            w_step = max(-vmax*dt, min(vmax*dt, w_des - self.width_cmd))
            w_des  = self.width_cmd + w_step

            at_min = (w_des <= wmin + 1e-3)
            at_max = (w_des >= wmax - 1e-3)

            if at_min and (self.F_filt > (Fset + dead*0.5)):
                w_des = min(wmax, self.width_cmd + max(1.0, vmax)*dt)
                self.I = 0.0

            if not (at_min or at_max) and self.pi_active:
                self.I += -ki * e_eff * dt
                self.I = max(-i_clamp, min(i_clamp, self.I))

            self.width_cmd = max(wmin, min(wmax, w_des))

        if (self._last_w is None) or (abs(self.width_cmd - self._last_w) >= 0.05) or (now - self.last_sent >= 0.05):
            q_cmd = self.width_to_joint(self.width_cmd)
            s = Float64(); s.data = q_cmd
            self.pub_f64.publish(s)
            self._last_w = self.width_cmd
            self.last_sent = now

        if now - self._dbg_t > 0.2:
            self._dbg_t = now
            self.get_logger().info(f"{self.state} | w={self.width_cmd:.2f}mm, F={self.F_filt:.1f}g, dFdt={self.dFdt:.1f}g/s")

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
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
