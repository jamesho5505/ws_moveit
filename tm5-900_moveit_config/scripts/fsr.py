#!/usr/bin/env python3
import re, math, argparse, serial, rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class FSRSerial(Node):
    def __init__(self, port, baud, topic):
        super().__init__('fsr_serial_node')
        self.pub = self.create_publisher(Float32MultiArray, topic, 10)
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.02)
        self.timer = self.create_timer(0.005, self.step)  # 200 Hz 嘗試讀

    def step(self):
        t0 = time.perf_counter_ns()
        line = self.ser.readline().decode(errors='ignore').strip()
        t1 = time.perf_counter_ns()
        if not line:
            return
        f1 = float('nan'); f2 = float('nan'); f_total = float('nan')
        parts = line.split(',')
        if len(parts) == 3:
            try:
                f1 = float(parts[0].strip())
                f2 = float(parts[1].strip())
                f_total = float(parts[2].strip())
            except Exception:
                return
        else:
            return  # 不是 CSV 兩欄就略過

        if any(math.isnan(x) for x in [f_total, f1, f2]):
            return
        msg = Float32MultiArray()
        msg.data = [f1, f2, f_total]
        t2 = time.perf_counter_ns()
        self.pub.publish(msg)
        t3 = time.perf_counter_ns()

        read_us = (t1 - t0) / 1000
        pub_us = (t3 - t2) / 1000
        total_us = (t3 - t0) / 1000
        if int(time.time()*2) % 40 == 0:
            self.get_logger().info(f"read_us:{read_us:.2f} pub_us:{pub_us:.2f} total_us:{total_us:.2f}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--topic', default='esp32/force')
    args = ap.parse_args()

    rclpy.init()
    node = FSRSerial(args.port, args.baud, args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node.destroy_node()
        except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass

if __name__ == '__main__':
    main()
