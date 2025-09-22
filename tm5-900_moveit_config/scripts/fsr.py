#!/usr/bin/env python3
import re, math, argparse, serial, rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

PAIR_RE = re.compile(r'([A-Za-z0-9_]+)(?:\([^)]+\))?=([+-]?(?:\d+(?:\.\d+)?))')

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
        # pairs = dict(PAIR_RE.findall(line))
        # try:
        #     f1 = float(pairs.get('Force1', 'nan'))
        #     f2 = float(pairs.get('Force2', 'nan'))
        # except ValueError:
        #     return
        f1 = float('nan'); f2 = float('nan')
        parts = line.split(',')
        if len(parts) == 2:
            try:
                f1 = float(parts[0].strip())
                f2 = float(parts[1].strip())
            except Exception:
                return
        else:
            return  # 不是 CSV 兩欄就略過

        if math.isnan(f1) or math.isnan(f2):
            return
        msg = Float32MultiArray()
        f1 = max(0.0, f1)
        f2 = max(0.0, f2)
        msg.data = [f1, f2, 0.5*(f1+f2)]
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
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=460800)
    ap.add_argument('--topic', default='arduino/force')
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
