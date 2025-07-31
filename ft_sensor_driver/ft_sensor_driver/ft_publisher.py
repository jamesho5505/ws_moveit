import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, Vector3
import serial
import time

from threading import Thread
from collections import deque

LABELS = ['Fx','Fy','Fz','Mx','My','Mz']
HEXSET = set(b'0123456789ABCDEF')

class FT6Publisher(Node):
    def __init__(self):
        super().__init__('ft6_publisher')
        self.publisher = self.create_publisher(WrenchStamped, 'ft_data', 10)
        
        self.ser = serial.Serial('/dev/ttyUSB0', 921600, timeout=1.0)
        self.ser.reset_input_buffer()
        self.ser.write(b'S')  # 進入連續模式

        self.buf = bytearray()
        self.thread = Thread(target=self.serial_loop, daemon=True)
        self.thread.start()

    def valid(self, pkt: bytes) -> bool:
        return (
            len(pkt) == 27 and
            pkt[0:25][-2:] != b'\r\n' and
            pkt[25:]   == b'\r\n' and
            pkt[0]     in b'0123456789' and
            all(c in HEXSET for c in pkt[:25])
        )

    def to_values(self, pkt25: bytes):
        nums = [int(pkt25[1+4*i : 1+4*(i+1)], 16) for i in range(6)]
        fx = (nums[0] - 16384) / 17.0
        fy = (nums[1] - 16384) / 17.0
        fz = (nums[2] - 16384) / 12.0
        mx = (nums[3] - 16384) / 400.0
        my = (nums[4] - 16384) / 400.0
        mz = (nums[5] - 16384) / 400.0
        return fx, fy, fz, mx, my, mz

    def serial_loop(self):
        while rclpy.ok():
            try:
                if self.ser.in_waiting:
                    self.buf.extend(self.ser.read(self.ser.in_waiting))
                
                while True:
                    lf = self.buf.find(b'\n')
                    if lf == -1:
                        break
                    line = bytes(self.buf[:lf+1])
                    del self.buf[:lf+1]

                    if not self.valid(line):
                        continue

                    fx, fy, fz, mx, my, mz = self.to_values(line[:25])
                    msg = WrenchStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "ft_sensor"
                    msg.wrench.force = Vector3(x=fx, y=fy, z=fz)
                    msg.wrench.torque = Vector3(x=mx, y=my, z=mz)

                    self.publisher.publish(msg)

            except Exception as e:
                self.get_logger().error(f"Serial error: {e}")
                time.sleep(0.1)

    def destroy_node(self):
        try:
            self.ser.write(b'E')  # 離開連續模式
        finally:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FT6Publisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
