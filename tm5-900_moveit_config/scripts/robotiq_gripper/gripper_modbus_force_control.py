#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from robotiq_gripper import RobotiqGripper # 引用你的 Modbus 通訊驅動程式
import minimalmodbus

class RobotiqGripperNode(Node):
    def __init__(self):
        super().__init__('robotiq_gripper_node')

        # 1. 宣告參數
        self.declare_parameter('portname', '/dev/ttyUSB0')
        self.declare_parameter('slaveaddress', 9)
        self.declare_parameter('baudrate', 115200)

        # 2. 建立底層通訊物件
        port = self.get_parameter('portname').value
        addr = self.get_parameter('slaveaddress').value
        baud = self.get_parameter('baudrate').value
        instrument = minimalmodbus.Instrument(port, addr, debug=True)
        instrument.serial.baudrate = baud
        self.gripper = RobotiqGripper(port, slaveaddress=addr)
        self.gripper.activate()
        self.gripper.calibrate(0, 85)
        self.gripper.goTo(0)  # 初始位置

        # 3. 建立 ROS2 publisher 和 subscriber
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/robotiq_gripper_controller/commands',
            self.gripper_cmd_cb,
            10
        )
        # 這裡可以再建立一個 publisher 來發布夾爪狀態

        # 4. 啟動定時器來定期讀取狀態
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.get_logger().info('Robotiq Gripper Node is ready.')

    def gripper_cmd_cb(self, msg: Float64MultiArray):
        if not msg.data:
            return
        # 假設 msg.data[0] 是目標位置（0-255）
        target_pos = msg.data[0]
        self.gripper.goTo(int(target_pos))

    def timer_cb(self):
        # 這裡可以讀取夾爪狀態並發布
        # pos, current = self.gripper.getPositionCurrent()
        # publish(pos, current)
        pass

def main():
    rclpy.init()
    node = RobotiqGripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()