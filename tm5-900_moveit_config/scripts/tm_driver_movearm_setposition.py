#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from tm_msgs.srv import SetPositions
from sensor_msgs.msg import JointState


class SingleMoveNode(Node):
    def __init__(self):
        super().__init__("single_move_node")

        # 只為了印目前關節，非必要但很有幫助
        self.joint_state_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.current_joint_positions = None

        # 建立 /set_positions client
        self.set_positions_client = self.create_client(SetPositions, "/set_positions")
        self.get_logger().info("Waiting for /set_positions service...")
        self.set_positions_client.wait_for_service()
        self.get_logger().info("/set_positions service available.")

        # 延遲 2 秒再送，等 joint_states 穩定
        self.create_timer(2.0, self.send_one_ptp_command)

        # 避免多次送 command，用 flag 控制
        self.command_sent = False

    def joint_state_callback(self, joint_state_message: JointState):
        self.current_joint_positions = list(joint_state_message.position)

    def send_one_ptp_command(self):
        if self.command_sent:
            return

        if self.current_joint_positions is None:
            self.get_logger().warn("No /joint_states received yet, skip this cycle.")
            return

        # 這裡請換成你想要去的關節角，單位是 rad，長度要 6
        # 例如直接用目前角度，稍微改一點 joint_1
        target_joint_positions = self.current_joint_positions.copy()
        target_joint_positions[0] -= 0.3  # joint_1 加 0.3 rad 當示範

        set_positions_request = SetPositions.Request()
        set_positions_request.motion_type = SetPositions.Request.PTP_J
        set_positions_request.positions = target_joint_positions

        # 速度 / 加速時間 / blend 自己調整
        set_positions_request.velocity = 1.0          # rad/s 上限
        set_positions_request.acc_time = 200.0        # ms
        set_positions_request.blend_percentage = 0    # 0 = 不 blending
        set_positions_request.fine_goal = True

        future = self.set_positions_client.call_async(set_positions_request)

        def handle_response(future_result):
            try:
                response = future_result.result()
                if response.ok:
                    self.get_logger().info("SetPositions command accepted by robot.")
                else:
                    self.get_logger().error("SetPositions command rejected by robot.")
            except Exception as exception:
                self.get_logger().error(f"Service call failed: {exception!r}")

        future.add_done_callback(handle_response)

        self.command_sent = True
        self.get_logger().info("PTP_J command sent.")

def main(args=None):
    rclpy.init(args=args)
    node = SingleMoveNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
