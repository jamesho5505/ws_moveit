#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from tm_msgs.msg import FeedbackState
from tm_msgs.srv import SendScript


class TmCartesianPvtVelocityNode(Node):
    """
    使用 TM PVTEnter(1) + PVTPoint 進行笛卡爾速度控制：
    - /feedback_states 提供 TCP 姿態 (tool_pose)
    - /cmd_vel 提供 TCP 速度命令 (m/s, rad/s)
    - 每個週期依照 V*dt 積分，送出一個新的 PVTPoint
    """

    def __init__(self):
        super().__init__("tm_cartesian_pvt_velocity_node")

        # 參數：TM IP 目前只做記錄
        # self.declare_parameter("robot_ip", "192.168.10.3")
        # self.robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value

        # 建立 /send_script client
        self.send_script_client = self.create_client(SendScript, "send_script")
        self.get_logger().info("Waiting for /send_script service...")
        self.send_script_client.wait_for_service()
        self.get_logger().info("/send_script service is ready.")

        # 狀態變數
        # 注意：請確認 FeedbackState.tool_pose 單位：
        # 通常 X,Y,Z 為 mm，RX,RY,RZ 為 deg
        self.current_tool_pose = [0.0] * 6  # 機器人實際位置（從 feedback）
        self.target_tool_pose = [0.0] * 6   # PVT 目標位置（計算用）
        self.has_tool_pose = False
        self.has_velocity_command = False
        self.last_velocity_command_time = None
        self.velocity_command_timeout_seconds = 0.50  # 例如 0.5 秒沒新指令就當作停

        # 目前的速度指令（給 PVT 用的單位：mm/s, deg/s）
        self.cartesian_velocity_command = [0.0] * 6

        # PVT 週期 (秒)
        self.pvt_duration_seconds = 1.0 / 60.0  # 60 Hz

        # 從 /feedback_states 取得 TCP 姿態
        self.feedback_subscription = self.create_subscription(
            FeedbackState,
            "feedback_states",
            self.feedback_callback,
            10,
        )

        # 從 /cmd_vel 取得速度指令
        self.twist_subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.twist_callback,
            10,
        )

        # 啟動 PVT 模式（Cartesian）
        self.send_script_command("PVTEnter(1)")
        self.get_logger().info("Sent PVTEnter(1), Cartesian PVT mode enabled.")

        # 週期性送 PVTPoint
        self.pvt_timer = self.create_timer(self.pvt_duration_seconds, self.pvt_timer_callback)

    # ------------------------------------------------------------------
    # 回授處理：更新 TCP 姿態
    # ------------------------------------------------------------------
    def feedback_callback(self, msg: FeedbackState) -> None:
        raw = list(msg.tool_pose)  # raw: [x_m, y_m, z_m, rx_rad, ry_rad, rz_rad]
        x_mm  = raw[0] * 1000.0
        y_mm  = raw[1] * 1000.0
        z_mm  = raw[2] * 1000.0
        rx_deg = raw[3] * 180.0 / math.pi
        ry_deg = raw[4] * 180.0 / math.pi
        rz_deg = raw[5] * 180.0 / math.pi
        
        self.current_tool_pose = [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
    
        if not self.has_tool_pose:
            # 第一次初始化時，目標位置 = 實際位置
            self.target_tool_pose = self.current_tool_pose.copy()
            self.has_tool_pose = True
        # self.current_tool_pose = [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
        # if not self.has_tool_pose:
        #      self.get_logger().info(f"Initial tool pose: {self.current_tool_pose}")
        # self.has_tool_pose = True # 總是要設為 True

    # ------------------------------------------------------------------
    # /cmd_vel → 內部速度指令 (mm/s, deg/s)
    # ------------------------------------------------------------------
    def twist_callback(self, twist_message: Twist) -> None:
        meters_per_second_to_mm_per_second = 1000.0
        radians_per_second_to_deg_per_second = 180.0 / math.pi

        had_velocity_command_before = self.has_velocity_command

        linear_x_mm_per_second = twist_message.linear.x * meters_per_second_to_mm_per_second
        linear_y_mm_per_second = twist_message.linear.y * meters_per_second_to_mm_per_second
        linear_z_mm_per_second = twist_message.linear.z * meters_per_second_to_mm_per_second

        angular_rx_deg_per_second = twist_message.angular.x * radians_per_second_to_deg_per_second
        angular_ry_deg_per_second = twist_message.angular.y * radians_per_second_to_deg_per_second
        angular_rz_deg_per_second = twist_message.angular.z * radians_per_second_to_deg_per_second

        self.cartesian_velocity_command = [
            linear_x_mm_per_second,
            linear_y_mm_per_second,
            linear_z_mm_per_second,
            angular_rx_deg_per_second,
            angular_ry_deg_per_second,
            angular_rz_deg_per_second,
        ]
        current_time_seconds = time.time()
        self.last_velocity_command_time = current_time_seconds
        self.get_logger().info(f"Received velocity command: {self.cartesian_velocity_command}")

        if any(abs(v) > 1e-6 for v in self.cartesian_velocity_command):
            self.has_velocity_command = True
        else:
            self.has_velocity_command = False


        # 再更新「現在」的狀態
        self.has_velocity_command = any(
            abs(command_value) > 1e-6 for command_value in self.cartesian_velocity_command
        )

        # 只有從 False → True 的那一瞬間才重送 PVTEnter(1)
        if self.has_velocity_command and not had_velocity_command_before:
            self.get_logger().info("New velocity command after stop, send PVTEnter(1) again.")
            self.send_script_command("PVTEnter(1)")

    # ------------------------------------------------------------------
    # 週期送出一個新的 PVTPoint
    # ------------------------------------------------------------------
    def pvt_timer_callback(self) -> None:
        if not self.has_tool_pose:
            return
        
        # 如果曾經收過指令，且超過 timeout 沒收到新指令 → 當作停
        if self.last_velocity_command_time is not None:
            elapsed_seconds = time.time() - self.last_velocity_command_time
            if elapsed_seconds > self.velocity_command_timeout_seconds:
                # 超過 timeout 才清掉，並送一次 PVTExit
                self.cartesian_velocity_command = [0.0] * 6
                if self.has_velocity_command:
                    # 只有在 PVTExit() 前才顯示 log
                    self.get_logger().info("Velocity timeout, send PVTExit().")
                    self.send_script_command("PVTExit()")
                    # 讓下次啟動時，current_tool_pose 必須從 feedback 重新取得
                
                self.has_velocity_command = False

        if not self.has_velocity_command:
            return
        dt = self.pvt_duration_seconds 
        # dt = 0.1
        max_linear_speed_mm_s = 100.0
        max_angular_speed_deg_s = 30.0

        next_tool_pose = []
        for axis_index in range(6):
            # current_position = self.current_tool_pose[axis_index]
            current_position = self.target_tool_pose[axis_index]
            raw_velocity = self.cartesian_velocity_command[axis_index]

            if axis_index < 3:
                clamped_velocity = max(-max_linear_speed_mm_s,
                                    min(max_linear_speed_mm_s, raw_velocity))
            else:
                clamped_velocity = max(-max_angular_speed_deg_s,
                                    min(max_angular_speed_deg_s, raw_velocity))

            # 用 clamp 後的速度積分
            next_position = current_position + clamped_velocity * dt

            self.cartesian_velocity_command[axis_index] = clamped_velocity
            # next_tool_pose.append(round(next_position, 4))
            next_tool_pose.append(next_position)



        position_string = "{" + ", ".join(str(v) for v in next_tool_pose) + "}"
        velocity_string = "{" + ", ".join(str(v) for v in self.cartesian_velocity_command) + "}"
        script_command = f"PVTPoint({position_string}, {velocity_string}, {dt})"

        self.send_script_command(script_command)
        self.target_tool_pose = next_tool_pose
        # self.target_tool_pose = self.current_tool_pose.copy()

    # ------------------------------------------------------------------
    # SendScript 封裝
    # ------------------------------------------------------------------
    def send_script_command(self, script_text: str) -> None:
        script_request = SendScript.Request()
        script_request.id = str(int(time.time() * 1000) % 100000)
        script_request.script = script_text
        self.send_script_client.call_async(script_request)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TmCartesianPvtVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 這裡很可能 context 還是好的（但也可能已被關）
        if rclpy.ok():
            # 只在還沒 shutdown 的情況下，送一次 PVTExit 並寫 log
            node.send_script_command("PVTExit()")
            node.get_logger().info("Sent PVTExit().")

        node.destroy_node()

        # 再次確認，避免重複 shutdown
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from tm_msgs.srv import SendScript
# import time

# class PvtTrajectoryNode(Node):
#     def __init__(self):
#         super().__init__("pvt_trajectory_node")

#         # 建立 /send_script 服務的客戶端
#         self.send_script_client = self.create_client(SendScript, "send_script")
#         self.get_logger().info("Waiting for /send_script service...")
#         self.send_script_client.wait_for_service()
#         self.get_logger().info("/send_script service is ready.")

#         # --- PVT 指令 ---
#         pvt_commands = [
#             "PVTEnter(1)",
#             # 起點 (只是一個初始點，位置、速度都一樣)
#             "PVTPoint(-7.63,-591.4,243.25,180,0,180,  0,0,0,0,0,0, 0.1)",

#             # 往 +Y 方向移動 (Y -591.4 → -566.4)
#             "PVTPoint(-7.63,-566.4,243.25,180,0,180,  0,50,0,0,0,0, 0.1)",
#             "PVTPoint(-7.63,-541.4,243.25,180,0,180,  0,50,0,0,0,0, 0.1)",
#             "PVTPoint(-7.63,-516.4,243.25,180,0,180,  0,50,0,0,0,0, 0.1)",
#             "PVTPoint(-7.63,-491.4,243.25,180,0,180,  0,50,0,0,0,0, 0.1)",

#             # 再走回來 (Y -491.4 → -591.4)
#             "PVTPoint(-7.63,-516.4,243.25,180,0,180,  0,-50,0,0,0,0, 0.1)",
#             "PVTPoint(-7.63,-541.4,243.25,180,0,180,  0,-50,0,0,0,0, 0.1)",
#             "PVTPoint(-7.63,-566.4,243.25,180,0,180,  0,-50,0,0,0,0, 0.1)",
#             "PVTPoint(-7.63,-591.4,243.25,180,0,180,  0,-50,0,0,0,0, 0.1)",
#             "PVTExit()",
#         ]
#         tm_script_command = "\n".join(pvt_commands)

#         self.get_logger().info("Constructed PVT script:\n" + tm_script_command)

#         # 準備服務請求
#         script_request = SendScript.Request()
#         script_request.id = "1"
#         script_request.script = tm_script_command

#         # 非同步呼叫服務，利用 lambda 把 request 一起帶進 callback
#         future = self.send_script_client.call_async(script_request)
#         future.add_done_callback(
#             lambda fut, req=script_request: self.handle_response(fut, req)
#         )

#     def handle_response(self, future, request):
#         """處理服務的回應"""
#         try:
#             response = future.result()
#             if response.ok:
#                 self.get_logger().info(f"Script '{request.id}' sent successfully.")
#             else:
#                 self.get_logger().error(f"Script '{request.id}' failed to execute.")
#         except Exception as exception_object:
#             self.get_logger().error(
#                 f"Service call for script '{request.id}' failed: {exception_object!r}"
#             )
#         finally:
#             self.get_logger().info("Shutting down node.")
#             # 這裡只呼叫 rclpy.shutdown，由 main 來 destroy_node
#             rclpy.shutdown()


# def main(args=None):
#     rclpy.init(args=args)
#     node = PvtTrajectoryNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         # 如果還沒被 handle_response 裡的 shutdown 關掉，這裡補一次
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == "__main__":
#     main()


# #!/usr/bin/env python3
# import math
# import time

# import rclpy
# from rclpy.node import Node

# from geometry_msgs.msg import Twist
# from tm_msgs.srv import SendScript
# from tm_msgs.msg import FeedbackState


# class TmCartesianPvtVelocityController(Node):
#     """
#     利用 PVTEnter(1) + PVTPoint 來做笛卡爾速度控制。
#     - 訂閱 /cmd_vel (m/s, rad/s)
#     - 轉成 TM PVT 所需的 mm / deg
#     - 週期性送出 PVTPoint({X,Y,Z,RX,RY,RZ}, {VX,VY,VZ,VRX,VRY,VRZ}, duration)
#     """

#     def __init__(self):
#         super().__init__('tm_cartesian_pvt_velocity_controller')

#         # 參數：機器人 IP (目前僅作記錄，不直接使用)
#         self.declare_parameter('robot_ip', '192.168.10.3')
#         self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value

#         # === SendScript 客戶端 ===
#         self.script_client = self.create_client(SendScript, 'send_script')
#         self.get_logger().info('Waiting for SendScript service...')
#         self.script_client.wait_for_service()
#         self.get_logger().info('SendScript service ready.')

#         # === 狀態變數 ===
#         # TM tool pose: {X, Y, Z, RX, RY, RZ}
#         # 注意：請確認 tm_driver 回來的單位 (多半是 X,Y,Z: mm，RX,RY,RZ: deg)
#         self.current_tool_pose = [0.0] * 6
#         self.is_tool_pose_initialized = False

#         # 速度指令 (TM PVT 用的單位: mm/s, deg/s)
#         self.current_cartesian_velocity_command = [0.0] * 6

#         # PVT 模式狀態
#         self.is_pvt_mode_active = False

#         # 週期時間 (秒) - PVTPoint 的 duration
#         self.pvt_duration_seconds = 0.05  # 50 ms, 20 Hz

#         # 零速度判斷閾值
#         self.velocity_epsilon = 1e-6

#         # === Topic 訂閱 ===
#         # 從 tm_driver 拿機器人回授狀態
#         self.feedback_subscription = self.create_subscription(
#             FeedbackState,
#             'feedback_states',
#             self.feedback_callback,
#             10
#         )

#         # 從上層接收速度命令 /cmd_vel
#         self.twist_subscription = self.create_subscription(
#             Twist,
#             '/cmd_vel',
#             self.twist_callback,
#             10
#         )

#         # === PVT 週期 Timer ===
#         self.pvt_timer = self.create_timer(self.pvt_duration_seconds, self.pvt_timer_callback)

#         self.get_logger().info('TM Cartesian PVT Velocity Controller initialized.')

#     # ----------------------------------------------------------------------
#     # 回授處理
#     # ----------------------------------------------------------------------
#     def feedback_callback(self, feedback_message: FeedbackState) -> None:
#         """
#         從 FeedbackState 取得 TCP 笛卡爾位置。
#         請確認 tm_msgs/FeedbackState 的定義中，tool_pose 是否為 {X,Y,Z,RX,RY,RZ}。
#         """
#         tool_pose_array = list(feedback_message.tool_pose)

#         if len(tool_pose_array) == 6:
#             self.current_tool_pose = tool_pose_array
#             if not self.is_tool_pose_initialized:
#                 self.is_tool_pose_initialized = True
#                 self.get_logger().info(f'Initial tool pose received: {self.current_tool_pose}')

#     # ----------------------------------------------------------------------
#     # /cmd_vel → 內部速度指令 (mm/s, deg/s)
#     # ----------------------------------------------------------------------
#     def twist_callback(self, twist_message: Twist) -> None:
#         """
#         /cmd_vel:
#         - linear: m/s
#         - angular: rad/s

#         轉成 TM PVT 用單位：
#         - X, Y, Z: mm/s
#         - RX, RY, RZ: deg/s
#         """

#         meters_per_second_to_mm_per_second = 1000.0
#         radians_per_second_to_deg_per_second = 180.0 / math.pi

#         linear_velocity_mm_per_second_x = twist_message.linear.x * meters_per_second_to_mm_per_second
#         linear_velocity_mm_per_second_y = twist_message.linear.y * meters_per_second_to_mm_per_second
#         linear_velocity_mm_per_second_z = twist_message.linear.z * meters_per_second_to_mm_per_second

#         angular_velocity_deg_per_second_rx = twist_message.angular.x * radians_per_second_to_deg_per_second
#         angular_velocity_deg_per_second_ry = twist_message.angular.y * radians_per_second_to_deg_per_second
#         angular_velocity_deg_per_second_rz = twist_message.angular.z * radians_per_second_to_deg_per_second

#         self.current_cartesian_velocity_command = [
#             linear_velocity_mm_per_second_x,
#             linear_velocity_mm_per_second_y,
#             linear_velocity_mm_per_second_z,
#             angular_velocity_deg_per_second_rx,
#             angular_velocity_deg_per_second_ry,
#             angular_velocity_deg_per_second_rz,
#         ]

#     # ----------------------------------------------------------------------
#     # PVT Timer 主迴圈
#     # ----------------------------------------------------------------------
#     def pvt_timer_callback(self) -> None:
#         """
#         週期性：
#         1. 檢查是否有速度指令
#         2. 進入 / 退出 PVT 模式
#         3. 計算下一個目標位置，送出 PVTPoint
#         """

#         if not self.is_tool_pose_initialized:
#             # 尚未有任何 tool_pose 回授，不能亂送 PVTPoint
#             return

#         has_nonzero_velocity_command = any(
#             abs(velocity_component) > self.velocity_epsilon
#             for velocity_component in self.current_cartesian_velocity_command
#         )

#         # ------------------------------------------------------------------
#         # 無速度指令 → 如有需要，退出 PVT 模式
#         # ------------------------------------------------------------------
#         if not has_nonzero_velocity_command:
#             if self.is_pvt_mode_active:
#                 self.send_script_command('PVTExit()')
#                 self.get_logger().info('PVT mode exited (zero velocity command).')
#                 self.is_pvt_mode_active = False
#             return

#         # ------------------------------------------------------------------
#         # 有速度指令 → 如需要，進入 PVT 模式
#         # ------------------------------------------------------------------
#         if not self.is_pvt_mode_active:
#             # 1 代表 Cartesian PVT 模式
#             self.send_script_command('PVTEnter(1)')
#             self.get_logger().info('PVT mode entered (Cartesian).')
#             self.is_pvt_mode_active = True

#         # ------------------------------------------------------------------
#         # 計算下一個目标位置
#         # current_tool_pose 單位假設為 {mm, mm, mm, deg, deg, deg}
#         # current_cartesian_velocity_command 單位為 {mm/s, mm/s, mm/s, deg/s, deg/s, deg/s}
#         # ------------------------------------------------------------------
#         next_tool_pose = []

#         for index_axis in range(6):
#             position_current = self.current_tool_pose[index_axis]
#             velocity_command = self.current_cartesian_velocity_command[index_axis]
#             position_next = position_current + velocity_command * self.pvt_duration_seconds
#             # 保留適當小數位
#             position_next_rounded = round(position_next, 4)
#             next_tool_pose.append(position_next_rounded)

#         # ------------------------------------------------------------------
#         # 準備 PVTPoint script
#         # ------------------------------------------------------------------
#         position_string = '{' + ', '.join(str(value) for value in next_tool_pose) + '}'
#         velocity_string = '{' + ', '.join(str(value) for value in self.current_cartesian_velocity_command) + '}'
#         duration_value = self.pvt_duration_seconds

#         script_command = f'PVTPoint({position_string}, {velocity_string}, {duration_value})'

#         # 寄出 script（非同步）
#         self.send_script_command(script_command)

#         # DEBUG 用 log，可視情況關閉
#         self.get_logger().debug(f'Sent PVTPoint: {script_command}')

#         # 更新 current_tool_pose 為下一點，避免下一次迴圈還用舊的 poses 做積分
#         self.current_tool_pose = next_tool_pose

#     # ----------------------------------------------------------------------
#     # 封裝 SendScript 呼叫
#     # ----------------------------------------------------------------------
#     def send_script_command(self, script_text: str) -> None:
#         """
#         通用的 SendScript 呼叫封裝。
#         目前僅非同步送出，不等待結果。
#         如需除錯，可以對 future 加 callback 看回傳。
#         """
#         send_script_request = SendScript.Request()
#         send_script_request.id = str(int(time.time() * 1000) % 100000)  # 簡單產生一個 ID
#         send_script_request.script = script_text

#         self.script_client.call_async(send_script_request)


# def main(args=None) -> None:
#     rclpy.init(args=args)
#     node = TmCartesianPvtVelocityController()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # 結束前，如果還在 PVT 模式，送一次 PVTExit()
#         if node.is_pvt_mode_active:
#             node.send_script_command('PVTExit()')
#             node.get_logger().info('PVT mode exited on shutdown.')

#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from tm_msgs.srv import SendScript # 您的 SendScript.srv 類型
# from tm_msgs.msg import FeedbackState # 用於獲取當前位置
# from geometry_msgs.msg import Twist
# import time

# class PtvVelocityController(Node):
#     def __init__(self):
#         super().__init__('ptv_velocity_controller')
#         # 1. 服務客戶端
#         self.script_client = self.create_client(SendScript, 'send_script')
#         while not self.script_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('SendScript service not available, waiting...')

#         # 2. 訂閱當前狀態和速度命令
#         self.feedback_sub = self.create_subscription(
#             FeedbackState, 'feedback_states', self.feedback_callback, 10
#         )
#         self.twist_sub = self.create_subscription(
#             Twist, '/cmd_vel', self.twist_callback, 10 # 訂閱速度命令Topic
#         )

#         # 狀態變量
#         self.current_pos = [0.0] * 6  # 當前笛卡爾位置 {X, Y, Z, RX, RY, RZ}
#         self.current_vel = [0.0] * 6  # 當前笛卡爾速度
#         self.cmd_vel = [0.0] * 6      # 目標速度命令
#         self.PVT_DURATION = 0.05      # 50 ms 週期 (20 Hz)

#         # 3. 週期性 PVT 點發送計時器
#         self.timer = self.create_timer(self.PVT_DURATION, self.timer_callback)

#         self.get_logger().info('PVT Velocity Controller initialized.')

#     # 獲取機器人當前狀態
#     def feedback_callback(self, msg):
#         # 假設 tm_driver 會將 CartesianInfo 填充到 msg 中
#         # 注意：您可能需要確認 FeedbackState 哪個欄位包含笛卡爾位置
#         if len(msg.tool_pose) == 6:
#              self.current_pos = msg.tool_pose 
#         if len(msg.joint_vel) == 6:
#              self.current_vel = msg.joint_vel

#     # 接收速度命令
#     def twist_callback(self, msg):
#         self.cmd_vel = [
#             msg.linear.x, msg.linear.y, msg.linear.z,
#             msg.angular.x, msg.angular.y, msg.angular.z
#         ]

#     # 4. PVT 點計算與發送
#     def timer_callback(self):
#         # 如果沒有速度命令，則停止手臂
#         if all(v == 0.0 for v in self.cmd_vel):
#             # 停止手臂
#             script_cmd = "PVTExit()"
#             self.send_script_request(script_cmd)
#             return
        
#         # 啟動 PVT 模式 (在第一次運動前或保證模式開啟)
#         # 實際應用中，只需啟動一次 PVTEnter(1) 即可
#         # self.send_script_request("PVTEnter(1)") 

#         # 1. 計算下一個目標位置 P_next
#         p_next = []
#         for i in range(6):
#             # P_next = P_current + V_cmd * T
#             next_p = self.current_pos[i] + self.cmd_vel[i] * self.PVT_DURATION
#             p_next.append(round(next_p, 4)) # 保持精度

#         # 2. 構造 PVTPoint 腳本
#         # PVTPoint({X, Y, Z, RX, RY, RZ}, {VX, VY, VZ, VRX, VRY, VRZ}, Duration)
#         pos_str = "{" + ", ".join(map(str, p_next)) + "}"
#         vel_str = "{" + ", ".join(map(str, self.cmd_vel)) + "}"
        
#         script_cmd = f"PVTPoint({pos_str}, {vel_str}, {self.PVT_DURATION})"
        
#         # 3. 發送腳本
#         self.send_script_request(script_cmd)
        
#         self.get_logger().info(f'Sent PVT Point: {script_cmd}')


#     def send_script_request(self, script_cmd):
#         req = SendScript.Request()
#         req.id = str(int(time.time() * 1000) % 10000) # 生成唯一ID
#         req.script = script_cmd
#         self.script_client.call_async(req)


# def main(args=None):
#     rclpy.init(args=args)
#     # 在開始 PVT 運動前，首先發送 PVTEnter(1) 腳本
#     temp_node = Node('pvt_starter')
#     temp_client = temp_node.create_client(SendScript, 'send_script')
#     temp_client.wait_for_service(timeout_sec=5.0)
    
#     req = SendScript.Request()
#     req.id = 'PVT_INIT'
#     req.script = 'PVTEnter(1)' # 笛卡爾速度控制
    
#     temp_client.call_async(req)
#     temp_node.get_logger().info('Sent PVTEnter(1) command.')
#     temp_node.destroy_node()


#     # 啟動主控制器節點
#     pvt_controller = PtvVelocityController()
#     try:
#         rclpy.spin(pvt_controller)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # 確保退出時發送 PVTExit()
#         pvt_controller.send_script_request("PVTExit()")
#         pvt_controller.get_logger().info('Sent PVTExit() command.')
#         pvt_controller.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
