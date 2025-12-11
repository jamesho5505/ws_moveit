#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tm_msgs.srv import SendScript
from geometry_msgs.msg import WrenchStamped, PoseStamped

class SinglePtpScriptNode(Node):
    def __init__(self):
        super().__init__("single_ptp_script_node")

        self.send_script_client = self.create_client(SendScript, "send_script")
        self.get_logger().info("Waiting for /send_script service...")
        self.send_script_client.wait_for_service()
        self.get_logger().info("/send_script service ready")
        x = -381.77
        y = -310.27
        z = 262.45
        rx = -179.78
        ry = -0.33
        rz = -30.21
        speed_rate = 100
        acc_time = 100
        blend_percentage = 70
        precise_positioning = "false"
        # tm_script_command = 'PTP("JPP",-120,0,90,0,90,0,35,100,0,false)'
        tm_script_command = (f'PTP("CPP",{x},{y},{z},{rx},{ry},{rz},{speed_rate},{acc_time},{blend_percentage},{precise_positioning})')

        script_request = SendScript.Request()
        script_request.id = "1"  # 當成 Script ID，會在 TM 回覆裡看到同樣的 ID
        script_request.script = tm_script_command

        future = self.send_script_client.call_async(script_request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"SendScript failed: {error}")
            rclpy.shutdown()
            return

        self.get_logger().info(f"SendScript response.ok = {response.ok}")
        # 這裡如果 tm_driver 有把 ERROR;... 或 OK;... 的字串 expose 出來，也可以印出來

        # 指令送完就結束 node
        rclpy.shutdown()

# class IncrementalApproachNode(Node):
#     def __init__(self):
#         super().__init__("incremental_approach_node")

#         # 建立 /send_script client
#         self.send_script_client = self.create_client(SendScript, "send_script")
#         self.get_logger().info("Waiting for /send_script service...")
#         self.send_script_client.wait_for_service()
#         self.get_logger().info("/send_script service ready")

#         # 訂閱力訊號
#         self.force_subscription = self.create_subscription(
#             WrenchStamped,
#             '/ft_compensated',
#             self.force_torque_callback,
#             10
#         )

#         # 訂閱 TCP 位姿（請確認實際 topic 名稱與型別）
#         self.tool_pose_subscription = self.create_subscription(
#             PoseStamped,
#             "/tool_pose",              # 如果你的 topic 名稱不同，這裡要改
#             self.tool_pose_callback,
#             10,
#         )

#         self.latest_force_magnitude = 0.0
#         # 接觸判斷門檻 (自己調數值，先設小一點測試)
#         self.force_threshold_newton = 5.0
#         # 是否已經偵測到接觸
#         self.contact_detected = False

#         # 最近一次收到的 TCP 位姿
#         self.latest_tool_pose_msg: PoseStamped | None = None
#         # 接觸當下的 TCP 位姿（記錄用）
#         self.contact_tool_pose_msg: PoseStamped | None = None

#         # 起始 TCP 姿態 [X, Y, Z, Rx, Ry, Rz]
#         # 單位：通常 X/Y/Z 是 mm，Rx/Ry/Rz 是度（依你的 TM 設定）
#         self.current_tcp_pose_xyzrpy = None   # 起點先不要在 __init__ 用 latest_tool_pose_msg
#         self.step_target_z_mm = None         # 這一步的目標 Z (mm)
#         self.step_in_progress = False        # 這一步還在走
#         self.position_tolerance_mm = 0.1     # 認為「到達」的 Z 誤差


        

#         # 每一步在 Y 軸的位移量（例如 0.5 mm，一點一點靠近）
#         self.step_delta_z = -0.20

#         # 控制頻率（秒）：越小越頻繁，但要避免塞爆 script queue
#         self.control_period_s = 0.1

#         # 簡單安全限制：最多走多少步，免得無限往前
#         self.max_step_count = 800
#         self.current_step_count = 0

#         # 避免 service 疊加
#         self.pending_future = None

#         # 啟動定時器迴圈
#         self.control_timer = self.create_timer(
#             self.control_period_s,
#             self.control_loop_callback,
#         )

#     def tool_pose_callback(self, msg: PoseStamped) -> None:
#         """持續更新最新的 TCP 位姿。"""
#         self.latest_tool_pose_msg = msg
#         # 若還沒有在走任何 step，就不用檢查
#         if not self.step_in_progress:
#             return

#         if self.step_target_z_mm is None:
#             return

#         # 假設 /tool_pose 的 position.z 是公尺 → 轉成 mm
#         current_z_mm = msg.pose.position.z * 1000.0

#         if abs(current_z_mm - self.step_target_z_mm) < self.position_tolerance_mm:
#             # 這一步已經走到目標附近 → 下一輪 timer 可以發下一步
#             self.step_in_progress = False
        
        

#     def force_torque_callback(self, msg: WrenchStamped) -> None:
#         """訂閱 /ft_compensated，計算力大小，判斷是否超過門檻。"""
#         force_x = msg.wrench.force.x
#         force_y = msg.wrench.force.y
#         force_z = msg.wrench.force.z

#         # 你也可以只看某一軸，例如 abs(force_z)
#         self.latest_force_magnitude = (force_x**2 + force_y**2 + force_z**2) ** 0.5
#         # self.latest_force_magnitude = abs(force_z)

#         if (not self.contact_detected and self.latest_force_magnitude > self.force_threshold_newton):
#             if self.latest_tool_pose_msg is not None:
#                 # 記錄接觸瞬間位姿
#                 self.contact_tool_pose_msg = self.latest_tool_pose_msg
#                 p = self.contact_tool_pose_msg.pose.position
#                 q = self.contact_tool_pose_msg.pose.orientation
#                 self.get_logger().info(
#                     "Contact pose recorded:\n"
#                     f"  position = ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})\n"
#                     f"  orientation (quat) = "
#                     f"({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})",
#                 )
#             else:
#                 self.get_logger().warn(
#                     "Force exceeded threshold, but no tool pose received yet.",
#                 )

#             self.contact_detected = True
#             self.get_logger().info(
#                 f"Contact detected: |F| = {self.latest_force_magnitude:.2f} N",
#             )
            
#             self.send_stop_and_clear_buffer()
#             self.control_timer.cancel()
        

#     def control_loop_callback(self):
#         # 如果已經偵測到接觸，就送停止指令並關閉 timer
#         if self.contact_detected:
#             # self.get_logger().info(
#             #     f"Force over threshold ({self.latest_force_magnitude:.2f} N), "
#             #     "sending StopAndClearBuffer(0) and stopping.",
#             # )
#             # self.send_stop_and_clear_buffer()
#             # self.control_timer.cancel()
#             return
        
#         if self.current_tcp_pose_xyzrpy is None:
#             if self.latest_tool_pose_msg is None:
#                 self.get_logger().info("Waiting for first /tool_pose...")
#                 return
#             pose = self.latest_tool_pose_msg.pose

#             # 注意：這裡要確認 /tool_pose 的單位
#             # 若是 m / rad，需要轉成 TM CPP 用的 mm / deg
#             tcp_x_mm = pose.position.x * 1000.0
#             tcp_y_mm = pose.position.y * 1000.0
#             tcp_z_mm = pose.position.z * 1000.0

#             # 這裡你現在直接用 quaternion 當 Rx,Ry,Rz 是不對的
#             # 正確作法應該是 quat → RPY(deg)，這裡先給一組暫時值
#             tcp_rx_deg = -180.0
#             tcp_ry_deg = -0.0
#             tcp_rz_deg = -140.0

#             self.current_tcp_pose_xyzrpy = [
#                 tcp_x_mm,
#                 tcp_y_mm,
#                 tcp_z_mm,
#                 tcp_rx_deg,
#                 tcp_ry_deg,
#                 tcp_rz_deg,
#             ]
#             self.get_logger().info(
#                 f"Init TCP from /tool_pose: "
#                 f"({tcp_x_mm:.2f}, {tcp_y_mm:.2f}, {tcp_z_mm:.2f})",
#             )
#             # 這一輪先不送 PTP，下一輪再開始 step
#             return

#         # 步數超過就停掉（保護一下）
#         if self.current_step_count >= self.max_step_count:
#             self.get_logger().info(
#                 f"Reached max steps ({self.max_step_count}), "
#                 "sending StopAndClearBuffer(0) and stopping.",
#             )
#             self.send_stop_and_clear_buffer()
#             self.control_timer.cancel()  # 停掉後續的 step
#             return

#         if self.step_in_progress:
#             return

#         # 如果上一個 /send_script 還沒回來，就先不要再送
#         if self.pending_future is not None and not self.pending_future.done():
#             return

#         # 更新目標 Z
#         self.current_tcp_pose_xyzrpy[2] += self.step_delta_z
#         self.current_step_count += 1

#         (
#             tcp_x_position,
#             tcp_y_position,
#             tcp_z_position,
#             tcp_rx_angle,
#             tcp_ry_angle,
#             tcp_rz_angle,
#         ) = self.current_tcp_pose_xyzrpy

#         # 單行 CPP PTP 指令（沒有 float[]，也沒有分號）
#         # PTP("CPP", x, y, z, rx, ry, rz, speed_percent, acc_time_ms, blend, fine)
#         tm_script_command = (
#             'PTP("CPP",'
#             f"{tcp_x_position:.3f},{tcp_y_position:.3f},{tcp_z_position:.3f},"
#             f"{tcp_rx_angle:.3f},{tcp_ry_angle:.3f},{tcp_rz_angle:.3f},"
#             "30,300,0,false)"
#         )

#         script_request = SendScript.Request()
#         script_request.id = "1"
#         script_request.script = tm_script_command

#         self.get_logger().info(
#             f"Step {self.current_step_count}: "
#             f"Z = {tcp_z_position:.3f}, "
#             f"|F| = {self.latest_force_magnitude:.2f} N, sending script.",
#         )
#         self.pending_future = self.send_script_client.call_async(script_request)
#         self.pending_future.add_done_callback(self.step_response_callback)

#     def send_stop_and_clear_buffer(self):
#         """送出 StopAndClearBuffer(0) 讓 TM 立刻停止運動。"""
#         stop_request = SendScript.Request()
#         stop_request.id = "2"
#         stop_request.script = "StopAndClearBuffer(0)"
#         self.send_script_client.call_async(stop_request)

#     def step_response_callback(self, future):
#         try:
#             response = future.result()
#         except Exception as error:  # noqa: BLE001
#             self.get_logger().error(f"SendScript failed: {error}")
#             return

#         self.get_logger().info(f"SendScript response.ok = {response.ok}")
#         # 如果這裡發現 response.ok 是 False，可以視情況停掉 node 或做其他處理

class IncrementalApproachNode(Node):
    def __init__(self):
        super().__init__("incremental_approach_node")

        # /send_script client
        self.send_script_client = self.create_client(SendScript, "send_script")
        self.get_logger().info("Waiting for /send_script service...")
        self.send_script_client.wait_for_service()
        self.get_logger().info("/send_script service ready")

        # 力訊號
        self.force_subscription = self.create_subscription(
            WrenchStamped,
            "/ft_compensated",
            self.force_torque_callback,
            10,
        )

        # TCP 位姿
        self.tool_pose_subscription = self.create_subscription(
            PoseStamped,
            "/tool_pose",
            self.tool_pose_callback,
            10,
        )

        # 力相關
        self.latest_force_magnitude = 0.0
        self.force_threshold_newton = 2.0
        self.contact_detected = False

        # 位姿相關
        self.latest_tool_pose_msg: PoseStamped | None = None
        self.contact_tool_pose_msg: PoseStamped | None = None

        # 當前 (目標) TCP 姿態 [X, Y, Z, Rx, Ry, Rz]，單位 mm / deg
        self.current_tcp_pose_xyzrpy = None  # 等第一筆 /tool_pose 來再設定

        # 每一步在 Z 軸的位移量 (mm)
        self.step_delta_z = -0.05

        # 用來判斷「這一步走完了沒」
        self.step_target_z_mm = None
        self.step_in_progress = False
        self.position_tolerance_mm = 0.05  # 認定「到達目標」的 Z 誤差

        # 控制頻率：這裡只是決定「多久檢查一次要不要送下一步」
        self.control_period_s = 0.01

        self.max_step_count = 800
        self.current_step_count = 0
        self.pending_future = None

        self.control_timer = self.create_timer(
            self.control_period_s,
            self.control_loop_callback,
        )

    # ========== 回呼：tool pose ==========

    def tool_pose_callback(self, msg: PoseStamped) -> None:
        self.latest_tool_pose_msg = msg

        # 如果這一步還在走，檢查 Z 是否接近目標
        if self.step_in_progress and self.step_target_z_mm is not None:
            # /tool_pose.position.z 是公尺 → 轉 mm
            current_z_mm = msg.pose.position.z * 1000.0

            if abs(current_z_mm - self.step_target_z_mm) < self.position_tolerance_mm:
                # 這一步已經走到目標附近，可以開始下一步
                self.step_in_progress = False

    # ========== 回呼：force / torque ==========

    def force_torque_callback(self, msg: WrenchStamped) -> None:
        force_x = msg.wrench.force.x
        force_y = msg.wrench.force.y
        force_z = msg.wrench.force.z

        # self.latest_force_magnitude = (
        #     force_x**2 + force_y**2 + force_z**2
        # ) ** 0.5

        self.latest_force_magnitude = -force_z

        if (
            not self.contact_detected
            and self.latest_force_magnitude > self.force_threshold_newton
        ):
            # 第一次超標 → 視為接觸瞬間
            if self.latest_tool_pose_msg is not None:
                self.contact_tool_pose_msg = self.latest_tool_pose_msg
                p = self.contact_tool_pose_msg.pose.position
                q = self.contact_tool_pose_msg.pose.orientation
                self.get_logger().info(
                    "Contact pose recorded:\n"
                    f"  position = ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})\n"
                    f"  orientation (quat) = "
                    f"({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})",
                )
            else:
                self.get_logger().warn(
                    "Force exceeded threshold, but no tool pose received yet.",
                )

            self.contact_detected = True
            self.get_logger().info(
                f"Contact detected: |F| = {self.latest_force_magnitude:.2f} N, "
                "sending StopAndClearBuffer(0).",
            )
            self.send_stop_and_clear_buffer()
            self.control_timer.cancel()

    # ========== 控制主迴圈 ==========

    def control_loop_callback(self):
        # 已經接觸 → 不再發指令
        if self.contact_detected:
            return

        # 第一次先等 /tool_pose 來，拿當下位置當初始點
        if self.current_tcp_pose_xyzrpy is None:
            if self.latest_tool_pose_msg is None:
                self.get_logger().info("Waiting for first /tool_pose...")
                return

            pose = self.latest_tool_pose_msg.pose

            # /tool_pose 是 m → 轉成 mm
            tcp_x_mm = pose.position.x * 1000.0
            tcp_y_mm = pose.position.y * 1000.0
            tcp_z_mm = pose.position.z * 1000.0

            # 姿態你可以用目前 quat 轉 RPY，
            # 這裡為了簡單先用你原本調好的角度常數
            tcp_rx_deg = -180.0
            tcp_ry_deg = 0.0
            tcp_rz_deg = -180.0

            self.current_tcp_pose_xyzrpy = [
                tcp_x_mm,
                tcp_y_mm,
                tcp_z_mm,
                tcp_rx_deg,
                tcp_ry_deg,
                tcp_rz_deg,
            ]
            self.get_logger().info(
                f"Init TCP from /tool_pose: "
                f"({tcp_x_mm:.2f}, {tcp_y_mm:.2f}, {tcp_z_mm:.2f})",
            )
            return  # 下一輪再開始走

        # 步數保護
        if self.current_step_count >= self.max_step_count:
            self.get_logger().info(
                f"Reached max steps ({self.max_step_count}), "
                "sending StopAndClearBuffer(0) and stopping.",
            )
            self.send_stop_and_clear_buffer()
            self.control_timer.cancel()
            return

        # 上一段 step 還在走 / 或 service 還沒回來 → 先不要送新的
        if self.step_in_progress:
            return

        if self.pending_future is not None and not self.pending_future.done():
            return

        # 開始新的一步
        self.current_tcp_pose_xyzrpy[2] += self.step_delta_z
        self.current_step_count += 1

        (
            tcp_x_position,
            tcp_y_position,
            tcp_z_position,
            tcp_rx_angle,
            tcp_ry_angle,
            tcp_rz_angle,
        ) = self.current_tcp_pose_xyzrpy

        tm_script_command = (
            'PTP("CPP",'
            f"{tcp_x_position:.3f},{tcp_y_position:.3f},{tcp_z_position:.3f},"
            f"{tcp_rx_angle:.3f},{tcp_ry_angle:.3f},{tcp_rz_angle:.3f},"
            "10,100,0,false)"
        )

        script_request = SendScript.Request()
        script_request.id = "1"
        script_request.script = tm_script_command

        self.step_target_z_mm = tcp_z_position
        self.step_in_progress = True

        self.get_logger().info(
            f"Step {self.current_step_count}: "
            f"Z_target = {tcp_z_position:.3f} mm, "
            f"|F| = {self.latest_force_magnitude:.2f} N, sending script.",
        )
        self.pending_future = self.send_script_client.call_async(script_request)
        self.pending_future.add_done_callback(self.step_response_callback)

    # ========== 停車 / service callback ==========

    def send_stop_and_clear_buffer(self):
        stop_request = SendScript.Request()
        stop_request.id = "2"
        stop_request.script = "StopAndClearBuffer(0)"
        self.send_script_client.call_async(stop_request)

    def step_response_callback(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"SendScript failed: {error}")
            return

        self.get_logger().info(f"SendScript response.ok = {response.ok}")



def main(args=None):
    rclpy.init(args=args)
    # node = SinglePtpScriptNode()
    node = IncrementalApproachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by KeyboardInterrupt.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
