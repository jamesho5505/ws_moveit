#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import struct
import threading
import time
from scipy.signal import butter, lfilter, lfilter_zi
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


PORT_NAME = "/dev/ttyUSB0"
SLAVE_ADDRESS = 9

BAUDRATE = 19200       # 依照 FT300-S 手冊 Modbus 設定
DATA_BITS = 8
STOP_BITS = 1
PARITY = "N"
READ_TIMEOUT = 0.05    # 讀取 timeout (秒)


# Butterworth filter
order = 4      # 濾波階數，可調 2~4
f_c = 2        # 截止頻率 Hz
f_s = 100      # 取樣頻率 Hz
b, a = butter(order, f_c / (f_s / 2), btype='low')

# ================= 濾波函式 =================
def butterworth_filter(x, z, b, a):
    y, zf = lfilter(b, a, [x], zi=z)
    return y[0], zf

zi_force = [lfilter_zi(b, a) * 0 for _ in range(3)]   # Fx, Fy, Fz
zi_torque = [lfilter_zi(b, a) * 0 for _ in range(3)]  # Tx, Ty, Tz



def compute_modbus_crc(message_bytes: bytes) -> int:
    """
    標準 Modbus RTU CRC-16 (poly=0xA001, init=0xFFFF).
    回傳 0~0xFFFF 的整數。
    """
    crc_value = 0xFFFF
    for one_byte in message_bytes:
        crc_value ^= one_byte
        for _ in range(8):
            if (crc_value & 0x0001) != 0:
                crc_value >>= 1
                crc_value ^= 0xA001
            else:
                crc_value >>= 1
    return crc_value & 0xFFFF


class Ft300StreamNode(Node):
    def __init__(self):
        super().__init__("ft300_stream_node")

        # 允許用參數覆蓋 port / slave id
        self.declare_parameter("port_name", PORT_NAME)
        self.declare_parameter("slave_address", SLAVE_ADDRESS)

        self.port_name = self.get_parameter("port_name").get_parameter_value().string_value
        self.slave_address = self.get_parameter("slave_address").get_parameter_value().integer_value

        if self.slave_address <= 0:
            self.slave_address = SLAVE_ADDRESS

        self.serial_port = None
        self.should_exit = False

        self.wrench_publisher = self.create_publisher(
            WrenchStamped, "/robotiq_force_torque_sensor_broadcaster/wrench", 100
        )
        self.wrench_filtered_publisher = self.create_publisher(
            WrenchStamped, "/robotiq_force_torque_sensor_broadcaster/wrench_filtered", 100
        )

        self.get_logger().info(f"Opening serial port {self.port_name} at {BAUDRATE} baud...")
        self._open_serial_port()
        self._stop_existing_stream()
        self._start_data_stream()

        # 背景執行緒，不阻塞 rclpy
        self.reader_thread = threading.Thread(
            target=self._data_stream_loop, daemon=True
        )
        self.reader_thread.start()

        self.get_logger().info("FT300-S data stream (100 Hz) node started.")

    def _open_serial_port(self):
        self.serial_port = serial.Serial(
            port=self.port_name,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=READ_TIMEOUT,
        )
        # 清空 buffer
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()

    def _stop_existing_stream(self):
        """
        手冊建議：送一串 0xFF 約 0.5 秒把舊的 data stream 關掉。:contentReference[oaicite:5]{index=5}
        """
        self.get_logger().info("Sending 0xFF sequence to stop any existing stream...")
        if not self.serial_port:
            return
        stop_bytes = b"\xff" * 50
        self.serial_port.write(stop_bytes)
        self.serial_port.flush()
        time.sleep(0.1)

    def _start_data_stream(self):
        """
        寫 0x0200 到 register 410 (0x019A) with FC16 (0x10) 來啟動 data stream。

        Modbus RTU frame:
        [SlaveID][0x10][0x01][0x9A][0x00][0x01][0x02][0x02][0x00][CRC_L][CRC_H]
        """
        self.get_logger().info("Sending FC16 to start data stream...")

        if not self.serial_port:
            raise RuntimeError("Serial port is not open")

        # Build Modbus frame without CRC
        frame_without_crc = bytearray()
        frame_without_crc.append(self.slave_address)  # Slave ID
        frame_without_crc.append(0x10)               # Function 16
        frame_without_crc.append(0x01)               # Address high (0x019A)
        frame_without_crc.append(0x9A)               # Address low
        frame_without_crc.append(0x00)               # Number of registers high
        frame_without_crc.append(0x01)               # Number of registers low (1 register)
        frame_without_crc.append(0x02)               # Byte count (1 register x 2 bytes)
        frame_without_crc.append(0x02)               # Value high (0x0200)
        frame_without_crc.append(0x00)               # Value low

        crc_value = compute_modbus_crc(frame_without_crc)
        crc_low = crc_value & 0xFF
        crc_high = (crc_value >> 8) & 0xFF

        full_frame = frame_without_crc + bytes([crc_low, crc_high])

        self.serial_port.write(full_frame)
        self.serial_port.flush()

        # 等待感測器切到 data stream 模式
        time.sleep(0.1)
        self.serial_port.reset_input_buffer()
        self.get_logger().info("Data stream command sent.")

    def _data_stream_loop(self):
        """
        讀取 16 bytes data stream，不斷解析 Fx,Fy,Fz,Mx,My,Mz 並發 ROS2 topic。

        封包格式：
        <0x20><0x4e><LSB_data1><MSB_data1> ... <LSB_data6><MSB_data6><LSB_crc><MSB_crc>
        data1 = Fx * 100 (N)
        data2 = Fy * 100 (N)
        data3 = Fz * 100 (N)
        data4 = Mx * 1000 (Nm)
        data5 = My * 1000 (Nm)
        data6 = Mz * 1000 (Nm)
        """
        buffer = bytearray()

        while rclpy.ok() and not self.should_exit:
            try:
                # 持續讀取一些 bytes
                incoming = self.serial_port.read(32)
                if not incoming:
                    continue

                buffer.extend(incoming)

                # 尋找 16-byte 封包
                while len(buffer) >= 16:
                    # 找 header 0x20 0x4E
                    if not (buffer[0] == 0x20 and buffer[1] == 0x4E):
                        # 丟掉第一個 byte 繼續找
                        buffer.pop(0)
                        continue

                    # 有 header，看看是否至少 16 bytes
                    if len(buffer) < 16:
                        break

                    frame = bytes(buffer[0:16])

                    # 檢查 CRC
                    crc_received_low = frame[14]
                    crc_received_high = frame[15]
                    crc_received = crc_received_low | (crc_received_high << 8)

                    crc_calculated = compute_modbus_crc(frame[0:14])
                    if crc_calculated != crc_received:
                        # CRC 錯誤，丟 1 byte 往後對齊重新找 header
                        self.get_logger().warn(
                            f"CRC mismatch: received=0x{crc_received:04X}, "
                            f"calculated=0x{crc_calculated:04X}"
                        )
                        buffer.pop(0)
                        continue

                    # 解析 data1..6 (little-endian int16)
                    data_bytes = frame[2:14]
                    raw_values = struct.unpack("<6h", data_bytes)
                    fx_raw, fy_raw, fz_raw, mx_raw, my_raw, mz_raw = raw_values

                    fx = fx_raw / 100.0
                    fy = fy_raw / 100.0
                    fz = fz_raw / 100.0
                    mx = mx_raw / 1000.0
                    my = my_raw / 1000.0
                    mz = mz_raw / 1000.0

                    fx_f, zi_force[0] = butterworth_filter(fx, zi_force[0], b, a)
                    fy_f, zi_force[1] = butterworth_filter(fy, zi_force[1], b, a)
                    fz_f, zi_force[2] = butterworth_filter(fz, zi_force[2], b, a)
                    tx_f, zi_torque[0] = butterworth_filter(mx, zi_torque[0], b, a)
                    ty_f, zi_torque[1] = butterworth_filter(my, zi_torque[1], b, a)
                    tz_f, zi_torque[2] = butterworth_filter(mz, zi_torque[2], b, a)

                    self._publish_wrench(fx, fy, fz, mx, my, mz)
                    self._publish_wrench_filtered(fx_f, fy_f, fz_f, tx_f, ty_f, tz_f)

                    # 移除已處理的 16 bytes
                    del buffer[0:16]

            except serial.SerialException as exc:
                self.get_logger().error(f"Serial error: {exc}")
                time.sleep(0.1)
            except Exception as exc:
                self.get_logger().error(f"Unexpected error in stream loop: {exc}")
                time.sleep(0.1)

        self.get_logger().info("Data stream loop exiting.")

    def _publish_wrench(self, fx, fy, fz, mx, my, mz):
        message = WrenchStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "robotiq_ft_frame_id"

        message.wrench.force.x = fx
        message.wrench.force.y = fy
        message.wrench.force.z = fz

        message.wrench.torque.x = mx
        message.wrench.torque.y = my
        message.wrench.torque.z = mz

        self.wrench_publisher.publish(message)

    def _publish_wrench_filtered(self, fx, fy, fz, mx, my, mz):
        message = WrenchStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "robotiq_ft_frame_id"

        message.wrench.force.x = fx
        message.wrench.force.y = fy
        message.wrench.force.z = fz

        message.wrench.torque.x = mx
        message.wrench.torque.y = my
        message.wrench.torque.z = mz

        self.wrench_filtered_publisher.publish(message)

    def destroy_node(self):
        self.should_exit = True
        if hasattr(self, "reader_thread") and self.reader_thread.is_alive():
            self.get_logger().info("Waiting for data stream thread to exit...")
            self.reader_thread.join(timeout=1.0) # Wait for the thread to finish

        time.sleep(0.1)
        if self.serial_port and self.serial_port.is_open:
            try:
                # 結束前送 0xFF 停止 data stream（非必要，但乾淨）
                self.serial_port.write(b"\xff" * 50)
                self.serial_port.flush()
            except Exception:
                pass
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Ft300StreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals() and rclpy.ok():
            node.get_logger().info("Shutting down...")
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
