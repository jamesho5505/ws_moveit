#!/usr/bin/env python3
# record_force_pair.py
import rclpy, threading, time, csv, serial
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# ---- load cell 解析（仿你 loadcell.py）----
def parse_frame(b: bytes):
    # 期望: 13 bytes, 0x02 ... 0x0D
    if len(b) != 13 or b[0] != 0x02 or b[-1] != 0x0D:
        return None
    sign_chr   = chr(b[1])
    num_str    = b[2:9].decode('ascii')        # 含空白與小數點
    unit_str   = b[9:11].decode('ascii').strip()
    status_chr = chr(b[11])                    # ' ' 或 'G'
    val = float(num_str.replace(' ', '0'))
    if sign_chr == '-':
        val = -val
    u = unit_str.lower()
    if u == 'g':   grams = val
    elif u == 'kg': grams = val*1000.0
    elif u == 'lb': grams = val*453.59237
    elif u == 'nt': grams = val*1000.0/9.80665
    elif u == 'kn': grams = val*1_000_000.0/9.80665
    else:          grams = val
    return -grams, unit_str, status_chr  # 面板受力方向相反，取負

class PairLogger(Node):
    def __init__(self,
                 topic='/esp32/force',
                 force_idx=2,
                 port='/dev/ttyUSB1',
                 baud=9600,
                 out_csv='force_pair_log.csv'):
        super().__init__('pair_logger')
        self.force_idx = force_idx
        self.F_fsr = float('nan')
        self.F_lc  = float('nan')
        self.last_unit = ''
        self.last_status = ' '
        self.t0 = time.time()

        # ROS 訂閱 ESP32 合力
        self.create_subscription(Float32MultiArray, topic, self.cb_force, 10)

        # 開 serial（JS-300 常見 7E2）
        self.ser = serial.Serial(
            port, baud,
            bytesize=serial.SEVENBITS,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_TWO,
            timeout=0.02
        )
        self.buf = bytearray()

        # 寫 CSV
        self.fh = open(out_csv, 'w', newline='')
        self.w = csv.writer(self.fh)
        self.w.writerow(['t_s','fsr_total_g','loadcell_g','loadcell_unit','loadcell_status'])

        # 定時器：讀 load cell 並落盤
        self.timer = self.create_timer(0.02, self.step)  # 50 Hz 記錄

        # 背景執行緒：快取 serial 原始資料
        self.stop_evt = threading.Event()
        self.th = threading.Thread(target=self._serial_pump, daemon=True)
        self.th.start()

        self.get_logger().info('pair logger started')

    def cb_force(self, msg: Float32MultiArray):
        if not msg.data:
            return
        i = 2
        if -len(msg.data) <= i < len(msg.data):
            self.F_fsr = float(msg.data[i])

    def _serial_pump(self):
        # 收集完整幀後即時更新 F_lc
        while not self.stop_evt.is_set():
            b = self.ser.read(1)
            if not b:
                continue
            c = b[0]
            if c == 0x02:               # STX
                self.buf = bytearray([c])
            else:
                self.buf.append(c)
                if c == 0x0D:           # CR
                    res = parse_frame(bytes(self.buf))
                    if res:
                        grams, unit, st = res
                        self.F_lc = grams
                        self.last_unit = unit
                        self.last_status = st
                    self.buf.clear()

    def step(self):
        t = time.time() - self.t0
        # 寫一行。兩邊都可能是 NaN，照實記錄。
        self.w.writerow([
            f'{t:.3f}',
            f'{self.F_fsr:.3f}',
            f'{self.F_lc:.3f}',
            self.last_unit,
            self.last_status
        ])

    def destroy_node(self):
        try:
            self.stop_evt.set()
            if hasattr(self, 'th'): self.th.join(timeout=0.5)
        except Exception:
            pass
        try:
            if hasattr(self, 'fh'): self.fh.close()
        except Exception:
            pass
        try:
            if hasattr(self, 'ser'): self.ser.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/esp32/force')
    ap.add_argument('--force_idx', type=int, default=2)      # 0=total
    ap.add_argument('--port', default='/dev/ttyUSB1')
    ap.add_argument('--baud', type=int, default=9600)
    ap.add_argument('--out', default='force_pair_log.csv')
    args = ap.parse_args()

    rclpy.init()
    node = PairLogger(args.topic, args.force_idx, args.port, args.baud, args.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
