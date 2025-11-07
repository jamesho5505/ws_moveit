# pip install pyserial
import serial, time, csv

PORT = "/dev/ttyUSB1"      # 你偵測到的 load cell
BAUD = 9600                # 與 JS-300 面板設定一致
ser = serial.Serial(
    PORT, BAUD,
    bytesize=serial.SEVENBITS,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    timeout=1.0
)

def parse_frame(b: bytes):
    # 期待 0x02 ... 0x0D，共 13 bytes
    if len(b) != 13 or b[0] != 0x02 or b[-1] != 0x0D:
        return None
    sign_chr   = chr(b[1])
    num_str    = b[2:9].decode('ascii')    # 含空白與小數點
    unit_str   = b[9:11].decode('ascii').strip()
    status_chr = chr(b[11])                # ' ' 或 'G'
    # 數值：空白代表未顯示之 0
    val = float(num_str.replace(' ', '0'))
    if sign_chr == '-':
        val = -val
    # 單位轉成「克」
    unit = unit_str.lower()
    if unit == 'g':
        grams = val
    elif unit == 'kg':
        grams = val * 1000.0
    elif unit == 'lb':
        grams = val * 453.59237
    elif unit == 'nt':      # 牛頓 → g，近似除以 9.80665
        grams = val * 1000.0 / 9.80665
    elif unit == 'kn':
        grams = val * 1000.0 * 1000.0 / 9.80665
    else:
        grams = val  # 不識別就原值
    return -grams, unit_str, status_chr

with open("loadcell_log.csv", "w", newline="") as f:
    w = csv.writer(f); w.writerow(["t_s","grams","unit","status"])
    buf = bytearray()
    t0 = time.time()
    while True:
        ch = ser.read(1)
        if not ch:
            continue
        c = ch[0]
        if c == 0x02:           # STX
            buf = bytearray([c])
        else:
            buf.append(c)
            if c == 0x0D:       # CR
                res = parse_frame(bytes(buf))
                if res:
                    grams, unit, st = res
                    w.writerow([time.time()-t0, f"{grams:.3f}", unit, st])
                    print(f"{grams:.3f} g  [{unit}]  status={st}")
                buf.clear()
