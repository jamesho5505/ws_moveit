import serial, time, csv, threading, re, sys

# ==== 配置 ====
LOAD_PORT = "/dev/ttyUSB1"     # JS-300 那個埠
LOAD_BAUD = 9600
# JS-300 參數：7E2 或 7O2，依你的面板。你原碼用 7O2（ODD）。
LOAD_BYTESIZE = serial.SEVENBITS
LOAD_PARITY   = serial.PARITY_ODD
LOAD_STOPBITS = serial.STOPBITS_TWO

FSR_PORT  = "/dev/ttyUSB0"     # ESP32 那個埠
FSR_BAUD  = 115200

CSV_PATH  = "calib_log.csv"

# 若要同時換算 vout/G，填入供電與分壓電阻
VCC = 3.236
# Rf  = 9500.0
Rf1  = 3870.0
Rf2  = 5120.0

# ==== 共享狀態 ====
last_load = {"t": None, "g": None, "unit": "", "st": ""}
last_fsr  = {"t": None, "adc1": None, "adc2": None}

lock = threading.Lock()
stop_flag = False

# ==== 解析 JS-300 幀 ====
def parse_frame(b: bytes):
    # 期待 13 bytes: 0x02 ... 0x0D
    if len(b)!=13 or b[0]!=0x02 or b[-1]!=0x0D:
        return None
    sign_chr   = chr(b[1])
    num_str    = b[2:9].decode('ascii')      # 含空白/小數點
    unit_str   = b[9:11].decode('ascii').strip()
    status_chr = chr(b[11])                   # ' ' 或 'G'
    val = float(num_str.replace(' ', '0'))
    if sign_chr == '-':
        val = -val
    unit = unit_str.lower()
    if unit == 'g':
        grams = val
    elif unit == 'kg':
        grams = val*1000.0
    elif unit == 'lb':
        grams = val*453.59237
    elif unit == 'nt':
        grams = val*1000.0/9.80665
    elif unit == 'kn':
        grams = val*1_000_000.0/9.80665
    else:
        grams = val
    return -grams, unit_str, status_chr

# ==== 線程：讀取 load cell ====
def load_thread():
    global stop_flag
    ser = serial.Serial(
        LOAD_PORT, LOAD_BAUD,
        bytesize=LOAD_BYTESIZE, parity=LOAD_PARITY, stopbits=LOAD_STOPBITS,
        timeout=1.0
    )
    buf = bytearray()
    try:
        while not stop_flag:
            ch = ser.read(1)
            if not ch:
                continue
            c = ch[0]
            if c == 0x02:
                buf = bytearray([c])
            else:
                buf.append(c)
                if c == 0x0D:
                    res = parse_frame(bytes(buf))
                    buf.clear()
                    if res:
                        grams, unit, st = res
                        t = time.time()
                        with lock:
                            last_load.update({"t": t, "g": grams, "unit": unit, "st": st})
    finally:
        ser.close()

# ==== 線程：讀取 ESP32（ADC: 1234） ====
ADC_RE = re.compile(r"(\d+),(\d+)")
def fsr_thread():
    global stop_flag
    ser = serial.Serial(FSR_PORT, FSR_BAUD, timeout=1.0)
    ser.reset_input_buffer()
    try:
        while not stop_flag:
            line = ser.readline()
            if not line:
                continue
            try:
                text = line.decode('utf-8', errors='ignore').strip()
            except:
                continue
            m = ADC_RE.search(text)
            if m:
                adc1 = int(m.group(1))
                adc2 = int(m.group(2))
                t = time.time()
                with lock:
                    last_fsr.update({"t": t, "adc1": adc1, "adc2": adc2})
    finally:
        ser.close()

# ==== 主程式：合併寫檔 ====
def calc_vout_and_G(adc, Rf):
    if adc is None: return None, None
    vout = (adc/4095.0)*VCC
    if vout<=0.0 or vout>=VCC: return vout, None
    G = vout/(Rf*(VCC - vout))
    return vout, G

def main():
    global stop_flag
    t0 = time.time()
    th1 = threading.Thread(target=load_thread, daemon=True)
    th2 = threading.Thread(target=fsr_thread,  daemon=True)
    th1.start(); th2.start()

    with open(CSV_PATH, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "t_s", "grams_g", "unit", "status",
            "adc1_cnt", "vout1_V", "G1_S",
            "adc2_cnt", "vout2_V", "G2_S"
        ])
        print("logging to", CSV_PATH, file=sys.stderr)
        last_written = (None, None)  # (t_load, t_fsr)
        try:
            while True:
                time.sleep(0.02)  # 50 Hz 迴圈
                with lock:
                    tl = last_load["t"]; tf = last_fsr["t"]
                    gl = last_load["g"]; un = last_load["unit"]; st = last_load["st"]
                    adc1 = last_fsr["adc1"]
                    adc2 = last_fsr["adc2"]
                # 只要任一來源有新資料就寫一列，帶入「最新已知」的另一來源
                if (tl, tf) != last_written and (tl is not None or tf is not None):
                    vout1, G1 = calc_vout_and_G(adc1, Rf1)
                    vout2, G2 = calc_vout_and_G(adc2, Rf2)
                    t_now = time.time() - t0
                    w.writerow([
                        f"{t_now:.3f}",
                        f"{gl:.3f}" if gl is not None else "",
                        un if un is not None else "",
                        st if st is not None else "",
                        adc1 if adc1 is not None else "",f"{vout1:.5f}" if vout1 is not None else "",
                        f"{G1:.8e}" if G1 is not None else "",
                        adc2 if adc2 is not None else "",
                        f"{vout2:.5f}" if vout2 is not None else "",
                        f"{G2:.8e}" if G2 is not None else ""
                    ])
                    # 建議同步在終端看一眼
                    print(f"{t_now:8.3f}s  {gl if gl is not None else '':>10} g  "
                          f"ADC1={adc1 if adc1 is not None else '':>4}  "
                          f"ADC2={adc2 if adc2 is not None else '':>4}  "
                          f"st={st if st is not None else ''}")
                    last_written = (tl, tf)
        except KeyboardInterrupt:
            stop_flag = True
            print("\nstopping...", file=sys.stderr)

if __name__ == "__main__":
    main()
