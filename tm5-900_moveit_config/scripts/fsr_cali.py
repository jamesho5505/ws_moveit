import serial, time, csv, threading, re, sys, signal

# ==== 配置 ====
LOAD_PORT = "/dev/ttyUSB0"     # JS-300
LOAD_BAUD = 9600
LOAD_BYTESIZE = serial.SEVENBITS
LOAD_PARITY   = serial.PARITY_ODD
LOAD_STOPBITS = serial.STOPBITS_TWO

FSR_PORT  = "/dev/ttyUSB1"     # ESP32
FSR_BAUD  = 115200

CSV_PATH  = "calib_log.csv"

# 分壓與供電，用於 Vout/G 換算
VCC = 3.236  # 3.236   # 3.308
Rf1 = 3554.0 # 3610.0  # 614.1  
Rf2 = 3860.0 # 3860.0  # 649.0 

# ==== 共享狀態 ====
last_load = {"t": None, "g": None, "unit": "", "st": ""}
last_fsr  = {
    "t": None,
    "adc1": None, "v1": None, "g1": None,
    "adc2": None, "v2": None, "g2": None,
}

lock = threading.Lock()
stop_flag = False

# ==== JS-300 幀解析 ====
def parse_frame(b: bytes):
    # 13 bytes: 0x02 ... 0x0D
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
    return -grams, unit_str, status_chr  # JS-300 拉力向上給負號時，改成正的在這裡調整

# ==== 線程：讀取 JS-300 ====
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

# ==== 線程：讀取 ESP32（每行：adc1,adc2） ====
FSR_RE = re.compile(r"^\s*(-?\d+(?:\.\d+)?)[,\s]+\s*(-?\d+(?:\.\d+)?)\s*$")

def calc_vout_and_G(adc, Rf):
    if adc is None:
        return None, None
    # 夾在 0..4095
    adc = max(0.0, min(4095.0, float(adc)))
    vout = (adc/4095.0)*VCC
    # 飽和或不合法直接回傳
    if vout <= 0.0 or vout >= VCC:
        return vout, None
    G = vout/(Rf*(VCC - vout))
    return vout, G

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
            m = FSR_RE.search(text)
            if not m:
                continue
            try:
                a1 = float(m.group(1))
                a2 = float(m.group(2))
            except ValueError:
                continue
            # 若韌體用 u16，請在 ESP32 端已轉 12bit；若未轉，可在此補轉：
            # a1 *= (4095.0/65535.0); a2 *= (4095.0/65535.0)

            v1, g1 = calc_vout_and_G(a1, Rf1)
            v2, g2 = calc_vout_and_G(a2, Rf2)
            t = time.time()
            with lock:
                last_fsr.update({
                    "t": t,
                    "adc1": a1, "v1": v1, "g1": g1,
                    "adc2": a2, "v2": v2, "g2": g2,
                })
    finally:
        ser.close()

# ==== 主程式：合併寫檔 ====
def main():
    global stop_flag
    def handle_sigint(signum, frame):
        global stop_flag
        stop_flag = True
    signal.signal(signal.SIGINT, handle_sigint)

    t0 = time.time()
    th1 = threading.Thread(target=load_thread, daemon=True)
    th2 = threading.Thread(target=fsr_thread,  daemon=True)
    th1.start(); th2.start()

    with open(CSV_PATH, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "t_s",
            "loadcell_g", "unit", "status",
            "fsr1_adc", "fsr1_vout_V", "fsr1_G_Siemens",
            "fsr2_adc", "fsr2_vout_V", "fsr2_G_Siemens",
        ])
        print("logging to", CSV_PATH, file=sys.stderr)
        last_written = (None, None)  # (t_load, t_fsr)

        while not stop_flag:
            time.sleep(0.02)  # 50 Hz
            with lock:
                tl = last_load["t"]; tf = last_fsr["t"]
                gl = last_load["g"]; un = last_load["unit"]; st = last_load["st"]
                a1 = last_fsr["adc1"]; v1 = last_fsr["v1"]; g1 = last_fsr["g1"]
                a2 = last_fsr["adc2"]; v2 = last_fsr["v2"]; g2 = last_fsr["g2"]

            # 任一來源有新資料就寫一列，帶入對方最新已知
            if (tl, tf) != last_written and (tl is not None or tf is not None):
                t_now = time.time() - t0
                w.writerow([
                    f"{t_now:.3f}",
                    f"{gl:.3f}" if gl is not None else "", un or "", st or "",
                    f"{a1:.0f}" if a1 is not None else "",
                    f"{v1:.6f}" if v1 is not None else "",
                    f"{g1:.9e}" if g1 is not None else "",
                    f"{a2:.0f}" if a2 is not None else "",
                    f"{v2:.6f}" if v2 is not None else "",
                    f"{g2:.9e}" if g2 is not None else "",
                ])
                f.flush()
                print(
                    f"{t_now:8.3f}s  "
                    f"{(gl if gl is not None else '')!s:>10} g  "
                    f"FSR1 adc={a1 if a1 is not None else '':>4}  "
                    f"V={f'{v1:.4f}' if v1 is not None else '':>6}  "
                    f"G={f'{g1:.3e}' if g1 is not None else '':>10}  |  "
                    f"FSR2 adc={a2 if a2 is not None else '':>4}  "
                    f"V={f'{v2:.4f}' if v2 is not None else '':>6}  "
                    f"G={f'{g2:.3e}' if g2 is not None else '':>10}  "
                    f"st={st or ''}"
                )
                last_written = (tl, tf)

    print("\nstopping...", file=sys.stderr)

if __name__ == "__main__":
    main()
