import serial, time, signal, sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

PORT      = '/dev/ttyUSB0'
BAUD      = 921_600
HIST_LEN  = 15000         # 畫面上保留 n 筆歷史
YLIM_F    = (-1200, 1200)
YLIM_M    = (-50,  50)

LABELS = ['Fx','Fy','Fz','Mx','My','Mz']
COLORS = ['tab:blue','tab:green','tab:red',
          'tab:cyan','tab:purple','tab:olive']
HEXSET = set(b'0123456789ABCDEF')

# ---------- serial ----------
ser = serial.Serial(PORT, BAUD, timeout=1.0)
ser.reset_input_buffer()
ser.write(b'S')               # 進入連續輸出模式

# ---------- Tk / MPL ----------
plt.style.use('seaborn-whitegrid')
fig, axes = plt.subplots(3, 2, figsize=(10, 7))
fig.canvas.manager.set_window_title("FT‑6 live viewer")

history = {k: deque([0]*HIST_LEN, maxlen=HIST_LEN) for k in LABELS}
lines   = []

for ax, lab, col in zip(axes.flat, LABELS, COLORS):
    ln, = ax.plot(history[lab], col)
    lines.append(ln)
    ax.set_title(lab)
    ax.set_xlim(0, HIST_LEN-1)
    ax.set_ylim(*(YLIM_F if lab.startswith('F') else YLIM_M))
    ax.grid(True)

fig.tight_layout()

# ---------- helpers ----------
def valid(pkt: bytes) -> bool:
    # 25 個 ASCII HEX (首位 0~9) + CR LF
    return (
        len(pkt) == 27 and
        pkt[0:25][-2:] != b'\r\n' and      # 確保 25 bytes 內容本身沒 CRLF
        pkt[25:]   == b'\r\n' and
        pkt[0]     in b'0123456789' and
        all(c in HEXSET for c in pkt[:25])
    )

def to_values(pkt25: bytes):
    # pkt25: 25 為 HEX 字元 (不含 CRLF)
    nums = [int(pkt25[1+4*i : 1+4*(i+1)], 16) for i in range(6)]
    fx = (nums[0] - 16384) / 17.0
    fy = (nums[1] - 16384) / 17.0
    fz = (nums[2] - 16384) / 12.0
    mx = (nums[3] - 16384) / 400.0
    my = (nums[4] - 16384) / 400.0
    mz = (nums[5] - 16384) / 400.0
    return (fx, fy, fz, mx, my, mz)

buf = bytearray()

def update(_):
    # 讀取序列埠
    if ser.in_waiting:
        buf.extend(ser.read(ser.in_waiting))

    # 以 CRLF 為切片
    while True:
        lf = buf.find(b'\n')
        if lf == -1:
            break                        # 還沒湊到一行
        line = bytes(buf[:lf+1])         # 含 LF
        del buf[:lf+1]

        if not valid(line):
            continue                     # 格式錯，丟掉

        vals = to_values(line[:25])
        print(" | ".join(f"{l}={v:.2f}" for l, v in zip(LABELS, vals)))

        # 更新資料
        for ln, lab, v in zip(lines, LABELS, vals):
            history[lab].append(v)
            ln.set_ydata(history[lab])

    return lines                         # for blit

ani = FuncAnimation(fig, update, interval=20, blit=True)

# ---------- clean exit ----------
def _quit(sig, frame):
    print("\nCaught Ctrl‑C, leaving Sequential mode…")
    try:
        ser.write(b'E')
    finally:
        ser.close()
        plt.close('all')
        sys.exit(0)

signal.signal(signal.SIGINT, _quit)

plt.show()