# Visualization of 3-frame transforms: base → flange (ZYX Euler), flange → sensor (identity)
# Pure NumPy + Matplotlib (no SciPy). Edit the angles or pick an index from R_list.
import os
import numpy as np
import matplotlib.pyplot as plt

save_dir = "frames_visualization"
os.makedirs(save_dir, exist_ok=True)
img_paths = []
F_meas_list = np.load("/home/jamesho5055/ws_moveit/1104_2/F_meas.npy")
F_comp_list = np.load("/home/jamesho5055/ws_moveit/1104_2/F_comp_list.npy")

# ------------------ helpers ------------------
def Rx(deg):
    a = np.deg2rad(deg)
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[1,0,0],[0,ca,-sa],[0,sa,ca]], dtype=float)

def Ry(deg):
    a = np.deg2rad(deg)
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[ca,0,sa],[0,1,0],[-sa,0,ca]], dtype=float)

def Rz(deg):
    a = np.deg2rad(deg)
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[ca,-sa,0],[sa,ca,0],[0,0,1]], dtype=float)

def R_from_euler_zyx(z_deg, y_deg, x_deg):
    # base → flange using ZYX convention: R = Rz(z) @ Ry(y) @ Rx(x)
    return Rz(z_deg) @ Ry(y_deg) @ Rx(x_deg)

def draw_frame(ax, R, t, label, L=0.12):
    # Draw 3 axis lines starting at origin t with directions = columns of R
    o = np.array(t, dtype=float).reshape(3)
    colors = ['r', 'g', 'b']
    for i, ch in enumerate("xyz"):
        v = R[:, i] * L
        ax.plot([o[0], o[0]+v[0]], [o[1], o[1]+v[1]], [o[2], o[2]+v[2]], color=colors[i], linewidth=2)
        # mark the axis end
        ax.scatter([o[0]+v[0]], [o[1]+v[1]], [o[2]+v[2]], color=colors[i])
    ax.text(o[0], o[1], o[2], f"{label}", fontsize=10)
    return

def set_axes_equal(ax):
    # Make 3D plot axes have equal scale
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    spans = limits[:,1] - limits[:,0]
    centers = np.mean(limits, axis=1)
    radius = 0.5 * max(spans)
    ax.set_xlim3d([centers[0]-radius, centers[0]+radius])
    ax.set_ylim3d([centers[1]-radius, centers[1]+radius])
    ax.set_zlim3d([centers[2]-radius, centers[2]+radius])

# ------------------ provided list ------------------
R_list = [
    [[0,0,1],[1,0,0],[0,1,0]],   [[0,0,-1],[1,0,0],[0,-1,0]],
    [[0,-1,0],[1,0,0],[0,0,1]],  [[0,1,0],[1,0,0],[0,0,-1]],
    [[1,0,0],[0,0,-1],[0,1,0]],  [[1,0,0],[0,0,1],[0,-1,0]],
    [[1,0,0],[0,1,0],[0,0,1]],   [[1,0,0],[0,-1,0],[0,0,-1]],
    [[0,0,1],[0,1,0],[-1,0,0]],  [[0,0,-1],[0,1,0],[1,0,0]],
    [[-1,0,0],[0,-1,0],[0,0,1]], [[-1,0,0],[0,1,0],[0,0,-1]],
    [[0,1,0],[0,0,-1],[-1,0,0]], [[0,1,0],[0,0,1],[1,0,0]],
    [[0,1,0],[-1,0,0],[0,0,1]],  [[0,1,0],[1,0,0],[0,0,-1]],
    [[0,0,1],[0,1,0],[-1,0,0]],  [[0,0,1],[0,-1,0],[1,0,0]],
    [[0,0,1],[1,0,0],[0,1,0]],   [[0,0,1],[-1,0,0],[0,-1,0]],
    [[0,-1,0],[0,0,1],[-1,0,0]], [[0,1,0],[0,0,1],[1,0,0]],
    [[1,0,0],[0,0,1],[0,-1,0]],  [[-1,0,0],[0,0,1],[0,1,0]],
]
R_list = [np.array(Ri, dtype=float) for Ri in R_list]

# ------------------ choose orientation ------------------
# Option A: specify Euler ZYX angles for base→flange
z_deg, y_deg, x_deg = 30.0, 20.0, 10.0  # edit as needed
R_b2f_from_euler = R_from_euler_zyx(z_deg, y_deg, x_deg)

Fb = np.array([-0.75975371, -0.01364686, -9.55280899])


# Option B: use one of the provided 24 rotations as base→flange
use_R_list = True
for idx in range(len(R_list)):

    R_b2f = R_list[idx] if use_R_list else R_b2f_from_euler

    # flange→sensor is identity
    R_f2s = np.eye(3)

    # compose to get base→sensor
    R_b2s = R_b2f @ R_f2s

    # ------------------ translation vectors ------------------
    t_b2f = np.array([0.2, 0, 0.5])   # base→flange 平移
    t_f2s = np.array([0.0, 0.0, 0.5])  # flange→sensor 平移

    # ----- theoretical gravity direction -----
    g_b = np.array([0.0, 0.0, -1.0])      # unit gravity in base
    g_s = R_b2s.T @ g_b                   # gravity direction expressed in sensor
    g_s = g_s / np.linalg.norm(g_s)       # ensure unit
    g_dir_in_base = R_b2s @ g_s           # 轉回 base 來畫箭頭（這會等於 g_b）

    # ------------------ plot ------------------
    fig = plt.figure(figsize=(6,6))
    ax = fig.add_subplot(111, projection='3d')



    # draw frames (origins all the same here; set offsets if you want translations)
    origin = np.zeros(3)
    origin_flange = t_b2f
    origin_sensor = t_b2f + R_b2f @ t_f2s
    draw_frame(ax, np.eye(3), origin, "base")
    draw_frame(ax, R_b2f, origin_flange, "flange")
    draw_frame(ax, R_b2s, origin_sensor, "sensor")
    
    # 加上力量箭頭（以 sensor 原點為起點）
    
    F_meas = F_meas_list[idx]
    F_meas = F_meas_list[idx]              # in sensor
    F0_s = F_meas - Fb                     # 去偏，仍在 sensor
    sgn = 1.0 if np.dot(F0_s/np.linalg.norm(F0_s), -g_s) >= 0 else -1.0
    L = 0.2

    F_res_s = F_comp_list[idx]                 # in sensor
    F_res_b = R_b2s @ F_res_s                  # to base
    F_par_s = np.dot(F_res_s, g_s) * g_s
    F_perp_s = F_res_s - F_par_s

    ax.quiver(origin_sensor[0], origin_sensor[1], origin_sensor[2],
          F_res_b[0], F_res_b[1], F_res_b[2],
          length=0.1, normalize=False, linewidth=3, color='m', label='F_res')
    ax.text(origin_sensor[0], origin_sensor[1], origin_sensor[2]-0.08,
        f"|F_res|={np.linalg.norm(F_res_s):.2f} N", fontsize=8)

    F_comp = F_comp_list[idx]
    F_comp_b = R_b2s @ F_comp              # to base
    scale = 0.25 / (np.linalg.norm(F_comp_b) + 1e-9)  # 讓箭頭不會太長
    # ax.quiver(origin_sensor[0], origin_sensor[1], origin_sensor[2],
    #         (F_comp_b*scale)[0], (F_comp_b*scale)[1], (F_comp_b*scale)[2],
    #         length=1.0, normalize=False, linewidth=2, color='m', label='F_comp (magnitude)')

    # ax.quiver(
    #     origin_sensor[0], origin_sensor[1], origin_sensor[2],
    #     F_comp[0], F_comp[1], F_comp[2],
    #     color='m', length=L, normalize=True, linewidth=5, label='F_comp'
    # )

    # ax.quiver(
    #     origin_sensor[0], origin_sensor[1], origin_sensor[2],
    #     F_meas[0], F_meas[1], F_meas[2],
    #     color='c', length=L, normalize=True, linewidth=5, label='F_meas'
    # )

    ax.quiver(origin_sensor[0], origin_sensor[1], origin_sensor[2],
          g_dir_in_base[0], g_dir_in_base[1], g_dir_in_base[2],
          length=L, normalize=True, linewidth=3, color='k', label='gravity dir')
    # 在圖旁標出重力在 sensor 座標的分量，方便核對
    ax.text(origin_sensor[0], origin_sensor[1] + 0.05, origin_sensor[2] + 0.05,
            f"g_s = [{g_s[0]:.3f}, {g_s[1]:.3f}, {g_s[2]:.3f}]", fontsize=8, color='k')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    ax.set_title(f"Frames: base (I), flange (R_b2f), sensor (R_b2s)", fontsize=10)

    # set viewing angle
    ax.view_init(elev=22, azim=35)
    set_axes_equal(ax)
    # plt.show()
    plt.tight_layout()
    img_path = os.path.join(save_dir, f"frame_{idx+1:02d}.png")
    plt.savefig(img_path)
    img_paths.append(img_path)
    # plt.show(block=False)
    # plt.pause(0.2)  # 顯示0.5秒
    plt.close(fig)

   

    
    # print(np.linalg.norm(g_s), g_s)

# import numpy as np
# from scipy.spatial.transform import Rotation as R

# # 把論文的 24 個 R_i 按順序放進來（你也可沿用現成的 R_list）
# R_list = [
#     [[0,0,1],[1,0,0],[0,1,0]],   [[0,0,-1],[1,0,0],[0,-1,0]],
#     [[0,-1,0],[1,0,0],[0,0,1]],  [[0,1,0],[1,0,0],[0,0,-1]],
#     [[1,0,0],[0,0,-1],[0,1,0]],  [[1,0,0],[0,0,1],[0,-1,0]],
#     [[1,0,0],[0,1,0],[0,0,1]],   [[1,0,0],[0,-1,0],[0,0,-1]],
#     [[0,0,1],[0,1,0],[-1,0,0]],  [[0,0,-1],[0,1,0],[1,0,0]],
#     [[-1,0,0],[0,-1,0],[0,0,1]], [[-1,0,0],[0,1,0],[0,0,-1]],
#     [[0,1,0],[0,0,-1],[-1,0,0]], [[0,1,0],[0,0,1],[1,0,0]],
#     [[0,1,0],[-1,0,0],[0,0,1]],  [[0,1,0],[1,0,0],[0,0,-1]],
#     [[0,0,1],[0,1,0],[-1,0,0]],  [[0,0,1],[0,-1,0],[1,0,0]],
#     [[0,0,1],[1,0,0],[0,1,0]],   [[0,0,1],[-1,0,0],[0,-1,0]],
#     [[0,-1,0],[0,0,1],[-1,0,0]], [[0,1,0],[0,0,1],[1,0,0]],
#     [[1,0,0],[0,0,1],[0,-1,0]],  [[-1,0,0],[0,0,1],[0,1,0]],
# ]

# # 轉為 Euler (XYZ) = [Rx, Ry, Rz]，單位：度
# eulers_deg = []
# for Ri in R_list:
#     ang = R.from_matrix(np.array(Ri)).as_euler('xyz', degrees=True)
#     # 讓角度落在 [-180,180)
#     ang = (ang + 180.0) % 360.0 - 180.0
#     eulers_deg.append(ang)

# eulers_deg = np.array(eulers_deg)
# for i, (rx,ry,rz) in enumerate(eulers_deg, 1):
#     print(f"R{i:02d}: Rx={rx:7.2f}°, Ry={ry:7.2f}°, Rz={rz:7.2f}°")

