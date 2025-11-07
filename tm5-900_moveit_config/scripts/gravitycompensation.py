import numpy as np
import csv
import matplotlib.pyplot as plt
import math

# 每組姿態分別儲存
Fx, Fy, Fz = [], [], []
Tx, Ty, Tz = [], [], []

for i in range(1, 25):  # 若有 R_1.csv ~ R_24.csv，要到 15
    file = f"/home/jamesho5055/ws_moveit/R{i}.csv"
    # file = f"wenyu/R{i}.csv"
    fxi, fyi, fzi = [], [], []
    txi, tyi, tzi = [], [], []
    with open(file, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        count = 0
        for row in reader:
            # if count >= 500:
            #     break
            fx, fy, fz, tx, ty, tz = map(float, row[1:])
            fxi.append(fx)
            fyi.append(fy)
            fzi.append(fz)
            txi.append(tx)
            tyi.append(ty)
            tzi.append(tz)
            # count += 1

    # 每組取平均代表該姿態的靜態量測
    Fx.append(np.mean(fxi))
    Fy.append(np.mean(fyi))
    Fz.append(np.mean(fzi))
    Tx.append(np.mean(txi))
    Ty.append(np.mean(tyi))
    Tz.append(np.mean(tzi))
    if i == 1:
        Fb_soft = np.array([np.median(fxi), np.median(fyi), np.median(fzi)])
        Tb_soft = np.array([np.median(txi), np.median(tyi), np.median(tzi)])

# 轉 numpy 陣列 (24,3)
F_meas = np.column_stack([Fx, Fy, Fz])
Tau_meas = np.column_stack([Tx, Ty, Tz])
# print("F_meas:", F_meas)
# F_meas_z = F_meas - Fb_soft
# Tau_meas_z = Tau_meas - Tb_soft


# 計算重力補償參數
def estimate_params(F_meas, Tau_meas, R_list, g=9.80665):
    # F_meas: (K,3), Tau_meas: (K,3), R_list: list of (3,3)
    gI = np.array([0,0,-g])
    G = np.stack([R.T @ gI for R in R_list], axis=0)  # (K,3)

    # 力三軸帶截距回歸，解 b 和 m（共用同一 m，單一 m 用合併法最穩）
    # 合併三軸：F = 1*b + m*G
    X = np.hstack([np.ones((G.shape[0]*3,1)),
                   np.reshape(G, (-1,1), order='F')]) # (3K,2)
    y = F_meas.reshape(-1, order='F')               # (3K,)
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    b_scalar = beta[0]         # 平均偏差投到三軸會有殘差，下面再單軸修正
    # m = beta[1]
    m = 1.040

    # 更精細：各軸分別帶截距以取 b_x,b_y,b_z（m 固定）
    b = []
    for axis in range(3):
        Xi = np.column_stack([np.ones((G.shape[0],)), G[:,axis]])
        yi = F_meas[:,axis]
        bi, _mi = np.linalg.lstsq(Xi, yi, rcond=None)[0]
        b.append(bi)
    Fb = np.array(b)

    # 去偏之力
    F_tilde = F_meas - Fb

    # 力矩偏差：帶截距回歸 tau = tau_b + m*(r × g)
    # 先用未知 r 的線性型式 tau = tau_b + m*A_i r
    A_blocks = []
    tau_stack = []
    for i in range(G.shape[0]):
        gx,gy,gz = G[i]
        Ai = np.array([[0,-gz, gy],
                       [gz, 0,-gx],
                       [-gy,gx, 0]])
        A_blocks.append(Ai)
        tau_stack.append(Tau_meas[i])
    A = np.vstack(A_blocks)             # (3K,3)
    tau_stack = np.vstack(tau_stack)    # (K,3)
    tau_vec = tau_stack.reshape(-1, order='F')  # (3K,)

    # 帶截距解 [tau_b(3), r(3)]
    Z = np.hstack([np.kron(np.ones((G.shape[0],1)), np.eye(3)).reshape(-1,3,order='F'),
                   m*A])                # (3K,6)
    params, *_ = np.linalg.lstsq(Z, tau_vec, rcond=None)
    Tau_b = params[:3]
    r = params[3:]/m

    # r = np.array([-0.0017, -0.000, 0.0717])
    return Fb, Tau_b, m, r

def compensate(F_meas, Tau_meas, R, Fb, Tau_b, m, r, g=9.80665):
    gI = np.array([0,0,-g]); gs = R.T @ gI
    Fg = m*gs
    Tg = np.cross(r, Fg)
    F_contact = F_meas - Fb - Fg
    T_contact = Tau_meas - Tau_b - Tg
    return F_contact, T_contact

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

assert len(R_list) == F_meas.shape[0] == Tau_meas.shape[0]
R_list = [np.array(R) for R in R_list]
Fb, Tau_b, m, r = estimate_params(F_meas, Tau_meas, R_list)
print("Force_bias:", Fb)
print("Tau_bias:", Tau_b)
print("m:", m)
print("r:", r)

for i, R in enumerate(R_list):
    Fc, Tc = compensate(F_meas[i], Tau_meas[i], R, Fb, Tau_b, m, r)
    # Fc, Tc = compensate(F_meas_z[i], Tau_meas_z[i], R, Fb, Tau_b, m, r)
    # print(f"Pose {i+1}:{Fc} |F|={np.linalg.norm(Fc):.4f}, |T|={np.linalg.norm(Tc):.6f}")


F_errors = [np.linalg.norm(compensate(F_meas[i], Tau_meas[i], R, Fb, Tau_b, m, r)[0]) for i,R in enumerate(R_list)]
# F_errors = [np.linalg.norm(compensate(F_meas_z[i], Tau_meas_z[i], R, Fb, Tau_b, m, r)[0]) for i,R in enumerate(R_list)]
plt.bar(range(1,25), F_errors)
plt.xlabel('Pose index')
plt.ylabel('|Residual Force| (N)')
# plt.show()

np.save("/home/jamesho5055/ws_moveit/1104_2/F_meas.npy", F_meas)
F_comp_list = [compensate(F_meas[i], Tau_meas[i], R, Fb, Tau_b, m, r)[0] for i,R in enumerate(R_list)]
np.save("/home/jamesho5055/ws_moveit/1104_2/F_comp_list.npy", F_comp_list)

# F_meas_obj_1 = np.array([0.173,0.420,1.876])
# Tau_meas_obj_1 = np.array([0.023,-0.006,-0.001])
# R_obj_1 = R_list[7]

# F_meas_obj_2 = [1.35, -11.98, -10.145]
# Tau_meas_obj_2 = [1.156, -0.089, 0.136]
# R_obj_2 = R_list[4]   # 夾重物時的旋轉矩陣

# F_meas_obj_3 = [11.62, 1.2, -13.0]
# Tau_meas_obj_3 = [-0.045, 1.089, -0.009]
# R_obj_3 = R_list[12]   # 夾重物時的旋轉矩陣

# F_meas_obj_4 = np.array([0.280,-0.015,1.987])
# Tau_meas_obj_4 = np.array([0.013,-0.01,-0.001])
# R_obj_4 = R_list[7]

# F_meas_obj_5 = [-14.864, -0.389, -8.435]
# Tau_meas_obj_5 = [0.083, -1.065, 0.078]
# R_obj_5 = R_list[13]   # 夾重物時的旋轉矩陣

# F_meas_obj_6 = [1.35, 11.78, -8.389]
# Tau_meas_obj_6 = [1.051, -0.0389, 0.197]
# R_obj_6 = R_list[5]   # 夾重物時的旋轉矩陣

# F_meas_obj_7 = np.array([3.25,0.983,-21.954])
# Tau_meas_obj_7 = np.array([0.261,-0.112,0.203])
# R_obj_7 = R_list[6]

F_meas_obj_1 = np.array([-0.05409,0.09723,1.876])
Tau_meas_obj_1 = np.array([0.023,-0.006,-0.001])
R_obj_1 = R_list[7]

F_meas_obj_2 = [1.35, -11.98, -10.145]
Tau_meas_obj_2 = [1.156, -0.089, 0.136]
R_obj_2 = R_list[4]   # 夾重物時的旋轉矩陣

F_meas_obj_3 = [11.62, 1.2, -13.0]
Tau_meas_obj_3 = [-0.045, 1.089, -0.009]
R_obj_3 = R_list[12]   # 夾重物時的旋轉矩陣

F_meas_obj_4 = np.array([0.280,-0.015,1.987])
Tau_meas_obj_4 = np.array([0.013,-0.01,-0.001])
R_obj_4 = R_list[7]

F_meas_obj_5 = [-14.864, -0.389, -8.435]
Tau_meas_obj_5 = [0.083, -1.065, 0.078]
R_obj_5 = R_list[13]   # 夾重物時的旋轉矩陣

F_meas_obj_6 = [1.35, 11.78, -8.389]
Tau_meas_obj_6 = [1.051, -0.0389, 0.197]
R_obj_6 = R_list[5]   # 夾重物時的旋轉矩陣

F_meas_obj_7 = np.array([3.25,0.983,-21.954])
Tau_meas_obj_7 = np.array([0.261,-0.112,0.203])
R_obj_7 = R_list[6]

m_obj = 269.5 / 1000.0  # kg
# # 補償

# print("補償後的力:", F_comp)
# print("理論重物重量:", m_obj * 9.80665)
# print("補償後力大小:", np.linalg.norm(F_comp))
# F_meas_obj = F_meas[n-1]
g = 9.80665
gI = np.array([0,0,-g])
for R in [R_obj_1, R_obj_2, R_obj_3, R_obj_4, R_obj_5, R_obj_6, R_obj_7]:
    gs = R.T @ gI                # world→sensor
    Fg_est = m * gs
    print("expected weight dir =", gs/np.linalg.norm(gs))


# F_meas_obj = np.array([-0.046,-0.068,-0.044])
# Tau_meas_obj = np.array([-0.003,-0.001,-0.001])
# R_obj = R_list[7]

F_meas_obj = np.array([0.02726,0.07942,0.03219])
Tau_meas_obj = np.array([-0.00125,-0.00040,0.00008])
R_obj = R_list[7]
# m = 1.413
# r = np.array([-0.0017, -0.000, 0.0717])
F_comp, Tau_comp = compensate(F_meas_obj, Tau_meas_obj, R_obj, Fb, Tau_b, m, r)
F_comp_1, Tau_comp_1 = compensate(F_meas_obj_1, Tau_meas_obj_1, R_obj_1, Fb, Tau_b, m, r)
F_comp_2, Tau_comp_2 = compensate(F_meas_obj_2, Tau_meas_obj_2, R_obj_2, Fb, Tau_b, m, r)
F_comp_3, Tau_comp_3 = compensate(F_meas_obj_3, Tau_meas_obj_3, R_obj_3, Fb, Tau_b, m, r)
F_comp_4, Tau_comp_4 = compensate(F_meas_obj_4, Tau_meas_obj_4, R_obj_4, Fb, Tau_b, m, r)
F_comp_5, Tau_comp_5 = compensate(F_meas_obj_5, Tau_meas_obj_5, R_obj_5, Fb, Tau_b, m, r)
F_comp_6, Tau_comp_6 = compensate(F_meas_obj_6, Tau_meas_obj_6, R_obj_6, Fb, Tau_b, m, r)
F_comp_7, Tau_comp_7 = compensate(F_meas_obj_7, Tau_meas_obj_7, R_obj_7, Fb, Tau_b, m, r)

print(f"補償後的力:{F_comp} |F|={ np.linalg.norm(F_comp/9.81):.4f} kg")
print(f"理論重物重量:{ m_obj * 9.80665:.4f} N, {m_obj:.4f} kg")
print(f"補償後力大小_1:{F_comp_1} |F_1|={ np.linalg.norm(F_comp_1/9.81):.4f} kg")
print(f"補償後力大小_2:{F_comp_2} |F_2|={ np.linalg.norm(F_comp_2/9.81):.4f} kg")
print(f"補償後力大小_3:{F_comp_3} |F_3|={ np.linalg.norm(F_comp_3/9.81):.4f} kg")
print(f"補償後力大小_4:{F_comp_4} |F_4|={ np.linalg.norm(F_comp_4/9.81):.4f} kg")
print(f"補償後力大小_5:{F_comp_5} |F_5|={ np.linalg.norm(F_comp_5/9.81):.4f} kg")
print(f"補償後力大小_6:{F_comp_6} |F_6|={ np.linalg.norm(F_comp_6/9.81):.4f} kg")
print(f"補償後力大小_7:{F_comp_7} |F_7|={ np.linalg.norm(F_comp_7/9.81):.4f} kg")