import numpy as np
import csv
import matplotlib.pyplot as plt
import math

# 每組姿態分別儲存
Fx, Fy, Fz = [], [], []
Tx, Ty, Tz = [], [], []

g_I = np.array([0, 0, -9.81])  # 重力加速度向量 (inertial frame)
g_s = np.zeros(3)  # 重力加速度向量 (sensor frame)

for i in range(1, 25):  # 若有 R_1.csv ~ R_24.csv，要到 15
    # if i == 1 or i == 4 or i == 5 or i == 6 or i == 7 or i == 11 or i == 14 or i == 16 or i == 24:
    #     continue
    file = f"/home/jamesho5055/ws_moveit/1117_5hz_nooffset/R{i}.csv"
    # file = f"wenyu/R{i}.csv"
    fxi, fyi, fzi = [], [], []
    txi, tyi, tzi = [], [], []
    with open(file, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        count = 0
        for idx, row in enumerate(reader):
            if idx < 50:
                continue  # skip first 10 lines to avoid transient
            elif idx >= 200:
                break  # only read 500 lines after skipping
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

# 轉 numpy 陣列 (24,3)
F_meas = np.column_stack([Fx, Fy, Fz])
# print("F_meas:", F_meas)
Tau_meas = np.column_stack([Tx, Ty, Tz])
# print("F_meas:", F_meas)
np.save("/home/jamesho5055/ws_moveit/F_meas_1117_5hz_nooffset.npy", F_meas)

R_list = [
    [[1,0,0],[0,1,0],[0,0,1]],   [[1,0,0],[0,-1,0],[0,0,-1]],
    [[1,0,0],[0,0,-1],[0,1,0]],  [[1,0,0],[0,0,1],[0,-1,0]],
    [[-1,0,0],[0,1,0],[0,0,-1]],  [[-1,0,0],[0,-1,0],[0,0,1]],
    [[-1,0,0],[0,0,1],[0,1,0]],   [[-1,0,0],[0,0,-1],[0,-1,0]],
    [[0,1,0],[1,0,0],[0,0,-1]],  [[0,-1,0],[1,0,0],[0,0,1]],
    [[0,0,1],[1,0,0],[0,1,0]], [[0,0,-1],[1,0,0],[0,-1,0]],
    [[0,1,0],[-1,0,0],[0,0,1]], [[0,-1,0],[-1,0,0],[0,0,-1]],
    [[0,0,-1],[-1,0,0],[0,1,0]],  [[0,0,1],[-1,0,0],[0,-1,0]],
    [[0,1,0],[0,0,1],[1,0,0]],  [[0,-1,0],[0,0,-1],[1,0,0]],
    [[0,0,-1],[0,1,0],[1,0,0]],   [[0,0,1],[0,-1,0],[1,0,0]],
    [[0,1,0],[0,0,-1],[-1,0,0]], [[0,-1,0],[0,0,1],[-1,0,0]],
    [[0,0,1],[0,1,0],[-1,0,0]],  [[0,0,-1],[0,-1,0],[-1,0,0]],
]
# R_list = [
#                                  [[1,0,0],[0,-1,0],[0,0,-1]],
#     [[1,0,0],[0,0,-1],[0,1,0]],  
    
#                                  [[-1,0,0],[0,0,-1],[0,-1,0]],
#     [[0,1,0],[1,0,0],[0,0,-1]],  [[0,-1,0],[1,0,0],[0,0,1]],
#                                  [[0,0,-1],[1,0,0],[0,-1,0]],
#     [[0,1,0],[-1,0,0],[0,0,1]], 
#     [[0,0,-1],[-1,0,0],[0,1,0]],  
#     [[0,1,0],[0,0,1],[1,0,0]],  [[0,-1,0],[0,0,-1],[1,0,0]],
#     [[0,0,-1],[0,1,0],[1,0,0]],   [[0,0,1],[0,-1,0],[1,0,0]],
#     [[0,1,0],[0,0,-1],[-1,0,0]], [[0,-1,0],[0,0,1],[-1,0,0]],
#     [[0,0,1],[0,1,0],[-1,0,0]],  
# ]

def estimate_params_1(F_meas, Tau_meas, R_list):
    # 計算 Fb, m_star, r_star
    Fb = np.mean((F_meas), axis=0)
    Tau_b = np.mean((Tau_meas), axis=0)
    print(f"Fb1: {Fb} N")
    print(f"Tau_b1: {Tau_b} Nm")
    m_star = 0
    r_star = np.zeros(3)
    K = len(R_list)
    G = []
    F_corrected = (F_meas) - Fb
    Tau_corrected = Tau_meas - Tau_b
    for i in range(len(R_list)):
        R_i = np.array(R_list[i]) @ np.diag([1, 1, -1])
        g_s = R_i.T @ g_I  # 將重力向量轉到 sensor 座標系
        G.append(g_s)
    G = np.array(G).reshape(-1)  # shape: (72,) = (24*3,)
    F = F_corrected.reshape(-1)  # shape: (72,)
    m_star = (G.T @ F) / (G.T @ G)

    A = []
    for i in range(K):
        R_i = np.array(R_list[i])
        g_si = R_i.T @ g_I
        g_si_hat = np.array([
            [0,        -g_si[2],  g_si[1]],
            [g_si[2],   0,       -g_si[0]],
            [-g_si[1],  g_si[0],  0]
        ])        
        A.append(g_si_hat)
    A = np.vstack(A)  # shape: (72,3)
    Tau = Tau_corrected.reshape(-1)  # shape: (72,)
    r_star = (1.0/m_star) * np.linalg.inv(A.T @ A) @ A.T @ Tau
    r_star = r_star.flatten()
    F_estimated = m_star * G
    force_error = np.sqrt(np.mean((F - F_estimated)**2))
    force_max_error = np.max(np.abs((F_corrected - m_star * np.array(G).reshape(K, 3)).reshape(-1)))
    
    # 力矩的均方根誤差
    Tau_estimated = m_star * A @ r_star.reshape(-1, 1)
    torque_error = np.sqrt(np.mean((Tau.flatten() - Tau_estimated.flatten())**2))
    torque_max_error = np.max(np.abs(Tau.flatten() - Tau_estimated.flatten()))
    
    print(f"\n=== Estimation Errors1 ===")
    print(f"Force RMS Error: {force_error:.4f} N")
    print(f"Force Max Error: {force_max_error:.4f} N")
    print(f"Torque RMS Error: {torque_error:.6f} Nm")
    print(f"Torque Max Error: {torque_max_error:.6f} Nm")

    # 對每個姿態，檢查補償後的殘差
    for i, R in enumerate(R_list):
        # 使用 gravity_test.py 的參數
        R = np.array(R)
        gs = R.T @ np.array([0, 0, -9.81])
        F_theory = Fb + m_star * gs
        Tau_theory = Tau_b + m_star * np.cross(r_star, gs)
        
        F_residual = np.linalg.norm(F_meas[i] - F_theory)
        Tau_residual = np.linalg.norm(Tau_meas[i] - Tau_theory)
        
        # print(f"R{i+1}: F_err={F_residual:.3f} N, Tau_err={Tau_residual:.4f} Nm")

    return Fb, Tau_b, m_star, r_star


def compensate(F_meas, Tau_meas, R, Fb, Tau_b, m, r, g=9.80665):
    gI = np.array([0,0,-g]); gs = R.T @ gI
    Fg = m*gs
    Tg = np.cross(r, Fg)
    F_contact = F_meas - Fb - Fg
    T_contact = Tau_meas - Tau_b - Tg
    return F_contact, T_contact

Fb_1, Tau_b_1, m_star_1, r_star_1 = estimate_params_1(F_meas, Tau_meas, R_list)

print(f"Estimated mass m*: {m_star_1:.5f} kg")
print(f"Estimated center of mass r*: {r_star_1} m")

# F1 = np.array([-0.6399999856948853, 1.809999942779541, 11.579999923706055])
# T1 = np.array([0.03099999949336052, 0.09000000357627869, -0.15299999713897705])
# R1 = np.array(R_list[1])
# F_comp, T_comp = compensate(F_meas[0], Tau_meas[0], R1, Fb, Tau_b, m_star, r_star)
# print(f"F_comp: {F_comp}")
# print(f"T_comp: {T_comp}")

# import numpy as np
# import csv
# import numpy as np
# import matplotlib.pyplot as plt

# # ========== 數據讀取（改進版）==========
# def load_measurement(file_path, skip_rows=20, max_rows=180):
#     """
#     讀取單個姿態的測量數據
    
#     Parameters:
#     -----------
#     file_path : str
#     skip_rows : int, 跳過前幾行以避免暫態
#     max_rows : int, 最多讀取幾行
#     """
#     fx_list, fy_list, fz_list = [], [], []
#     tx_list, ty_list, tz_list = [], [], []
    
#     with open(file_path, 'r') as f:
#         reader = csv.reader(f)
#         next(reader)  # skip header
        
#         for idx, row in enumerate(reader):
#             if idx < skip_rows:
#                 continue
#             elif idx >= skip_rows + max_rows:
#                 break
            
#             fx, fy, fz, tx, ty, tz = map(float, row[1:])
#             fx_list.append(fx)
#             fy_list.append(fy)
#             fz_list.append(fz)
#             tx_list.append(tx)
#             ty_list.append(ty)
#             tz_list.append(tz)
    
#     # 計算平均值和標準差（用於檢查數據品質）
#     force = np.array([fx_list, fy_list, fz_list])
#     torque = np.array([tx_list, ty_list, tz_list])
    
#     f_mean = np.mean(force, axis=1)
#     f_std = np.std(force, axis=1)
#     t_mean = np.mean(torque, axis=1)
#     t_std = np.std(torque, axis=1)
    
#     return f_mean, f_std, t_mean, t_std

# # 讀取所有 24 組數據
# print("Loading measurements...")
# F_meas = []
# Tau_meas = []
# F_std_list = []
# Tau_std_list = []

# for i in range(1, 25):
#     file = f"/home/jamesho5055/ws_moveit/1117_2hz_nooffset/R{i}.csv"
#     f_mean, f_std, t_mean, t_std = load_measurement(file, skip_rows=20, max_rows=180)
    
#     F_meas.append(f_mean)
#     Tau_meas.append(t_mean)
#     F_std_list.append(f_std)
#     Tau_std_list.append(t_std)
    
#     # 檢查數據品質（標準差不應太大）
#     if np.max(f_std) > 0.5 or np.max(t_std) > 0.01:
#         print(f"Warning: R{i} has high noise - F_std: {f_std}, T_std: {t_std}")

# F_meas = np.array(F_meas)  # (24, 3)
# Tau_meas = np.array(Tau_meas)  # (24, 3)

# # ========== 旋轉矩陣 ==========
# R_list = [
#     [[1,0,0],[0,1,0],[0,0,1]],   [[1,0,0],[0,-1,0],[0,0,-1]],
#     [[1,0,0],[0,0,-1],[0,1,0]],  [[1,0,0],[0,0,1],[0,-1,0]],
#     [[-1,0,0],[0,1,0],[0,0,-1]], [[-1,0,0],[0,-1,0],[0,0,1]],
#     [[-1,0,0],[0,0,1],[0,1,0]],  [[-1,0,0],[0,0,-1],[0,-1,0]],
#     [[0,1,0],[1,0,0],[0,0,-1]],  [[0,-1,0],[1,0,0],[0,0,1]],
#     [[0,0,1],[1,0,0],[0,1,0]],   [[0,0,-1],[1,0,0],[0,-1,0]],
#     [[0,1,0],[-1,0,0],[0,0,1]],  [[0,-1,0],[-1,0,0],[0,0,-1]],
#     [[0,0,-1],[-1,0,0],[0,1,0]], [[0,0,1],[-1,0,0],[0,-1,0]],
#     [[0,1,0],[0,0,1],[1,0,0]],   [[0,-1,0],[0,0,-1],[1,0,0]],
#     [[0,0,-1],[0,1,0],[1,0,0]],  [[0,0,1],[0,-1,0],[1,0,0]],
#     [[0,1,0],[0,0,-1],[-1,0,0]], [[0,-1,0],[0,0,1],[-1,0,0]],
#     [[0,0,1],[0,1,0],[-1,0,0]],  [[0,0,-1],[0,-1,0],[-1,0,0]],
# ]

# g_I = np.array([0, 0, -9.81])  # 重力向量

# # ========== 參數估計 ==========
# def estimate_params(F_meas, Tau_meas, R_list, g_I):
#     K = len(R_list)
    
#     # Step 1: 估計 Bias
#     Fb = np.mean(F_meas, axis=0)
#     Tau_b = np.mean(Tau_meas, axis=0)
    
#     print("="*50)
#     print("BIAS ESTIMATION")
#     print("="*50)
#     print(f"Force Bias (Fb):  [{Fb[0]:7.4f}, {Fb[1]:7.4f}, {Fb[2]:7.4f}] N")
#     print(f"Torque Bias (Tb): [{Tau_b[0]:7.5f}, {Tau_b[1]:7.5f}, {Tau_b[2]:7.5f}] Nm")
    
#     # Step 2: 減去 Bias
#     F_corrected = F_meas - Fb
#     Tau_corrected = Tau_meas - Tau_b
    
#     # Step 3: 估計質量
#     G = []
#     for i in range(K):
#         R_i = np.array(R_list[i])
#         g_si = R_i.T @ g_I
#         G.append(g_si)
    
#     G = np.array(G).reshape(-1)  # (72,)
#     F = F_corrected.reshape(-1)  # (72,)
    
#     m_star = (G.T @ F) / (G.T @ G)
    
#     print("\n" + "="*50)
#     print("MASS ESTIMATION")
#     print("="*50)
#     print(f"Estimated Mass (m*): {m_star:.5f} kg")
#     print(f"Expected gravity force: {m_star * 9.81:.4f} N")
    
#     # Step 4: 估計質心
#     A = []
#     for i in range(K):
#         R_i = np.array(R_list[i])
#         g_si = R_i.T @ g_I
        
#         A_i = np.array([
#             [0,        -g_si[2],  g_si[1]],
#             [g_si[2],   0,       -g_si[0]],
#             [-g_si[1],  g_si[0],  0]
#         ])
#         A.append(A_i)
    
#     A = np.vstack(A)  # (72, 3)
#     Tau = Tau_corrected.reshape(-1)  # (72,)
    
#     r_star = (1.0 / m_star) * np.linalg.inv(A.T @ A) @ A.T @ Tau
#     r_star = r_star.flatten()
    
#     print("\n" + "="*50)
#     print("CENTER OF MASS ESTIMATION")
#     print("="*50)
#     print(f"Center of Mass (r*): [{r_star[0]*1000:7.4f}, {r_star[1]*1000:7.4f}, {r_star[2]*1000:7.4f}] mm")
#     print(f"Distance from origin: {np.linalg.norm(r_star)*1000:.4f} mm")
    
#     # 計算估計誤差
#     F_estimated = m_star * G
#     Tau_estimated = m_star * A @ r_star
    
#     force_residuals = F - F_estimated
#     torque_residuals = Tau - Tau_estimated
    
#     force_rms = np.sqrt(np.mean(force_residuals**2))
#     force_max = np.max(np.abs(force_residuals))
    
#     torque_rms = np.sqrt(np.mean(torque_residuals**2))
#     torque_max = np.max(np.abs(torque_residuals))
    
#     print("\n" + "="*50)
#     print("ESTIMATION ERRORS")
#     print("="*50)
#     print(f"Force  RMS Error: {force_rms:.4f} N")
#     print(f"Force  Max Error: {force_max:.4f} N")
#     print(f"Torque RMS Error: {torque_rms:.6f} Nm")
#     print(f"Torque Max Error: {torque_max:.6f} Nm")
    
#     # 每個軸的誤差分析
#     F_reshaped = force_residuals.reshape(K, 3)
#     T_reshaped = torque_residuals.reshape(K, 3)
    
#     print("\nPer-axis Force Errors (RMS):")
#     print(f"  Fx: {np.sqrt(np.mean(F_reshaped[:,0]**2)):.4f} N")
#     print(f"  Fy: {np.sqrt(np.mean(F_reshaped[:,1]**2)):.4f} N")
#     print(f"  Fz: {np.sqrt(np.mean(F_reshaped[:,2]**2)):.4f} N")
    
#     print("\nPer-axis Torque Errors (RMS):")
#     print(f"  Tx: {np.sqrt(np.mean(T_reshaped[:,0]**2)):.6f} Nm")
#     print(f"  Ty: {np.sqrt(np.mean(T_reshaped[:,1]**2)):.6f} Nm")
#     print(f"  Tz: {np.sqrt(np.mean(T_reshaped[:,2]**2)):.6f} Nm")
    
#     return Fb, Tau_b, m_star, r_star

# # 執行估計
# Fb, Tau_b, m_star, r_star = estimate_params(F_meas, Tau_meas, R_list, g_I)

# # ========== 重力補償函數 ==========
# def gravity_compensation(R, m_star, r_star, g_I):
#     """計算給定姿態的重力補償"""
#     g_s = R.T @ g_I
#     F_gravity = m_star * g_s
#     Tau_gravity = m_star * np.cross(r_star, g_s)
#     return np.concatenate([F_gravity, Tau_gravity])

# # ========== 驗證：重新計算補償後的讀數 ==========
# print("\n" + "="*50)
# print("VALIDATION: Compensated Readings for First 3 Poses")
# print("="*50)

# for i in range(3):
#     R = np.array(R_list[i])
#     F_measured = F_meas[i]
#     Tau_measured = Tau_meas[i]
    
#     # 計算補償值
#     compensation = gravity_compensation(R, m_star, r_star, g_I)
#     F_comp = compensation[:3]
#     Tau_comp = compensation[3:]
    
#     # 補償後的讀數（應該接近 Bias）
#     F_after = F_measured - F_comp
#     Tau_after = Tau_measured - Tau_comp
    
#     print(f"\nPose {i+1}:")
#     print(f"  Force after compensation:  [{F_after[0]:7.4f}, {F_after[1]:7.4f}, {F_after[2]:7.4f}] N")
#     print(f"  Expected (Fb):             [{Fb[0]:7.4f}, {Fb[1]:7.4f}, {Fb[2]:7.4f}] N")
#     print(f"  Torque after compensation: [{Tau_after[0]:7.5f}, {Tau_after[1]:7.5f}, {Tau_after[2]:7.5f}] Nm")
#     print(f"  Expected (Tb):             [{Tau_b[0]:7.5f}, {Tau_b[1]:7.5f}, {Tau_b[2]:7.5f}] Nm")

#     # ========== 診斷：找出問題姿態 ==========
# def diagnose_measurements(F_meas, Tau_meas, R_list, g_I, Fb, Tau_b, m_star, r_star):
#     """
#     分析每個姿態的測量誤差，找出異常值
#     """
#     K = len(R_list)
    
#     force_errors = []
#     torque_errors = []
    
#     print("\n" + "="*80)
#     print("DETAILED RESIDUAL ANALYSIS FOR EACH POSE")
#     print("="*80)
#     print(f"{'Pose':<6} {'F_err(N)':<12} {'T_err(Nm)':<12} {'Status':<15} {'Notes'}")
#     print("-"*80)
    
#     for i in range(K):
#         R = np.array(R_list[i])
        
#         # 計算該姿態的理論值
#         g_s = R.T @ g_I
#         F_theory = Fb + m_star * g_s
#         Tau_theory = Tau_b + m_star * np.cross(r_star, g_s)
        
#         # 實際測量值
#         F_actual = F_meas[i]
#         Tau_actual = Tau_meas[i]
        
#         # 計算誤差
#         F_error = np.linalg.norm(F_actual - F_theory)
#         T_error = np.linalg.norm(Tau_actual - Tau_theory)
        
#         force_errors.append(F_error)
#         torque_errors.append(T_error)
        
#         # 判斷狀態
#         status = "OK"
#         notes = ""
#         if F_error > 2.0:
#             status = "⚠️ HIGH FORCE"
#             notes += "Force outlier "
#         if T_error > 0.08:
#             status = "⚠️ HIGH TORQUE"
#             notes += "Torque outlier "
#         if F_error > 3.0 or T_error > 0.15:
#             status = "❌ CRITICAL"
        
#         print(f"R{i+1:<5} {F_error:<12.4f} {T_error:<12.6f} {status:<15} {notes}")
    
#     force_errors = np.array(force_errors)
#     torque_errors = np.array(torque_errors)
    
#     print("-"*80)
#     print(f"Force Error:  Mean={np.mean(force_errors):.4f} N, "
#           f"Median={np.median(force_errors):.4f} N, "
#           f"Max={np.max(force_errors):.4f} N")
#     print(f"Torque Error: Mean={np.mean(torque_errors):.6f} Nm, "
#           f"Median={np.median(torque_errors):.6f} Nm, "
#           f"Max={np.max(torque_errors):.6f} Nm")
    
#     # 找出最差的姿態
#     worst_force_idx = np.argmax(force_errors)
#     worst_torque_idx = np.argmax(torque_errors)
    
#     print("\n" + "="*80)
#     print("WORST POSES")
#     print("="*80)
#     print(f"Worst Force Error:  R{worst_force_idx+1} with {force_errors[worst_force_idx]:.4f} N")
#     print(f"Worst Torque Error: R{worst_torque_idx+1} with {torque_errors[worst_torque_idx]:.6f} Nm")
    
#     # 視覺化
#     fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
#     # 力誤差分布
#     ax1.bar(range(1, K+1), force_errors, color=['red' if e > 2.0 else 'blue' for e in force_errors])
#     ax1.axhline(y=np.mean(force_errors), color='g', linestyle='--', label=f'Mean: {np.mean(force_errors):.3f} N')
#     ax1.axhline(y=2.0, color='orange', linestyle='--', label='Threshold: 2.0 N')
#     ax1.set_xlabel('Pose Number')
#     ax1.set_ylabel('Force Error (N)')
#     ax1.set_title('Force Error by Pose')
#     ax1.legend()
#     ax1.grid(True, alpha=0.3)
    
#     # 力矩誤差分布
#     ax2.bar(range(1, K+1), torque_errors, color=['red' if e > 0.08 else 'blue' for e in torque_errors])
#     ax2.axhline(y=np.mean(torque_errors), color='g', linestyle='--', label=f'Mean: {np.mean(torque_errors):.5f} Nm')
#     ax2.axhline(y=0.08, color='orange', linestyle='--', label='Threshold: 0.08 Nm')
#     ax2.set_xlabel('Pose Number')
#     ax2.set_ylabel('Torque Error (Nm)')
#     ax2.set_title('Torque Error by Pose')
#     ax2.legend()
#     ax2.grid(True, alpha=0.3)
    
#     plt.tight_layout()
#     plt.savefig('pose_error_analysis.png', dpi=150)
#     print(f"\nPlot saved as 'pose_error_analysis.png'")
#     plt.show()
    
#     return force_errors, torque_errors

# # 執行診斷
# force_errors, torque_errors = diagnose_measurements(
#     F_meas, Tau_meas, R_list, g_I, Fb, Tau_b, m_star, r_star
# )

# # ========== 可選：重新估計（排除異常值）==========
# def estimate_params_robust(F_meas, Tau_meas, R_list, g_I, force_threshold=3.0):
#     """
#     使用健壯估計（排除明顯的異常值）
#     """
#     K = len(R_list)
    
#     # 第一輪：粗略估計
#     Fb_rough = np.mean(F_meas, axis=0)
#     Tau_b_rough = np.mean(Tau_meas, axis=0)
    
#     F_corrected = F_meas - Fb_rough
    
#     G = []
#     for i in range(K):
#         R_i = np.array(R_list[i])
#         g_si = R_i.T @ g_I
#         G.append(g_si)
#     G = np.array(G).reshape(-1)
#     F = F_corrected.reshape(-1)
#     m_rough = (G.T @ F) / (G.T @ G)
    
#     # 找出異常值
#     valid_indices = []
#     for i in range(K):
#         R = np.array(R_list[i])
#         g_s = R.T @ g_I
#         F_expected = m_rough * g_s
#         F_error = np.linalg.norm(F_corrected[i] - F_expected)
        
#         if F_error < force_threshold:
#             valid_indices.append(i)
    
#     print("\n" + "="*80)
#     print("ROBUST ESTIMATION (Outlier Removal)")
#     print("="*80)
#     print(f"Valid poses: {len(valid_indices)}/{K}")
#     print(f"Excluded poses: {[i+1 for i in range(K) if i not in valid_indices]}")
    
#     # 使用有效數據重新估計
#     F_valid = F_meas[valid_indices]
#     Tau_valid = Tau_meas[valid_indices]
#     R_valid = [R_list[i] for i in valid_indices]
    
#     Fb, Tau_b, m_star, r_star = estimate_params(F_valid, Tau_valid, R_valid, g_I)
    
#     return Fb, Tau_b, m_star, r_star

# # 嘗試健壯估計
# print("\n" + "="*80)
# print("ATTEMPTING ROBUST ESTIMATION")
# print("="*80)
# Fb_robust, Tau_b_robust, m_robust, r_robust = estimate_params_robust(
#     F_meas, Tau_meas, R_list, g_I, force_threshold=3.0
# )

# # 簡單測試：只看 R1 和 R7
# print("\n" + "="*80)
# print("QUICK TEST - R1 and R7")
# print("="*80)

# print("\nR1 (First pose):")
# print(f"  Measured: {F_meas[0]}")
# print(f"  Expected: Should be close to [-0.59, -0.38, -8.75] ± 1N")

# print("\nR7 (Identity pose - sensor aligned with world):")
# print(f"  Measured: {F_meas[6]}")
# print(f"  Expected: Z should be ~-8.7N, X and Y should be small")

# # 如果 R7 的測量值不符合預期，座標系定義有問題
# if not (abs(F_meas[6][2]) > 7 and abs(F_meas[6][0]) < 2 and abs(F_meas[6][1]) < 2):
#     print("\n🔴 PROBLEM DETECTED at R7!")
#     print("   Your coordinate system definition may be wrong.")