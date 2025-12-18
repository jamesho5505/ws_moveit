# #!/usr/bin/env python3
# import numpy as np
# import cv2
# import math


# def rotation_matrix_to_rpy_xyz(rotation_matrix: np.ndarray):
#     """Convert 3x3 rotation matrix to RPY (XYZ convention)."""
#     sy = math.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2)
#     singular = sy < 1e-9

#     if not singular:
#         roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
#         pitch = math.atan2(-rotation_matrix[2, 0], sy)
#         yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
#     else:
#         roll = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
#         pitch = math.atan2(-rotation_matrix[2, 0], sy)
#         yaw = 0.0

#     return np.array([roll, pitch, yaw], dtype=float)


# def main():
#     npz_data = np.load("handeye_data.npz")
#     R_gripper2base = npz_data["R_gripper2base"]   # (N,3,3)
#     t_gripper2base = npz_data["t_gripper2base"]   # (N,3,1)
#     R_target2cam = npz_data["R_target2cam"]       # (N,3,3)
#     t_target2cam = npz_data["t_target2cam"]       # (N,3,1)

#     # OpenCV 需要 list[3x3], list[3x1]
#     R_g2b_list = [R_gripper2base[i] for i in range(R_gripper2base.shape[0])]
#     t_g2b_list = [t_gripper2base[i] for i in range(t_gripper2base.shape[0])]
#     R_t2c_list = [R_target2cam[i] for i in range(R_target2cam.shape[0])]
#     t_t2c_list = [t_target2cam[i] for i in range(t_target2cam.shape[0])]

#     # 求解 hand-eye：回傳的是 camera in gripper frame (^F R_C, ^F t_C)
#     R_cam_in_gripper, t_cam_in_gripper = cv2.calibrateHandEye(
#         R_g2b_list,
#         t_g2b_list,
#         R_t2c_list,
#         t_t2c_list,
#         method=cv2.CALIB_HAND_EYE_TSAI
#     )

#     # 組成 4x4
#     T_F_C = np.eye(4, dtype=float)
#     T_F_C[:3, :3] = R_cam_in_gripper
#     T_F_C[:3, 3] = t_cam_in_gripper.flatten()

#     print("F_T_C (camera w.r.t flange):")
#     np.set_printoptions(suppress=True, precision=9)
#     print(T_F_C)

#     # 轉成 URDF 格式
#     translation = t_cam_in_gripper.flatten()
#     rpy = rotation_matrix_to_rpy_xyz(R_cam_in_gripper)

#     print("\nURDF origin:")
#     print(f'xyz="{translation[0]:.9f} {translation[1]:.9f} {translation[2]:.9f}"')
#     print(f'rpy="{rpy[0]:.9f} {rpy[1]:.9f} {rpy[2]:.9f}"')

#     # 儲存
#     np.save("F_T_C.npy", T_F_C)
#     print("\nSaved F_T_C.npy")


# if __name__ == "__main__":
#     main()

# import numpy as np
# import pandas as pd

# DATA = np.array([

#     # [ 0.068844,  -0.036704, 0.333000, 0.158349409,  0.475409390, 0.094599530],
#     # [ 0.040005,  -0.021965, 0.334000, 0.129026110,  0.460949449, 0.094811704],
#     # [ -0.074938,  -0.043322, 0.332000, 0.012996763, 0.484297741, 0.094912128],
#     # [ 0.028667, 0.044338, 0.286000, 0.117042958, 0.394095475, 0.141455744],
#     # [ 0.025012, -0.073921, 0.287000, 0.114966217,  0.513165847, 0.141870675],
#     # [ -0.064090,  0.058629, 0.358000, 0.022270796, 0.381438078, 0.069921731],
#     # [ 0.073856,  -0.062750, 0.360000, 0.162932430,  0.501042729, 0.070732704],
#     # [ -0.083428,  -0.045800, 0.293000, 0.005152425,  0.486390033, 0.135945351],
#     # [ 0.053868,  -0.070632, 0.292000, 0.143776185, 0.509287133, 0.136008470],
#     # [ 0.067229, -0.049217, 0.264000, 0.156796346,  0.487284967, 0.164840741],
#     # [ 0.007498, -0.025145, 0.306000, 0.096397397, 0.464742549, 0.121330438],
#     # [ 0.114513,  0.085798, 0.307000, 0.201265793, 0.352412851, 0.120904923],

#     # [ -0.047025,  0.011665, 0.332000, 0.040269119,  0.428470858, 0.096405730],
#     # [ 0.061758,  0.023892, 0.332000, 0.150078702,  0.414449468, 0.095564060],
#     # [ 0.017719,  0.013717, 0.291000, 0.106223397, 0.425295976, 0.136465678],
#     # [ 0.109025, 0.048416, 0.286000, 0.196753769, 0.389442995, 0.141503419],
#     # [ -0.038158, 0.043150, 0.357000, 0.048527577,  0.396734178, 0.071099543],
#     # [ -0.009359, -0.002588, 0.496000, 0.077503550,  0.442671877, -0.067948483],

 
# #     [ 0.006117382,  0.024262574, 0.329000, 0.094032415, 0.414977772, 0.098954185],
# #     [ -0.091544235, 0.044919823, 0.353000, -0.005122798, 0.395963044, 0.074957179],
# #     [ 0.029175867, -0.079606955, 0.283000, 0.119173156,  0.518626394, 0.145680922],
# #     [ -0.032013894, 0.038358224, 0.262000, 0.055481413,  0.401119383, 0.165455865],


# #Check moi mui nhon
#     [ 0.003114,  -0.030439, 0.368000, 0.091566779, 0.470108250, 0.059172573],
#     [ 0.038156, -0.030897, 0.369000, 0.126880807, 0.469529563, 0.058738157],
#     [ 0.003520, 0.004479, 0.367000, 0.091021235,  0.434946281, 0.058751681],
#     [ 0.038418, 0.004091, 0.367000, 0.126267408,  0.434411052, 0.058939596],

# #Check moi chuan, lay Z rieng, X,Y sau, hole
#     [ 0.034403136,  0.007890912, 0.332000, 0.122352842, 0.430375795, 0.093981151],
#     [ 0.022052417, -0.018048907, 0.333000, 0.110511023, 0.456904241, 0.093803586],
#     [ 0.195586310,  0.066720475, 0.36000, 0.280247594, 0.368326119, 0.067389198],
#     [ 0.119788968, -0.019979896, 0.337000, 0.206910800, 0.455843014, 0.093861271],
#     [ 0.087023598, 0.048848367, 0.358000, 0.173736309, 0.388155642, 0.067977571], 
    
#     [ 0.069880082, 0.001633285, 0.286000, 0.158409726, 0.435692112, 0.141555611], 
#     [ -0.034071744,-0.001251125, 0.284000, 0.053430111, 0.441259185, 0.141447960], 
#     [ 0.050129454, -0.049890154, 0.261000, 0.139925146, 0.487772761, 0.166464987], 
#     [ 0.014964660, 0.010081686, 0.261000, 0.103048588, 0.428634824, 0.166279663], 
#     [ 0.013898128, -0.004927687, 0.332000, 0.101898138, 0.443950693, 0.095143671], 

# ])

# def compute_T_and_rms(data: np.ndarray):
#     XC, YC, ZC = data[:,0], data[:,1], data[:,2]
#     XA, YA, ZA = data[:,3], data[:,4], data[:,5]
#     C = np.column_stack([XC, YC, ZC, np.ones_like(XC)])

#     params_x = np.linalg.pinv(C) @ XA
#     params_y = np.linalg.pinv(C) @ YA
#     params_z = np.linalg.pinv(C) @ ZA

#     T = np.eye(4)
#     T[0,:4] = params_x
#     T[1,:4] = params_y
#     T[2,:4] = params_z

#     PC = np.column_stack([XC, YC, ZC, np.ones_like(XC)])
#     PA_pred = (PC @ T[:3,:4].T)
#     PA_true = np.column_stack([XA, YA, ZA])
#     err = PA_pred - PA_true
#     rms = float(np.sqrt(np.mean(np.sum(err**2, axis=1))))
#     return T, rms

# def main():
#     T, rms = compute_T_and_rms(DATA)
#     np.set_printoptions(suppress=True)
#     print("Calibration matrix T (A <- C):")
#     print(T)
#     print("\nRMS reprojection error (m):", rms)

# if __name__ == "__main__":
#     main()

# import numpy as np
# import pandas as pd

# def compute_T_and_rms(data: np.ndarray):
#     XC, YC, ZC = data[:,0], data[:,1], data[:,2]
#     XA, YA, ZA = data[:,3], data[:,4], data[:,5]
#     C = np.column_stack([XC, YC, ZC, np.ones_like(XC)])

#     params_x = np.linalg.pinv(C) @ XA
#     params_y = np.linalg.pinv(C) @ YA
#     params_z = np.linalg.pinv(C) @ ZA

#     T = np.eye(4)
#     T[0,:4] = params_x
#     T[1,:4] = params_y
#     T[2,:4] = params_z

#     PC = np.column_stack([XC, YC, ZC, np.ones_like(XC)])
#     PA_pred = (PC @ T[:3,:4].T)
#     PA_true = np.column_stack([XA, YA, ZA])
#     err = PA_pred - PA_true
    
#     err_norms = np.sqrt(np.sum(err**2, axis=1))
#     rms = float(np.sqrt(np.mean(err_norms**2)))
    
#     return T, rms, err_norms

# # 讀取數據
# df = pd.read_csv("handeye_simple_calibration.csv")
# DATA = df[['Xc', 'Yc', 'Zc', 'Xr', 'Yr', 'Zr']].values

# print(f"Total points: {len(DATA)}")

# # 標定
# T, rms, err_norms = compute_T_and_rms(DATA)

# np.set_printoptions(suppress=True, precision=6)
# print("\n=== Calibration Result ===")
# print("Transform Camera → TCP:")
# print(T)
# print(f"\nRMS error: {rms*1000:.2f} mm")
# print(f"Max error: {err_norms.max()*1000:.2f} mm")
# print(f"Mean error: {err_norms.mean()*1000:.2f} mm")

# # 顯示誤差最大的 5 個點
# worst_indices = np.argsort(err_norms)[-5:]
# print("\nWorst 5 points:")
# for idx in worst_indices:
#     print(f"  Corner {idx}: {err_norms[idx]*1000:.2f} mm")

# 保存
# np.save("camera_to_tcp_transform.npy", T)
# print("\n✓ Saved to 'camera_to_tcp_transform.npy'")


# import numpy as np
# import pandas as pd

# def solve_rigid_transform(P, Q):
#     """
#     Solve rigid transformation Q = R * P + t
#     P: Nx3 (camera)
#     Q: Nx3 (robot/tcp)
#     """
#     centroid_P = np.mean(P, axis=0)
#     centroid_Q = np.mean(Q, axis=0)

#     P_centered = P - centroid_P
#     Q_centered = Q - centroid_Q

#     H = P_centered.T @ Q_centered
#     U, S, Vt = np.linalg.svd(H)

#     R = Vt.T @ U.T

#     if np.linalg.det(R) < 0:
#         Vt[2, :] *= -1
#         R = Vt.T @ U.T

#     t = centroid_Q - R @ centroid_P

#     T = np.eye(4)
#     T[:3, :3] = R
#     T[:3, 3] = t
#     return T


# # 讀取四點資料
# df = pd.read_csv("handeye_simple_calibration.csv")
# Pc = df[['Xc','Yc','Zc']].values
# Pr = df[['Xr','Yr','Zr']].values

# T = solve_rigid_transform(Pc, Pr)

# print("=== Rigid Camera → TCP Transform ===")
# print(T)


# import numpy as np
# import pandas as pd
# import math

# def rpy_to_rot(rx, ry, rz):
#     cx, cy, cz = np.cos([rx, ry, rz])
#     sx, sy, sz = np.sin([rx, ry, rz])

#     Rz = np.array([[ cz, -sz, 0],
#                    [ sz,  cz, 0],
#                    [  0,   0, 1]])
#     Ry = np.array([[ cy, 0, sy],
#                    [  0, 1,  0],
#                    [-sy, 0, cy]])
#     Rx = np.array([[ 1,  0,   0],
#                    [ 0, cx, -sx],
#                    [ 0, sx,  cx]])
#     return Rz @ Ry @ Rx

# def invT(T):
#     R, t = T[:3,:3], T[:3,3]
#     Ti = np.eye(4)
#     Ti[:3,:3] = R.T
#     Ti[:3,3] = -R.T @ t
#     return Ti

# def solve_rigid(P, Q):
#     Pc = P - P.mean(0)
#     Qc = Q - Q.mean(0)
#     H = Pc.T @ Qc
#     U, S, Vt = np.linalg.svd(H)
#     R = Vt.T @ U.T
#     if np.linalg.det(R) < 0:
#         Vt[2,:] *= -1
#         R = Vt.T @ U.T
#     t = Q.mean(0) - R @ P.mean(0)

#     T = np.eye(4)
#     T[:3,:3] = R
#     T[:3,3] = t
#     return T

# # 讀資料
# df = pd.read_csv("handeye_simple_calibration.csv")

# Pc = df[['Xc','Yc','Zc']].values
# Pr = df[['Xr','Yr','Zr']].values
# cap = df[['tcp_cap_x','tcp_cap_y','tcp_cap_z','tcp_cap_rx','tcp_cap_ry','tcp_cap_rz']].values[0]

# # 1. 相機→base
# T_A_C = solve_rigid(Pc, Pr)

# # 2. base→flange (拍照姿勢)
# tx,ty,tz, rx,ry,rz = cap
# R = rpy_to_rot(rx,ry,rz)
# T_A_F = np.eye(4)
# T_A_F[:3,:3] = R
# T_A_F[:3,3]  = [tx,ty,tz]

# # 3. flange→camera
# T_F_C = invT(T_A_F) @ T_A_C

# # 你已經算好的
# T_tcp_C = T_F_C              # Camera -> TCP

# # 來自 tf2_echo flange tcp
# T_tcp_F = np.eye(4)
# T_tcp_F[:3, 3] = [-0.009, 0.032, 0.196]

# # Flange -> Camera
# T_C_F = invT(T_tcp_C) @ T_tcp_F



# print("T_A_C (Camera→Base):\n", T_A_C)
# print("T_A_F (Tcp→Base):\n", T_A_F)
# print("T_F_C (Camera→Tcp):\n", T_F_C)
# print("T_C_F (Flange -> Camera):\n", T_C_F)

import numpy as np
import pandas as pd
import math
from scipy.spatial.transform import Rotation as R

def rpy_to_rot(rx, ry, rz):
    cx, cy, cz = np.cos([rx, ry, rz])
    sx, sy, sz = np.sin([rx, ry, rz])
    
    Rz = np.array([[ cz, -sz, 0],
                   [ sz,  cz, 0],
                   [ 0,   0,  1]])
    Ry = np.array([[ cy, 0, sy],
                   [ 0,  1,  0],
                   [-sy, 0, cy]])
    Rx = np.array([[ 1,  0,   0],
                   [ 0, cx, -sx],
                   [ 0, sx,  cx]])
    return Rz @ Ry @ Rx

def invT(T):
    R, t = T[:3,:3], T[:3,3]
    Ti = np.eye(4)
    Ti[:3,:3] = R.T
    Ti[:3,3] = -R.T @ t
    return Ti

def solve_rigid(P, Q):
    Pc = P - P.mean(0)
    Qc = Q - Q.mean(0)
    H = Pc.T @ Qc
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[2,:] *= -1
        R = Vt.T @ U.T
    t = Q.mean(0) - R @ P.mean(0)
    
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = t
    return T

# 讀資料
df = pd.read_csv("handeye_simple_calibration.csv")

Pc = df[['Xc','Yc','Zc']].values
Pr = df[['Xr','Yr','Zr']].values
tcp_cap = df[['tcp_cap_x','tcp_cap_y','tcp_cap_z','tcp_cap_rx','tcp_cap_ry','tcp_cap_rz']].values[0]

# 1. 相機→base
T_base_camera = solve_rigid(Pc, Pr)

# 2. base→tcp (拍照時的姿勢)
tx, ty, tz, rx, ry, rz = tcp_cap
R_rot = rpy_to_rot(rx, ry, rz)
T_base_tcp = np.eye(4)
T_base_tcp[:3,:3] = R_rot
T_base_tcp[:3,3] = [tx, ty, tz]

# 3. tcp→camera
T_tcp_camera = invT(T_base_tcp) @ T_base_camera

# 4. flange→tcp (從 tf2_echo 得到的固定轉換)
T_flange_tcp = np.eye(4)
T_flange_tcp[:3,3] = [-0.009, 0.032, 0.196]  # 平移
# 旋轉是單位矩陣 (RPY都是0)

# 5. flange→camera (最終結果)
T_flange_camera = T_flange_tcp @ T_tcp_camera

print("=" * 60)
print("T_base_camera (Camera→Base):")
print(T_base_camera)
print("\n" + "=" * 60)
print("T_base_tcp (TCP→Base, 拍照時的姿勢):")
print(T_base_tcp)
print("\n" + "=" * 60)
print("T_tcp_camera (Camera→TCP):")
print(T_tcp_camera)
print("\n" + "=" * 60)
print("T_flange_tcp (TCP→Flange, 固定轉換):")
print(T_flange_tcp)
print("\n" + "=" * 60)
print("**T_flange_camera (Camera→Flange, 最終結果):**")
print(T_flange_camera)
print("=" * 60)

# 驗證:檢查轉換鏈是否正確
T_base_camera_verify = T_base_tcp @ invT(T_flange_tcp) @ T_flange_camera
print("\n驗證 (應該與 T_base_camera 相同):")
print(T_base_camera_verify)
print("\n誤差:")
print(np.abs(T_base_camera - T_base_camera_verify).max())



R_flange_camera = T_flange_camera[:3, :3]
t_flange_camera = T_flange_camera[:3, 3]

rpy = R.from_matrix(R_flange_camera).as_euler('xyz', degrees=False)

print("xyz (m):", t_flange_camera)
print("rpy (rad):", rpy)


# import numpy as np

# def invT(T):
#     R, t = T[:3,:3], T[:3,3]
#     Ti = np.eye(4)
#     Ti[:3,:3] = R.T
#     Ti[:3,3] = -R.T @ t
#     return Ti

# def rot_to_rpy(R):
#     sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
#     if sy > 1e-6:
#         rx = np.arctan2(R[2,1], R[2,2])
#         ry = np.arctan2(-R[2,0], sy)
#         rz = np.arctan2(R[1,0], R[0,0])
#     else:
#         rx = np.arctan2(-R[1,2], R[1,1])
#         ry = np.arctan2(-R[2,0], sy)
#         rz = 0
#     return rx, ry, rz

# def rpy_to_rot(rx, ry, rz):
#     cx, cy, cz = np.cos([rx, ry, rz])
#     sx, sy, sz = np.sin([rx, ry, rz])
    
#     Rz = np.array([[ cz, -sz, 0],
#                    [ sz,  cz, 0],
#                    [ 0,   0,  1]])
#     Ry = np.array([[ cy, 0, sy],
#                    [ 0,  1,  0],
#                    [-sy, 0, cy]])
#     Rx = np.array([[ 1,  0,   0],
#                    [ 0, cx, -sx],
#                    [ 0, sx,  cx]])
#     return Rz @ Ry @ Rx

# print("=" * 80)
# print("從 Hand-Eye Calibration 結果推算 URDF 參數")
# print("=" * 80)

# # 你的 calibration 結果: flange → camera_color_optical_frame
# T_flange_optical = np.array([
#     [-9.98862654e-01,  3.31795067e-02, -3.42420729e-02,  3.41631617e-02],
#     [-3.31353957e-02, -9.99449150e-01, -1.85504144e-03, -8.31009190e-02],
#     [-3.42847601e-02, -7.18306973e-04,  9.99411847e-01,  6.30180245e-02],
#     [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
# ])

# print("T_flange_optical (Flange → camera_color_optical_frame):")
# print(T_flange_optical)
# print()

# # RealSense D435 內部轉換 (從 _d435.urdf.xacro)
# # 1. camera_bottom_screw_frame → camera_link
# d435_mesh_x_offset = 0.0149 - 0.1e-3 - 4.2e-3  # = 0.0106
# d435_cam_depth_py = 0.0175
# d435_cam_depth_pz = 0.025 / 2  # = 0.0125

# T_screw_link = np.eye(4)
# T_screw_link[:3, 3] = [d435_mesh_x_offset, d435_cam_depth_py, d435_cam_depth_pz]

# print("T_screw_link (camera_bottom_screw_frame → camera_link):")
# print(T_screw_link)
# print()

# # 2. camera_link → camera_color_frame
# T_link_color = np.eye(4)
# T_link_color[:3, 3] = [0, 0.015, 0]  # y 偏移 15mm

# print("T_link_color (camera_link → camera_color_frame):")
# print(T_link_color)
# print()

# # 3. camera_color_frame → camera_color_optical_frame
# R_optical = rpy_to_rot(0, -np.pi/2, -np.pi/2)
# T_color_optical = np.eye(4)
# T_color_optical[:3, :3] = R_optical

# print("T_color_optical (camera_color_frame → camera_color_optical_frame):")
# print(T_color_optical)
# print()

# # 組合: camera_bottom_screw_frame → camera_color_optical_frame
# T_screw_optical = T_screw_link @ T_link_color @ T_color_optical

# print("T_screw_optical (camera_bottom_screw_frame → camera_color_optical_frame):")
# print(T_screw_optical)
# print()

# # 反推: flange → camera_bottom_screw_frame
# T_flange_screw = T_flange_optical @ invT(T_screw_optical)

# print("=" * 80)
# print("最終結果: T_flange_screw (Flange → camera_bottom_screw_frame)")
# print("=" * 80)
# print(T_flange_screw)
# print()

# # 提取 RPY
# R = T_flange_screw[:3, :3]
# t = T_flange_screw[:3, 3]
# rx, ry, rz = rot_to_rpy(R)

# print("=" * 80)
# print("URDF 參數:")
# print("=" * 80)
# print(f'<origin xyz="{t[0]:.6f} {t[1]:.6f} {t[2]:.6f}" rpy="{rx:.6f} {ry:.6f} {rz:.6f}"/>')
# print()
# print("詳細數值:")
# print(f"x     = {t[0]:.9f} m = {t[0]*1000:.2f} mm")
# print(f"y     = {t[1]:.9f} m = {t[1]*1000:.2f} mm")
# print(f"z     = {t[2]:.9f} m = {t[2]*1000:.2f} mm")
# print(f"roll  = {rx:.9f} rad = {np.degrees(rx):10.6f}°")
# print(f"pitch = {ry:.9f} rad = {np.degrees(ry):10.6f}°")
# print(f"yaw   = {rz:.9f} rad = {np.degrees(rz):10.6f}°")
# print()

# # 驗證
# print("=" * 80)
# print("驗證:")
# print("=" * 80)
# R_verify = rpy_to_rot(rx, ry, rz)
# print(f"旋轉矩陣重建誤差: {np.abs(R - R_verify).max():.12f}")

# T_verify = T_flange_screw @ T_screw_optical
# print(f"完整轉換鏈誤差: {np.abs(T_flange_optical - T_verify).max():.12f}")
# print()

# # 比較角度
# print("=" * 80)
# print("與標準安裝角度比較:")
# print("=" * 80)
# print("標準 D435 安裝 (rpy): 0, -π/2, -π/2")
# print(f"                     0, {-np.pi/2:.6f}, {-np.pi/2:.6f}")
# print(f"你的 calibration (rpy): {rx:.6f}, {ry:.6f}, {rz:.6f}")
# print()

# # 判斷是否接近標準安裝
# angle_diff_y = abs(ry + np.pi/2)
# angle_diff_z = abs(rz + np.pi/2)

# if angle_diff_y < 0.1 and angle_diff_z < 0.1:  # 約 5.7 度
#     print("✅ 安裝角度接近標準 D435 方向！")
#     print("   可以考慮簡化為標準角度:")
#     print(f'   <origin xyz="{t[0]:.6f} {t[1]:.6f} {t[2]:.6f}" rpy="0 ${{-pi/2}} ${{-pi/2}}"/>')
# else:
#     print(f"⚠️  安裝角度與標準方向有差異:")
#     print(f"   pitch 差異: {np.degrees(angle_diff_y):.2f}°")
#     print(f"   yaw 差異:   {np.degrees(angle_diff_z):.2f}°")
#     print("   建議使用完整的 calibration 結果")

# print("=" * 80)