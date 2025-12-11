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


# # #Check moi mui nhon
# #     [ 0.003114,  -0.030439, 0.368000, 0.091566779, 0.470108250, 0.059172573],
# #     [ 0.038156, -0.030897, 0.369000, 0.126880807, 0.469529563, 0.058738157],
# #     [ 0.003520, 0.004479, 0.367000, 0.091021235,  0.434946281, 0.058751681],
# #     [ 0.038418, 0.004091, 0.367000, 0.126267408,  0.434411052, 0.058939596],

# # #Check moi chuan, lay Z rieng, X,Y sau, hole
# #     [ 0.034403136,  0.007890912, 0.332000, 0.122352842, 0.430375795, 0.093981151],
# #     [ 0.022052417, -0.018048907, 0.333000, 0.110511023, 0.456904241, 0.093803586],
# #     [ 0.195586310,  0.066720475, 0.36000, 0.280247594, 0.368326119, 0.067389198],
# #     [ 0.119788968, -0.019979896, 0.337000, 0.206910800, 0.455843014, 0.093861271],
# #     [ 0.087023598, 0.048848367, 0.358000, 0.173736309, 0.388155642, 0.067977571], 
    
# #     [ 0.069880082, 0.001633285, 0.286000, 0.158409726, 0.435692112, 0.141555611], 
# #     [ -0.034071744,-0.001251125, 0.284000, 0.053430111, 0.441259185, 0.141447960], 
# #     [ 0.050129454, -0.049890154, 0.261000, 0.139925146, 0.487772761, 0.166464987], 
# #     [ 0.014964660, 0.010081686, 0.261000, 0.103048588, 0.428634824, 0.166279663], 
# #     [ 0.013898128, -0.004927687, 0.332000, 0.101898138, 0.443950693, 0.095143671], 

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

import numpy as np
import pandas as pd

def compute_T_and_rms(data: np.ndarray):
    XC, YC, ZC = data[:,0], data[:,1], data[:,2]
    XA, YA, ZA = data[:,3], data[:,4], data[:,5]
    C = np.column_stack([XC, YC, ZC, np.ones_like(XC)])

    params_x = np.linalg.pinv(C) @ XA
    params_y = np.linalg.pinv(C) @ YA
    params_z = np.linalg.pinv(C) @ ZA

    T = np.eye(4)
    T[0,:4] = params_x
    T[1,:4] = params_y
    T[2,:4] = params_z

    PC = np.column_stack([XC, YC, ZC, np.ones_like(XC)])
    PA_pred = (PC @ T[:3,:4].T)
    PA_true = np.column_stack([XA, YA, ZA])
    err = PA_pred - PA_true
    
    err_norms = np.sqrt(np.sum(err**2, axis=1))
    rms = float(np.sqrt(np.mean(err_norms**2)))
    
    return T, rms, err_norms

# 讀取數據
df = pd.read_csv("handeye_simple_calibration.csv")
DATA = df[['Xc', 'Yc', 'Zc', 'Xr', 'Yr', 'Zr']].values

print(f"Total points: {len(DATA)}")

# 標定
T, rms, err_norms = compute_T_and_rms(DATA)

np.set_printoptions(suppress=True, precision=6)
print("\n=== Calibration Result ===")
print("Transform Camera → TCP:")
print(T)
print(f"\nRMS error: {rms*1000:.2f} mm")
print(f"Max error: {err_norms.max()*1000:.2f} mm")
print(f"Mean error: {err_norms.mean()*1000:.2f} mm")

# 顯示誤差最大的 5 個點
worst_indices = np.argsort(err_norms)[-5:]
print("\nWorst 5 points:")
for idx in worst_indices:
    print(f"  Corner {idx}: {err_norms[idx]*1000:.2f} mm")

# 保存
np.save("camera_to_tcp_transform.npy", T)
print("\n✓ Saved to 'camera_to_tcp_transform.npy'")