import numpy as np
from scipy.spatial.transform import Rotation as Rot
# T_flange_camera_optical= np.array([
#     [-9.99420242e-01,  3.40429613e-02, -5.06696784e-04,  2.52975073e-02],
#     [-3.40443073e-02, -9.99416001e-01,  2.93985984e-03, -8.53863223e-02],
#     [-4.06319338e-04,  2.95540558e-03,  9.99995550e-01,  6.43833999e-02],
#     [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

# T_flange_camera_optical = np.array([
#     [-0.99934372,  0.0341722,   0.01201646,  0.02955424],
#     [-0.03437293, -0.99926592, -0.01691517, -0.08029949],
#     [ 0.01142961, -0.01731711,  0.99978472,  0.0642658 ],
#     [ 0.,          0.,          0.,          1.,        ]])

T_flange_camera_optical= np.array([
    [-0.99942486,  0.03322565,  0.00678327,  0.03121642],
    [-0.03325844, -0.99943536, -0.00477955, -0.08367524],
    [ 0.00662064, -0.00500241,  0.99996557,  0.0654927 ],
    [ 0.,          0.,          0.,          1.]
])

R_optical_to_link = np.array([
            [ 0,  0,  1],
            [-1,  0,  0],
            [ 0, -1,  0]
        ])
T_optical_to_link = np.eye(4)
T_optical_to_link[:3, :3] = np.linalg.inv(R_optical_to_link)
T_optical_to_link[:3, 3] = [0, 0.0, 0]

T_flange_camera_link = T_flange_camera_optical @ T_optical_to_link
print("T_flange_camera_link:\n", T_flange_camera_link)
print("T_flange_camera_optical:\n", T_flange_camera_optical)
print("T_optical_to_link:\n", T_optical_to_link)
print("T_flange_camera_optical =\n", T_flange_camera_optical)

t = T_flange_camera_link[:3, 3]
rpy = Rot.from_matrix(T_flange_camera_link[:3, :3]).as_euler('xyz', degrees=False)

print("\n" + "=" * 70)
print("最終結果:")
print("=" * 70)
print(f'<origin xyz="{t[0]:.6f} {t[1]:.6f} {t[2]:.6f}" rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/>')

# import numpy as np
# import cv2
# import pandas as pd
# from scipy.spatial.transform import Rotation as R

# # =========================
# # 工具
# # =========================

# def invT(T):
#     Rm = T[:3, :3]
#     t = T[:3, 3]
#     Ti = np.eye(4)
#     Ti[:3, :3] = Rm.T
#     Ti[:3, 3] = -Rm.T @ t
#     return Ti


# def pose6d_to_T(p):
#     """
#     p = [x, y, z, rx, ry, rz]
#     rx,ry,rz = ZYX Euler (robot feedback)
#     """
#     T = np.eye(4)
#     T[:3, 3] = p[:3]
#     T[:3, :3] = R.from_euler('zyx', p[3:], degrees=False).as_matrix()
#     return T


# def solvePnP_to_T(rvec, tvec):
#     Rm, _ = cv2.Rodrigues(rvec)
#     T = np.eye(4)
#     T[:3, :3] = Rm
#     T[:3, 3] = tvec.reshape(3)
#     return T


# # =========================
# # 主程式
# # =========================

# # 1. 讀 CSV（只取第一筆拍照姿勢）
# df = pd.read_csv("handeye_simple_calibration.csv")
# Pr = df.loc[:, ["Xr","Yr","Zr"]].values
# flange_cap = df.loc[0, [
#     "flange_cap_x","flange_cap_y","flange_cap_z",
#     "flange_cap_rx","flange_cap_ry","flange_cap_rz"
# ]].values

# T_base_flange = pose6d_to_T(flange_cap)

# # 2. 讀 solvePnP 當下存的 rvec/tvec
# data = np.load("pnp_capture.npz")
# rvec = data["rvec"]
# tvec = data["tvec"]

# T_camera_chessboard = solvePnP_to_T(rvec, tvec)

# # 3. 棋盤在 base 下的位姿（你實際量到的，填一次就好）
# T_base_chessboard = np.array([
#     [1,0,0, Pr[0,0]],
#     [0,1,0, Pr[0,1]],
#     [0,0,1, Pr[0,2]],
#     [0,0,0,       1]
# ])

# # 4. 閉環計算 flange → camera (optical)
# T_flange_camera_optical = (
#     invT(T_base_flange)
#     @ T_base_chessboard
#     @ invT(T_camera_chessboard)
# )

# # 5. Optical → camera_link（REP-105）
# R_opt_to_link = np.array([
#     [0,  0,  1],
#     [-1, 0,  0],
#     [0, -1,  0]
# ])

# T_opt_to_link = np.eye(4)
# T_opt_to_link[:3, :3] = R_opt_to_link

# T_flange_camera_link = T_flange_camera_optical @ T_opt_to_link

# # =========================
# # URDF 輸出
# # =========================

# t = T_flange_camera_link[:3, 3]
# rpy = R.from_matrix(T_flange_camera_link[:3, :3]).as_euler('xyz', degrees=False)

# print("\nURDF origin:")
# print(f'<origin xyz="{t[0]:.6f} {t[1]:.6f} {t[2]:.6f}" '
#       f'rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/>')





# import numpy as np
# import pandas as pd
# import math
# from scipy.spatial.transform import Rotation as R

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

# # ★★★ 修改：讀取 flange 姿勢，不是 tcp ★★★
# flange_cap = df[['flange_cap_x','flange_cap_y','flange_cap_z',
#                  'flange_cap_rx','flange_cap_ry','flange_cap_rz']].values[0]

# print("=" * 80)
# print("Hand-Eye Calibration: 完整轉換鏈計算 (FLANGE-based)")
# print("=" * 80)
# print(f"數據點數量: {len(df)}")
# print()

# # 1. camera_optical → base (你的 calibration 結果)
# T_base_optical = solve_rigid(Pc, Pr)
# print("1. T_base_optical (Camera Optical Frame → Base):")
# print(T_base_optical)

# # 2. base → flange (拍照時的姿勢)
# # ★★★ 修改：這是 flange 姿勢，不是 tcp ★★★
# tx, ty, tz, rx, ry, rz = flange_cap
# R_rot = rpy_to_rot(rx, ry, rz)
# T_base_flange = np.eye(4)
# T_base_flange[:3,:3] = R_rot
# T_base_flange[:3,3] = [tx, ty, tz]
# print("\n2. T_base_flange (Flange → Base, 拍照姿勢):")
# print(T_base_flange)

# # 3. flange → camera_optical
# # ★★★ 修改：直接從 flange 到 camera_optical，不需要經過 tcp ★★★
# T_flange_optical = invT(T_base_flange) @ T_base_optical
# print("\n3. T_flange_optical (Camera Optical → Flange):")
# print(T_flange_optical)

# # 4. **關鍵轉換**: camera_optical → camera_link (ROS2 Frame)
# # 根據你的圖片: X_ROS2=Z_opt, Y_ROS2=-X_opt, Z_ROS2=-Y_opt
# R_optical_to_ros2 = np.array([
#     [ 0,  0,  1],  # X_ROS2 = Z_optical
#     [-1,  0,  0],  # Y_ROS2 = -X_optical
#     [ 0, -1,  0]   # Z_ROS2 = -Y_optical
# ])

# T_optical_ros2 = np.eye(4)
# T_optical_ros2[:3, :3] = R_optical_to_ros2

# print("\n4. T_optical_ros2 (ROS2 Camera Link → Optical Frame):")
# print(T_optical_ros2)

# # 驗證這個旋轉矩陣 (應該等於 rpy(0, -pi/2, -pi/2))
# rpy_check = R.from_matrix(R_optical_to_ros2).as_euler('xyz', degrees=True)
# print(f"   驗證 RPY (度): [{rpy_check[0]:.1f}, {rpy_check[1]:.1f}, {rpy_check[2]:.1f}]")
# print(f"   應該接近: [0, -90, -90] 或等效旋轉")

# # 5. flange → camera_link (最終 URDF 結果！)
# T_flange_camera_link = T_flange_optical @ T_optical_ros2

# print("\n" + "=" * 80)
# print("**最終結果: T_flange_camera_link (Camera Link → Flange)**")
# print("=" * 80)
# print(T_flange_camera_link)

# # 提取 xyz 和 rpy
# t = T_flange_camera_link[:3, 3]
# R_mat = T_flange_camera_link[:3, :3]
# rpy = R.from_matrix(R_mat).as_euler('xyz', degrees=False)  # xyz 順序

# print("\n" + "=" * 80)
# print("URDF 參數 (填入你的 tm5-900.urdf.xacro):")
# print("=" * 80)
# print(f'<origin xyz="{t[0]:.6f} {t[1]:.6f} {t[2]:.6f}" rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/>')
# print()
# print("詳細數值:")
# print(f"x     = {t[0]:.9f} m = {t[0]*1000:.2f} mm")
# print(f"y     = {t[1]:.9f} m = {t[1]*1000:.2f} mm")
# print(f"z     = {t[2]:.9f} m = {t[2]*1000:.2f} mm")
# print(f"roll  = {rpy[0]:.9f} rad = {np.degrees(rpy[0]):10.6f}°")
# print(f"pitch = {rpy[1]:.9f} rad = {np.degrees(rpy[1]):10.6f}°")
# print(f"yaw   = {rpy[2]:.9f} rad = {np.degrees(rpy[2]):10.6f}°")

# # 驗證完整轉換鏈
# print("\n" + "=" * 80)
# print("驗證:")
# print("=" * 80)
# T_verify = T_base_flange @ T_flange_camera_link @ invT(T_optical_ros2)
# print("重建的 T_base_optical:")
# print(T_verify)
# print(f"\n與原始 T_base_optical 的誤差: {np.abs(T_base_optical - T_verify).max():.12f}")

# if np.abs(T_base_optical - T_verify).max() < 1e-10:
#     print("✅ 驗證成功！轉換鏈正確")
# else:
#     print("❌ 驗證失敗！請檢查轉換鏈")

# # 計算重投影誤差
# print("\n" + "=" * 80)
# print("重投影誤差分析:")
# print("=" * 80)

# errors = []
# for i in range(len(Pc)):
#     # 點在相機座標系
#     pc_homo = np.array([Pc[i, 0], Pc[i, 1], Pc[i, 2], 1.0])
    
#     # 通過轉換到 base
#     pr_pred = T_base_optical @ pc_homo
    
#     # 實際位置
#     pr_actual = np.array([Pr[i, 0], Pr[i, 1], Pr[i, 2]])
    
#     # 誤差
#     error = np.linalg.norm(pr_pred[:3] - pr_actual)
#     errors.append(error)
    
#     print(f"點 {i+1}: 誤差 = {error*1000:.3f} mm")

# errors = np.array(errors)
# print()
# print(f"平均誤差:  {np.mean(errors)*1000:.3f} mm")
# print(f"最大誤差:  {np.max(errors)*1000:.3f} mm")
# print(f"RMS 誤差:  {np.sqrt(np.mean(errors**2))*1000:.3f} mm")
# print(f"標準差:    {np.std(errors)*1000:.3f} mm")

# if np.sqrt(np.mean(errors**2)) * 1000 < 1.0:
#     print("\n✅ 標定精度優秀 (RMS < 1mm)")
# elif np.sqrt(np.mean(errors**2)) * 1000 < 3.0:
#     print("\n✅ 標定精度良好 (RMS < 3mm)")
# elif np.sqrt(np.mean(errors**2)) * 1000 < 5.0:
#     print("\n⚠️  標定精度可接受 (RMS < 5mm)")
# else:
#     print("\n❌ 標定精度較差 (RMS > 5mm)，建議重新標定")

# print("\n" + "=" * 80)
# print("完成！請將上面的 URDF 參數填入你的 URDF 檔案")
# print("=" * 80)