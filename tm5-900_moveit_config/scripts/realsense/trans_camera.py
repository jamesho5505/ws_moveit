# #!/usr/bin/env python3
# # trans_camera.py — compute F_T_C (camera w.r.t flange) from calibration

# import numpy as np
# import math

# # ---------- linear algebra helpers ----------
# def invT(T: np.ndarray) -> np.ndarray:
#     """矩陣反轉"""
#     R, t = T[:3, :3], T[:3, 3]
#     Ti = np.eye(4)
#     Ti[:3, :3] = R.T
#     Ti[:3, 3]  = -R.T @ t
#     return Ti

# def orthonormalize(R: np.ndarray) -> np.ndarray:
#     """正交化旋轉矩陣"""
#     U, _, Vt = np.linalg.svd(R)
#     Rn = U @ Vt
#     if np.linalg.det(Rn) < 0:
#         U[:, -1] *= -1
#         Rn = U @ Vt
#     return Rn

# def rot_to_rpy_xyz(R: np.ndarray):
#     """旋轉矩陣轉 RPY"""
#     sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
#     if sy < 1e-9:
#         roll  = math.atan2(-R[1,2], R[1,1])
#         pitch = math.atan2(-R[2,0], sy)
#         yaw = 0.0
#     else:
#         roll  = math.atan2(R[2,1], R[2,2])
#         pitch = math.atan2(-R[2,0], sy)
#         yaw   = math.atan2(R[1,0], R[0,0])
#     return np.array([roll, pitch, yaw], dtype=float)

# def rpy_to_rotation_matrix(rx, ry, rz):
#     """RPY (弧度) 轉旋轉矩陣"""
#     # Rotation around X
#     Rx = np.array([
#         [1, 0, 0],
#         [0, np.cos(rx), -np.sin(rx)],
#         [0, np.sin(rx), np.cos(rx)]
#     ])
#     # Rotation around Y
#     Ry = np.array([
#         [np.cos(ry), 0, np.sin(ry)],
#         [0, 1, 0],
#         [-np.sin(ry), 0, np.cos(ry)]
#     ])
#     # Rotation around Z
#     Rz = np.array([
#         [np.cos(rz), -np.sin(rz), 0],
#         [np.sin(rz), np.cos(rz), 0],
#         [0, 0, 1]
#     ])
#     return Rz @ Ry @ Rx

# # ---------- 輸入數據 ----------

# # 1. 從標定結果載入 T_cam2tcp (Camera → TCP)
# T_cam2tcp = np.array([
#     [ 0.999148, -0.068919,  0.042433, -0.010169],
#     [-0.033941, -0.798608, -0.626585, -0.461181],
#     [-0.001995, -0.002713,  0.009765,  0.025876],
#     [ 0.,        0.,        0.,        1.      ]
# ], dtype=float)

# # 2. 從 feedback_states 取得 tool0_pose (Base → TCP)
# # tool0_pose: [X, Y, Z, Rx, Ry, Rz] 單位：米, 弧度
# tool0_pose = [
#     0.08409877014160157,   # X
#     -0.5298729248046875,   # Y
#     0.15545707702636719,   # Z
#     -2.094458086152976,    # Rx (弧度)
#     8.112148138452e-05,    # Ry (弧度)
#     3.1067843391941303     # Rz (弧度)
# ]

# # 構建 T_base2tcp
# T_base2tcp = np.eye(4)
# T_base2tcp[:3, :3] = rpy_to_rotation_matrix(tool0_pose[3], tool0_pose[4], tool0_pose[5])
# T_base2tcp[:3, 3] = tool0_pose[:3]

# # ---------- 計算 ----------
# def compute_camera_transform():
#     """
#     計算相機在 base 和 flange 座標系中的位置
#     """
#     print("=" * 70)
#     print("Hand-Eye Calibration Result Processing")
#     print("=" * 70)
    
#     # 步驟 1：計算 T_tcp2cam = inv(T_cam2tcp)
#     T_tcp2cam = invT(T_cam2tcp)
#     T_tcp2cam[:3, :3] = orthonormalize(T_tcp2cam[:3, :3])
    
#     print("\n1. T_tcp2cam (TCP → Camera):")
#     print(T_tcp2cam)
    
#     # 步驟 2：計算 T_base2cam = T_base2tcp @ T_tcp2cam
#     T_base2cam = T_base2tcp @ T_tcp2cam
#     T_base2cam[:3, :3] = orthonormalize(T_base2cam[:3, :3])
    
#     print("\n2. T_base2cam (Base → Camera):")
#     print(T_base2cam)
    
#     # 步驟 3：驗證反推 T_flange2cam
#     # T_flange2cam = inv(T_base2tcp) @ T_base2cam
#     T_flange2cam = invT(T_base2tcp) @ T_base2cam
#     T_flange2cam[:3, :3] = orthonormalize(T_flange2cam[:3, :3])
    
#     print("\n3. T_flange2cam (Flange → Camera, verification):")
#     print(T_flange2cam)
    
#     # 檢查是否等於 T_tcp2cam
#     diff = np.linalg.norm(T_flange2cam - T_tcp2cam)
#     print(f"\nDifference between T_flange2cam and T_tcp2cam: {diff:.9f}")
#     if diff < 1e-6:
#         print("✓ Verification passed!")
#     else:
#         print("⚠️ WARNING: Large difference detected!")
    
#     return T_tcp2cam, T_base2cam

# def main():
#     np.set_printoptions(suppress=True, precision=9)
    
#     T_tcp2cam, T_base2cam = compute_camera_transform()
    
#     # 生成 URDF
#     xyz = T_tcp2cam[:3, 3]
#     rpy = rot_to_rpy_xyz(T_tcp2cam[:3, :3])
    
#     print("\n" + "=" * 70)
#     print("URDF <origin> tag (Flange → Camera):")
#     print("=" * 70)
#     print(f'<origin xyz="{xyz[0]:.9f} {xyz[1]:.9f} {xyz[2]:.9f}"')
#     print(f'        rpy="{rpy[0]:.9f} {rpy[1]:.9f} {rpy[2]:.9f}" />')
    
#     print("\n" + "=" * 70)
#     print("Translation (meters):")
#     print("=" * 70)
#     print(f"  X: {xyz[0]:+.6f} m ({xyz[0]*1000:+.2f} mm)")
#     print(f"  Y: {xyz[1]:+.6f} m ({xyz[1]*1000:+.2f} mm)")
#     print(f"  Z: {xyz[2]:+.6f} m ({xyz[2]*1000:+.2f} mm)")
    
#     print("\n" + "=" * 70)
#     print("Rotation (degrees):")
#     print("=" * 70)
#     print(f"  Roll:  {math.degrees(rpy[0]):+.3f}°")
#     print(f"  Pitch: {math.degrees(rpy[1]):+.3f}°")
#     print(f"  Yaw:   {math.degrees(rpy[2]):+.3f}°")
    
#     # Sanity checks
#     print("\n" + "=" * 70)
#     print("Sanity Checks:")
#     print("=" * 70)
    
#     dist = float(np.linalg.norm(xyz))
#     print(f"Camera distance from TCP: {dist:.3f} m ({dist*1000:.1f} mm)")
    
#     if dist > 0.6:
#         print("⚠️  WARNING: Distance > 0.6m seems large!")
#     elif dist < 0.05:
#         print("⚠️  WARNING: Distance < 50mm seems small!")
#     else:
#         print("✓ Distance looks reasonable")
    
#     R = T_tcp2cam[:3, :3]
#     orthogonality_error = np.linalg.norm(R @ R.T - np.eye(3))
#     print(f"\nRotation matrix orthogonality error: {orthogonality_error:.9f}")
    
#     if orthogonality_error < 1e-6:
#         print("✓ Rotation matrix is orthogonal")
#     else:
#         print("⚠️  Rotation matrix has issues")
    
#     det = np.linalg.det(R)
#     print(f"Rotation matrix determinant: {det:.9f}")
    
#     if abs(det - 1.0) < 1e-6:
#         print("✓ Determinant is +1")
#     else:
#         print("⚠️  Determinant is not +1")

# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
# trans_camera.py — compute F_T_C (camera w.r.t flange) from A_T_C and A_T_F(obs)

import numpy as np
import math

# ---------- linear algebra helpers ----------
def invT(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3]  = -R.T @ t
    return Ti

def orthonormalize(R: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(R)
    Rn = U @ Vt
    if np.linalg.det(Rn) < 0:  # fix improper rotation
        U[:, -1] *= -1
        Rn = U @ Vt
    return Rn

def rot_to_rpy_xyz(R: np.ndarray):
    sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
    if sy < 1e-9:
        roll  = math.atan2(-R[1,2], R[1,1]); pitch = math.atan2(-R[2,0], sy); yaw = 0.0
    else:
        roll  = math.atan2(R[2,1], R[2,2])
        pitch = math.atan2(-R[2,0], sy)
        yaw   = math.atan2(R[1,0], R[0,0])
    return np.array([roll, pitch, yaw], dtype=float)

# ---------- fill your inputs ----------
# ^A T_C (camera w.r.t base) — từ phép fit. ĐƠN VỊ PHẢI LÀ MÉT.
T_A_C = np.array([
    # [ 1.00209167, -0.02519967,  0.11098865,  0.05069174],
    # [-0.02309193, -1.00017401, -0.15746694,  0.49128907],
    # [-1.03775047,  1.61076824, 42.10744209, -13.57211370],
    # [ 0., 0., 0., 1. ]

    # [ 1.00613151, -0.01859431, -0.00712982,  0.09039068],
    # [-0.01389467, -1.00147827,  0.00311466, 0.43842151],
    # [ 0.00218016, -0.00799811, -0.99691601,  0.42706775],
    # [ 0.,          0.,          0.,          1.        ],

    [ 0.9962597,  -0.02670584, -0.00599904,  0.08993219],
    [-0.0227818,  -1.00411728,  0.00384315,  0.4379672 ],
    [ 0.00583895, -0.00947345, -1.00363753,  0.42825773],
    [ 0.,          0.,          0.,          1.        ]
], dtype=float)

# Nếu cột tịnh tiến của bạn đang ở **mm**, bỏ comment dòng dưới:
# T_A_C[:3, 3] /= 1000.0

# ^A T_F(obs) (flange w.r.t base) — lấy từ tf2_echo base->flange ở pose quan sát (đơn vị mét)
# Ví dụ: translation (tx,ty,tz) và ma trận/quat đã đổi sang 4x4:

T_A_F = np.array([
    [-1.0,  -0.0,  -0.0, 0.122266364],
    [ -0.0, 1.0,  0.0, 0.517153570],
    [ 0.0,  0.0,  -1.0, 0.461111498],
    [ 0.0,  0.0,  0.0, 1.0]
], dtype=float)
# Gợi ý: nếu bạn có quaternion q(x,y,z,w) thì tự dựng 4x4 trước khi gán T_A_F.

# ---------- compute F_T_C ----------
def compute_F_T_C(T_A_C_in: np.ndarray, T_A_F_obs: np.ndarray):
    T_A_Cn = T_A_C_in.copy()
    T_A_Cn[:3, :3] = orthonormalize(T_A_Cn[:3, :3])  # loại shear/scale do fit affine

    T_F_C = invT(T_A_F_obs) @ T_A_Cn
    T_F_C[:3, :3] = orthonormalize(T_F_C[:3, :3])

    # sanity check khoảng cách flange->camera
    dist = float(np.linalg.norm(T_F_C[:3, 3]))
    if dist > 0.5:
        print("[WARN] flange->camera distance > 0.5 m. Kiểm tra đơn vị hoặc T_A_C.")
    return T_F_C

def main():
    np.set_printoptions(suppress=True, precision=9)
    T_F_C = compute_F_T_C(T_A_C, T_A_F)

    xyz = T_F_C[:3, 3]
    rpy = rot_to_rpy_xyz(T_F_C[:3, :3])

    print("F_T_C (camera w.r.t flange):")
    print(T_F_C)
    print("\nURDF origin:")
    print(f'xyz="{xyz[0]:.9f} {xyz[1]:.9f} {xyz[2]:.9f}"')
    print(f'rpy="{rpy[0]:.9f} {rpy[1]:.9f} {rpy[2]:.9f}"')

if __name__ == "__main__":
    main()