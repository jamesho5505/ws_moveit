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
    [-1.000,  0.000, -0.000,  0.041],
    [ 0.000,  1.000,  0.000, -0.632],
    [ 0.000,  0.000, -1.000,  0.346],
    [ 0.000,  0.000,  0.000,  1.000]
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