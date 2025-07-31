#!/usr/bin/env python3
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

# Your original DH Table
ORIGINAL_DH = [
    (0.0,          0.000,   0.1451, -math.pi/2),
    (-math.pi/2,   0.4290,  0.0000,  0.0),
    (0.0,          0.4115,  0.0000,  0.0),
    (math.pi/2,    0.000,  -0.1222,  math.pi/2),
    (0.0,          0.000,   0.1060,  math.pi/2),
    (0.0,          0.000,   0.1144,  0.0),
]

def inverse_kinematics_multiple_solutions(target_pos, target_rpy, dh_params):
    """
    Improved inverse kinematics that returns multiple solutions
    to help identify which one matches rviz2
    """
    # Convert RPY to rotation matrix
    r = R.from_euler('xyz', np.radians(target_rpy))
    R06 = r.as_matrix()
    p = np.array(target_pos)

    # Extract DH parameters
    d1 = dh_params[0][2]
    d6 = dh_params[5][2]
    a1 = dh_params[1][1]
    a2 = dh_params[2][1]

    # Calculate wrist center
    z_ee = R06[:, 2]  # z-axis of end effector
    pw = p - d6 * z_ee
    px, py, pz = pw

    solutions = []

    # ---------------------------
    # θ1: Two possible solutions
    # ---------------------------
    theta1_options = [
        math.atan2(py, px),
        math.atan2(py, px) + math.pi
    ]

    for theta1 in theta1_options:
        # Normalize theta1 to [-π, π]
        theta1 = math.atan2(math.sin(theta1), math.cos(theta1))
        
        # ---------------------------
        # θ2, θ3: Elbow up/down solutions
        # ---------------------------
        r = math.hypot(px, py)
        z = pz - d1

        D = (r**2 + z**2 - a1**2 - a2**2) / (2 * a1 * a2)
        
        if abs(D) > 1:
            continue  # Skip unreachable solutions

        # Two elbow configurations
        theta3_options = [
            math.atan2(math.sqrt(1 - D**2), D),   # elbow-up
            math.atan2(-math.sqrt(1 - D**2), D)   # elbow-down
        ]

        for theta3 in theta3_options:
            # Calculate θ2
            phi1 = math.atan2(z, r)
            phi2 = math.atan2(a2 * math.sin(theta3), a1 + a2 * math.cos(theta3))
            theta2 = phi1 - phi2

            # ---------------------------
            # θ4, θ5, θ6: Wrist orientation
            # ---------------------------
            try:
                # Get R03 using forward kinematics for first 3 joints
                joint_angles_first3 = [math.degrees(theta1), math.degrees(theta2), math.degrees(theta3)]
                _, R03, _ = forward_kinematics_with_dh(joint_angles_first3, dh_params[:3])
                R03 = np.array(R03)
                
                # Calculate R36
                R36 = R03.T @ R06
                
                # Solve for wrist angles using proper spherical wrist equations
                theta5 = math.acos(np.clip(R36[2, 2], -1, 1))
                
                if abs(math.sin(theta5)) > 1e-6:  # Non-singular case
                    theta4 = math.atan2(R36[1, 2], R36[0, 2])
                    theta6 = math.atan2(R36[2, 1], -R36[2, 0])
                else:  # Singular case
                    theta4 = 0  # Set to zero by convention
                    if R36[2, 2] > 0:  # theta5 ≈ 0
                        theta6 = math.atan2(-R36[0, 1], R36[0, 0])
                    else:  # theta5 ≈ π
                        theta6 = math.atan2(R36[0, 1], R36[0, 0])

                # Also try the alternative θ5 solution
                theta5_alt = -theta5
                if abs(math.sin(theta5_alt)) > 1e-6:
                    theta4_alt = math.atan2(-R36[1, 2], -R36[0, 2])
                    theta6_alt = math.atan2(-R36[2, 1], R36[2, 0])
                else:
                    theta4_alt = theta4
                    theta6_alt = theta6

                # Store both wrist solutions
                for th4, th5, th6 in [(theta4, theta5, theta6), (theta4_alt, theta5_alt, theta6_alt)]:
                    solution = [
                        math.degrees(theta1),
                        math.degrees(theta2),
                        math.degrees(theta3),
                        math.degrees(th4),
                        math.degrees(th5),
                        math.degrees(th6)
                    ]
                    solutions.append(solution)
                    
            except Exception as e:
                continue  # Skip if calculation fails

    return solutions

def forward_kinematics_with_dh(joint_angles_deg, dh_params):
    """Forward kinematics using DH parameters"""
    joint_angles_rad = [math.radians(angle) for angle in joint_angles_deg]
    
    T = np.eye(4)
    
    for i, (theta_offset, a, d, alpha) in enumerate(dh_params):
        theta = joint_angles_rad[i] + theta_offset
        T_i = homogeneous_transform(theta, a, d, alpha)
        T = T @ T_i
    
    return T[:3, 3], T[:3, :3], None

def homogeneous_transform(theta, a, d, alpha):
    """DH transformation matrix"""
    c, s = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [   c,   -s*ca,    s*sa,    a * c],
        [   s,    c*ca,   -c*sa,    a * s],
        [   0,      sa,       ca,       d ],
        [   0,       0,        0,       1 ],
    ])

def rotation_to_rpy(R):
    """Convert rotation matrix to RPY angles"""
    sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
    if sy > 1e-6:
        x = math.atan2(R[2,1], R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return [math.degrees(x), math.degrees(y), math.degrees(z)]

def find_closest_solution(solutions, rviz_angles):
    """Find the solution closest to rviz2 result"""
    rviz_array = np.array(rviz_angles)
    min_error = float('inf')
    best_solution = None
    best_index = -1
    
    for i, solution in enumerate(solutions):
        solution_array = np.array(solution)
        
        # Calculate angular difference (considering angle wrapping)
        diff = solution_array - rviz_array
        diff = np.array([math.atan2(math.sin(math.radians(d)), math.cos(math.radians(d))) 
                        for d in diff])
        diff = np.degrees(diff)
        
        error = np.linalg.norm(diff)
        
        if error < min_error:
            min_error = error
            best_solution = solution
            best_index = i
    
    return best_solution, best_index, min_error

# Test the improved inverse kinematics
if __name__ == "__main__":
    # Target pose
    target_pos = [0.5, -0.1, 0.32]
    target_rpy = [-180.0, 0.0, 50.0]
    
    # rviz2 reference result
    rviz_result = [154.67, -0.04, -109.58, 19.51, -90.04, 14.76]
    
    print("Target position:", target_pos)
    print("Target RPY:", target_rpy)
    print("rviz2 result:", rviz_result)
    print("\n" + "="*60)
    
    # Find all possible solutions
    solutions = inverse_kinematics_multiple_solutions(target_pos, target_rpy, ORIGINAL_DH)
    
    print(f"Found {len(solutions)} possible solutions:")
    for i, solution in enumerate(solutions):
        print(f"Solution {i+1}: [{', '.join(f'{angle:7.2f}' for angle in solution)}]")
        
        # Verify each solution with forward kinematics
        pos_fk, rot_fk, _ = forward_kinematics_with_dh(solution, ORIGINAL_DH)
        rpy_fk = rotation_to_rpy(rot_fk)
        
        pos_error = np.linalg.norm(np.array(pos_fk) - np.array(target_pos))
        rpy_error = np.linalg.norm(np.array(rpy_fk) - np.array(target_rpy))
        
        print(f"         Verification - Pos error: {pos_error*1000:.2f}mm, RPY error: {rpy_error:.2f}°")
    
    # Find closest solution to rviz2
    if solutions:
        best_solution, best_index, error = find_closest_solution(solutions, rviz_result)
        print(f"\nClosest to rviz2:")
        print(f"Solution {best_index+1}: [{', '.join(f'{angle:7.2f}' for angle in best_solution)}]")
        print(f"Angular error: {error:.2f}°")
        
        # Analyze the differences
        print(f"\nDifference analysis:")
        for i, (calc, rviz) in enumerate(zip(best_solution, rviz_result)):
            diff = calc - rviz
            print(f"Joint {i+1}: {calc:7.2f}° vs {rviz:7.2f}° (diff: {diff:7.2f}°)")

# #!/usr/bin/env python3
# import math, sys
# import numpy as np
# from scipy.spatial.transform import Rotation as R


# # ------------------- 使用者輸入 -------------------
# # 1. 給定 TM5-900 TCP 的目標位置與姿態（XYZ + 四元數）
# # xd, yd, zd = 0.45, 0.65, 0.35
# # qx, qy, qz, qw = 0.1, 0.0, 0.0, 1.0     # 四元數 (x,y,z,w)
# # --------------------------------------------------

# # TM5-900 的 DH 參數表（對應你提供的 Table 2）
# # 每一行是 (theta_offset, a_{i-1}, d_i, alpha_{i-1})
# DH = [
#     (0.0,          0.000,   0.1451, -math.pi/2),   # joint 1
#     (-math.pi/2,   0.4290,  0.0000,  0.0     ),   # joint 2
#     (0.0,          0.4115,  0.0000,  0.0     ),   # joint 3
#     (math.pi/2,    0.000,  -0.1222,  math.pi/2),   # joint 4
#     (0.0,          0.000,   0.1060,  math.pi/2),   # joint 5
#     (0.0,          0.000,   0.1144,  0.0     ),   # joint 6 (TCP 偏移)
#     # (0.0,          0.000,   0.2884,  0.0     ),   # joint 6 (robotiq TCP (0.174) 偏移)
#     # (0.0,          0.000,   0.3184,  0.0     ),   # joint 6 (robotiq TCP (0.174) + camera holder (0.03假設) 偏移)
# ]

# # DH = [
# #     (0.0,        0.0,     0.1452,  0.0),           # J1
# #     (-math.pi/2, 0.0,     0.0,     -math.pi/2),    # J2 (軸轉換，但無 a)
# #     (0.0,        0.4290,  0.0,     0.0),           # J3 (a1)
# #     (0.0,        0.4115, -0.1223,  math.pi/2),     # J4 (a2)
# #     (0.0,        0.0,     0.1060,  math.pi/2),     # J5
# #     (0.0,        0.0,     0.11315, math.pi/2),     # J6
# # ]

# def rot_from_quat(qx, qy, qz, qw):
#     """由四元數回傳 3×3 旋轉矩陣"""
#     R = np.zeros((3,3))
#     norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
#     qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
#     R[0,0] = 1 - 2*(qy*qy + qz*qz)
#     R[0,1] =     2*(qx*qy - qz*qw)
#     R[0,2] =     2*(qx*qz + qy*qw)
#     R[1,0] =     2*(qx*qy + qz*qw)
#     R[1,1] = 1 - 2*(qx*qx + qz*qz)
#     R[1,2] =     2*(qy*qz - qx*qw)
#     R[2,0] =     2*(qx*qz - qy*qw)
#     R[2,1] =     2*(qy*qz + qx*qw)
#     R[2,2] = 1 - 2*(qx*qx + qy*qy)
#     return R

# def homogeneous_transform(theta, a, d, alpha):
#     """
#     根據 DH 公式，輸出對應的 4x4 變換矩陣 A_i(θ).
#     theta, a, d, alpha 都是 scalar (rad, m, m, rad)。
#     """
#     c, s = math.cos(theta), math.sin(theta)
#     ca, sa = math.cos(alpha), math.sin(alpha)
#     return np.array([
#         [   c,   -s*ca,    s*sa,    a * c],
#         [   s,    c*ca,   -c*sa,    a * s],
#         [   0,      sa,       ca,       d ],
#         [   0,       0,        0,       1 ],
#     ])

# # def ik_tm5(xd, yd, zd, R_des):
# #     """
# #     解析式 IK for TM5-900（考慮新的 DH 參數與偏移）。

# #     輸入：
# #       - xd, yd, zd : 目標 TCP 位置 (m)
# #       - R_des      : 3x3 旋轉矩陣（由四元數轉來）
# #     回傳：
# #       - 一組「真正」的 [θ1, θ2, …, θ6]（rad） 或 None （若找不到解）。
# #     """

# #     # ------------------------------------------------------------------------
# #     # Step 1. 計算「手腕中心 (Wrist Center)」的位置 p_wc 
# #     #         = p_tcp - d6 * z_tcp。這裡第 6 節的 d6 = DH[5][2]。
# #     # ------------------------------------------------------------------------
# #     d6   = DH[5][2]       # 讀自 DH 第 6 行 (0.1144 m)
# #     z_tcp = R_des[:, 2]   # 第 6 節末端 (TCP) 的 Z 軸方向
# #     p_tcp = np.array([xd, yd, zd])
# #     p_wc  = p_tcp - d6 * z_tcp
# #     xwc, ywc, zwc = p_wc

# #     # ------------------------------------------------------------------------
# #     # Step 2. 解 θ1  →  θ1 = atan2( ywc, xwc )
# #     # ------------------------------------------------------------------------
# #     theta1_var = math.atan2(ywc, xwc)   # 這裡先算「變數解」 θ1_var

# #     # ------------------------------------------------------------------------
# #     # Step 3. 解 θ2_var, θ3_var：投影到手腕中心所在的垂直平面做余弦定理
# #     # ------------------------------------------------------------------------
# #     #   a1 = DH[1][1] = 0.4290, a2 = DH[2][1] = 0.4115
# #     #   d1 = DH[0][2] = 0.1451
# #     a1 = DH[1][1]
# #     a2 = DH[2][1]
# #     d1 = DH[0][2]
# #     r = math.hypot(xwc, ywc)        # r = sqrt(xwc^2 + ywc^2)
# #     s = zwc - d1

# #     # 余弦定理算 D = cos(θ3_var) 
# #     D = (r*r + s*s - a1*a1 - a2*a2) / (2 * a1 * a2)
# #     if abs(D) > 1.0:
# #         return None   # 無實解 (超出 reach)

# #     # 取其中一種常見的「肘下 (elbow-down)」方案
# #     theta3_var = math.atan2(-math.sqrt(1 - D*D), D)

# #     # 令 φ = atan2(s, r)，ψ = atan2( a2*sin(θ3_var), a1 + a2*cos(θ3_var) )
# #     phi = math.atan2(s, r)
# #     psi = math.atan2(a2 * math.sin(theta3_var),
# #                      a1 + a2 * math.cos(theta3_var))

# #     theta2_var = phi - psi

# #     # ------------------------------------------------------------------------
# #     # Step 4. 解 θ4_var, θ5_var, θ6_var：由 R3_6 = R0_3^T * R_des 拆出
# #     # ------------------------------------------------------------------------
# #     # 先算出前三節的 R0_3 ＝ R_z(θ1+offset1) · R_x(alpha0) ·R_z(θ2+offset2) · R_x(alpha1) · R_z(θ3+offset3) · R_x(alpha2)
# #     # 這邊直接算 4×4 Homogeneous 變換再取左上 3×3
# #     th1 = theta1_var + DH[0][0]  # 真正用在第 1 節的角度 = θ1_var + offset1
# #     th2 = theta2_var + DH[1][0]  # 第 2 節：θ2_var + offset2
# #     th3 = theta3_var + DH[2][0]  # 第 3 節：θ3_var + offset3

# #     T0_1 = homogeneous_transform(th1, DH[0][1], DH[0][2], DH[0][3])
# #     T1_2 = homogeneous_transform(th2, DH[1][1], DH[1][2], DH[1][3])
# #     T2_3 = homogeneous_transform(th3, DH[2][1], DH[2][2], DH[2][3])
# #     R0_3 = (T0_1 @ T1_2 @ T2_3)[:3, :3]

# #     # R3_6 必須滿足 R0_3 @ R3_6 = R_des  →  R3_6 = R0_3^T @ R_des
# #     R3_6 = R0_3.T @ R_des

# #     # 由 R3_6 拆出 θ4_var, θ5_var, θ6_var 
# #     # 注意第 4 節 α3 = DH[3][3] = +pi/2，第 5 節 α4 = DH[4][3] = +pi/2。
# #     # 具體拆法見下方註解。
# #     #
# #     # R3_6 = R_z(θ4_var + offset4) · R_x(alpha3) · R_z(θ5_var + offset5) · R_x(alpha4) · R_z(θ6_var + offset6)
# #     # 但由 DH 表可知
# #     #   offset4 = DH[3][0] = +pi/2
# #     #   offset5 = DH[4][0] = 0
# #     #   offset6 = DH[5][0] = 0
# #     #   alpha3 = DH[3][3] =  pi/2
# #     #   alpha4 = DH[4][3] =  pi/2
# #     #
# #     # 經過簡化(detailed derivation omitted)，最終可用：
# #     #
# #     #   θ5_var = atan2( sqrt( (R3_6[0,2])^2 + (R3_6[1,2])^2 ),  R3_6[2,2] )
# #     #   θ4_var = atan2( R3_6[1,2],  R3_6[0,2] )
# #     #   θ6_var = atan2( R3_6[2,1], -R3_6[2,0] )
# #     #
# #     # 這是「前提是 sin(θ5_var) ≠ 0」的非奇異解。
# #     #
# #     # 若要完整涵蓋奇異情況，要再多做 case-by-case 處理，但此處示範常用解。
# #     #

# #     # 先取出 R3_6 中 (row,column) = (0,2),(1,2),(2,2),(2,1),(2,0) 等元素
# #     r02, r12, r22 = R3_6[0,2], R3_6[1,2], R3_6[2,2]
# #     r21, r20       = R3_6[2,1], R3_6[2,0]

# #     # 計算 θ5_var, θ4_var, θ6_var
# #     theta5_var = math.atan2(math.hypot(r02, r12), r22)
# #     # 若 r22 = ±1 或 sqrt(r02^2 + r12^2)=0，就可能是奇異點，需要特殊處理。
# #     # 這裡先假定 sin(θ5_var) ≠ 0
# #     theta4_var = math.atan2(r12, r02)
# #     theta6_var = math.atan2(r21, -r20)

# #     # ------------------------------------------------------------------------
# #     # Step 5. 合併「變數解」與「偏移」，得到真正的 θ1..θ6
# #     # ------------------------------------------------------------------------
# #     theta1 = theta1_var + DH[0][0]
# #     theta2 = theta2_var + DH[1][0]
# #     theta3 = theta3_var + DH[2][0]
# #     theta4 = theta4_var + DH[3][0]
# #     theta5 = theta5_var + DH[4][0]
# #     theta6 = theta6_var + DH[5][0]

# #     return [theta1, theta2, theta3, theta4, theta5, theta6]

# def ik_tm5_all_solutions(xd, yd, zd, R_des):
#     d6 = DH[5][2]
#     z_tcp = R_des[:, 2]
#     p_tcp = np.array([xd, yd, zd])
#     p_wc = p_tcp - d6 * z_tcp
#     xwc, ywc, zwc = p_wc

#     a1 = DH[2][1]
#     a2 = DH[3][1]
#     d1 = DH[0][2]
#     r = math.hypot(xwc, ywc)
#     s = zwc - d1

#     D = (r*r + s*s - a1*a1 - a2*a2) / (2 * a1 * a2)
#     if abs(D) > 1.0:
#         return []  # 無解

#     theta1_var = math.atan2(ywc, xwc)

#     solutions = []
#     for elbow_sign in [+1, -1]:  # theta3 的正根 / 負根
#         theta3_var = math.atan2(elbow_sign * math.sqrt(1 - D*D), D)
#         phi = math.atan2(s, r)
#         psi = math.atan2(a2 * math.sin(theta3_var),
#                          a1 + a2 * math.cos(theta3_var))
#         theta2_var = phi - psi

#         th1 = theta1_var + DH[0][0]
#         th2 = theta2_var + DH[1][0]
#         th3 = theta3_var + DH[2][0]

#         T0_1 = homogeneous_transform(th1, *DH[0][1:])
#         T1_2 = homogeneous_transform(th2, *DH[1][1:])
#         T2_3 = homogeneous_transform(th3, *DH[2][1:])
#         R0_3 = (T0_1 @ T1_2 @ T2_3)[:3, :3]
#         R3_6 = R0_3.T @ R_des

#         r02, r12, r22 = R3_6[0,2], R3_6[1,2], R3_6[2,2]
#         r21, r20       = R3_6[2,1], R3_6[2,0]

#         for wrist_sign in [+1, -1]:  # theta5 正根 / 負根
#             theta5_var = math.atan2(wrist_sign * math.hypot(r02, r12), r22)
#             if abs(math.sin(theta5_var)) < 1e-6:
#                 continue  # 避免奇異點

#             theta4_var = math.atan2(r12, r02)
#             theta6_var = math.atan2(r21, -r20)

#             # 加上偏移量
#             theta1 = theta1_var + DH[0][0]
#             theta2 = theta2_var + DH[1][0]
#             theta3 = theta3_var + DH[2][0]
#             theta4 = theta4_var + DH[3][0]
#             theta5 = theta5_var + DH[4][0]
#             theta6 = theta6_var + DH[5][0]

#             solutions.append([theta1, theta2, theta3, theta4, theta5, theta6])
#     return solutions

# if __name__ == "__main__":
#     # 1. 輸入目標位置與旋轉向量
#     # xd = float(input("目標位置 x (m)："))
#     # yd = float(input("目標位置 y (m)："))
#     # zd = float(input("目標位置 z (m)："))
#     # rx = float(input("旋轉向量 rx (degree)："))
#     # ry = float(input("旋轉向量 ry (degree)："))
#     # rz = float(input("旋轉向量 rz (degree)："))
#     xd, yd, zd = 0.5, -0.1, 0.32
#     rx, ry, rz = -180.0, 0.0, 50.0  # 旋轉向量 (degree)
#     print("目標位置 (m)：", (xd, yd, zd))
#     print("目標姿態 (旋轉向量)：", (rx, ry, rz))
#     rx, ry, rz = rx*math.pi/180, ry*math.pi/180, rz*math.pi/180  # 轉換為弧度
#     # 2. 由旋轉向量轉為旋轉矩陣
#     rotvec = np.array([rx, ry, rz])
#     R_des = R.from_rotvec(rotvec).as_matrix()
#     print("目標姿態 (旋轉矩陣)：\n", R_des)

#     # 3. 呼叫 IK 解算器
#     sols = ik_tm5_all_solutions(xd, yd, zd, R_des)
#     if sols is None or len(sols) == 0:
#         print("❌ 無 IK 解")
#     else:
#         for i, sol in enumerate(sols):
#             print(f"解 {i+1}:")
#             degs = [math.degrees(t) for t in sol]
#             print(f"✅ IK 解{i+1} (rad)：", [f"{t:+.4f}" for t in sol])
#             print("        (deg)：", [f"{d:+.1f}°" for d in degs])
#     # # 1. 把目標四元數轉成 3×3 旋轉矩陣
#     # xd = float(input("目標位置 x (m)："))
#     # yd = float(input("目標位置 y (m)："))
#     # zd = float(input("目標位置 z (m)："))
#     # qx = float(input("目標姿態四元數 x："))
#     # qy = float(input("目標姿態四元數 y："))
#     # qz = float(input("目標姿態四元數 z："))
#     # qw = float(input("目標姿態四元數 w："))
#     # print("目標位置 (m)：", (xd, yd, zd))
#     # print("目標姿態 (四元數)：", (qx, qy, qz, qw))
#     # R_des = rot_from_quat(qx, qy, qz, qw)
#     # print("目標姿態 (旋轉矩陣)：\n", R_des)

#     # # 2. 呼叫 IK 解算器
#     # sols = ik_tm5_all_solutions(xd, yd, zd, R_des)
#     # if sols is None:
#     #     print("❌ 無 IK 解")
#     # else:
#     #     for i, sol in enumerate(sols):
#     #         print(f"解 {i+1}:")
#     #         degs = [math.degrees(t) for t in sol]
#     #         print(f"✅ IK 解{i+1} (rad)：", [f"{t:+.4f}" for t in sol])
#     #         print("        (deg)：", [f"{d:+.1f}°" for d in degs])
