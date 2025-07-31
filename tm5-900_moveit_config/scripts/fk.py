#!/usr/bin/env python3
import math
import numpy as np

# TM5-900 DH 參數表
DH = [
    (0.0,          0.000,   0.1451, -math.pi/2),   # joint 1
    (-math.pi/2,   0.4290,  0.0000,  0.0     ),   # joint 2
    (0.0,          0.4115,  0.0000,  0.0     ),   # joint 3
    (math.pi/2,    0.000,  -0.1222,  math.pi/2),   # joint 4
    (0.0,          0.000,   0.1060,  math.pi/2),   # joint 5
    (0.0,          0.000,   0.1144,  0.0     ),   # joint 6
]

def homogeneous_transform(theta, a, d, alpha):
    c, s = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [   c,   -s*ca,    s*sa,    a * c],
        [   s,    c*ca,   -c*sa,    a * s],
        [   0,      sa,       ca,       d ],
        [   0,       0,        0,       1 ],
    ])

def forward_kinematics(joint_angles_deg):
    """
    正向運動學：從關節角度計算TCP位置和姿態
    """
    # 轉換為弧度
    joint_angles_rad = [math.radians(angle) for angle in joint_angles_deg]
    
    # 計算變換矩陣
    T = np.eye(4)
    for i, (theta_offset, a, d, alpha) in enumerate(DH):
        theta = joint_angles_rad[i] + theta_offset
        T_i = homogeneous_transform(theta, a, d, alpha)
        T = T @ T_i
        print(f"Joint {i+1}: theta={math.degrees(theta):6.2f}°, T{i}_frame:")
        print(f"  Position: [{T[0,3]:7.4f}, {T[1,3]:7.4f}, {T[2,3]:7.4f}]")
    
    # 提取TCP位置和旋轉矩陣
    tcp_position = T[:3, 3]
    tcp_rotation = T[:3, :3]
    
    return tcp_position, tcp_rotation, T

def rotation_matrix_to_euler_zyx(R):
    """將旋轉矩陣轉換為ZYX歐拉角（RPY）"""
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

if __name__ == "__main__":
    # 使用RViz中觀察到的角度
    rviz_angles = [154.67, -0.04, -109.58, 19.51, -90.04, 14.76]
    print("=== 使用 RViz 觀察到的關節角度進行正向運動學 ===")
    print("關節角度 (degrees):", rviz_angles)
    print()
    
    tcp_pos, tcp_rot, T_final = forward_kinematics(rviz_angles)
    
    print("=== 計算結果 ===")
    print(f"TCP 位置 (m): [{tcp_pos[0]:7.4f}, {tcp_pos[1]:7.4f}, {tcp_pos[2]:7.4f}]")
    print(f"TCP 旋轉矩陣:")
    for row in tcp_rot:
        print(f"  [{row[0]:8.5f}, {row[1]:8.5f}, {row[2]:8.5f}]")
    
    # 轉換為RPY角度
    rpy = rotation_matrix_to_euler_zyx(tcp_rot)
    print(f"TCP 姿態 (RPY degrees): [{rpy[0]:6.2f}, {rpy[1]:6.2f}, {rpy[2]:6.2f}]")
    
    print("\n=== 完整變換矩陣 ===")
    print(T_final)
    
    print("\n=== 分析 ===")
    print("請比較這個計算出的TCP位置和姿態與您的目標值：")
    print("目標位置: [0.5, -0.1, 0.32]")
    print("目標姿態: 旋轉向量 [-180°, 0°, 50°]")
    
    # 也測試您計算出的角度
    print("\n" + "="*50)
    print("=== 使用您的 IK 解進行驗證 ===")
    your_ik_angles = [-8.9, -23.6, -83.6, -162.7, -111.9, -117.9]
    print("您的 IK 解 (degrees):", your_ik_angles)
    
    tcp_pos2, tcp_rot2, T_final2 = forward_kinematics(your_ik_angles)
    print(f"TCP 位置 (m): [{tcp_pos2[0]:7.4f}, {tcp_pos2[1]:7.4f}, {tcp_pos2[2]:7.4f}]")
    rpy2 = rotation_matrix_to_euler_zyx(tcp_rot2)
    print(f"TCP 姿態 (RPY degrees): [{rpy2[0]:6.2f}, {rpy2[1]:6.2f}, {rpy2[2]:6.2f}]")