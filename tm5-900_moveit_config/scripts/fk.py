# #!/usr/bin/env python3
# import math
# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # TM5-900 DH 參數表
# # DH = [
# #     (0.0,          0.000,   0.1451,  0.0),          # joint 1
# #     (-math.pi/2,   0.000,   0.0000,  -math.pi/2),   # joint 2
# #     (0.0,          0.4290,  0.0000,  0.0),          # joint 3
# #     (0.0,          0.4115,  -0.1222, math.pi/2),    # joint 4
# #     (math.pi/2,    0.000,   0.1060,  0.0),          # joint 5
# #     (math.pi/2,    0.000,   0.11315,  0.0),         # joint 6
# # ]

# # 修正後的 TM5-900 DH 參數表 (Standard DH)
# # alpha, a, d, theta_offset
# DH = [
#     # (theta_offset, a, d, alpha)
#     (0.0,                0.0,    0.1452,  0.0),
#     (-math.pi/2.0,       0.0,    0.0,     -math.pi/2.0),
#     (0.0,                0.4290, 0.0,      0.0),
#     (math.pi/2.0,        0.4115, -0.1223,  0.0),
#     (0.0,                0.0,    0.1060,   math.pi/2.0),
#     (0.0,                0.0,    0.11315,  math.pi/2.0),
# ]
# def homogeneous_transform(theta, a, d, alpha):
#     c, s = math.cos(theta), math.sin(theta)
#     ca, sa = math.cos(alpha), math.sin(alpha)
#     return np.array([
#         [   c,   -s*ca,    s*sa,    a * c],
#         [   s,    c*ca,   -c*sa,    a * s],
#         [   0,      sa,       ca,       d ],
#         [   0,       0,        0,       1 ],
#     ])

# # def forward_kinematics(joint_angles_deg):
# #     """
# #     正向運動學：從關節角度計算TCP位置和姿態
# #     """
# #     # 轉換為弧度
# #     joint_angles_rad = [math.radians(angle) for angle in joint_angles_deg]
    
# #     # 計算變換矩陣
# #     T = np.eye(4)
# #     for i, (theta_offset, a, d, alpha) in enumerate(DH):
# #         theta = joint_angles_rad[i] + theta_offset
# #         T_i = homogeneous_transform(theta, a, d, alpha)
# #         T = T @ T_i
# #         print(f"Joint {i+1}: theta={math.degrees(theta):6.2f}°, T{i}_frame:")
# #         print(f"  Position: [{T[0,3]:7.4f}, {T[1,3]:7.4f}, {T[2,3]:7.4f}]")
    
# #     # 提取TCP位置和旋轉矩陣
# #     tcp_position = T[:3, 3]
# #     tcp_rotation = T[:3, :3]
    
# #     return tcp_position, tcp_rotation, T

# def flange_to_tcp_transform():
#     translation_flange_to_tcp = np.array([
#         [-0.01428],
#         [-0.000161],
#         [ 0.21794]
#     ])

#     rotation_flange_to_tcp = np.eye(3)

#     transformation_flange_to_tcp = np.eye(4)
#     transformation_flange_to_tcp[:3, :3] = rotation_flange_to_tcp
#     transformation_flange_to_tcp[:3, 3:4] = translation_flange_to_tcp

#     return transformation_flange_to_tcp


# def forward_kinematics_with_frames(joint_angles_deg):
#     joint_angles_rad = [math.radians(angle) for angle in joint_angles_deg]

#     transformation_base_to_current = np.eye(4)

#     joint_positions_base = []      # 每個 joint 的位置 (x,y,z)
#     joint_transforms_base = []     # 每個 joint 的 4x4 T

#     joint_positions_base.append(transformation_base_to_current[:3, 3])
#     joint_transforms_base.append(transformation_base_to_current.copy())

#     for joint_index, (theta_offset, link_length_a, link_offset_d, link_twist_alpha) in enumerate(DH):
#         joint_angle = joint_angles_rad[joint_index] + theta_offset

#         transform_current = homogeneous_transform(
#             joint_angle,
#             link_length_a,
#             link_offset_d,
#             link_twist_alpha
#         )

#         transformation_base_to_current = transformation_base_to_current @ transform_current

#         joint_positions_base.append(transformation_base_to_current[:3, 3])
#         joint_transforms_base.append(transformation_base_to_current.copy())

#     tcp_position = transformation_base_to_current[:3, 3]
#     tcp_rotation = transformation_base_to_current[:3, :3]

#     return (
#         np.array(joint_positions_base),
#         joint_transforms_base,
#         tcp_position,
#         tcp_rotation
#     )

# def plot_robot_3d(joint_positions_base):
#     x_coords = joint_positions_base[:, 0]
#     y_coords = joint_positions_base[:, 1]
#     z_coords = joint_positions_base[:, 2]

#     fig = plt.figure(figsize=(10, 8))
#     ax = fig.add_subplot(111, projection="3d")

#     # 畫出連桿
#     ax.plot(x_coords, y_coords, z_coords, "-o", linewidth=5, markersize=8, markerfacecolor="black", label="Arm Segments")
    
#     # 標註各個關節點
#     for i, (x, y, z) in enumerate(joint_positions_base):
#         ax.text(x, y, z, f' J{i}', color='blue')

#     # TCP 特別標註
#     ax.scatter(x_coords[-1], y_coords[-1], z_coords[-1], color="red", s=100, label="TCP")

#     ax.set_xlabel("X (m)")
#     ax.set_ylabel("Y (m)")
#     ax.set_zlabel("Z (m)")
    
#     # 強制設定坐標軸範圍，防止變形
#     ax.set_xlim3d([-0.6, 0.6])
#     ax.set_ylim3d([-0.6, 0.6])
#     ax.set_zlim3d([0, 1.0])
    
#     ax.set_title("TM5-900 Kinematic Structure")
#     ax.legend()
#     plt.show()




# def rotation_matrix_to_euler_zyx(R):
#     """將旋轉矩陣轉換為ZYX歐拉角（RPY）"""
#     sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
    
#     if sy > 1e-6:
#         x = math.atan2(R[2,1], R[2,2])
#         y = math.atan2(-R[2,0], sy)
#         z = math.atan2(R[1,0], R[0,0])
#     else:
#         x = math.atan2(-R[1,2], R[1,1])
#         y = math.atan2(-R[2,0], sy)
#         z = 0
    
#     return [math.degrees(x), math.degrees(y), math.degrees(z)]

# if __name__ == "__main__":
#     # 使用RViz中觀察到的角度
#     # rviz_angles = [154.67, -0.04, -109.58, 19.51, -90.04, 14.76]
#     # print("=== 使用 RViz 觀察到的關節角度進行正向運動學 ===")
#     # print("關節角度 (degrees):", rviz_angles)
#     # print()
    
#     # tcp_pos, tcp_rot, T_final = forward_kinematics(rviz_angles)
    
#     # print("=== 計算結果 ===")
#     # print(f"TCP 位置 (m): [{tcp_pos[0]:7.4f}, {tcp_pos[1]:7.4f}, {tcp_pos[2]:7.4f}]")
#     # print(f"TCP 旋轉矩陣:")
#     # for row in tcp_rot:
#     #     print(f"  [{row[0]:8.5f}, {row[1]:8.5f}, {row[2]:8.5f}]")
    
#     # # 轉換為RPY角度
#     # rpy = rotation_matrix_to_euler_zyx(tcp_rot)
#     # print(f"TCP 姿態 (RPY degrees): [{rpy[0]:6.2f}, {rpy[1]:6.2f}, {rpy[2]:6.2f}]")
    
#     # print("\n=== 完整變換矩陣 ===")
#     # print(T_final)
    
#     # print("\n=== 分析 ===")
#     # print("請比較這個計算出的TCP位置和姿態與您的目標值：")
#     # print("目標位置: [0.5, -0.1, 0.32]")
#     # print("目標姿態: 旋轉向量 [-180°, 0°, 50°]")
    
#     # # 也測試您計算出的角度
#     # print("\n" + "="*50)
#     # print("=== 使用您的 IK 解進行驗證 ===")
#     # your_ik_angles = [-8.9, -23.6, -83.6, -162.7, -111.9, -117.9]
#     # print("您的 IK 解 (degrees):", your_ik_angles)
    
#     # tcp_pos2, tcp_rot2, T_final2 = forward_kinematics(your_ik_angles)
#     # print(f"TCP 位置 (m): [{tcp_pos2[0]:7.4f}, {tcp_pos2[1]:7.4f}, {tcp_pos2[2]:7.4f}]")
#     # rpy2 = rotation_matrix_to_euler_zyx(tcp_rot2)
#     # print(f"TCP 姿態 (RPY degrees): [{rpy2[0]:6.2f}, {rpy2[1]:6.2f}, {rpy2[2]:6.2f}]")
# # if __name__ == "__main__":
# #     rviz_angles = [0.517, -0.122, 0.461, -179.988, -0.052, 90.000]

# #     (
# #         joint_positions_base,
# #         joint_transforms_base,
# #         tcp_position,
# #         tcp_rotation
# #     ) = forward_kinematics_with_frames(rviz_angles)

# #     plot_robot_3d(joint_positions_base)

#     # 如果這是弧度，請確保函數內部不要再做一次 math.radians()
#     rviz_angles_rad = [0.517, -0.122, 0.461, -179.988, -0.052, 90.000] # 測試直立姿態
    
#     (
#         joint_positions_base,
#         joint_transforms_base,
#         tcp_position,
#         tcp_rotation
#     ) = forward_kinematics_with_frames(rviz_angles_rad)

#     plot_robot_3d(joint_positions_base)

import math
import numpy as np
import matplotlib.pyplot as plt

JOINT_FRAME_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",   # flange
]

TCP_FRAME_NAME = "tcp"

# URDF-equivalent DH table
URDF_EQUIVALENT_DH = [
    # (theta_offset, a, d, alpha)
    (0.0,                0.0,    0.1452,  0.0),
    (-math.pi/2.0,       0.0,    0.0,     -math.pi/2.0),
    (0.0,                0.4290, 0.0,      0.0),
    (math.pi/2.0,        0.4115, -0.1223,  0.0),
    (0.0,                0.0,    0.1060,   math.pi/2.0),
    (0.0,                0.0,    0.11315,  math.pi/2.0),
]

def rotation_x(alpha_rad: float) -> np.ndarray:
    cos_alpha = math.cos(alpha_rad)
    sin_alpha = math.sin(alpha_rad)
    return np.array([
        [1.0, 0.0,      0.0     ],
        [0.0, cos_alpha, -sin_alpha],
        [0.0, sin_alpha,  cos_alpha],
    ])

def rotation_z(theta_rad: float) -> np.ndarray:
    cos_theta = math.cos(theta_rad)
    sin_theta = math.sin(theta_rad)
    return np.array([
        [ cos_theta, -sin_theta, 0.0],
        [ sin_theta,  cos_theta, 0.0],
        [ 0.0,        0.0,       1.0],
    ])

def homogeneous_from_rotation_translation(rotation_3x3: np.ndarray, translation_3: np.ndarray) -> np.ndarray:
    transform_4x4 = np.eye(4)
    transform_4x4[:3, :3] = rotation_3x3
    transform_4x4[:3, 3] = translation_3
    return transform_4x4

def modified_dh_transform(alpha_i: float, a_i: float, theta_i: float, d_i: float) -> np.ndarray:
    # T = Rx(alpha) Tx(a) Rz(theta) Tz(d)
    transform = np.eye(4)

    transform[:3, :3] = rotation_x(alpha_i) @ rotation_z(theta_i)
    # translation = Rx(alpha) * [a, 0, d]
    translation_local = np.array([a_i, 0.0, d_i])
    translation_world = rotation_x(alpha_i) @ translation_local
    transform[:3, 3] = translation_world

    return transform

# def forward_kinematics_urdf_equivalent(joint_angles_deg: list[float]) -> np.ndarray:
#     joint_angles_rad = [math.radians(angle_deg) for angle_deg in joint_angles_deg]

#     transform_base_to_current = np.eye(4)
#     for joint_index, (theta_offset, a_i, d_i, alpha_i) in enumerate(URDF_EQUIVALENT_DH):
#         theta_i = joint_angles_rad[joint_index] + theta_offset
#         transform_current = modified_dh_transform(alpha_i, a_i, theta_i, d_i)
#         transform_base_to_current = transform_base_to_current @ transform_current

#     return transform_base_to_current  # this equals base->link_6 (URDF)

def forward_kinematics_with_all_frames(joint_angles_deg: list[float]):
    joint_angles_rad = [math.radians(angle) for angle in joint_angles_deg]

    transform_base_to_current = np.eye(4)

    transforms_base_to_links = []   # base -> link_i
    positions_base = []             # link_i position

    for joint_index, (theta_offset, a_i, d_i, alpha_i) in enumerate(URDF_EQUIVALENT_DH):
        theta_i = joint_angles_rad[joint_index] + theta_offset
        transform_current = modified_dh_transform(alpha_i, a_i, theta_i, d_i)

        transform_base_to_current = transform_base_to_current @ transform_current

        transforms_base_to_links.append(transform_base_to_current.copy())
        positions_base.append(transform_base_to_current[:3, 3].copy())

    # flange (= link_6)
    transform_base_to_flange = transforms_base_to_links[-1]

    # tcp
    transform_base_to_tcp = (
        transform_base_to_flange
        @ flange_to_tcp_transform()
    )

    return (
        transforms_base_to_links,
        np.array(positions_base),
        transform_base_to_tcp
    )

def flange_to_tcp_transform() -> np.ndarray:
    transform_flange_to_tcp = np.eye(4)
    transform_flange_to_tcp[:3, 3] = np.array([
        -0.01428,
        -0.000161,
         0.21794
    ])
    return transform_flange_to_tcp

def draw_coordinate_frame(axis_3d, transform, axis_length=0.05):
    origin = transform[:3, 3]
    rotation = transform[:3, :3]

    x_axis = origin + axis_length * rotation[:, 0]
    y_axis = origin + axis_length * rotation[:, 1]
    z_axis = origin + axis_length * rotation[:, 2]

    axis_3d.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]], color="r")
    axis_3d.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]], color="g")
    axis_3d.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]], color="b")

def draw_frame_label(axis_3d, position_3, label_text, offset=(0.01, 0.01, 0.01)):
    axis_3d.text(
        position_3[0] + offset[0],
        position_3[1] + offset[1],
        position_3[2] + offset[2],
        label_text,
        fontsize=10,
        color="black"
    )

def visualize_robot(transforms_base_to_links, positions_base, transform_base_to_tcp):
    figure = plt.figure(figsize=(7, 7))
    axis_3d = figure.add_subplot(111, projection="3d")

    # joints + links
    for i in range(0,5):
        draw_coordinate_frame(axis_3d, transforms_base_to_links[i])
        axis_3d.plot(
            [positions_base[i][0], positions_base[i+1][0]],
            [positions_base[i][1], positions_base[i+1][1]],
            [positions_base[i][2], positions_base[i+1][2]],
            "-o",
            linewidth=4,
            markersize=8,
        )

    for joint_index, transform in enumerate(transforms_base_to_links):
        draw_coordinate_frame(axis_3d, transform)
        joint_position = transform[:3, 3]
        joint_name = JOINT_FRAME_NAMES[joint_index]

        if joint_index == 0:
            offset = (0.03, 0.0, 0.0)
        else:
            offset = (0.0, 0.015, 0.0)
        draw_frame_label(
            axis_3d,
            joint_position,
            joint_name,
            offset=offset
        )
    # axis_3d.plot(
    #     positions_base[:, 0],
    #     positions_base[:, 1],
    #     positions_base[:, 2],
    #     "-o",
    #     linewidth=2,
    #     markersize=6,
    #     label="Joints"
    # )

    # draw joint frames
    for transform in transforms_base_to_links:
        draw_coordinate_frame(axis_3d, transform)

    # TCP
    tcp_position = transform_base_to_tcp[:3, 3]
    axis_3d.scatter(
        tcp_position[0],
        tcp_position[1],
        tcp_position[2],
        color="magenta",
        s=80,
        label="TCP"
    )

    draw_coordinate_frame(axis_3d, transform_base_to_tcp, axis_length=0.08)

    draw_frame_label(
        axis_3d,
        tcp_position,
        f"{TCP_FRAME_NAME}: {tcp_position[0]:.3f}, {tcp_position[1]:.3f}, {tcp_position[2]:.3f} m",
        offset=(0.03, 0.0, 0.015)
    )

    axis_3d.set_xlabel("X (m)")
    axis_3d.set_ylabel("Y (m)")
    axis_3d.set_zlabel("Z (m)")
    axis_3d.set_title("TM5-900 FK Visualization (Joints + TCP)")
    axis_3d.set_box_aspect([1, 1, 1])
    axis_3d.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    rviz_angles = [0, 0, 90, 0, 90, 0]

    (
        transforms_base_to_links,
        positions_base,
        transform_base_to_tcp
    ) = forward_kinematics_with_all_frames(rviz_angles)

    visualize_robot(
        transforms_base_to_links,
        positions_base,
        transform_base_to_tcp
    )

