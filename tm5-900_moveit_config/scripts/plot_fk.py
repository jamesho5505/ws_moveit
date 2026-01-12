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
        -0.00314, 0.01973, 0.21463
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

def draw_force_vector(
    axis_3d,
    origin,
    force_vector,
    scale=0.01,
    color="k",
    label=None
):
    """
    繪製力向量，箭頭長度與力的大小成正比
    
    Args:
        origin: 起點位置
        force_vector: 力向量（未縮放）
        scale: 縮放因子（例如 0.01 表示 1N = 0.01m）
    """
    scaled_force = force_vector * scale
    
    # 避免繪製零向量
    if np.linalg.norm(scaled_force) < 1e-9:
        return
    
    axis_3d.quiver(
        origin[0], origin[1], origin[2],
        scaled_force[0], scaled_force[1], scaled_force[2],
        color=color,
        arrow_length_ratio=0.2,
        linewidth=2
    )

    if label is not None:
        axis_3d.text(
            origin[0] + scaled_force[0] * 1.1,
            origin[1] + scaled_force[1] * 1.1,
            origin[2] + scaled_force[2] * 1.1,
            label,
            color=color,
            fontsize=10
        )

def compute_normal_force_and_tangential_direction(force_xy):
    """
    從 f_x, f_y 計算法向力（在 XY 平面上）
    並推導出切線方向（垂直於法向力）
    
    Args:
        force_xy: np.array([f_x, f_y]) - 在 XY 平面上的力
        
    Returns:
        force_normal: 3D 法向力向量
        tangential_direction: 3D 切線方向向量（單位向量）
        normal_magnitude: 法向力的大小
    """
    # 法向力就是 XY 平面上的力向量
    force_normal = np.array([
        force_xy[0],
        force_xy[1],
        0.0
    ])
    
    normal_magnitude = np.linalg.norm(force_normal)
    print(f"Normal force vector: {force_normal}")
    print(f"Normal force magnitude: {normal_magnitude:.2f} N")
    
    # 切線方向垂直於法向力
    # 在 2D 平面上，垂直向量可以透過旋轉 90 度得到
    # (x, y) -> (-y, x) 或 (y, -x)
    # 這裡選擇逆時針旋轉 90 度
    tangential_direction = np.array([
        -force_xy[1],
        force_xy[0],
        0.0
    ])
    
    # 正規化為單位向量
    if np.linalg.norm(tangential_direction) > 1e-6:
        tangential_direction = tangential_direction / np.linalg.norm(tangential_direction)
    else:
        # 如果法向力為零，則設定預設切線方向
        tangential_direction = np.array([1.0, 0.0, 0.0])
    print(f"Tangential direction (unit vector): {tangential_direction}")
    
    return force_normal, tangential_direction, normal_magnitude


def visualize_robot(transforms_base_to_links, positions_base, transform_base_to_tcp, force_xy):
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

    # 計算法向力和切線方向
    (
        force_normal,
        tangential_direction,
        normal_magnitude
    ) = compute_normal_force_and_tangential_direction(force_xy)

    tcp_origin = transform_base_to_tcp[:3, 3]

    # 繪製Fx
    draw_force_vector(
        axis_3d,
        tcp_origin,
        np.array([force_xy[0], 0.0, 0.0]),
        scale=0.01,
        color="black",
        label=f"Fx ({force_xy[0]:.2f} N)"
    )

    # 繪製Fy
    draw_force_vector(
        axis_3d,
        tcp_origin,
        np.array([0.0, force_xy[1], 0.0]),
        scale=0.01,
        color="navy",
        label=f"Fy ({force_xy[1]:.2f} N)"
    )

    # 繪製法向力
    draw_force_vector(
        axis_3d,
        tcp_origin,
        force_normal,
        scale=0.01,
        color="purple",
        label=f"F_normal ({normal_magnitude:.2f} N)"
    )

    # 繪製切線方向
    draw_force_vector(
        axis_3d,
        tcp_origin,
        tangential_direction * normal_magnitude,
        scale=0.01,
        color="brown",
        label=f"Tangential: {tangential_direction[0]:.2f}, {tangential_direction[1]:.2f}, {tangential_direction[2]:.2f}"
    )

    axis_3d.set_xlabel("X (m)")
    axis_3d.set_ylabel("Y (m)")
    axis_3d.set_zlabel("Z (m)")
    axis_3d.set_title("TM5-900 FK Visualization (Joints + TCP + Forces)")
    axis_3d.set_box_aspect([1, 1, 1])
    axis_3d.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    rviz_angles = [-58.0, -2.0, 120.0, -28.0, 90.0, -148.0]
    force_xy = np.array([3.0, 4.0, 0.0])  # N (f_x, f_y)

    (
        transforms_base_to_links,
        positions_base,
        transform_base_to_tcp
    ) = forward_kinematics_with_all_frames(rviz_angles)

    visualize_robot(
        transforms_base_to_links,
        positions_base,
        transform_base_to_tcp,
        force_xy
    )