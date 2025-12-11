import csv 
import time
import numpy as np
from math import pi
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
import tf_transformations

class PoseListener(Node):
    def __init__(self):
        super().__init__('tm_pose_listener')
        self.pose_sub = self.create_subscription(PoseStamped, '/tool_pose', self.pose_callback, 10)
        # self.force_sub = self.create_subscription(WrenchStamped, 'robotiq_force_torque_sensor_broadcaster/wrench', self.force_callback, 10)
        self.R_b2s = np.eye(3)
        self.R = np.eye(3)

    def pose_callback(self, msg):
        q = msg.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        euler_angles = tf_transformations.euler_from_quaternion([x, y, z, w])
        rx = euler_angles[0]
        ry = euler_angles[1]
        rz = euler_angles[2]
        # roatation matrix
        self.R_b2s = np.array([
            [np.cos(rz)*np.cos(ry), np.cos(rz)*np.sin(ry)*np.sin(rx)-np.sin(rz)*np.cos(rx), np.cos(rz)*np.sin(ry)*np.cos(rx)+np.sin(rz)*np.sin(rx)],
            [np.sin(rz)*np.cos(ry),  np.sin(rz)*np.sin(ry)*np.sin(rx)+np.cos(rz)*np.cos(rx),  np.sin(rz)*np.sin(ry)*np.cos(rx)-np.cos(rz)*np.sin(rx)],
            [-np.sin(ry),            np.cos(ry)*np.sin(rx),                                   np.cos(ry)*np.cos(rx)]
        ])

        # quaternion → rotation matrix
        self.R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
        ])

    def get_rotation_from_b2s(self):
        return self.R_b2s
    
    def get_rotation(self):
        return self.R
    

def load_rotation_matrices(num_pose = 9):
    R_list = []
    for i in range(1, num_pose+1):
        file = f"/home/jamesho5055/ws_moveit/1120_2hz_hong/R{i}_matrix.npy"
        # R_matrix = np.loadtxt(file, delimiter=',')
        R_matrix = np.load(file)
        R_matrix = np.array(R_matrix) #@ np.diag([1, 1, -1])
        R_list.append(R_matrix)
    return np.array(R_list)

def load_force_torque_data():
    # 每組姿態分別儲存
    Fx, Fy, Fz = [], [], []
    Tx, Ty, Tz = [], [], []

    for i in range(1, 10): 
        file = f"/home/jamesho5055/ws_moveit/1120_2hz_hong/R{i}.csv"
        # file = f"wenyu/R{i}.csv"
        fxi, fyi, fzi = [], [], []
        txi, tyi, tzi = [], [], []
        with open(file, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # skip header
            for idx, row in enumerate(reader):
                if idx < 300:
                    continue  # skip first 30 lines to avoid transient
                elif idx >= 800:
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
        # print("F_meas:\n", F_meas)
        Tau_meas = np.column_stack([Tx, Ty, Tz])
    return F_meas, Tau_meas





def estimate_payload_with_tilt(force_measured, torque_measured, rotation_list):
    """
    force_measured : shape (N, 3)  每姿態 [Fx, Fy, Fz]
    torque_measured: shape (N, 3)  每姿態 [Tx, Ty, Tz]
    rotation_list  : list of N 個 3x3 矩陣, 對應論文中的 B_FT R (base -> sensor)

    回傳 dict:
        fx0, fy0, fz0
        tx0, ty0, tz0
        mass
        cx, cy, cz
        tilt_u, tilt_v   # base 安裝傾角（rad）
        Lx, Ly, Lz       # Eq. (15) 的 L 常數
    """
    GRAVITY_MAGNITUDE = 9.787  # m/s^2

    number_of_poses = force_measured.shape[0]

    # =========================
    # 1) Eq. (14)–(20): 用力辨識 Lx,Ly,Lz 與 Fx0,Fy0,Fz0
    # =========================

    # 建 R (3N x 6) 和 f (3N x 1) 對應 Eq. (17)–(20)
    coefficient_rows_force = []
    force_vector_entries = []

    for pose_index in range(number_of_poses):
        rotation_base_to_sensor = np.array(rotation_list[pose_index]) # B_FT R
        rotation_sensor_to_base = rotation_base_to_sensor.T            # B_FT R^T, Eq. (16) 中的 R^T

        # [R^T  I3]  對應 Eq. (16)
        rotation_block = np.hstack([rotation_sensor_to_base, np.eye(3)])
        coefficient_rows_force.append(rotation_block)

        measured_force = force_measured[pose_index, :]
        force_vector_entries.extend(measured_force.tolist())

    coefficient_matrix_force = np.vstack(coefficient_rows_force)   # R in Eq. (18)
    force_vector = np.array(force_vector_entries)                  # f in Eq. (18)

    # Eq. (20): l = (R^T R)^(-1) R^T f
    parameter_vector_force, residuals_force, rank_force, singular_values_force = np.linalg.lstsq(
        coefficient_matrix_force, force_vector, rcond=None
    )

    Lx, Ly, Lz = parameter_vector_force[0:3]  # Eq. (19) 的前三個: Lx,Ly,Lz
    print(f"Lx,Ly,Lz = {Lx:.4f}, {Ly:.4f}, {Lz:.4f} N")
    fx0, fy0, fz0 = parameter_vector_force[3:6]  # 三個力零點 Fx0,Fy0,Fz0
    print(f"Fx0,Fy0,Fz0 = {fx0:.4f}, {fy0:.4f}, {fz0:.4f} N")

    # Eq. (22): G = sqrt(Lx^2 + Ly^2 + Lz^2)
    load_weight = np.sqrt(Lx*Lx + Ly*Ly + Lz*Lz)
    mass = load_weight / GRAVITY_MAGNITUDE

    # Eq. (23): U, V
    # U = arcsin( -Ly / G )
    # V = arctan( -Lx / Lz )
    tilt_u = np.arcsin(-Ly / load_weight)
    tilt_v = np.arctan2(-Lx, Lz)  # 用 atan2 比較穩定, 等價於 Eq. (23) 的 arctan(-Lx/Lz)

    # =========================
    # 2) Eq. (3)–(6): 用力矩辨識 x,y,z 與 k1,k2,k3
    # =========================

    # 這裡的 Fx,Fy,Fz 用原始測量值 (對應 Eq. (3)(5)(6))
    coefficient_rows_torque = []
    torque_vector_entries = []

    for pose_index in range(number_of_poses):
        fx, fy, fz = force_measured[pose_index, :]
        tx, ty, tz = torque_measured[pose_index, :]

        # Eq. (5): [Tx,Ty,Tz]^T = M * [x,y,z,k1,k2,k3]^T
        row_tx = np.array([0.0,   -fz,   fy, 1.0, 0.0, 0.0])
        row_ty = np.array([fz,    0.0,  -fx, 0.0, 1.0, 0.0])
        row_tz = np.array([-fy,   fx,   0.0, 0.0, 0.0, 1.0])

        coefficient_rows_torque.append(row_tx)
        coefficient_rows_torque.append(row_ty)
        coefficient_rows_torque.append(row_tz)

        torque_vector_entries.extend([tx, ty, tz])

    coefficient_matrix_torque = np.vstack(coefficient_rows_torque)  # F in Eq. (7)
    torque_vector = np.array(torque_vector_entries)                 # τ in Eq. (7)

    # Eq. (9): p = (F^T F)^(-1) F^T τ
    parameter_vector_torque, residuals_torque, rank_torque, singular_values_torque = np.linalg.lstsq(
        coefficient_matrix_torque, torque_vector, rcond=None
    )

    cx, cy, cz = parameter_vector_torque[0:3]    # x,y,z
    k1, k2, k3 = parameter_vector_torque[3:6]    # k1,k2,k3

    # =========================
    # 3) Eq. (21): 用 k1,k2,k3 + Fx0,Fy0,Fz0 + x,y,z 求 Tx0,Ty0,Tz0
    # =========================

    tx0 = k1 - fz0 * cy + fy0 * cz
    ty0 = k2 - fx0 * cz + fz0 * cx
    tz0 = k3 - fy0 * cx + fx0 * cy

    

    return {
        "fx0": fx0,
        "fy0": fy0,
        "fz0": fz0,
        "tx0": tx0,
        "ty0": ty0,
        "tz0": tz0,
        "mass": mass,
        "cx": cx,
        "cy": cy,
        "cz": cz,
        "tilt_u": tilt_u,   # rad
        "tilt_v": tilt_v,   # rad
        "Lx": Lx,
        "Ly": Ly,
        "Lz": Lz,
        "residuals_force": residuals_force,
        "residuals_torque": residuals_torque,
    }


def compensate(F_meas, Tau_meas, R, Fb, Tau_b, m, r, tilt_u, tilt_v, g=9.787):
    R_w2b = (
        np.array([[1, 0, 0],
                  [0, np.cos(tilt_u), -np.sin(tilt_u)],
                  [0, np.sin(tilt_u), np.cos(tilt_u)]])
        @
        np.array([[np.cos(tilt_v), 0, np.sin(tilt_v)],
                  [0, 1, 0],
                  [-np.sin(tilt_v), 0, np.cos(tilt_v)]])
    )

    # 轉換重力向量從世界座標系到基座座標系
    gI = np.array([0, 0, -g])  # 重力向量（世界座標系）
    gs = R_w2b.T @ gI  # 重力在基座座標系的表示

    # 計算重力補償力 Fg 和力矩 Tg
    Fg = m * gs  # 重力補償力
    Tg = np.cross(r, Fg)  # 重力力矩補償

    # 計算接觸力和接觸力矩的補償
    F_contact = F_meas - Fb - Fg
    T_contact = Tau_meas - Tau_b - Tg
    
    return F_contact, T_contact


def main():
    rclpy.init()
    pose_node = PoseListener()
    print("Pose listener started, waiting for /tool_pose message...")
    time.sleep(1.0) # 等待節點初始化並接收第一則訊息
    R_list  = load_rotation_matrices(num_pose=9)
    # print(f"R:{R_list}, det:{np.linalg.det(R_list)}")
    F_meas, Tau_meas = load_force_torque_data()
    results = estimate_payload_with_tilt(F_meas, Tau_meas, R_list)
    force_bias  = np.array([results['fx0'], results['fy0'], results['fz0']])
    torque_bias = np.array([results['tx0'], results['ty0'], results['tz0']])
    m_star = results['mass']
    r_star  = np.array([results['cx'], results['cy'], results['cz']])
    tilt_u       = results['tilt_u'] * (180/pi)  # 轉成 degree
    tilt_v       = results['tilt_v'] * (180/pi)  # 轉成 degree
    print(f"Fx0,Fy0,Fz0 = {force_bias[0]:.4f}, {force_bias[1]:.4f}, {force_bias[2]:.4f} N")
    print(f"Tx0,Ty0,Tz0 = {torque_bias[0]:.4f}, {torque_bias[1]:.4f}, {torque_bias[2]:.4f} N.m")
    print(f"mass        = {m_star:.4f} kg")
    print(f"x,y,z       = {r_star[0]:.4f}, {r_star[1]:.4f}, {r_star[2]:.4f} m")
    print(f"tilt U,V (degree) = {tilt_u:.4f}, {tilt_v:.4f} degree")
    print(f"\n=== Compensation check on {F_meas.shape[0]} poses ===")

    for pose_index in range(F_meas.shape[0]):
        rotation_current = R_list[pose_index]
        force_avg  = F_meas[pose_index]
        torque_avg = Tau_meas[pose_index]

        compensated_force, compensated_torque = compensate(
            force_avg,
            torque_avg,
            rotation_current,
            force_bias,
            torque_bias,
            m_star,
            r_star,
            0,
            0
        )

        print(f"Pose {pose_index+1}: "
              f"F_comp = {compensated_force} N, T_comp = {compensated_torque} N.m, "
              f"|F_comp| = {np.linalg.norm(compensated_force):.4f} N, "
              f"|T_comp| = {np.linalg.norm(compensated_torque):.4f} N·m")      


    # --- RMSE Calculation ---
    force_residuals = []
    torque_residuals = []

    for pose_index in range(F_meas.shape[0]):
        rotation_current = R_list[pose_index]
        force_avg  = F_meas[pose_index]
        torque_avg = Tau_meas[pose_index]

        compensated_force, compensated_torque = compensate(
            force_avg,
            torque_avg,
            rotation_current,
            force_bias,
            torque_bias,
            m_star,
            r_star,
            0,
            0
        )
        force_residuals.append(compensated_force)
        torque_residuals.append(compensated_torque)

    force_rmse = np.sqrt(np.mean(np.square(force_residuals)))
    torque_rmse = np.sqrt(np.mean(np.square(torque_residuals)))

    print("\n" + "="*30)
    print("Overall Model Error (RMSE):")
    print(f"Force RMSE:  {force_rmse:.4f} N")
    print(f"Torque RMSE: {torque_rmse:.4f} N·m")
    print("="*30)

    # try:
    #     while True:
    #         rclpy.spin_once(pose_node, timeout_sec=0.1) # 處理 ROS 回呼
    #         time.sleep(1)
    # except KeyboardInterrupt:
    #     print("Shutting down...")
    # finally:
    #     pose_node.destroy_node()
    #     rclpy.shutdown()


if __name__ == '__main__':
    main()