from scipy.spatial.transform import Rotation as R
import numpy as np

x_p = 0.4
y_p = 0.0
z_p = 0.2
rx_p = 0.0
ry_p = 0.0
rz_p = 0.0

pose_before = [x_p, y_p, z_p, rx_p, ry_p, rz_p]

x = 0.7
y = 0.2
z = 0.6
rx = 0.1
ry = 0.3
rz = 0.0

pose_after = [x, y, z, rx, ry, rz]

def get_transform_matrix(pose_before, pose_after):
    R1 = R.from_euler('xyz', pose_before[3:]).as_matrix()
    T1 = np.eye(4)
    T1[:3, :3] = R1
    T1[:3, 3] = pose_before[:3]

    R2 = R.from_euler('xyz', pose_after[3:]).as_matrix()
    T2 = np.eye(4)
    T2[:3, :3] = R2
    T2[:3, 3] = pose_after[:3]
    T = np.linalg.inv(T1) @ T2
    return T
def main():
    T = get_transform_matrix(pose_before, pose_after)
    print("Transform Matrix:")
    print(T)

    # Extract translation and rotation
    translation = T[:3, 3]
    rotation_matrix = T[:3, :3]
    
    # Convert rotation matrix to Euler angles (ZYX)
    r = R.from_matrix(rotation_matrix)
    euler_angles = r.as_euler('xyz', degrees=True)

    print("Translation:", translation)
    print("Rotation (Euler angles in degrees):", euler_angles)

if __name__ == "__main__":
    main()
