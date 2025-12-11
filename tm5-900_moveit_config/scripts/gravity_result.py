import rosbag2_py
import rclpy.serialization
from geometry_msgs.msg import WrenchStamped

import matplotlib.pyplot as plt


def read_rosbag_wrench_series(bag_path, target_topic):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    timestamp_list = []
    force_x_list = []
    force_y_list = []
    force_z_list = []
    torque_x_list = []
    torque_y_list = []
    torque_z_list = []


    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic != target_topic:
            continue

        wrench_msg = rclpy.serialization.deserialize_message(data, WrenchStamped)

        timestamp_list.append(t * 1e-9)  # ns → sec
        force_x_list.append(wrench_msg.wrench.force.x)
        force_y_list.append(wrench_msg.wrench.force.y)
        force_z_list.append(wrench_msg.wrench.force.z)
        torque_x_list.append(wrench_msg.wrench.torque.x)
        torque_y_list.append(wrench_msg.wrench.torque.y)
        torque_z_list.append(wrench_msg.wrench.torque.z)

    return {
        "timestamp": timestamp_list,
        "fx": force_x_list,
        "fy": force_y_list,
        "fz": force_z_list,
        "tx": torque_x_list,
        "ty": torque_y_list,
        "tz": torque_z_list,
    }


def main():
    bag_path = "rosbag2_2025_12_08-12_49_19/rosbag2_2025_12_08-12_49_19_0.db3"   # 給第一個 DB3 即可，自動讀取序列
    topic_compensated = "/ft_compensated"
    topic_filtered = "/robotiq_force_torque_sensor_broadcaster/wrench_filtered"

    compensated_series = read_rosbag_wrench_series(bag_path, topic_compensated)
    filtered_series = read_rosbag_wrench_series(bag_path, topic_filtered)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)

    ax1 = axes[0,0]
    ax2 = axes[0,1]
    ax3 = axes[1,0]
    ax4 = axes[1,1]

    

    ax1.plot(compensated_series["timestamp"], compensated_series["fx"], label="Fx")
    ax1.plot(compensated_series["timestamp"], compensated_series["fy"], label="Fy")
    ax1.plot(compensated_series["timestamp"], compensated_series["fz"], label="Fz")
    ax1.set_title("Force compensated")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Force (N)")
    ax1.grid(True)
    ax1.legend()
    ax2.plot(compensated_series["timestamp"], compensated_series["tx"], label="Tx")
    ax2.plot(compensated_series["timestamp"], compensated_series["ty"], label="Ty")
    ax2.plot(compensated_series["timestamp"], compensated_series["tz"], label="Tz")
    ax2.set_title("Torque compensated")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Torque (Nm)")
    ax2.grid(True)
    ax2.legend()   

    ax3.plot(filtered_series["timestamp"], filtered_series["fx"], label="Fx")
    ax3.plot(filtered_series["timestamp"], filtered_series["fy"], label="Fy")
    ax3.plot(filtered_series["timestamp"], filtered_series["fz"], label="Fz")
    ax3.set_title("Raw Force")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Force (N)")
    ax3.grid(True)
    ax3.legend()
    ax4.plot(filtered_series["timestamp"], filtered_series["tx"], label="Tx")
    ax4.plot(filtered_series["timestamp"], filtered_series["ty"], label="Ty")
    ax4.plot(filtered_series["timestamp"], filtered_series["tz"], label="Tz")
    ax4.set_title("Raw Force")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Torque (Nm)")
    ax4.grid(True)
    ax4.legend()
    filename = 'result.png'
    plt.tight_layout()
    plt.savefig(filename, dpi=300)   # 先存檔
    plt.show()                       # 再顯示



if __name__ == "__main__":
    main()
