# save_pointcloud.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import pcl  # 使用 python-pcl
import numpy as np

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        self.sub = self.create_subscription(PointCloud2,
                                            '/camera_1/points',
                                            self.callback, 10)
        self.once = True

    def callback(self, msg):
        if not self.once:
            return
        self.once = False

        # 將點轉成純 list
        points = [
            [pt[0], pt[1], pt[2]]
            for pt in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ]

        if len(points) == 0:
            self.get_logger().warn("PointCloud is empty")
            return

        np_points = np.array(points, dtype=np.float32)  # shape: (N, 3)
        
        # 轉成 pcl PointCloud
        pcl_cloud = pcl.PointCloud()
        pcl_cloud.from_array(np_points)

        # 儲存成 .pcd
        pcl.save(pcl_cloud, "realsense_snapshot.pcd")
        self.get_logger().info("Saved point cloud to realsense_snapshot.pcd")

rclpy.init()
node = PointCloudSaver()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
