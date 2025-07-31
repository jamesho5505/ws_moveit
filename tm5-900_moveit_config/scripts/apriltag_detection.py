#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import pupil_apriltags
import numpy as np
import transforms3d
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
import time
from geometry_msgs.msg import TransformStamped



class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        self.K = None
        self.pose_pub = self.create_publisher(PoseStamped, '/apriltag_pose', 10)
        self.img_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10)
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_raw',
            self.depth_callback,
            10)
        
        self.bridge = CvBridge()
        self.detector = pupil_apriltags.Detector()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.get_logger().info('Waiting for TF tree...')
        # # 查詢從 'camera_link' 到 'base' 的最新轉換，並設定 1.0 秒的超時
        # # rclpy.time.Time() 作為時間參數表示查詢最新時間的轉換
        # while not self.tf_buffer.can_transform('base', 'camera_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0)):
        #     self.get_logger().warn('Waiting for transform base -> camera_link...')
        #     time.sleep(0.5) # 可以縮短等待時間，讓檢查更頻繁
        # self.get_logger().info('TF tree is ready')
        
        # 初始化 MoveIt action client
        # self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
    
    def transform_to_base(self, pose_stamped):
        try:
            # 等待 tf tree 準備好
            if not self.tf_buffer.can_transform('base', 'camera_link', rclpy.time.Time()):
                self.get_logger().warn('Transform not available yet')
                return None

            # 取得變換
            transform = self.tf_buffer.lookup_transform(
                'base',
                'camera_link',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )

            # 使用 tf2_geometry_msgs 進行轉換
            pose_base = tf2_geometry_msgs.do_transform_pose(
                pose_stamped,
                transform
            )

            # 驗證轉換結果
            if not hasattr(pose_base.pose, 'position'):
                self.get_logger().error('Transform result invalid')
                return None

            self.get_logger().info('Transform successful')
            return pose_base

        except Exception as e:
            self.get_logger().error(f'Transform failed: {e}')
            return None
        
    def move_to_target(self, pose_base):
        # 建立 MoveIt goal
        goal = MoveGroup.Goal()
        goal.request.group_name = "tm_arm"
        goal.request.pose_stamped = pose_base
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 10
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        
        # 送出 goal
        self.move_group_client.wait_for_server()
        future = self.move_group_client.send_goal_async(goal)
        
        return future
    def camera_info_callback(self, msg):
        # 只設定一次
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"Camera K: {self.K}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'Depth error: {e}')

    def listener_callback(self, data):
        if self.K is None:
            self.get_logger().warn("Camera intrinsic matrix (K) is not yet available. Skipping frame.")
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(gray_image)

        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        camera_matrix = np.array([[fx, 0, cx],
                                  [0, fy, cy],
                                  [0, 0, 1]])
        tag_size = 0.08

        half_tag_size = tag_size / 2.0
        obj_pts = np.array([
            [-half_tag_size, -half_tag_size, 0],
            [ half_tag_size, -half_tag_size, 0],
            [ half_tag_size,  half_tag_size, 0],
            [-half_tag_size,  half_tag_size, 0]
        ], dtype=np.float32)


        

        for tag in tags:
            (x, y, w, h) = (int(tag.corners[0][0]), int(tag.corners[0][1]), int(tag.corners[2][0] - tag.corners[0][0]), int(tag.corners[2][1] - tag.corners[0][1]))
            # print(f"tag: {tag.tag_id}, x: {x}, y: {y}, w: {w}, h: {h}")
            (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
            img_pts = tag.corners.astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix, None)
            if success:
                x, y, z = tvec.flatten()
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "camera_link"
                
                pose_msg.pose.position.x = float(x)
                pose_msg.pose.position.y = float(y)
                pose_msg.pose.position.z = float(z)
                # rvec 轉 quaternion
                rot_mat, _ = cv2.Rodrigues(rvec)
                quat = transforms3d.quaternions.mat2quat(rot_mat)
                pose_msg.pose.orientation.w = float(quat[0])
                pose_msg.pose.orientation.x = float(quat[1])
                pose_msg.pose.orientation.y = float(quat[2])
                pose_msg.pose.orientation.z = float(quat[3])
                # 這裡直接發布 tag frame 的 Pose
                self.pose_pub.publish(pose_msg)
                self.get_logger().info(
                    f"Tag {tag.tag_id} pose in camera_link: "
                    f"pos=({x:.3f},{y:.3f},{z:.3f}), quat=({quat[0]:.3f},{quat[1]:.3f},{quat[2]:.3f},{quat[3]:.3f})"
                    )
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "camera_link"
                t.child_frame_id = f"tag_{tag.tag_id}"
                t.transform.translation.x = float(x)
                t.transform.translation.y = float(y)
                t.transform.translation.z = float(z)
                t.transform.rotation.w = float(quat[0])
                t.transform.rotation.x = float(quat[1])
                t.transform.rotation.y = float(quat[2])
                t.transform.rotation.z = float(quat[3])
                self.tf_broadcaster.sendTransform(t)

                pose_base = None
                try:
                    pose_base = self.tf_buffer.transform(
                        pose_msg,
                        'base',  # 目的座標系
                        timeout=rclpy.duration.Duration(seconds=0.5)
                    )
                    self.get_logger().info(
                        f"[pose_base] x={pose_base.pose.position.x:.3f}, "
                        f"y={pose_base.pose.position.y:.3f}, "
                        f"z={pose_base.pose.position.z:.3f}"
                    )
                except Exception as e:
                    self.get_logger().error(f"tf2轉換失敗: {e}")
                    return


                self.get_logger().info(f"Tag ID: {tag.tag_id}, Position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
                depth_value = None
                if hasattr(self, 'depth_image'):
                    h, w = self.depth_image.shape[:2]
                    # 確保座標不超出範圍
                    if 0 <= cY < h and 0 <= cX < w:
                        depth_value = self.depth_image[cY, cX]
                        # 若是16位元整數（常見於depth image），轉成公尺
                        if self.depth_image.dtype == np.uint16:
                            depth_value = float(depth_value) / 1000.0
                msg = f"Tag ID: {tag.tag_id}, solvePnP z={z:.3f} m"
                if depth_value is not None:
                    msg += f", depth={depth_value:.3f} m, diff={abs(z-depth_value):.3f} m"
                self.get_logger().info(msg)
            

            b = (tuple(tag.corners[0].astype(int))[0], tuple(tag.corners[0].astype(int))[1])
            c = (tuple(tag.corners[1].astype(int))[0], tuple(tag.corners[1].astype(int))[1])
            d = (tuple(tag.corners[2].astype(int))[0], tuple(tag.corners[2].astype(int))[1])
            a = (tuple(tag.corners[3].astype(int))[0], tuple(tag.corners[3].astype(int))[1])

            cv2.line(cv_image, a, b, (0, 255, 0), 2, lineType=cv2.LINE_AA)
            cv2.line(cv_image, b, c, (255, 255, 0), 2, lineType=cv2.LINE_AA)
            cv2.line(cv_image, c, d, (255, 0, 0), 2, lineType=cv2.LINE_AA)
            cv2.line(cv_image, d, a, (0, 0, 255), 2, lineType=cv2.LINE_AA)
            cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1)
            ab = np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
            ab_real = ab * z /fx
            print(f"Tag size: {tag_size}, ab: {ab:.3f} pixel, ab_real: {ab_real:.3f} m")
            time.sleep(1)

            cv2.putText(cv_image, f"ID: {tag.tag_id}", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv2.imshow("AprilTag Detection", cv_image)
        cv2.waitKey(1)

 

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()

# img = cv2.imread('/home/jamesho5055/tm_ws/src/tmr_ros2/tm_moveit/tm5-900_moveit_config/scripts/1.jpg')

# cv2.imshow('Image', img)

# res = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# detector = pupil_apriltags.Detector()
# tags = detector.detect(res)
# print(f"Detected {len(tags)} tags")

# for tag in tags:
#     b = (tuple(tag.corners[0].astype(int))[0], tuple(tag.corners[0].astype(int))[1])
#     c = (tuple(tag.corners[1].astype(int))[0], tuple(tag.corners[1].astype(int))[1])
#     d = (tuple(tag.corners[2].astype(int))[0], tuple(tag.corners[2].astype(int))[1])
#     a = (tuple(tag.corners[3].astype(int))[0], tuple(tag.corners[3].astype(int))[1])

#     cv2.line(img, a, b, (0, 255, 0), 2, lineType=cv2.LINE_AA)
#     cv2.line(img, b, c, (255, 255, 0), 2, lineType=cv2.LINE_AA)
#     cv2.line(img, c, d, (255, 0, 0), 2, lineType=cv2.LINE_AA)
#     cv2.line(img, d, a, (0, 0, 255), 2, lineType=cv2.LINE_AA)
#     (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
#     cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)

#     tagFamily = tag.tag_family.decode("utf-8")
#     cv2.putText(img, f"ID: {tag.tag_id}", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

# cv2.imshow("res", img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()