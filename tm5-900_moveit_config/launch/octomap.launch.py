from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',          # Humble 的可執行檔名稱
        name='octomap_server',
        output='screen',
        parameters=[{
            'frame_id': 'map',          # 產生地圖所用座標系
            'base_frame_id': 'camera_depth_optical_frame', 
            'resolution': 0.05,         # 體素大小 (m)
            'sensor_model/max_range': 5.0,  # 感測上限 (m)
            'colorize_map': True        # 將地圖著色以利除錯
        }],
        remappings=[
            ('cloud_in', '/camera/camera/depth/color/points')  # 依你的感測器話題調整
        ]
    )

    return LaunchDescription([octomap_node])
