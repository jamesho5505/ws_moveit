from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("tm5-900").to_dict()

    # MTC Demo node
    pick_place_demo = Node(
        package="tmr_mtc_tutorial",
        executable="tmr_mtc_node",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])