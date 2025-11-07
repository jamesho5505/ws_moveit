# fsr_and_gripper.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    fsr_script   = LaunchConfiguration('fsr_script')
    gripper_script = LaunchConfiguration('gripper_script')

    fsr_port   = LaunchConfiguration('fsr_port')
    fsr_baud   = LaunchConfiguration('fsr_baud')
    fsr_topic  = LaunchConfiguration('fsr_topic')

    rq_port    = LaunchConfiguration('rq_port')
    rq_baud    = LaunchConfiguration('rq_baud')
    force_idx  = LaunchConfiguration('force_idx')
    cmd_rate   = LaunchConfiguration('cmd_rate_hz')
    rq_speed   = LaunchConfiguration('rq_speed')
    rq_force   = LaunchConfiguration('rq_force')
    force_set  = LaunchConfiguration('force_set_g')
    rise_time  = LaunchConfiguration('rise_time_s')

    return LaunchDescription([
        # paths to scripts
        DeclareLaunchArgument('fsr_script', default_value='/home/jamesho5055/ws_moveit/src/tm5-900_moveit_config/scripts/fsr.py'),
        DeclareLaunchArgument('gripper_script', default_value='/home/jamesho5055/ws_moveit/src/tm5-900_moveit_config/scripts/robotiq_gripper/gripper_modbus_force_control.py'),

        # FSR args
        DeclareLaunchArgument('fsr_port',  default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('fsr_baud',  default_value='115200'),
        DeclareLaunchArgument('fsr_topic', default_value='/esp32/force'),

        # Gripper params
        DeclareLaunchArgument('rq_port',   default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('rq_baud',   default_value='115200'),
        DeclareLaunchArgument('force_idx', default_value='2'),       # 需要時改成 2
        DeclareLaunchArgument('cmd_rate_hz', default_value='120.0'),
        DeclareLaunchArgument('rq_speed',  default_value='150'),
        DeclareLaunchArgument('rq_force',  default_value='200'),
        DeclareLaunchArgument('force_set_g', default_value='1000.0'),
        DeclareLaunchArgument('rise_time_s', default_value='0.25'),

        # FSR node (script)
        ExecuteProcess(
            cmd=[
                'python3', fsr_script,
                '--port', fsr_port,
                '--baud', fsr_baud,
                '--topic', fsr_topic
            ],
            output='screen',
            respawn=True, respawn_delay=2.0
        ),

        # Gripper node (script with ROS params)
        ExecuteProcess(
            cmd=[
                'python3', gripper_script,
                '--ros-args',
                '-p', ['portname:=', rq_port],
                '-p', ['baudrate:=', rq_baud],
                '-p', ['force_idx:=', force_idx],
                '-p', ['cmd_rate_hz:=', cmd_rate],
                '-p', ['rq_speed:=', rq_speed],
                '-p', ['rq_force:=', rq_force],
                '-p', ['force_set_g:=', force_set],
                '-p', ['rise_time_s:=', rise_time],
            ],
            output='screen',
            respawn=True, respawn_delay=2.0
        ),
    ])
