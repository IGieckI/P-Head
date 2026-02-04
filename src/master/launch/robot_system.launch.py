from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ip_cam1 = LaunchConfiguration('ip_cam1')
    ip_cam2 = LaunchConfiguration('ip_cam2')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ip_cam1',
            default_value='192.168.1.101',
            description='IP address for the cam1 camera'
        ),
        DeclareLaunchArgument(
            'ip_cam2',
            default_value='192.168.1.102',
            description='IP address for the cam2 camera'
        ),

        # --- cam1 Camera Driver ---
        Node(
            package='master',
            executable='camera_driver',
            name='camera_cam1_node',
            parameters=[
                {'camera_ip': ip_cam1},
                {'camera_name': 'camera_cam1'}
            ]
        ),

        # --- cam2 Camera Driver ---
        Node(
            package='master',
            executable='camera_driver',
            name='camera_cam2_node',
            parameters=[
                {'camera_ip': ip_cam2},
                {'camera_name': 'camera_cam2'}
            ]
        ),

        # --- Robot Master Controller ---
        Node(
            package='master',
            executable='robot_controller',
            name='robot_master_controller',
            output='screen'
        ),

        # Connection to the Micro-ROS Agent
        # If connected with USB cable: 'udp4' -> 'serial' and '--port 8888' -> '--port /dev/ttyUSB0'
        ExecuteProcess(
            cmd=['docker', 'run', '-it', '--rm', '--net=host', 'microros/micro-ros-agent:jazzy', 'udp4', '--port', '8888'],
            output='screen'
        )
    ])