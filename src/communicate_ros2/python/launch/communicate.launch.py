from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 串口 device/baudrate 以 config/ports.yaml 的 serial 段为唯一来源；
    # 想临时覆盖，用 ros2 run ... --ros-args -p device:=/dev/ttyUSB1 -p baudrate:=9600
    return LaunchDescription([
        DeclareLaunchArgument('ports_yaml', default_value=''),
        Node(
            package='communicate_py',
            executable='communicate_node',
            name='communicate',
            parameters=[{'ports_yaml': LaunchConfiguration('ports_yaml')}],
            output='screen',
        ),
    ])
