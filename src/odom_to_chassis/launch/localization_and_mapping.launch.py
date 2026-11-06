import os.path
import datetime
import launch

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

# use ros2 launch <file name> config_file:= <that not mid360> rviz:=true (if you want)
def generate_launch_description():
    fast_lio_path = get_package_share_directory('fast_lio')
    fast_lio_config_path = os.path.join(fast_lio_path, 'config')
    fast_lio_rviz_path = os.path.join(
        fast_lio_path, 'rviz', 'fastlio.rviz')

    livox_driver_path = get_package_share_directory("livox_ros_driver2")
    odom_to_chassis_pkg_path = get_package_share_directory('odom_to_chassis')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    bag_enable = LaunchConfiguration('rosbag')
    prt_odom_log = LaunchConfiguration('debug')  # Set true to print some thing like roll, pitch , yaw,...

    declare_rosbag = DeclareLaunchArgument(
        'rosbag', default_value='false',
        description="use rosbag to record"
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=fast_lio_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=fast_lio_rviz_path,
        description='RViz config file path'
    )
    declare_log_odom = DeclareLaunchArgument(
        "debug", default_value ='False',
        description="Enable to output chassis orientation and position"
    )

    mid360_driver = IncludeLaunchDescription(os.path.join(livox_driver_path, "launch_ROS2/msg_MID360_launch.py"))

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )
    odom_subs_node = Node(
        package="odom_to_chassis",
        executable="odom_to_chassis",
        parameters=[os.path.join(odom_to_chassis_pkg_path, "config", "extrinsic.yaml"),
                    {'debug': prt_odom_log}]
    )
    rosbag_name = os.path.join("/home/anatman/livox_ws/dataset/", str(datetime.datetime.now().timestamp()) + ".bag")
    rosbag_action = launch.actions.ExecuteProcess(cmd=['ros2', 'bag', 'record', '--all-topic', '-o', rosbag_name])

    ld = LaunchDescription()
    ld.add_action(mid360_driver)
    ld.add_action(declare_rosbag)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_log_odom)
    ld.add_action(declare_rviz_config_path_cmd)
    print(bag_enable)
    if bag_enable == 'true':
        ld.add_action(rosbag_action)
    ld.add_action(fast_lio_node)
    ld.add_action(odom_subs_node)
    ld.add_action(rviz_node)
    return ld
