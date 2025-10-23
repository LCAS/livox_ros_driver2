import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    cur_path = os.path.split(os.path.realpath(__file__))[0]
    cur_config_path = os.path.join(cur_path, '../config')
    default_user_config = os.path.join(cur_config_path, 'multi_lidar_hunter.json')
    rviz_config_path = os.path.join(cur_config_path, 'display_point_cloud_ROS2.rviz')

    user_config_arg = DeclareLaunchArgument(
        'user_config_path',
        default_value=default_user_config,
        description='Path to Livox user configuration JSON file'
    )

    livox_ros2_params = [
        {"xfer_format": 0},
        {"multi_topic": 1},
        {"data_src": 0},
        {"publish_freq": 10.0},
        {"output_data_type": 0},
        {"frame_id": 'livox_frame'},
        {"lvx_file_path": '/home/livox/livox_test.lvx'},
        {"user_config_path": LaunchConfiguration('user_config_path')},
        {"cmdline_input_bd_code": 'livox0000000001'}
    ]

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
        remappings=[
            ("/livox/lidar_192_168_1_102", "/front_lidar/points"),
            ("/livox/lidar_192_168_1_130", "/back_lidar/points")
        ]
    )

    return LaunchDescription([
        user_config_arg,
        livox_driver,
    ])
