import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Dynamically find the path to your config file after installation
    config_path = os.path.join(
        get_package_share_directory('kitti_converter'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_path],
            remappings=[
                ('/odometry/filtered', '/ekf_odom')
            ]
        )
    ])