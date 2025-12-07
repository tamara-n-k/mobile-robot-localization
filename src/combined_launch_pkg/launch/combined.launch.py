from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    fake_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                FindPackageShare('fake_robot').find('fake_robot'),
                'launch',
                'fake_robot.launch.py'
            )
        ])
    )

    positioning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                FindPackageShare('kalman_positioning').find('kalman_positioning'),
                'launch',
                'positioning.launch.py'
            )
        ])
    )

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', 'src/combined_launch_pkg/my_bag',
            '/robot_gt',
            '/robot_estimated_odometry'
        ],
        output='screen'
    )

    return LaunchDescription([
        fake_robot_launch,
        positioning_launch,
        rosbag_record
    ])
