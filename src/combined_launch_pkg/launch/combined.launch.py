from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from launch_ros.actions import Node


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
                FindPackageShare('mcl_positioning').find('mcl_positioning'),
                'launch',
                'mcl_positioning.launch.py'
            )
        ])
    )
    delayed_nodes = TimerAction(
        period=1.0,  # delay in seconds
        actions=[positioning_launch],
    )

    odom_plot_node = Node(
    package='combined_launch_pkg',
    executable='mcl_liveplot',
    name='mcl_convergence_plot',
    output='screen'
    )


    return LaunchDescription([
        odom_plot_node,
        fake_robot_launch,
        delayed_nodes
        # rosbag_record,
        # stop_launch_timer
    ])
