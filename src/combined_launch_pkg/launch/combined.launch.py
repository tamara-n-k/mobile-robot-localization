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
                FindPackageShare('kalman_positioning').find('kalman_positioning'),
                'launch',
                'positioning.launch.py'
            )
        ])
    )
    delayed_nodes = TimerAction(
        period=1.0,  # delay in seconds
        actions=[positioning_launch, fake_robot_launch],
    )

    odom_plot_node = Node(
    package='combined_launch_pkg',
    executable='liveplot',
    name='odom_convergence_plot',
    output='screen'
    )


    
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', 'src/combined_launch_pkg/mn_0.0001',
            '/robot_gt',
            '/robot_estimated_odometry'
        ],
        output='screen'

    )


    stop_launch_timer = TimerAction(
        period=30.0,
        actions=[
            Shutdown(reason='Fixed-duration recording completed.')
        ]
    )

    return LaunchDescription([
        odom_plot_node,
        delayed_nodes
        # rosbag_record,
        # stop_launch_timer
    ])
