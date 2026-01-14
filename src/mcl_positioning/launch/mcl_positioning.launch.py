from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    
    # Get the package directory
    pkg_dir = get_package_share_directory('mcl_positioning')
    
    # Default landmarks CSV path
    landmarks_csv_default = os.path.join(os.getcwd(), 'src/mcl_positioning/landmarks.csv')
    
    # Launch arguments with descriptions
    landmarks_csv_arg = DeclareLaunchArgument(
        'landmarks_csv_path',
        default_value=landmarks_csv_default,
        description='Path to CSV file containing landmark positions (id,x,y format)'
    )
    
    num_particles = DeclareLaunchArgument(
        'num_particles',
        default_value='100'
    )
    
    x_min = DeclareLaunchArgument(
        'x_min',
        default_value='-10.0'
    )
    
    x_max = DeclareLaunchArgument(
        'x_max',
        default_value='10.0'
    )
    
    y_min = DeclareLaunchArgument(
        'y_min',
        default_value='-10.0'
    )

    y_max = DeclareLaunchArgument(
        'y_max',
        default_value='10.0'
    )
    
    # Optional logging level argument
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal'],
        description='Logging level for the positioning node'
    )
    
    # Optional namespace argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS namespace for the node (empty string for no namespace)'
    )
    
    # Positioning Node
    positioning_node = Node(
        package='mcl_positioning',
        executable='mcl_positioning_node',
        name='mcl_positioning_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            {
                'landmarks_csv_path': LaunchConfiguration('landmarks_csv_path'),
                'num_particles': LaunchConfiguration('num_particles'),
                'x_min': LaunchConfiguration('x_min'),
                'x_max': LaunchConfiguration('x_max'),
                'y_min': LaunchConfiguration('y_min'),
                'y_max': LaunchConfiguration('y_max'),
            }
        ],
        remappings=[
            # Uncomment and modify if you want to remap input/output topics
            # ('/robot_noisy', '/odometry/noisy'),
            # ('/landmarks_observed', '/sensor/landmarks'),
            # ('/robot_estimated_odometry', '/odometry/filtered'),
        ]
    )
    
    # Info message about the launch
    log_info = LogInfo(
        msg=['Starting MCL Positioning Node with parameters:',
             ' - Landmarks CSV: ', LaunchConfiguration('landmarks_csv_path'),
             ' - Num Particles: ', LaunchConfiguration('num_particles')
            ]
    )
    
    return LaunchDescription([
        landmarks_csv_arg,
        num_particles,
        x_min,
        x_max,
        y_min,
        y_max,
        log_level_arg,
        namespace_arg,
        log_info,
        positioning_node,
    ])
