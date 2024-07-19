import os

from ament_index_python.packages import get_package_share_directory

from socket import gethostname

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    robot_id = gethostname()

    # adding identification
    communication_identification = Node(
        package='communication_controller',
        executable='identification',
        name=f'identification_{robot_id}',
        output='screen'
    )

    # adding exploration
    communication_exploration = Node(
        package='communication_controller',
        executable='exploration',
        name=f'exploration_{robot_id}',
        output='screen',
        remappings=[('/cmd_vel', f'{robot_id}/cmd_vel'), ('/scan', f'{robot_id}/scan')],
    )

    # adding info_status
    communication_info_status = Node(
        package='communication_controller',
        executable='info_status',
        name=f'info_status_{robot_id}',
        parameters=[
            {'robot_id': robot_id}
        ],
        output='screen'
    )

    limo_chassis_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('limo_bringup'), 'launch/limo_start.launch.py')
        ),
        launch_arguments={
            'odom_topic_name': f'{robot_id}/odom'
        }.items()
    )

    limo_cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('limo_bringup'), 'launch/cartographer.launch.py')
        ),
    )

    return LaunchDescription([
        communication_identification,
        communication_exploration,
        communication_info_status,
        limo_chassis_driver_launch,
        limo_cartographer_launch,
    ])
