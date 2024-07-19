# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    
    execute_action = ExecuteProcess(
        cmd=['python3', 'src/simulation_bringup/launch/generate_world.py'],
        output='screen'
    )

    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('simulation_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gazebo_pkg')
    pkg_project_description = get_package_share_directory('limo_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the SDF file from "description" package
    sdf_file = os.path.join(pkg_project_description, 'models', 'limo_diff_drive', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    sdf_file = os.path.join(pkg_project_description, 'models', 'limo_diff_drive1', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc1 = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'diff_drive.sdf'
        ])}.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        namespace='limo_0',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    robot_state_publisher1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher1',
        output='both',
        namespace='limo_1',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc1},
        ]
    )

    # adding identification
    communication_identification = Node(
        package='communication_controller',
        executable='identification',
        name='identification_limo_0',
        output='screen'
    )

    communication_identification1 = Node(
        package='communication_controller',
        executable='identification',
        name='identification_limo_1',
        output='screen'
    )

    # adding exploration
    communication_exploration = Node(
        package='communication_controller',
        executable='exploration',
        name='exploration_limo_0',
        output='screen',
        remappings=[('/cmd_vel', 'limo_0/limo/cmd_vel'), ('/scan', 'limo_0/limo/scan')],
    )

    communication_exploration1 = Node(
        package='communication_controller',
        executable='exploration',
        name='exploration_limo_1',
        output='screen',
        remappings=[('/cmd_vel', 'limo_1/limo/cmd_vel'), ('/scan', 'limo_1/limo/scan')],
    )

    # adding info_status
    communication_info_status = Node(
        package='communication_controller',
        executable='info_status',
        name='info_status_limo_0',
        parameters=[
            {'robot_id': 'limo_0'}
        ],
        output='screen'
    )

    communication_info_status1 = Node(
        package='communication_controller',
        executable='info_status',
        name='info_status_limo_1',
        parameters=[
            {'robot_id': 'limo_1'}
        ],
        output='screen'
    )

    limo_status_publisher = Node(
        package='limo_stub',
        executable='status_publisher',
        name='status_publisher',
        output='screen'
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'limo_diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge0',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge_0.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen',
    )

    bridge1 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge1',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge_1.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen',
    )

    # adding cartographer
    cartographer_config_dir = os.path.join(get_package_share_directory('simulation_bringup'), 'config')
    configuration_basename = 'limo_diff_drive_cartographer.lua'
    configuration_basename1 = 'limo_diff_drive_cartographer1.lua'
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
        namespace='limo_0/limo',
        remappings=[
            ('/points2', '/depth/image/points'),
        ],
    )

    cartographer_occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '0.5'],
        namespace='limo_0/limo',
    )

    cartographer1 = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node1',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename1],
        namespace='limo_1/limo',
        remappings=[
            ('/points2', '/depth/image/points'),
        ],
    )

    cartographer_occupancy_grid1 = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node1',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '0.5'],
        namespace='limo_1/limo',
    )

    # adding map merger and frame broadcaster
    static_map_broadcaster = Node(
        package='communication_controller',
        executable='static_map_broadcaster',
        name='static_map_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gz_sim,
        execute_action,
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        bridge,
        bridge1,
        robot_state_publisher,
        robot_state_publisher1,
        communication_identification,
        communication_identification1,
        communication_exploration,
        communication_exploration1,
        communication_info_status,
        communication_info_status1,
        limo_status_publisher,
        #node_tf2_limo,
        rviz,
        cartographer,
        cartographer1,
        cartographer_occupancy_grid,
        cartographer_occupancy_grid1,
        static_map_broadcaster,
    ])

