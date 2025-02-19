# Copyright 2021 Open Source Robotics Foundation, Inc.
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    #ignition_ros2_iiwa7_demo_path = os.path.join(
    #    get_package_share_directory('lbr_description'),)

    #xacro_file = os.path.join(ignition_ros2_iiwa7_demo_path,
    #                          'urdf',
    #                          'iiwa7',
    #                          'iiwa7.xacro.urdf')

    ignition_ros2_iiwa7_demo_path = os.path.join(
        get_package_share_directory('iiwa7_moveit_config'),)

    xacro_file = os.path.join(ignition_ros2_iiwa7_demo_path,
                              'config',
                              'iiwa7.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    robot_description = {'robot_description': doc.toxml()}

    #package_path = get_package_share_directory('lbr_description')
    #xacro_file = os.path.join(package_path, 'urdf', 'iiwa7', 'iiwa7.xacro.urdf')
    #controllers_yaml = os.path.join(package_path, 'config', 'iiwa7', 'iiwa7_controllers.yaml')
    package_path = get_package_share_directory('iiwa7_moveit_config')
    xacro_file = os.path.join(package_path, 'config', 'iiwa7.urdf.xacro')
    controllers_yaml = os.path.join(package_path, 'config', 'ros2_controllers.yaml')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'iiwa7',
                   '-allow_renaming', 'true'],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'iiwa7_arm_controller'],
        output='screen'
    )

    load_joint_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            #'/joint_trajectory_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory@ignition.msgs.JointTrajectory',
            #'/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
            ],
        output='screen'
    )
    
    # Controller Manager Node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, controllers_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    lbr_path = get_package_share_directory('lbr_description')

    return LaunchDescription([
        # Launch gazebo environment
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                               'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]),
        ignition_spawn_entity,

        #RegisterEventHandler(
        #    event_handler=OnProcessExit(
        #        target_action=ignition_spawn_entity,
        #        on_exit=[load_joint_state_broadcaster],
        #    )
        #),
        #RegisterEventHandler(
        #    event_handler=OnProcessExit(
        #        target_action=load_joint_state_broadcaster,
        #        on_exit=[load_joint_trajectory_controller],
        #    )
        #),
        #RegisterEventHandler(
        #    event_handler=OnProcessExit(
        #        target_action=load_joint_state_broadcaster,
        #        on_exit=[load_joint_position_controller],
        #    )
        #),
        node_robot_state_publisher,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        bridge,
        #controller_manager_node
    ])