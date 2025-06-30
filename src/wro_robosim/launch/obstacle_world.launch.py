import math
import os
from random import randint, choice

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command

from launch_ros.actions import Node, RosTimer

cards = [
    [2,0,0,0,0,0],
    [1,0,0,0,0,0],
    [0,0,2,0,0,0],
    [0,0,1,0,0,0],
    [0,0,0,0,2,0],
    [0,0,0,0,1,0],
    [0,2,0,0,0,0],
    [0,1,0,0,0,0],
    [0,0,0,2,0,0],
    [0,0,0,1,0,0],
    [0,0,0,0,2,0],
    [0,0,0,0,1,0],
    [0,2,0,0,2,0],
    [0,2,0,0,1,0],
    [0,1,0,0,2,0],
    [0,2,0,0,1,0],
    [0,1,0,0,2,0],
    [0,1,0,0,1,0],
    [2,0,0,0,0,2],
    [2,0,0,0,0,1],
    [1,0,0,0,0,2],
    [2,0,0,0,0,1],
    [1,0,0,0,0,2],
    [1,0,0,0,0,1],
    [2,0,0,0,2,0],
    [2,0,0,0,1,0],
    [1,0,0,0,2,0],
    [2,0,0,0,1,0],
    [1,0,0,0,2,0],
    [1,0,0,0,1,0],
    [0,2,0,0,0,2],
    [0,2,0,0,0,1],
    [0,1,0,0,0,2],
    [0,2,0,0,0,1],
    [0,1,0,0,0,2],
    [0,1,0,0,0,1]
]

def generate_launch_description():
    first_section_options = [0, 1, 2, 3, 4, 5, 10, 11, 24, 25, 26, 27, 28, 29]
    driving_direction = randint(0, 1)
    sections_data = [cards[choice(first_section_options) if i == 0 else randint(0, 35)] for i in range(4)]

    print("Driving direction from coin toss:", ["anticlockwise", "clockwise"][driving_direction])
    print("Sections configuration from card selection:", sections_data)

    package_name='wro_robosim' 
    pkg_path = os.path.join(get_package_share_directory(package_name))


    data = str(sections_data).replace(' ', '')
    world_xacro_file = os.path.join(pkg_path,'description', 'wro_obstacle_world', 'obstacle_mat.urdf.xacro')
    world_description_config = Command(['xacro ', world_xacro_file, f' data:="{data}"'])
    world_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': world_description_config, 'use_sim_time': True}],
        remappings=[
            ("/robot_description", "/world_description")
        ]
    )


    spawn_world_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'world_description',
                                   '-entity', 'wro_world', "-z 0.01"],
                        output='screen')


    robot_xacro_file = os.path.join(pkg_path,'description', 'wro_car', 'robot.urdf.xacro')
    robot_description_config = Command(['xacro ', robot_xacro_file])
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )


    gazebo_params_file = os.path.join(pkg_path,'config','gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )


    cl = 0.184
    rotate = (driving_direction+0.5)*math.pi
    spawn_robot_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'wro_car', f"-x {0.5+0.9}", f"-y {0.5-1.5*cl/2}", f"-z 0.5", f"-Y {rotate}"],
                        output='screen')


    ak_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ak_controller"],
    )
    delayed_ak_drive_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot_entity,
            on_exit=[ak_drive_spawner],
        )
    )


    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster"],
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=ak_drive_spawner,
            on_exit=[joint_broad_spawner],
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
    )

    # Launch them all!
    return LaunchDescription([
        world_state_publisher,
        robot_state_publisher,
        gazebo,
        RosTimer(period=4.0, actions=[spawn_world_entity]),
        RosTimer(period=8.0, actions=[spawn_robot_entity]),
        delayed_ak_drive_spawner,
        delayed_joint_broad_spawner,
        # rviz,
    ])