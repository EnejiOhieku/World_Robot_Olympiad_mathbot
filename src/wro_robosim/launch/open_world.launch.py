import math
import os
from random import randint, choice

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command

from launch_ros.actions import Node, RosTimer


def generate_launch_description():
    driving_direction = randint(0, 1)
    walls_extension = [randint(0, 1) for _ in range(4)]
    spawn_locations = [(0.2, 0.75), (0.5, 0.75), (0.8, 0.75), (0.2, 0.25), (0.5, 0.25), (0.8, 0.25)]
    roll_choice = choice([1, 2, 3, 4, 5, 6]) if not walls_extension[0] else choice([2, 3, 5, 6])

    print("Driving direction from coin toss:", ["anticlockwise", "clockwise"][driving_direction])
    print("Wall configuration from coin toss:", walls_extension)
    print("Starting zone selection from die roll is ", roll_choice)

    package_name='wro_robosim' 
    pkg_path = os.path.join(get_package_share_directory(package_name))


    mat_config = str(walls_extension).replace(' ', '')
    world_xacro_file = os.path.join(pkg_path,'description', 'wro_open_world', 'open_mat.urdf.xacro')
    world_description_config = Command(['xacro ', world_xacro_file, f' mat_config:="{mat_config}"'])
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
                                   '-entity', 'wro_world', "-z 0.1"],
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


    spawn_location = spawn_locations[roll_choice-1]
    x, y, rotate = 0.5+spawn_location[0], -0.5+spawn_location[1], (driving_direction+0.5)*math.pi
    spawn_robot_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'wro_car', f"-x {x}", f"-y {y}", f"-z 0.2", f"-Y {rotate}"],
                        output='screen')


    ak_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ak_controller"],
    )
    delayed_ak_drive_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_robot_entity,
            on_start=[ak_drive_spawner],
        )
    )


    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster"],
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_robot_entity,
            on_start=[joint_broad_spawner],
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
    )

    # Launch them all!
    return LaunchDescription([
        gazebo,
        world_state_publisher,
        robot_state_publisher,
        RosTimer(period=4.0, actions=[spawn_world_entity]),
        RosTimer(period=6.0, actions=[spawn_robot_entity]),
        delayed_ak_drive_spawner,
        delayed_joint_broad_spawner,
        # rviz,
    ])