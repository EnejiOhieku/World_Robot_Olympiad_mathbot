import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command

from launch_ros.actions import Node, RosTimer



def generate_launch_description():

    package_name='wro_robosim' 

    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'description', 'wro_car', 'robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    
    params = {'robot_description': robot_description_config, 'use_sim_time': True}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'wro_car', "-z 0.2"],
                        output='screen')


    ak_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ak_controller"],
    )
    delayed_ak_drive_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_entity,
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
            target_action=spawn_entity,
            on_start=[joint_broad_spawner],
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
    )

    # Launch them all!
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        RosTimer(period=6.0, actions=[spawn_entity]),
        delayed_ak_drive_spawner,
        delayed_joint_broad_spawner,
        rviz,
    ])