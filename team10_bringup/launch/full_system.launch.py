from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    world_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        FindPackageShare('metr4202_test_worlds'), '/launch/', world, '.launch.py'
    ]),
    launch_arguments={'gui': gui}.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('turtlebot3_navigation2'), '/launch/navigation2.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time, 'slam': 'True'}.items(),
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('mapping_pkg'), '/launch/camera_only.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    explorer_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        FindPackageShare('exploration'), '/launch/exploration_only.launch.py'
    ]),
    launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='marker_gauntlet',
                            description='horseshoe | marker_gauntlet | slam_stress'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='True'),

        GroupAction([world_launch]),
        TimerAction(period=10.0, actions=[
            GroupAction([nav2_launch]),
            GroupAction([camera_launch]),
        ]),
        TimerAction(period=20.0, actions=[GroupAction([explorer_launch])]),
    ])