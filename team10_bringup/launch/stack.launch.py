"""Start Nav2 + SLAM, the camera nodes and the explorer. No Gazebo here, so this
runs on top of whatever world is already up, or on the real robot.

    ros2 launch team10_bringup stack.launch.py
    ros2 launch team10_bringup stack.launch.py use_sim_time:=false \
        use_compressed:=true image_topic:=/camera/image_raw/compressed
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

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
        launch_arguments={
            'use_sim_time': use_sim_time,
            'image_topic': LaunchConfiguration('image_topic'),
            'use_compressed': LaunchConfiguration('use_compressed'),
        }.items(),
    )

    explorer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('exploration'), '/launch/exploration_only.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='true in Gazebo, false on the real robot'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('use_compressed', default_value='false',
                              description='true when image_topic is a CompressedImage'),

        nav2_launch,
        camera_launch,
        # give SLAM a few seconds to publish the first map before we look for frontiers
        TimerAction(period=10.0, actions=[explorer_launch]),
    ])
