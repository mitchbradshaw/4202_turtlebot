"""Start only the ArUco camera node.

    ros2 launch mapping_pkg camera_only.launch.py
    ros2 launch mapping_pkg camera_only.launch.py use_compressed:=true \
        image_topic:=/camera/image_raw/compressed
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use /clock from Gazebo or `ros2 bag play --clock`'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('use_compressed', default_value='false'),
        DeclareLaunchArgument('marker_size', default_value='0.1',
                              description='Black-square side length in metres (0.1 from world SDF)'),
        DeclareLaunchArgument('aruco_dictionary', default_value='DICT_6X6_1000'),
    ]

    camera_node = Node(
        package='mapping_pkg',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            # ParameterValue(..., value_type=...) makes launch pass real
            # bool/float parameters instead of strings; declare_parameter in
            # the node would otherwise reject a string for a bool/double.
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'use_compressed': ParameterValue(LaunchConfiguration('use_compressed'), value_type=bool),
            'marker_size': ParameterValue(LaunchConfiguration('marker_size'), value_type=float),
            'aruco_dictionary': LaunchConfiguration('aruco_dictionary'),
        }],
    )

    return LaunchDescription(args + [camera_node])
