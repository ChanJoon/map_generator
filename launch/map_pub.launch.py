from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("pcd_file", description="Path to PCD file"),
        DeclareLaunchArgument("add_boundary", default_value="0"),
        DeclareLaunchArgument("is_bridge", default_value="0"),
        DeclareLaunchArgument("downsample_res", default_value="0.1"),
        DeclareLaunchArgument("map_offset_x", default_value="0.0"),
        DeclareLaunchArgument("map_offset_y", default_value="0.0"),
        DeclareLaunchArgument("map_offset_z", default_value="0.0"),
        Node(
            package="map_generator",
            executable="map_pub",
            output="screen",
            parameters=[
                {
                    "add_boundary": LaunchConfiguration("add_boundary"),
                    "is_bridge": LaunchConfiguration("is_bridge"),
                    "downsample_res": PythonExpression([
                        "float(", LaunchConfiguration("downsample_res"), ")"
                    ]),
                    "map_offset_x": PythonExpression([
                        "float(", LaunchConfiguration("map_offset_x"), ")"
                    ]),
                    "map_offset_y": PythonExpression([
                        "float(", LaunchConfiguration("map_offset_y"), ")"
                    ]),
                    "map_offset_z": PythonExpression([
                        "float(", LaunchConfiguration("map_offset_z"), ")"
                    ]),
                }
            ],
            arguments=[LaunchConfiguration("pcd_file")],
        ),
    ])
