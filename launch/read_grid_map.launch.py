from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    float_args = {
        "map_x_size": "20.0",
        "map_y_size": "10.0",
        "map_z_size": "5.0",
        "map_x_origin": "-5.0",
        "map_y_origin": "-10.0",
        "map_z_origin": "0.0",
        "map_resolution": "0.1",
        "map_inflate_radius": "0.2",
        "img_occ_th": "0.6",
    }

    int_args = {
        "map_mode": "1",
        "img_negate": "0",
    }

    bool_args = {
        "map_auto_change": "false",
        "map_publish_grid_centers": "true",
        "use_folder": "true",
    }

    from ament_index_python.packages import get_package_share_directory
    pkg_share = get_package_share_directory('map_generator')
    default_folder = f"{pkg_share}/data/img/maze"
    default_file = f"{default_folder}/maze8.png"

    string_args = {
        "map_frame_id": "map",
        "bag_topic": "point_clouds_topic",
        "folder_path": default_folder,
        "file_path": default_file,
    }

    declarations = [DeclareLaunchArgument(name, default_value=value) for name, value in float_args.items()]
    declarations += [DeclareLaunchArgument(name, default_value=value) for name, value in int_args.items()]
    declarations += [DeclareLaunchArgument(name, default_value=value) for name, value in bool_args.items()]
    declarations += [DeclareLaunchArgument(name, default_value=value) for name, value in string_args.items()]

    def as_float(name):
        return PythonExpression(["float(", LaunchConfiguration(name), ")"])

    return LaunchDescription(
        declarations
        + [
            Node(
                package="map_generator",
                executable="read_grid_map",
                output="screen",
                parameters=[
                    {
                        "map/x_size": as_float("map_x_size"),
                        "map/y_size": as_float("map_y_size"),
                        "map/z_size": as_float("map_z_size"),
                        "map/x_origin": as_float("map_x_origin"),
                        "map/y_origin": as_float("map_y_origin"),
                        "map/z_origin": as_float("map_z_origin"),
                        "map/resolution": as_float("map_resolution"),
                        "map/frame_id": LaunchConfiguration("map_frame_id"),
                        "map/inflate_radius": as_float("map_inflate_radius"),
                        "map/auto_change": LaunchConfiguration("map_auto_change"),
                        "map/publish_grid_centers": LaunchConfiguration("map_publish_grid_centers"),
                        "bag_topic": LaunchConfiguration("bag_topic"),
                        "img/negate": LaunchConfiguration("img_negate"),
                        "img/occ_th": as_float("img_occ_th"),
                        "map/mode": LaunchConfiguration("map_mode"),
                        "folder_path": LaunchConfiguration("folder_path"),
                        "use_folder": LaunchConfiguration("use_folder"),
                        "file_path": LaunchConfiguration("file_path"),
                    }
                ],
            )
        ]
    )
