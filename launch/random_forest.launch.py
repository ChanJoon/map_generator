from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    float_args = {
        "init_state_x": "0.0",
        "init_state_y": "0.0",
        "map_x_size": "50.0",
        "map_y_size": "50.0",
        "map_z_size": "5.0",
        "map_resolution": "0.1",
        "ObstacleShape_lower_rad": "0.3",
        "ObstacleShape_upper_rad": "0.8",
        "ObstacleShape_lower_hei": "3.0",
        "ObstacleShape_upper_hei": "7.0",
        "ObstacleShape_radius_l": "7.0",
        "ObstacleShape_radius_h": "7.0",
        "ObstacleShape_z_l": "7.0",
        "ObstacleShape_z_h": "7.0",
        "ObstacleShape_theta": "7.0",
        "sensing_radius": "10.0",
        "sensing_rate": "10.0",
    }

    int_args = {
        "map_obs_num": "30",
        "map_circle_num": "30",
    }

    declarations = [DeclareLaunchArgument(name, default_value=val) for name, val in float_args.items()]
    declarations += [DeclareLaunchArgument(name, default_value=val) for name, val in int_args.items()]

    def as_float(name):
        return PythonExpression(["float(", LaunchConfiguration(name), ")"])

    return LaunchDescription(
        declarations
        + [
            Node(
                package="map_generator",
                executable="random_forest",
                output="screen",
                parameters=[
                    {
                        "init_state_x": as_float("init_state_x"),
                        "init_state_y": as_float("init_state_y"),
                        "map/x_size": as_float("map_x_size"),
                        "map/y_size": as_float("map_y_size"),
                        "map/z_size": as_float("map_z_size"),
                        "map/obs_num": LaunchConfiguration("map_obs_num"),
                        "map/resolution": as_float("map_resolution"),
                        "map/circle_num": LaunchConfiguration("map_circle_num"),
                        "ObstacleShape/lower_rad": as_float("ObstacleShape_lower_rad"),
                        "ObstacleShape/upper_rad": as_float("ObstacleShape_upper_rad"),
                        "ObstacleShape/lower_hei": as_float("ObstacleShape_lower_hei"),
                        "ObstacleShape/upper_hei": as_float("ObstacleShape_upper_hei"),
                        "ObstacleShape/radius_l": as_float("ObstacleShape_radius_l"),
                        "ObstacleShape/radius_h": as_float("ObstacleShape_radius_h"),
                        "ObstacleShape/z_l": as_float("ObstacleShape_z_l"),
                        "ObstacleShape/z_h": as_float("ObstacleShape_z_h"),
                        "ObstacleShape/theta": as_float("ObstacleShape_theta"),
                        "sensing/radius": as_float("sensing_radius"),
                        "sensing/rate": as_float("sensing_rate"),
                    }
                ],
            )
        ]
    )
