from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    float_args = {
        "update_freq": "1.0",
        "resolution": "0.1",
        "width_min": "0.6",
        "width_max": "1.5",
        "complexity": "0.03",
        "fill": "0.3",
        "attenuation": "0.1",
        "road_width": "0.5",
        "connectivity": "0.8",
    }

    int_args = {
        "seed": "511",
        "x_length": "10",
        "y_length": "10",
        "z_length": "3",
        "type": "1",
        "obstacle_number": "50",
        "fractal": "1",
        "add_wall_x": "0",
        "add_wall_y": "1",
        "maze_type": "1",
        "numNodes": "40",
        "nodeRad": "1",
        "roadRad": "10",
    }

    declarations = [DeclareLaunchArgument(k, default_value=v) for k, v in float_args.items()]
    declarations += [DeclareLaunchArgument(k, default_value=v) for k, v in int_args.items()]

    def as_float(name):
        return PythonExpression(["float(", LaunchConfiguration(name), ")"])

    return LaunchDescription(
        declarations
        + [
            Node(
                package="map_generator",
                executable="mockamap_node",
                output="screen",
                parameters=[
                    {
                        "seed": LaunchConfiguration("seed"),
                        "update_freq": as_float("update_freq"),
                        "resolution": as_float("resolution"),
                        "x_length": LaunchConfiguration("x_length"),
                        "y_length": LaunchConfiguration("y_length"),
                        "z_length": LaunchConfiguration("z_length"),
                        "type": LaunchConfiguration("type"),
                        "width_min": as_float("width_min"),
                        "width_max": as_float("width_max"),
                        "obstacle_number": LaunchConfiguration("obstacle_number"),
                        "complexity": as_float("complexity"),
                        "fill": as_float("fill"),
                        "fractal": LaunchConfiguration("fractal"),
                        "attenuation": as_float("attenuation"),
                        "road_width": as_float("road_width"),
                        "add_wall_x": LaunchConfiguration("add_wall_x"),
                        "add_wall_y": LaunchConfiguration("add_wall_y"),
                        "maze_type": LaunchConfiguration("maze_type"),
                        "numNodes": LaunchConfiguration("numNodes"),
                        "connectivity": as_float("connectivity"),
                        "nodeRad": LaunchConfiguration("nodeRad"),
                        "roadRad": LaunchConfiguration("roadRad"),
                    }
                ],
            )
        ]
    )
