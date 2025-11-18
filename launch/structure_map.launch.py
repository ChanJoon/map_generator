from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    float_args = {
        "map_x_size": "30.0",
        "map_y_size": "30.0",
        "map_z_size": "5.0",
        "map_x_origin": "-15.0",
        "map_y_origin": "-15.0",
        "map_z_origin": "0.0",
        "map_resolution": "0.1",
        "map_inflate_radius": "0.1",
        "params_cylinder_ratio": "0.1",
        "params_circle_ratio": "0.0",
        "params_gate_ratio": "0.0",
        "params_ellip_ratio": "0.0",
        "params_poly_ratio": "0.01",
        "params_w1": "0.1",
        "params_w2": "0.5",
        "params_w3": "2.0",
        "params_seed": "1.0",
        "dyn_v_x_h": "0.1",
        "dyn_v_y_h": "0.1",
        "dyn_v_z_h": "0.0",
        "dyn_dt": "10.0",
    }

    bool_args = {
        "map_simple_2d": "false",
        "map_auto_change": "false",
        "params_add_noise": "false",
        "dataset_save_map": "true",
        "clear_pos": "true",
        "dyn_dyn_mode": "false",
    }

    int_args = {
        "dataset_samples_num": "10000",
        "dataset_start_index": "100000",
    }

    string_args = {
        "map_frame_id": "map",
        "dataset_path": "./data/clear_pos/",
        "clear_path": "./data/clear_pos/clear_3d.csv",
    }

    declarations = [DeclareLaunchArgument(name, default_value=value) for name, value in float_args.items()]
    declarations += [DeclareLaunchArgument(name, default_value=value) for name, value in bool_args.items()]
    declarations += [DeclareLaunchArgument(name, default_value=value) for name, value in int_args.items()]
    declarations += [DeclareLaunchArgument(name, default_value=value) for name, value in string_args.items()]

    def as_float(name):
        return PythonExpression(["float(", LaunchConfiguration(name), ")"])

    return LaunchDescription(
        declarations
        + [
            Node(
                package="map_generator",
                executable="structure_map",
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
                        "map/inflate_radius": as_float("map_inflate_radius"),
                        "map/simple_2d": LaunchConfiguration("map_simple_2d"),
                        "map/frame_id": LaunchConfiguration("map_frame_id"),
                        "map/auto_change": LaunchConfiguration("map_auto_change"),
                        "params/cylinder_ratio": as_float("params_cylinder_ratio"),
                        "params/circle_ratio": as_float("params_circle_ratio"),
                        "params/gate_ratio": as_float("params_gate_ratio"),
                        "params/ellip_ratio": as_float("params_ellip_ratio"),
                        "params/poly_ratio": as_float("params_poly_ratio"),
                        "params/w1": as_float("params_w1"),
                        "params/w2": as_float("params_w2"),
                        "params/w3": as_float("params_w3"),
                        "params/add_noise": LaunchConfiguration("params_add_noise"),
                        "params/seed": as_float("params_seed"),
                        "dataset/save_map": LaunchConfiguration("dataset_save_map"),
                        "dataset/samples_num": LaunchConfiguration("dataset_samples_num"),
                        "dataset/start_index": LaunchConfiguration("dataset_start_index"),
                        "dataset/path": LaunchConfiguration("dataset_path"),
                        "clear_path": LaunchConfiguration("clear_path"),
                        "clear_pos": LaunchConfiguration("clear_pos"),
                        "dyn/v_x_h": as_float("dyn_v_x_h"),
                        "dyn/v_y_h": as_float("dyn_v_y_h"),
                        "dyn/v_z_h": as_float("dyn_v_z_h"),
                        "dyn/dt": as_float("dyn_dt"),
                        "dyn/dyn_mode": LaunchConfiguration("dyn_dyn_mode"),
                    }
                ],
            )
        ]
    )
