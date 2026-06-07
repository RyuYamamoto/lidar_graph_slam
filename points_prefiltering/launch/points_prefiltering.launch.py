from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    node = Node(
        package='points_prefiltering',
        executable='points_prefiltering_node',
        name='points_prefiltering_node',
        output='screen',
        remappings=[('points_raw', 'velodyne_points')],
        parameters=[{
            'use_sim_time': use_sim_time,
            'leaf_size': 0.1,
            'random_sample_num': 1500.0,
            'mean_k': 30,
            'stddev': 1.2,
            'min_x': -100.0,
            'max_x': 100.0,
            'min_y': -100.0,
            'max_y': 100.0,
            'min_z': -100.0,
            'max_z': 100.0,
            'min_distance_cloud': 1.0,
            'max_distance_cloud': 200.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        node,
    ])
