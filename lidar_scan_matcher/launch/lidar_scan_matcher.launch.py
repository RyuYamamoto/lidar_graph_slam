from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    param_path = LaunchConfiguration('lidar_scan_matcher_param_path')

    node = Node(
        package='lidar_scan_matcher',
        executable='lidar_scan_matcher_node',
        name='lidar_scan_matcher_node',
        output='screen',
        remappings=[('points_raw', 'filtered_points')],
        parameters=[param_path, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'lidar_scan_matcher_param_path',
            default_value=PathJoinSubstitution(
                [FindPackageShare('lidar_scan_matcher'), 'config', 'lidar_scan_matcher.param.yaml'])),
        node,
    ])
