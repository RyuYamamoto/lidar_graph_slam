from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    param_path = LaunchConfiguration('graph_based_slam_param_path')

    node = Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        name='graph_based_slam_node',
        output='screen',
        parameters=[param_path, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'graph_based_slam_param_path',
            default_value=PathJoinSubstitution(
                [FindPackageShare('graph_based_slam'), 'config', 'graph_based_slam.param.yaml'])),
        node,
    ])
