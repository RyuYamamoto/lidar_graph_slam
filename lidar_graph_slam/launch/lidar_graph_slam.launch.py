from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    scan_matcher_param = PathJoinSubstitution(
        [FindPackageShare('lidar_scan_matcher'), 'config', 'lidar_scan_matcher.param.yaml'])
    graph_based_slam_param = PathJoinSubstitution(
        [FindPackageShare('graph_based_slam'), 'config', 'graph_based_slam.param.yaml'])

    container = ComposableNodeContainer(
        name='lidar_graph_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='points_prefiltering',
                plugin='PointsPreFiltering',
                name='points_prefiltering_node',
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
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='lidar_scan_matcher',
                plugin='LidarScanMatcher',
                name='lidar_scan_matcher_node',
                remappings=[('points_raw', 'filtered_points')],
                parameters=[scan_matcher_param, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='graph_based_slam',
                plugin='GraphBasedSLAM',
                name='graph_based_slam_node',
                parameters=[graph_based_slam_param, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('lidar_graph_slam'), 'rviz', 'rviz.config'])),
        container,
        rviz_node,
    ])
