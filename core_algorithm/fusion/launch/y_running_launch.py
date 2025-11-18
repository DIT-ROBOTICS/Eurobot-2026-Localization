from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rival_name = LaunchConfiguration('rival_name')
    side = LaunchConfiguration('side')

    local_filter_launch_path = PathJoinSubstitution([
        FindPackageShare('local_filter'),
        'launch',
        'local_filter_whole.xml'
    ])

    # rplidar_launch = PathJoinSubstitution([
    #     FindPackageShare('lidar_localization_pkg'),
    #     'launch',
    #     'firmware',
    #     'rplidar_s3_launch.py'
    # ])
    ydlidar_launch = PathJoinSubstitution([
        FindPackageShare('lidar_localization_pkg'),
        'launch',
        'firmware',
        'ydlidar_launch.py'
    ])

    obstacle_extractor_launch = PathJoinSubstitution([
        FindPackageShare('lidar_localization_pkg'),
        'launch',
        'obstacle_extractor_launch.xml'
    ])

    healthcheck_node = Node(
        package='healthcheck',
        executable='healthcheck_node',
        name='healthcheck_node',
        output='screen'
        # ,remappings=[
        #     ['/vision/aruco/robot/single/average_pose', '/vision/aruco/robot_pose']
        # ]
    )

    ekf_node = Node(
        package='ekf',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[{
            'use_cam': 0,
            'robot_parent_frame_id': 'map',
            'robot_frame_id': 'base_footprint',
            '/use_sim_time': False,
            'q_linear': 1.2e-6,
            'q_angular': 1.7e-7,
            'r_camra_linear': 1e-2,
            'r_camra_angular': 0.1,
            'r_threshold_xy': 1e-2,
            'r_threshold_theta': 1e-1
        }],
        remappings=[
            ('initalpose', ['initial_pose'])
        ]

    )

    lidar_node = Node(
        package='lidar_localization_pkg',
        executable='lidar_localization',
        name='lidar_localization',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        parameters=[{
            'side': side,
            'debug_mode': False,
            'visualize_candidate': True,
            'likelihood_threshold': 0.8,
            'consistency_threshold': 0.95,
            'lidar_multiplier': 0.987
        }]
    )

    local_filter_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(local_filter_launch_path)
    )

    rival_node = Node(
        package='rival_localization',
        executable='rival_localization',
        name='rival_localization',
        output='screen',
        parameters=[
            {
                'robot_name': 'robot',
                'frequency': 10.0,
                'x_max': 3.0,
                'x_min': 0.0,
                'y_max': 2.0,
                'y_min': 0.0,
                'vel_lpf_gain': 0.9,
                'locking_rad': 0.3,
                'lockrad_growing_rate': 0.3,
                'is_me': 0.3,
                'cam_weight': 6.0,
                'obs_weight': 10.0,
                'side_weight': 2.0,
                'cam_side_threshold': 0.2,
                'side_obs_threshold': 0.2,
                'obs_cam_threshold': 0.2,
                'crossed_areas': [
                    2.55, 3.0, 0.65, 1.1,
                    2.4, 2.85, 1.55, 2.0,
                    2.0, 3.0, 0.0, 0.15,
                    1.0, 2.0, 0.0, 0.4,
                    0.0, 1.0, 0.0, 0.15,
                    0.0, 0.45, 0.65, 1.1,
                    0.15, 0.6, 1.55, 2.0,
                    1.05, 1.95, 1.50, 2.0
                ]
            }
        ],
        remappings=[
            ('raw_pose', [rival_name, '/raw_pose'])
            (['/ceiling_rival/pose','/vision/aruco/rival_pose'])
        ]
    )

    rival_obstacle_node = Node(
        package='obstacle_detector',
        executable='obstacle_extractor_node',
        name='obstacle_detector_to_map',
        parameters=[{
            'frame_id': 'map',
            'active': True,
            'use_scan': True,
            'use_pcl': False,
            'use_split_and_merge': True,
            'circles_from_visibles': True,
            'discard_converted_segments': False,
            'transform_coordinates': True,
            'min_group_points': 5,
            'max_group_distance': 0.04,
            'distance_proportion': 0.00628,
            'max_split_distance': 0.02,
            'max_merge_separation': 0.25,
            'max_merge_spread': 0.02,
            'max_circle_radius': 0.2,
            'radius_enlargement': 0.05,
            'pose_array': True
        }],
        remappings=[
            ('raw_obstacles', '/obstacles_to_map'),
            ('scan', '/scan'),
            ('raw_obstacles_visualization_pcl', 'raw_obstacle_visualization_to_map_pcl')
        ]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_to_laser',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '-1.633628',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'laser'
        ]
    )

    # rplidar_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(rplidar_launch))
    ydlidar_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(ydlidar_launch))
    obstacle_extractor_include = IncludeLaunchDescription(AnyLaunchDescriptionSource(obstacle_extractor_launch))

    return LaunchDescription([
        DeclareLaunchArgument('rival_name', default_value='rival'),
        DeclareLaunchArgument('side', default_value='0'),

        static_tf,
        ydlidar_include,
        obstacle_extractor_include,

        ekf_node,
        healthcheck_node,

        # TimerAction(period=2.0, actions=[ekf_node]),
        TimerAction(period=4.0, actions=[lidar_node]),
        TimerAction(period=6.0, actions=[local_filter_launch]),
        TimerAction(period=8.0, actions=[rival_node, rival_obstacle_node])
    ])