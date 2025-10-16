import launch
import launch_ros.actions
import yaml
import os
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
def generate_launch_description():
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("pgo"), "rviz", "pgo.rviz"]
    )
    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo.yaml"]
    )
    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    rm_nav_bringup_dir = get_package_share_directory('rm_nav_bringup')
    slam_toolbox_mapping_file_dir = os.path.join(rm_nav_bringup_dir, 'config', 'reality', 'mapper_params_online_async_real.yaml')
    segmentation_params = os.path.join(rm_nav_bringup_dir, 'config', 'reality', 'segmentation_real.yaml')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use reality (Gazebo) clock if true')
    launch_params = yaml.safe_load(open(os.path.join(get_package_share_directory('rm_nav_bringup'), 'config', 'reality', 'measurement_params_real.yaml')))
    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_nav_bringup'), 'urdf', 'sentry_robot_real.xacro'),
        ' xyz:=', launch_params['base_link2livox_frame']['xyz'], ' rpy:=', launch_params['base_link2livox_frame']['rpy']])
    
    use_nav_rviz = LaunchConfiguration('nav_rviz')
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                '--frame-id', 'odom',
                '--child-frame-id', 'lidar_odom'
            ],
            ),
            launch_ros.actions.Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{
                    'robot_description': robot_description
                }],
            ),
            launch_ros.actions.Node(
                package="linefit_ground_segmentation_ros",
                executable="ground_segmentation_node",
                output="screen",
                parameters=[segmentation_params]
            ),
            launch_ros.actions.Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                remappings=[('cloud_in',  ['/segmentation/obstacle']),
                    ('scan',  ['/scan'])],
                parameters=[{
                    'target_frame': 'heading_frame',
                    'transform_tolerance': 0.03,
                    'min_height': -0.5,
                    'max_height': 0.5,
                    'angle_min': -1.57,  # -M_PI/2
                    'angle_max': 1.57,   # M_PI/2
                    'angle_increment': 0.0043,  # M_PI/360.0
                    'scan_time': 0.3333,
                    'range_min': 0.15,
                    'range_max': 15.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0
                    }],
                    name='pointcloud_to_laserscan'
            ),
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[{'use_sim_time': use_sim_time,"config_path": lio_config_path.perform(launch.LaunchContext())}]
            ),
            launch_ros.actions.Node(
                package="pgo",
                namespace="pgo",
                executable="pgo_node",
                name="pgo_node",
                output="screen",
                parameters=[{'use_sim_time': use_sim_time,"config_path": pgo_config_path.perform(launch.LaunchContext())}]
            ),
            launch_ros.actions.Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[
            slam_toolbox_mapping_file_dir
        ],
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="pgo",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            )
        ]
    )
