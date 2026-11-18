"""Saved-map localization and Nav2. Hardware drive is OFF by default."""
import json
import copy
import math
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


HERE = Path(__file__).resolve().parent


def setup(context):
    map_path = str(Path(LaunchConfiguration('map').perform(context)).expanduser().resolve())
    state_path = str(Path(LaunchConfiguration('state').perform(context)).expanduser().resolve())
    hardware = LaunchConfiguration('hardware').perform(context).lower() == 'true'
    settings_path = Path(LaunchConfiguration('settings').perform(context)).expanduser().resolve()
    settings = json.loads(settings_path.read_text())
    if not Path(map_path).is_file() or not Path(state_path).is_file():
        raise RuntimeError('Map YAML and saved PBStream must both exist')
    if hardware:
        import sys
        sys.path.insert(0, str(HERE))
        from safety import missing_calibration
        missing = missing_calibration(settings)
        if missing:
            raise RuntimeError('Hardware launch blocked: '+', '.join(missing))
    bringup = Path(get_package_share_directory('nav2_bringup'))
    params = yaml.safe_load((bringup/'params/nav2_params.yaml').read_text())
    def change_times(obj):
        if isinstance(obj, dict):
            for key, value in obj.items():
                if key == 'use_sim_time':
                    obj[key] = False
                else:
                    change_times(value)
    change_times(params)
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(HERE/'navigate.xml')
    controller = params['controller_server']['ros__parameters']
    controller.update(controller_frequency=10.0, min_x_velocity_threshold=0.001,
                      min_y_velocity_threshold=0.001, min_theta_velocity_threshold=0.001,
                      progress_checker_plugin='progress_checker', goal_checker_plugins=['goal_checker'])
    controller['progress_checker'] = dict(plugin='nav2_controller::SimpleProgressChecker', required_movement_radius=0.1, movement_time_allowance=15.0)
    controller['goal_checker'] = dict(plugin='nav2_controller::SimpleGoalChecker', xy_goal_tolerance=0.2, yaw_goal_tolerance=0.2, stateful=True)
    controller['FollowPath'] = dict(plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        desired_linear_vel=0.15, lookahead_dist=0.45, min_lookahead_dist=0.3, max_lookahead_dist=0.7,
        lookahead_time=1.5, rotate_to_heading_angular_vel=0.3, transform_tolerance=0.3,
        use_velocity_scaled_lookahead_dist=False, min_approach_linear_velocity=0.03,
        approach_velocity_scaling_dist=0.6, use_collision_detection=True,
        max_allowed_time_to_collision_up_to_carrot=2.0, use_regulated_linear_velocity_scaling=True,
        use_cost_regulated_linear_velocity_scaling=True, cost_scaling_dist=0.5, cost_scaling_gain=1.0,
        inflation_cost_scaling_factor=3.0, regulated_linear_scaling_min_radius=0.8,
        regulated_linear_scaling_min_speed=0.03, use_rotate_to_heading=True,
        allow_reversing=False, rotate_to_heading_min_angle=0.5, max_angular_accel=0.4)
    radius = settings['provisional_planning_radius_m']
    if settings.get('width_m') and settings.get('length_m'):
        radius = math.hypot(settings['width_m'], settings['length_m'])/2
    for name in ('local_costmap', 'global_costmap'):
        costmap = params[name][name]['ros__parameters']
        costmap['robot_radius'] = float(radius)
        costmap['transform_tolerance'] = 0.3
        costmap['inflation_layer']['inflation_radius'] = max(0.65, radius+0.25)
        if name == 'local_costmap':
            costmap['width'] = costmap['height'] = 4
            costmap['plugins'] = ['obstacle_layer', 'inflation_layer']
            costmap['obstacle_layer'] = copy.deepcopy(params['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer'])
        costmap['obstacle_layer']['scan'].update(obstacle_max_range=5.0, raytrace_max_range=6.0, obstacle_min_range=0.12)
    planner = params['planner_server']['ros__parameters']
    planner['expected_planner_frequency'] = 1.0
    planner['GridBased'].update(allow_unknown=False, use_astar=True, tolerance=0.15)
    params['behavior_server']['ros__parameters']['behavior_plugins'] = ['wait']
    params['velocity_smoother']['ros__parameters'].update(max_velocity=[0.15, 0.0, 0.3],
        min_velocity=[0.0, 0.0, -0.3], max_accel=[0.15, 0.0, 0.4], max_decel=[-0.3, 0.0, -0.6], velocity_timeout=0.3)
    params['waypoint_follower']['ros__parameters']['stop_on_failure'] = True
    generated = Path.home()/'.ros/robot_navigation_params.yaml'
    generated.parent.mkdir(parents=True, exist_ok=True)
    generated.write_text(yaml.safe_dump(params))
    lidar = str(Path(get_package_share_directory('ydlidar_ros2_driver'))/'params/X2.yaml')
    return [
        Node(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node', parameters=[lidar], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame']),
        Node(package='cartographer_ros', executable='cartographer_node', output='screen', arguments=[
            '-configuration_directory', str(HERE), '-configuration_basename', 'localization.lua',
            '-load_state_filename', state_path, '-load_frozen_state=true']),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': map_path, 'use_sim_time': False}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
             parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': False}]),
        ExecuteProcess(cmd=['python3', str(HERE/'tf_odometry.py')], output='screen'),
        ExecuteProcess(cmd=['python3', str(HERE/'motor_gate.py'), '--settings', str(settings_path)]+(['--hardware'] if hardware else []), output='screen'),
        ExecuteProcess(cmd=['python3', str(HERE/'api.py'), '--map', map_path], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup/'launch/navigation_launch.py')),
            launch_arguments={'params_file': str(generated), 'use_sim_time': 'false', 'autostart': 'true', 'use_composition': 'False'}.items()),
    ]


def generate_launch_description():
    default = str(Path.home()/'maps/manual_floor_20261117_160302')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default+'.yaml'),
        DeclareLaunchArgument('state', default_value=default+'.pbstream'),
        DeclareLaunchArgument('hardware', default_value='false'),
        DeclareLaunchArgument('settings', default_value=str(HERE/'robot_settings.json')),
        OpaqueFunction(function=setup),
    ])
