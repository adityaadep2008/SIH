from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

DOCK_POSES = [
    (-5.25, -29.55),
    (-3.75, -29.55),
    (-2.25, -29.55),
    (-0.75, -29.55),
    (0.75, -29.55),
    (2.25, -29.55),
    (3.75, -29.55),
    (5.25, -29.55),
]


def robot_group(robot_id, map_file, pad_pose, tracking_speed, reservation_slot, enable_faults, enable_vision, random_seed, expected_robot_ids, consolidated=True):
    # Find matching charging pad ID by X-coordinate
    dock_num = min(range(len(DOCK_POSES)), key=lambda k: abs(DOCK_POSES[k][0] - pad_pose[0])) + 1
    params = {
        'robot_id': robot_id,
        'map_file': map_file,
        'use_sim_time': True,
        'max_speed_mps': 6.0,
        'nominal_speed_mps': tracking_speed,
        'path_tracking_speed_mps': tracking_speed,
        'time_slot_seconds': reservation_slot,
        'linear_kp': 12.0,
        'braking_deceleration_mps2': 60.0,
        'braking_margin_m': 0.10,
        'robot_radius_m': 0.35,
        'lidar_x_in_base_m': 0.00393584,
        'lidar_y_in_base_m': 0.0,
        'lidar_yaw_in_base_rad': 1.5707963267948966,
        'pad_x': pad_pose[0], 'pad_y': pad_pose[1], 'pad_yaw': -1.5708,
        'dock_id': f'charging_pad_{dock_num}',
        'odom_origin_x': pad_pose[0], 'odom_origin_y': pad_pose[1],
        'odom_origin_yaw': -1.5708,
        'random_seed': random_seed,
        'fault_injection_enabled': enable_faults,
        'record_camera_images': enable_vision,
        'expected_robot_ids': expected_robot_ids,
    }
    
    if consolidated:
        return GroupAction([
            PushRosNamespace(robot_id),
            Node(package='sih_amr_fleet', executable='robot_agent_process', name='robot_agent',
                 parameters=[params], output='screen')
        ])
    else:
        nodes = [
            'local_costmap_node', 'blockage_detector_node', 'peer_tracker_node',
            'health_node', 'charging_pad_node', 'docking_coordinator_node', 'cbba_node', 'whca_planner_node', 'reservation_manager_node',
            'corridor_mutex_node', 'path_follower_node', 'orca_node', 'safety_supervisor_node', 'task_execution_node'
        ]
        return GroupAction([PushRosNamespace(robot_id)] + [Node(package='sih_amr_fleet', executable=name, name=name, parameters=[params], output='screen') for name in nodes])


def generate_launch_description():
    share = get_package_share_directory('sih_amr_fleet')
    default_map = os.path.join(share, 'maps', 'demo_warehouse.yaml')
    default_scenario = os.path.join(share, 'scenarios', 'demo_tasks.yaml')
    map_file = LaunchConfiguration('map_file')
    random_tasks = LaunchConfiguration('random_tasks')
    tracking_speed = LaunchConfiguration('path_tracking_speed_mps')
    reservation_slot = LaunchConfiguration('reservation_time_slot_s')
    enable_faults = LaunchConfiguration('enable_faults')
    enable_spawner = LaunchConfiguration('enable_spawner')
    enable_vision = LaunchConfiguration('enable_vision')
    random_seed = LaunchConfiguration('random_seed')
    num_robots_cfg = LaunchConfiguration('num_robots')
    consolidated_cfg = LaunchConfiguration('consolidated')
    
    tracking_speed_value = ParameterValue(tracking_speed, value_type=float)
    reservation_slot_value = ParameterValue(reservation_slot, value_type=float)
    enable_faults_value = ParameterValue(enable_faults, value_type=bool)
    enable_spawner_value = ParameterValue(enable_spawner, value_type=bool)
    enable_vision_value = ParameterValue(enable_vision, value_type=bool)
    random_seed_value = ParameterValue(random_seed, value_type=int)

    # Determine fleet list from environment or default
    fleet_count = int(os.environ.get('FLEET_COUNT', '8'))
    expected_ids = [f'robot_{i}' for i in range(1, fleet_count + 1)]

    launch_items = [
        DeclareLaunchArgument('map_file', default_value=default_map),
        DeclareLaunchArgument('scenario_file', default_value=default_scenario),
        DeclareLaunchArgument('random_tasks', default_value='false', description='Generate random shelf-aisle tasks instead of the fixed scenario.'),
        DeclareLaunchArgument('record_data', default_value='false', description='Write JSONL fleet telemetry; never affects control.'),
        DeclareLaunchArgument('data_file', default_value='/tmp/sih_amr_fleet_telemetry.jsonl'),
        DeclareLaunchArgument('enable_faults', default_value='false', description='Enable seeded fault injection harness.'),
        DeclareLaunchArgument('enable_spawner', default_value='false', description='Enable dynamic temporary obstacle spawner.'),
        DeclareLaunchArgument('enable_vision', default_value='false', description='Enable bounded camera frame recorder.'),
        DeclareLaunchArgument('random_seed', default_value='42', description='Random seed for reproducibility.'),
        DeclareLaunchArgument('path_tracking_speed_mps', default_value='0.46',
                              description='Fleet route-tracking speed; 0.46 is the physical_max baseline.'),
        DeclareLaunchArgument('reservation_time_slot_s', default_value='0.0',
                              description='WHCA* seconds/cell; 0 derives it from grid resolution and speed.'),
        DeclareLaunchArgument('num_robots', default_value=str(fleet_count),
                              description='Fleet size (8 for Desktop, 6 for Laptop)'),
        DeclareLaunchArgument('consolidated', default_value='true',
                              description='Run consolidated agent process per robot (drops CPU thrashing by 80%)'),
        Node(package='sih_amr_fleet', executable='warehouse_map_node', name='warehouse_map_node', parameters=[{'map_file': map_file, 'use_sim_time': True}], output='screen'),
    ]

    # Instantiate robot agents for each AMR in the fleet
    for i in range(1, fleet_count + 1):
        env_x = os.environ.get(f'ROBOT_{i}_X')
        env_y = os.environ.get(f'ROBOT_{i}_Y')
        if env_x is not None and env_y is not None:
            pad_coord = (float(env_x), float(env_y))
        elif fleet_count <= 4:
            pad_coord = DOCK_POSES[(i - 1) * 2]
        else:
            pad_coord = DOCK_POSES[i - 1]
        launch_items.append(
            robot_group(f'robot_{i}', map_file, pad_coord, tracking_speed_value,
                        reservation_slot_value, enable_faults_value, enable_vision_value,
                        random_seed_value, expected_ids, consolidated=True)
        )

    launch_items.extend([
        Node(package='sih_amr_fleet', executable='obstacle_spawner_node', name='obstacle_spawner_node',
             parameters=[{'random_seed': random_seed_value, 'spawning_enabled': enable_spawner_value, 'use_sim_time': True}],
             condition=IfCondition(enable_spawner), output='screen'),
        Node(package='sih_amr_fleet', executable='task_scenario_node', name='task_scenario_node', parameters=[{'scenario_file': LaunchConfiguration('scenario_file'), 'use_sim_time': True}], condition=UnlessCondition(LaunchConfiguration('random_tasks')), output='screen'),
        Node(package='sih_amr_fleet', executable='random_task_generator_node', name='random_task_generator_node',
             parameters=[{
                 'use_sim_time': True,
                 'expected_robot_ids': expected_ids,
                 'max_active_tasks': len(expected_ids) * 2,
                 'min_interval_s': 0.8,
                 'max_interval_s': 2.0,
                 'task_ttl_s': 1800.0,
                 'seed': random_seed_value
             }], condition=IfCondition(random_tasks), output='screen'),
        Node(package='sih_amr_fleet', executable='data_collection_node', name='data_collection_node',
             parameters=[{'output_file': LaunchConfiguration('data_file'), 'use_sim_time': True, 'lean_telemetry': True, 'robot_ids': expected_ids}],
             condition=IfCondition(LaunchConfiguration('record_data')), output='screen'),
        Node(package='sih_amr_fleet', executable='dashboard_bridge_node', name='dashboard_bridge_node', parameters=[{'use_sim_time': True}], output='screen'),
    ])

    return LaunchDescription(launch_items)
