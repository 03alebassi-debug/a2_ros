"""
Autonomous exploration launch for A2 simulation using TARE planner.

Starts the full exploration stack on top of the running sim:
  - terrain_analysis     : builds /terrain_map from /registered_scan + /state_estimation
  - terrain_analysis_ext : builds /terrain_map_ext (global terrain)
  - local_planner        : obstacle-aware path selection
  - pathFollower         : converts waypoints to velocity, /nav_vel (twist_mux input)
  - tare_planner         : autonomous coverage exploration (replaces far_planner)

Prerequisites (provided by sim.launch.py + a2_bridge):
  /state_estimation  - ground-truth odometry (published by a2_bridge in a2_sim_utils)
  /registered_scan   - world-frame lidar cloud (published by a2_bridge in a2_sim_utils)
  /clock             - sim time clock (published by sim_clock in a2_sim_utils)

Usage:
  # Terminal 1
  ros2 launch a2_ros sim.launch.py scene:=scene_obstacles.xml

  # Terminal 2
  cd src/control/a2_locomotion_controller/scripts
  ./control_mode.sh --stand
  ./control_mode.sh --walk

  # Terminal 3
  ros2 launch a2_ros exploration.launch.py rviz:=true

The robot will begin exploring autonomously.
"""

import os
from pprint import pformat
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

def _format_params(name, params):
    return f"\n[EXPLORE PARAMS] {name}\n{pformat(params, sort_dicts=True)}"


def _read_file(path):
    try:
        with open(path, 'r', encoding='utf-8') as file:
            return file.read()
    except OSError as exc:
        return f"<failed to read {path}: {exc}>"


def generate_launch_description():
    description_dir = get_package_share_directory('a2_description')
    a2_ros_dir      = get_package_share_directory('a2_ros')
    rviz_path        = os.path.join(a2_ros_dir, 'rviz', 'exploration.rviz')
    tare_config      = os.path.join(a2_ros_dir, 'config', 'autonomy', 'tare_a2.yaml')
    far_config       = os.path.join(a2_ros_dir, 'config', 'autonomy', 'far_a2.yaml')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )

    exploration_duration_arg = DeclareLaunchArgument(
        'exploration_duration',
        default_value='60.0',
        description='Seconds of autonomous exploration before the robot returns home'
    )

    terrain_analysis_params = {
        'scanVoxelSize':       0.05,
        'decayTime':           5.0,
        'noDecayDis':          0.0,
        'clearingDis':         8.0,
        'useSorting':          True,
        'quantileZ':           0.25,
        'considerDrop':        True,
        'limitGroundLift':     True,
        'maxGroundLift':       0.25,
        'clearDyObs':          False,
        'minDyObsDis':         0.3,
        'minDyObsAngle':       0.0,
        'minDyObsRelZ':        -0.5,
        'absDyObsRelZThre':    0.2,
        'minDyObsVFOV':        -16.0,
        'maxDyObsVFOV':        16.0,
        'minDyObsPointNum':    1,
        'noDataObstacle':      False,
        'noDataBlockSkipNum':  0,
        'minBlockPointNum':    10,
        'vehicleHeight':       0.5,
        'voxelPointUpdateThre': 100,
        'voxelTimeUpdateThre': 2.0,
        'minRelZ':             -1.0,
        'maxRelZ':             1.0,
        'disRatioZ':           0.2,
    }

    terrain_analysis_ext_params = {
        'scanVoxelSize':        0.1,
        'decayTime':            10.0,
        'noDecayDis':           0.0,
        'clearingDis':          30.0,
        'useSorting':           True,
        'quantileZ':            0.25,
        'vehicleHeight':        0.5,
        'voxelPointUpdateThre': 100,
        'voxelTimeUpdateThre':  2.0,
        'lowerBoundZ':          -1.0,
        'upperBoundZ':          1.0,
        'disRatioZ':            0.1,
        'checkTerrainConn':     True,
        'terrainUnderVehicle':  -0.75,
        'terrainConnThre':      0.5,
        'ceilingFilteringThre': 2.0,
        'localTerrainMapRadius': 4.0,
    }

    local_planner_params = {
        'pathFolder':          get_package_share_directory('local_planner') + '/paths',
        'vehicleLength':       0.65,
        'vehicleWidth':        0.40,
        'sensorOffsetX':       0.0,
        'sensorOffsetY':       0.0,
        'twoWayDrive':         False,
        'laserVoxelSize':      0.05,
        'terrainVoxelSize':    0.2,
        'useTerrainAnalysis':  True,
        'checkObstacle':       True,
        'checkRotObstacle':    True,
        'adjacentRange':       2.0,
        'obstacleHeightThre':  0.2,
        'groundHeightThre':    0.1,
        'costHeightThre':      0.1,
        'costScore':           0.02,
        'useCost':             False,
        'pointPerPathThre':    2,
        'minRelZ':             -0.5,
        'maxRelZ':             0.8,
        'maxSpeed':            0.5,
        'dirWeight':           0.1,
        'dirThre':             90.0,
        'dirToVehicle':        False,
        'pathScale':           1.0,
        'minPathScale':        0.75,
        'pathScaleStep':       0.25,
        'pathScaleBySpeed':    False,
        'minPathRange':        1.0,
        'pathRangeStep':       0.5,
        'pathRangeBySpeed':    False,
        'pathCropByGoal':      True,
        'autonomyMode':        True,
        'autonomySpeed':       1.0,
        'joyToSpeedDelay':     2.0,
        'joyToCheckObstacleDelay': 5.0,
        'goalReachedDist':     0.35,
        'goalClearRange':      0.3,
        'goalX':               0.0,
        'goalY':               0.0,
    }

    path_follower_params = {
        'sensorOffsetX':    0.0,
        'sensorOffsetY':    0.0,
        'pubSkipNum':       1,
        'twoWayDrive':      False,
        'lookAheadDis':     0.4,
        'yawRateGain':      5.0,
        'stopYawRateGain':  4.0,
        'maxYawRate':       30.0,
        'maxSpeed':         0.5,
        'maxAccel':         2.0,
        'switchTimeThre':   1.0,
        'dirDiffThre':      0.1,
        'stopDisThre':      0.3,
        'slowDwnDisThre':   0.6,
        'useInclRateToSlow': False,
        'inclRateThre':     120.0,
        'slowRate1':        0.25,
        'slowRate2':        0.5,
        'slowTime1':        2.0,
        'slowTime2':        2.0,
        'useInclToStop':    False,
        'inclThre':         45.0,
        'stopTime':         5.0,
        'noRotAtStop':      False,
        'noRotAtGoal':      True,
        'autonomyMode':     True,
        'autonomySpeed':    2.0,
        'joyToSpeedDelay':  2.0,
    }

    mission_manager_params = {
        'exploration_duration': ParameterValue(
            LaunchConfiguration('exploration_duration'), value_type=float),
        'watchdog_enabled': True,
        'watchdog_stuck_timeout': 5.0,
        'watchdog_min_progress': 0.15,
        'watchdog_min_waypoint_distance': 0.8,
        'watchdog_goal_reached_dist': 0.5,
        'watchdog_stopped_linear_threshold': 0.03,
        'watchdog_stopped_angular_threshold': 0.03,
        'watchdog_blacklist_radius': 1.0,
        'watchdog_blacklist_ttl': 15.0,
        'watchdog_reset_cooldown': 3.0,
        'watchdog_escape_distance': 1.5,
        'watchdog_escape_hold_time': 10.0,
    }

    nodes = [
        rviz_arg,
        exploration_duration_arg,
        SetParameter(name='use_sim_time', value=False),
        LogInfo(msg=_format_params('terrainAnalysis', terrain_analysis_params)),
        LogInfo(msg=_format_params('terrainAnalysisExt', terrain_analysis_ext_params)),
        LogInfo(msg=_format_params('localPlanner', local_planner_params)),
        LogInfo(msg=_format_params('pathFollower', path_follower_params)),
        LogInfo(msg=_format_params('mission_manager', mission_manager_params)),
        LogInfo(msg=f"\n[EXPLORE PARAMS] tare_planner YAML path\n{tare_config}"),
        LogInfo(msg=f"\n[EXPLORE PARAMS] tare_planner YAML contents\n{_read_file(tare_config)}"),
        LogInfo(msg=f"\n[EXPLORE PARAMS] far_planner YAML path\n{far_config}"),
        LogInfo(msg=f"\n[EXPLORE PARAMS] far_planner YAML contents\n{_read_file(far_config)}"),

        # ---- terrain analysis (local map) ----
        Node(
            package='terrain_analysis',
            executable='terrainAnalysis',
            name='terrainAnalysis',
            output='screen',
            parameters=[terrain_analysis_params],
        ),

        # ---- terrain analysis ext (global map) ----
        Node(
            package='terrain_analysis_ext',
            executable='terrainAnalysisExt',
            name='terrainAnalysisExt',
            output='screen',
            parameters=[terrain_analysis_ext_params],
        ),
        # ---- local planner ----
        Node(
            package='local_planner',
            executable='localPlanner',
            name='localPlanner',
            output='screen',
            remappings=[('/way_point', '/selected_waypoint')],
            parameters=[local_planner_params],
        ),

        Node(
            package='local_planner',
            executable='pathFollower',
            name='pathFollower',
            output='screen',
            parameters=[path_follower_params],
        ),

        # ---- TARE planner (autonomous exploration) ----
        Node(
            package='tare_planner',
            executable='tare_planner_node',
            name='tare_planner_node',
            output='log',
            parameters=[tare_config],
        ),

        # ---- FAR planner (global path planner used during return home) ----
        # Runs in the background during exploration (builds terrain map).
        # /way_point and /navigation_boundary outputs are remapped so they do
        # not interfere with TARE during exploration; mission_manager forwards
        # them to the real topics only when the return phase begins.
        Node(
            package='far_planner',
            executable='far_planner',
            name='far_planner',
            output='screen',
            additional_env={'QT_QPA_PLATFORM': 'offscreen'},
            parameters=[far_config],
            remappings=[
                ('/odom_world',          '/state_estimation'),
                ('/terrain_cloud',       '/terrain_map_ext'),
                ('/scan_cloud',          '/registered_scan'),
                ('/terrain_local_cloud', '/terrain_map'),
                ('/way_point',           '/far_waypoint'),
                ('/navigation_boundary', '/far_navigation_boundary'),
            ],
        ),

        # ---- Mission manager (exploration → return-home switch) ----
        # Forwards /way_point (TARE) → /selected_waypoint during exploration.
        # After exploration_duration seconds, sends home goal to FAR and
        # forwards /far_waypoint → /selected_waypoint for return navigation.
        Node(
            package='a2_ros',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[mission_manager_params],
        ),

        # ---- RViz ----
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]

    return LaunchDescription(nodes)
