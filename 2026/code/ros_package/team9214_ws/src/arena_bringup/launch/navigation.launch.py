# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node, PushROSNamespace, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def launch_config_as_bool(name: str):
    # Compatibility helper for ROS distros where nav2_common.launch does not
    # export LaunchConfigAsBool.
    return PythonExpression(["'", LaunchConfiguration(name), "'.lower() in ['true', '1', 'yes', 'on']"])


def generate_launch_description() -> LaunchDescription:
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    # Added map-server launch args and default map yaml from arena_tag_map package
    arena_tag_map_dir = get_package_share_directory('arena_tag_map')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = launch_config_as_bool('use_sim_time')
    autostart = launch_config_as_bool('autostart')
    launch_map_server = launch_config_as_bool('launch_map_server')
    map_yaml = LaunchConfiguration('map')
    graph_filepath = LaunchConfiguration('graph')
    params_file = LaunchConfiguration('params_file')
    use_composition = launch_config_as_bool('use_composition')
    use_intra_process_comms = launch_config_as_bool('use_intra_process_comms')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = launch_config_as_bool('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_keepout_zones = LaunchConfiguration('use_keepout_zones')
    use_speed_zones = LaunchConfiguration('use_speed_zones')
    enable_cmd_vel_nt4_bridge = launch_config_as_bool('enable_cmd_vel_nt4_bridge')
    cmd_vel_nt4_topic = LaunchConfiguration('cmd_vel_nt4_topic')
    nt4_client_name = LaunchConfiguration('nt4_client_name')
    nt4_server = LaunchConfiguration('nt4_server')
    nt4_team = LaunchConfiguration('nt4_team')
    nt4_server_port = LaunchConfiguration('nt4_server_port')
    nt4_table = LaunchConfiguration('nt4_table')
    enable_autonomy_mode_manager = launch_config_as_bool('enable_autonomy_mode_manager')
    autonomy_mode_topic = LaunchConfiguration('autonomy_mode_topic')
    autonomy_mode_value = LaunchConfiguration('autonomy_mode_value')
    autonomy_cancel_on_exit = launch_config_as_bool('autonomy_cancel_on_exit')
    autonomy_action_name = LaunchConfiguration('autonomy_action_name')
    autonomy_goal_frame_id = LaunchConfiguration('autonomy_goal_frame_id')
    autonomy_goal_x = LaunchConfiguration('autonomy_goal_x')
    autonomy_goal_y = LaunchConfiguration('autonomy_goal_y')
    autonomy_goal_yaw = LaunchConfiguration('autonomy_goal_yaw')
    autonomy_goal_mode = LaunchConfiguration('autonomy_goal_mode')
    autonomy_waypoints = LaunchConfiguration('autonomy_waypoints')
    autonomy_loop_on_success = launch_config_as_bool('autonomy_loop_on_success')
    autonomy_skip_failed_waypoint = launch_config_as_bool('autonomy_skip_failed_waypoint')
    autonomy_goal_behavior_tree = LaunchConfiguration('autonomy_goal_behavior_tree')
    enable_nt4_mode_bridge = launch_config_as_bool('enable_nt4_mode_bridge')
    nt4_mode_client_name = LaunchConfiguration('nt4_mode_client_name')
    nt4_mode_server = LaunchConfiguration('nt4_mode_server')
    nt4_mode_team = LaunchConfiguration('nt4_mode_team')
    nt4_mode_server_port = LaunchConfiguration('nt4_mode_server_port')
    nt4_mode_table = LaunchConfiguration('nt4_mode_table')
    nt4_mode_key = LaunchConfiguration('nt4_mode_key')
    ros_mode_topic = LaunchConfiguration('ros_mode_topic')

    lifecycle_nodes = [
        'controller_server', # Controls robots movement
        'smoother_server',
        'planner_server', # Responsible calculate plan
        'route_server',
        'behavior_server', # All robots behaviors. Action servers that are called by a client
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator', # Runs the behavior tree. Contains the client that calls the action server
        'waypoint_follower',
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {'autostart': autostart}

    # RewrittenYaml: Adds namespace to the parameters file as a root key
    # Note: Make sure that all frames are correctly namespaced in the parameters file
    # Do not add namespace to topics in the parameters file, as they will be remapped
    # by the root key only if they are not prefixed with a forward slash.
    # e.g. 'map' will be remapped to '/<namespace>/map', but '/map' will not be remapped.
    # IMPORTANT: to make your yaml file dynamic you can refer to humble branch under
    # nav2_bringup/launch/bringup_launch.py to see how the parameters file is configured
    # using ReplaceString <robot_namespace>
    configured_params_source = ReplaceString(
        source_file=params_file,
        replacements={
            'KEEPOUT_ZONE_ENABLED': use_keepout_zones,
            'SPEED_ZONE_ENABLED': use_speed_zones,
        },
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=configured_params_source,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )
    # New arg launch_map_server defaulted to true, 
    # meaning the map server will be launched by default with this launch file. 
    declare_launch_map_server_cmd = DeclareLaunchArgument(
        'launch_map_server',
        default_value='True',
        description='Whether to launch nav2_map_server and lifecycle manager for map',
    )
    # The map yaml arg is also added, with a default pointing to a blank field map 
    # in the arena_tag_map package.
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(arena_tag_map_dir, 'config', 'frc2026_field_blank.yaml'),
        description='Full path to map yaml file to load in map_server',
    )

    declare_graph_file_cmd = DeclareLaunchArgument(
        'graph',
        default_value='', description='Path to the graph file to load'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True',
    )

    declare_use_intra_process_comms_cmd = DeclareLaunchArgument(
        'use_intra_process_comms',
        default_value='False',
        description='Whether to use intra process communication',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of container that nodes will load in if use composition',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    declare_use_keepout_zones_cmd = DeclareLaunchArgument(
        'use_keepout_zones', default_value='True',
        description='Whether to enable keepout zones or not'
    )

    declare_use_speed_zones_cmd = DeclareLaunchArgument(
        'use_speed_zones', default_value='True',
        description='Whether to enable speed zones or not'
    )
    declare_enable_cmd_vel_nt4_bridge_cmd = DeclareLaunchArgument(
        'enable_cmd_vel_nt4_bridge', default_value='False',
        description='Launch a cmd_vel_nav -> NT4 bridge node for RoboRIO integration'
    )
    declare_cmd_vel_nt4_topic_cmd = DeclareLaunchArgument(
        'cmd_vel_nt4_topic', default_value='cmd_vel_nav',
        description='Velocity topic to bridge to NT4'
    )
    declare_nt4_client_name_cmd = DeclareLaunchArgument(
        'nt4_client_name', default_value='arena_cmd_vel_bridge',
        description='NT4 client identity'
    )
    declare_nt4_server_cmd = DeclareLaunchArgument(
        'nt4_server', default_value='10.92.14.2',
        description='NT4 server hostname/IP (ignored if nt4_team >= 0)'
    )
    declare_nt4_team_cmd = DeclareLaunchArgument(
        'nt4_team', default_value='-1',
        description='FRC team number for NT4 discovery; set >= 0 to use team server'
    )
    declare_nt4_server_port_cmd = DeclareLaunchArgument(
        'nt4_server_port', default_value='-1',
        description='Optional NT4 server port override'
    )
    declare_nt4_table_cmd = DeclareLaunchArgument(
        'nt4_table', default_value='ROS',
        description='NetworkTables table used for cmd_vel topics'
    )
    declare_enable_autonomy_mode_manager_cmd = DeclareLaunchArgument(
        'enable_autonomy_mode_manager', default_value='False',
        description='Launch mode-driven autonomy manager that sends/cancels Nav2 goals'
    )
    declare_autonomy_mode_topic_cmd = DeclareLaunchArgument(
        'autonomy_mode_topic', default_value='robot_mode',
        description='Topic with current robot mode (std_msgs/String)'
    )
    declare_autonomy_mode_value_cmd = DeclareLaunchArgument(
        'autonomy_mode_value', default_value='autonomous',
        description='Mode string value that triggers autonomous goal start'
    )
    declare_autonomy_cancel_on_exit_cmd = DeclareLaunchArgument(
        'autonomy_cancel_on_exit', default_value='True',
        description='Cancel active Nav2 goal when mode exits autonomous'
    )
    declare_autonomy_action_name_cmd = DeclareLaunchArgument(
        'autonomy_action_name', default_value='navigate_to_pose',
        description='NavigateToPose action name'
    )
    declare_autonomy_goal_frame_id_cmd = DeclareLaunchArgument(
        'autonomy_goal_frame_id', default_value='map',
        description='Frame for default autonomous goal'
    )
    declare_autonomy_goal_x_cmd = DeclareLaunchArgument(
        'autonomy_goal_x', default_value='1.0',
        description='Default autonomous goal x'
    )
    declare_autonomy_goal_y_cmd = DeclareLaunchArgument(
        'autonomy_goal_y', default_value='0.0',
        description='Default autonomous goal y'
    )
    declare_autonomy_goal_yaw_cmd = DeclareLaunchArgument(
        'autonomy_goal_yaw', default_value='0.0',
        description='Default autonomous goal yaw (radians)'
    )
    declare_autonomy_goal_mode_cmd = DeclareLaunchArgument(
        'autonomy_goal_mode', default_value='single_goal',
        description="Autonomy goal mode: 'single_goal' or 'loop_waypoints'"
    )
    declare_autonomy_waypoints_cmd = DeclareLaunchArgument(
        'autonomy_waypoints', default_value='',
        description="Semicolon-separated waypoints as x,y,yaw;x,y,yaw for loop_waypoints mode"
    )
    declare_autonomy_loop_on_success_cmd = DeclareLaunchArgument(
        'autonomy_loop_on_success', default_value='True',
        description='Continue looping to next waypoint after each successful goal'
    )
    declare_autonomy_skip_failed_waypoint_cmd = DeclareLaunchArgument(
        'autonomy_skip_failed_waypoint', default_value='True',
        description='Skip to next waypoint when a loop waypoint goal fails'
    )
    # Launch arg autonomy_goal_behavior_tree_cmd defined.
    #
    # Provides behavior tree support to the autonomy manager with
    # BT XML file. Default is empty, meaning no specific BT will be requested, 
    # and the planner will use its default behavior for NavigateToPose goals.
    #
    # If provided, the autonomy manager will include this BT name 
    # in the NavigateToPose goal message, and the BT Navigator 
    # will execute that BT if it supports it. 
    # This allows custom autonomous behaviors to run when the robot
    # enters autonomous mode, 
    # 
    # References the Nav2 NavigateToPose action trigger from
    # arena_bringup/autonomy_mode_manager.py, 
    declare_autonomy_goal_behavior_tree_cmd = DeclareLaunchArgument(
        'autonomy_goal_behavior_tree', default_value='',
        description='Optional BT XML path for NavigateToPose goal'
    )
    declare_enable_nt4_mode_bridge_cmd = DeclareLaunchArgument(
        'enable_nt4_mode_bridge', default_value='False',
        description='Launch NT4->ROS mode bridge for autonomy mode triggering'
    )
    declare_nt4_mode_client_name_cmd = DeclareLaunchArgument(
        'nt4_mode_client_name', default_value='arena_mode_bridge',
        description='NT4 client identity for mode bridge'
    )
    declare_nt4_mode_server_cmd = DeclareLaunchArgument(
        'nt4_mode_server', default_value='10.92.14.2',
        description='NT4 server hostname/IP for mode bridge'
    )
    declare_nt4_mode_team_cmd = DeclareLaunchArgument(
        'nt4_mode_team', default_value='-1',
        description='FRC team number for NT4 mode bridge; set >= 0 to use team server'
    )
    declare_nt4_mode_server_port_cmd = DeclareLaunchArgument(
        'nt4_mode_server_port', default_value='-1',
        description='Optional NT4 mode bridge server port override'
    )
    declare_nt4_mode_table_cmd = DeclareLaunchArgument(
        'nt4_mode_table', default_value='ROS',
        description='NetworkTables table used by mode bridge'
    )
    declare_nt4_mode_key_cmd = DeclareLaunchArgument(
        'nt4_mode_key', default_value='robot_mode',
        description='NetworkTables key holding mode string'
    )
    declare_ros_mode_topic_cmd = DeclareLaunchArgument(
        'ros_mode_topic', default_value='robot_mode',
        description='ROS topic where mode bridge publishes std_msgs/String mode'
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            PushROSNamespace(namespace=namespace),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_route',
                executable='route_server',
                name='route_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'graph_filepath': graph_filepath}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
                + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    configured_params,
                    {'autostart': autostart}, {'node_names': lifecycle_nodes}
                ],
            ),
        ],
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            PushROSNamespace(namespace=namespace),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                    ComposableNode(
                        package='nav2_smoother',
                        plugin='nav2_smoother::SmootherServer',
                        name='smoother_server',
                        parameters=[configured_params],
                        remappings=remappings,
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_params],
                        remappings=remappings,
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                    ComposableNode(
                        package='nav2_route',
                        plugin='nav2_route::RouteServer',
                        name='route_server',
                        parameters=[configured_params, {'graph_filepath': graph_filepath}],
                        remappings=remappings,
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='behavior_server::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_params],
                        remappings=remappings,
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                    ComposableNode(
                        package='nav2_waypoint_follower',
                        plugin='nav2_waypoint_follower::WaypointFollower',
                        name='waypoint_follower',
                        parameters=[configured_params],
                        remappings=remappings,
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                    ComposableNode(
                        package='nav2_velocity_smoother',
                        plugin='nav2_velocity_smoother::VelocitySmoother',
                        name='velocity_smoother',
                        parameters=[configured_params],
                        remappings=remappings
                        + [('cmd_vel', 'cmd_vel_nav')],
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                    ComposableNode(
                        package='nav2_collision_monitor',
                        plugin='nav2_collision_monitor::CollisionMonitor',
                        name='collision_monitor',
                        parameters=[configured_params],
                        remappings=remappings,
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[
                            configured_params,
                            {'autostart': autostart, 'node_names': lifecycle_nodes}
                        ],
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                ],
            ),
        ],
    )

    cmd_vel_nt4_bridge = Node(
        condition=IfCondition(enable_cmd_vel_nt4_bridge),
        package='arena_bringup',
        executable='cmd_vel_nt4_bridge',
        name='cmd_vel_nt4_bridge',
        namespace=namespace,
        output='screen',
        parameters=[{
            'cmd_vel_topic': cmd_vel_nt4_topic,
            'nt4_client_name': nt4_client_name,
            'nt4_server': nt4_server,
            'nt4_team': nt4_team,
            'nt4_server_port': nt4_server_port,
            'nt4_table': nt4_table,
        }],
    )
    autonomy_mode_manager = Node(
        condition=IfCondition(enable_autonomy_mode_manager),
        package='arena_bringup',
        executable='autonomy_mode_manager',
        name='autonomy_mode_manager',
        namespace=namespace,
        output='screen',
        parameters=[{
            'mode_topic': autonomy_mode_topic,
            'autonomous_mode_value': autonomy_mode_value,
            'cancel_on_mode_exit': autonomy_cancel_on_exit,
            'navigate_action_name': autonomy_action_name,
            'goal_frame_id': autonomy_goal_frame_id,
            'goal_x': autonomy_goal_x,
            'goal_y': autonomy_goal_y,
            'goal_yaw': autonomy_goal_yaw,
            'goal_mode': autonomy_goal_mode,
            'waypoints': autonomy_waypoints,
            'loop_on_success': autonomy_loop_on_success,
            'skip_failed_waypoint': autonomy_skip_failed_waypoint,
            'goal_behavior_tree': autonomy_goal_behavior_tree,
        }],
    )
    nt4_mode_bridge = Node(
        condition=IfCondition(enable_nt4_mode_bridge),
        package='arena_bringup',
        executable='nt4_mode_bridge',
        name='nt4_mode_bridge',
        namespace=namespace,
        output='screen',
        parameters=[{
            'nt4_client_name': nt4_mode_client_name,
            'nt4_server': nt4_mode_server,
            'nt4_team': nt4_mode_team,
            'nt4_server_port': nt4_mode_server_port,
            'nt4_table': nt4_mode_table,
            'nt4_mode_key': nt4_mode_key,
            'ros_mode_topic': ros_mode_topic,
        }],
    )
    # map_server node added with condition to only launch if launch_map_server is true.
    map_server = Node(
        condition=IfCondition(launch_map_server),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params, {'yaml_filename': map_yaml}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )
    # lifecycle manager for map server added, also conditioned on launch_map_server
    lifecycle_manager_map = Node(
        condition=IfCondition(launch_map_server),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server'],
        }],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_launch_map_server_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_graph_file_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_intra_process_comms_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_keepout_zones_cmd)
    ld.add_action(declare_use_speed_zones_cmd)
    ld.add_action(declare_enable_cmd_vel_nt4_bridge_cmd)
    ld.add_action(declare_cmd_vel_nt4_topic_cmd)
    ld.add_action(declare_nt4_client_name_cmd)
    ld.add_action(declare_nt4_server_cmd)
    ld.add_action(declare_nt4_team_cmd)
    ld.add_action(declare_nt4_server_port_cmd)
    ld.add_action(declare_nt4_table_cmd)
    ld.add_action(declare_enable_autonomy_mode_manager_cmd)
    ld.add_action(declare_autonomy_mode_topic_cmd)
    ld.add_action(declare_autonomy_mode_value_cmd)
    ld.add_action(declare_autonomy_cancel_on_exit_cmd)
    ld.add_action(declare_autonomy_action_name_cmd)
    ld.add_action(declare_autonomy_goal_frame_id_cmd)
    ld.add_action(declare_autonomy_goal_x_cmd)
    ld.add_action(declare_autonomy_goal_y_cmd)
    ld.add_action(declare_autonomy_goal_yaw_cmd)
    ld.add_action(declare_autonomy_goal_mode_cmd)
    ld.add_action(declare_autonomy_waypoints_cmd)
    ld.add_action(declare_autonomy_loop_on_success_cmd)
    ld.add_action(declare_autonomy_skip_failed_waypoint_cmd)
    ld.add_action(declare_autonomy_goal_behavior_tree_cmd)
    ld.add_action(declare_enable_nt4_mode_bridge_cmd)
    ld.add_action(declare_nt4_mode_client_name_cmd)
    ld.add_action(declare_nt4_mode_server_cmd)
    ld.add_action(declare_nt4_mode_team_cmd)
    ld.add_action(declare_nt4_mode_server_port_cmd)
    ld.add_action(declare_nt4_mode_table_cmd)
    ld.add_action(declare_nt4_mode_key_cmd)
    ld.add_action(declare_ros_mode_topic_cmd)
    # Wired both map_server and its lifecycle manager before Nav2 stack actions
    ld.add_action(map_server)
    ld.add_action(lifecycle_manager_map)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    ld.add_action(cmd_vel_nt4_bridge)
    ld.add_action(autonomy_mode_manager)
    ld.add_action(nt4_mode_bridge)

    return ld
