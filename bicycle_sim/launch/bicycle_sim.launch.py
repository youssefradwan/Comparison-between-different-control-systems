"""
Launch file for the Kinematic Bicycle simulation stack.
Processes racecar.xacro, launches robot_state_publisher, simulation plant (bicycle_sim),
path generator and lap analyzer (track_environment), RViz2, and controller (bicycle_control).
"""

import os
from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    try:
        pkg_share = get_package_share_directory('bicycle_sim')
        rviz_config = os.path.join(pkg_share, 'bicycle.rviz')
        urdf_file = os.path.join(pkg_share, 'urdf', 'racecar.urdf')
    except (PackageNotFoundError, Exception):
        pkg_share = None
        rviz_config = None
        urdf_file = None

    # Fallback to local source path if package is not yet installed in share
    if not urdf_file or not os.path.isfile(urdf_file):
        urdf_file = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'urdf', 'racecar.urdf')
        )
    if not rviz_config or not os.path.isfile(rviz_config):
        rviz_config = os.path.abspath(
            os.path.join(os.path.dirname(__file__), 'bicycle.rviz')
        )

    # Launch configuration variables
    use_rviz = LaunchConfiguration('rviz')
    track_file = LaunchConfiguration('track_file')
    trajectory_type = LaunchConfiguration('trajectory_type')
    controller = LaunchConfiguration('controller')
    use_analyzer = LaunchConfiguration('analyzer')
    target_speed = LaunchConfiguration('target_speed')

    # Load URDF XML directly without requiring external xacro CLI
    if os.path.isfile(urdf_file):
        with open(urdf_file, 'r') as f:
            robot_description = f.read()
    else:
        xacro_file = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'urdf', 'racecar.xacro')
        )
        try:
            import xacro
            robot_description = xacro.process_file(xacro_file).toxml()
        except Exception:
            robot_description = ParameterValue(
                Command(['xacro ', xacro_file]),
                value_type=str
            )

    def controller_node(mode):
        """One conditional controller node per control mode."""
        return Node(
            package='bicycle_control',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[{
                'control_mode': mode,
                'target_speed': ParameterValue(target_speed, value_type=float),
                'velocity_mode': 'curvature',
            }],
            condition=IfCondition(
                PythonExpression(["'", controller, "'.lower() == '" + mode + "'"])
            )
        )

    lap_analyzer = Node(
        package='track_environment',
        executable='lap_analyzer',
        name='lap_analyzer',
        output='screen',
        parameters=[{
            'log_file': ParameterValue(LaunchConfiguration('log_file'), value_type=str),
            'controller_name': ParameterValue(controller, value_type=str),
            'max_laps': ParameterValue(LaunchConfiguration('max_laps'), value_type=int),
        }],
        condition=IfCondition(use_analyzer)
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2 for visualization'),
        DeclareLaunchArgument('track_file', default_value='centerline_0.csv',
                              description='CSV track file for the path and start pose'),
        DeclareLaunchArgument('trajectory_type', default_value='centerline',
                              description='Trajectory type: centerline, sp, or iqp'),
        DeclareLaunchArgument('controller', default_value='none',
                              description='none, pure_pursuit, lateral_pid, mpc, teleop'),
        DeclareLaunchArgument('analyzer', default_value='true',
                              description='Launch lap analyzer and real-time HUD'),
        DeclareLaunchArgument('use_cruise_control', default_value='false',
                              description='Closed-loop cruise control in teleop bridge'),
        DeclareLaunchArgument('target_speed', default_value='4.0',
                              description='Base target speed [m/s] for autonomous modes'),
        DeclareLaunchArgument('log_file', default_value='',
                              description='CSV file the lap analyzer appends lap stats to'),
        DeclareLaunchArgument('max_laps', default_value='0',
                              description='Stop the whole launch after N laps (0 = never)'),

        # 1. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # 2. Kinematic Bicycle Simulator Plant Node
        Node(
            package='bicycle_sim',
            executable='sim_node',
            name='kinematic_bicycle',
            output='screen',
            arguments=['--track', track_file],
            parameters=[{
                'wheelbase_length': 1.25,
                'dt': 0.1,
                'car_name': 'ego_racecar'
            }]
        ),

        # 3. Path Generator Node
        Node(
            package='track_environment',
            executable='path_gen',
            name='path_gen',
            output='screen',
            parameters=[{
                'track_file': track_file,
                'trajectory_type': trajectory_type,
                'close_loop': True
            }]
        ),

        # 4. Lap Analyzer (+ shut everything down when it finishes max_laps)
        lap_analyzer,
        RegisterEventHandler(
            OnProcessExit(
                target_action=lap_analyzer,
                on_exit=[EmitEvent(event=Shutdown(reason='lap analyzer finished'))]
            )
        ),

        # 5. RViz2 Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz)
        ),

        # 6. Controllers from bicycle_control
        controller_node('lateral_pid'),
        controller_node('pure_pursuit'),
        controller_node('mpc'),

        # 7. Keyboard Teleoperation Bridge
        Node(
            package='bicycle_control',
            executable='teleop_bridge',
            name='teleop_bridge',
            output='screen',
            parameters=[{
                'use_cruise_control': ParameterValue(
                    LaunchConfiguration('use_cruise_control'), value_type=bool)
            }],
            condition=IfCondition(
                PythonExpression(["'", controller, "'.lower() == 'teleop'"])
            )
        ),
    ])