from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('aubo_workcell_description')
    workcell_sdf = os.path.join(pkg, 'models', 'workcell_environment', 'model.sdf')
    ground_sdf = os.path.join(pkg, 'models', 'workcell_ground', 'model.sdf')

    workcell_x = LaunchConfiguration('workcell_x')
    workcell_y = LaunchConfiguration('workcell_y')
    workcell_z = LaunchConfiguration('workcell_z')
    ground_x = LaunchConfiguration('ground_x')
    ground_y = LaunchConfiguration('ground_y')
    ground_z = LaunchConfiguration('ground_z')

    # The JAKA base_link stays at world z=0. The separate workcell CAD has its
    # arm mounting point +0.95 m above its origin, so spawn it at -0.95 m.
    return LaunchDescription([
        DeclareLaunchArgument('workcell_x', default_value='0.0'),
        DeclareLaunchArgument('workcell_y', default_value='0.0'),
        DeclareLaunchArgument('workcell_z', default_value='-0.95'),
        DeclareLaunchArgument('ground_x', default_value='0.0'),
        DeclareLaunchArgument('ground_y', default_value='0.0'),
        DeclareLaunchArgument('ground_z', default_value='-0.95'),

        # Remove Gazebo's default ground at z=0 so it does not hide the lowered workcell.
        TimerAction(
            period=0.5,
            actions=[ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/delete_entity',
                     'gazebo_msgs/srv/DeleteEntity', "{name: 'ground_plane'}"],
                output='screen'
            )]
        ),

        # Spawn replacement ground at the physical bottom of the imported workcell.
        TimerAction(
            period=1.5,
            actions=[ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-entity', 'workcell_ground', '-file', ground_sdf,
                     '-x', ground_x, '-y', ground_y, '-z', ground_z],
                output='screen'
            )]
        ),

        # Spawn the imported non-arm scene. Its original arm mount lands at world z=0.
        TimerAction(
            period=2.0,
            actions=[ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-entity', 'workcell_environment', '-file', workcell_sdf,
                     '-x', workcell_x, '-y', workcell_y, '-z', workcell_z],
                output='screen'
            )]
        ),
    ])
