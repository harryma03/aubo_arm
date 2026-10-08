"""Launch one detector and one queue with one shared lifecycle."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    queue = Node(
        package='aubo_vision_grasp',
        executable='visual_pick_queue',
        name='visual_pick_queue',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}],
    )
    detector = Node(
        package='aubo_vision_grasp',
        executable='red_target_detector',
        name='red_target_detector',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    stop_when_queue_finishes = RegisterEventHandler(
        OnProcessExit(
            target_action=queue,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason='visual pick queue finished')
                )
            ],
        )
    )
    stop_when_detector_finishes = RegisterEventHandler(
        OnProcessExit(
            target_action=detector,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason='vision detector stopped')
                )
            ],
        )
    )

    # Start the queue first so its subscription exists before the detector's
    # first publication. Both nodes then shut down as one process group.
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        stop_when_queue_finishes,
        stop_when_detector_finishes,
        queue,
        detector,
    ])
