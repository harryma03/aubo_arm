from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):
    # Command-line arguments
    robot_xacro_file = LaunchConfiguration("robot_xacro_file")
    support_package = LaunchConfiguration("support_package")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    aubo_type = LaunchConfiguration("aubo_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")

    # Planning context
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(support_package), "urdf/xacro/inc/", robot_xacro_file]
            ),
            " ",
            "aubo_type:=",
            aubo_type,
            " ",
            "robot_x:=",
            robot_x,
            " ",
            "robot_y:=",
            robot_y,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "config", moveit_config_file]
            ),
            " ",
            "aubo_type:=",
            aubo_type,
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content.perform(
            context
        )
    }

    kinematics_yaml = load_yaml(
        "aubo_moveit_config", "config/kinematics.yaml"
    )

    robot_model = aubo_type.perform(context)
    joint_limits_file = "config/joint_limits_jaka_Zu12.yaml"
    joint_limits_yaml = {
        "robot_description_planning": load_yaml(
            "aubo_moveit_config", joint_limits_file
        )
    }

    # Planning functionality. Use the current MoveIt multi-pipeline layout so
    # both move_group and the RViz MotionPlanning panel discover OMPL.
    ompl_pipeline = {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
        "start_state_max_bounds_error": 0.1,
        "sample_duration": 0.005,
    }
    ompl_pipeline.update(load_yaml(
        "aubo_moveit_config", "config/ompl_planning.yaml"
    ))
    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_pipeline,
    }

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        "aubo_moveit_config", "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        # MoveIt does not handle controller switching automatically
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/move_group/planning_scene_monitor",
            "publish_planning_scene_topic": "/move_group/publish_planning_scene",
            "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
           }
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {"use_sim_time": True},
        ],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("aubo_moveit_config"), "config"
    )
    rviz_config = os.path.join(rviz_base, "view_robot.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            joint_limits_yaml,
            {"use_sim_time": True},
        ],
        condition=IfCondition(launch_rviz),
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aubo_world_base_tf",
        output="log",
        arguments=[
            "--x", robot_x, "--y", robot_y, "--z", "0.0",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "world", "--child-frame-id", "base_link",
        ],
    )

    # robot_state_publisher is owned by aubo_gazebo.launch.py in the
    # combined simulation startup.  Starting another one here duplicates
    # robot_description and republishes the same robot TF tree.
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[robot_description]
        )

    nodes_to_start = [move_group_node, rviz_node, static_tf_node]
    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("robot_x", default_value="0.0", description="JAKA base X offset in world coordinates."))
    declared_arguments.append(DeclareLaunchArgument("robot_y", default_value="0.0", description="JAKA base Y offset in world coordinates."))

    declared_arguments.append(
        DeclareLaunchArgument(
            "aubo_type",
            default_value="jaka_Zu12",
            choices=["jaka_Zu12"],
            description=(
                "Robot model implementing the canonical workcell interface. "
                "The public startup selector is ROBOT_MODEL."
            ),
        )
    )

    # TODO(andyz): add other options
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_xacro_file",
            default_value="aubo_ros2.xacro",
            description="Xacro describing the robot.",
            choices=["aubo_ros2.xacro"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "support_package",
            default_value="aubo_description",
            description="Name of the support package",
            choices=["aubo_description"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="aubo_moveit_config",
            description="Name of the support package",
            choices=["aubo_moveit_config"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            description="Name of the SRDF file",
            default_value="aubo_robot.srdf",
            choices=["aubo_robot.srdf"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch the MoveIt RViz interface.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
