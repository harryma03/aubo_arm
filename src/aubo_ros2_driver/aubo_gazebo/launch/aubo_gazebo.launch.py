from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    robot_type = LaunchConfiguration("robot_type")

    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare(description_package),
                "urdf/xacro/inc",
                description_file,
            ]),
            " ",
            "aubo_type:=",
            robot_type,
            " ",
            "sim_gazebo:=true",
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

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Hold the JAKA spawn pose before the workcell/camera startup delay.
    jaka_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_jaka_trajectory_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[jaka_trajectory_controller_spawner],
        )
    )

    # Use the official Gazebo ROS launch wrapper so it prepares the
    # Gazebo model/plugin/resource paths for gazebo_ros2_control and the
    # suction plugin.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={"gui": gazebo_gui}.items(),
    )

    # Spawn the exact same URDF consumed by robot_state_publisher and MoveIt.
    # Keeping a second hand-edited SDF here caused Gazebo link transforms to
    # diverge from the RViz model.
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_jaka",
        arguments=["-entity", "jaka_Zu12", "-topic", "robot_description"],
        output="screen",
    )

    # The model plugin holds this same pose while ros2_control starts. Once the
    # trajectory controller is active, send the matching command so controller
    # ownership begins without a jump or a path through the workcell.
    set_jaka_initial_pose = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--once",
            "/joint_trajectory_controller/joint_trajectory",
            "trajectory_msgs/msg/JointTrajectory",
            "{joint_names: [shoulder_joint, upperArm_joint, foreArm_joint, "
            "wrist1_joint, wrist2_joint, wrist3_joint], "
            "points: [{positions: [-1.5707963267948966, 1.5707963267948966, 0.0, "
            "3.141592653589793, 1.5707963267948966, 1.5707963267948966], "
            "time_from_start: {sec: 1, nanosec: 0}}]}"
        ],
        output="screen",
    )
    set_initial_pose_after_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jaka_trajectory_controller_spawner,
            on_exit=[set_jaka_initial_pose],
        )
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        gazebo,
        gazebo_spawn_robot,
    ]

    start_controller = start_joint_controller.perform(context).lower()
    if (
        robot_type.perform(context) == "jaka_Zu12"
        and start_controller in ("true", "1", "yes")
    ):
        nodes_to_start.append(delay_jaka_trajectory_controller_spawner)
        nodes_to_start.append(set_initial_pose_after_controller)

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("robot_x", default_value="0.0", description="JAKA base X offset in world coordinates."))
    declared_arguments.append(DeclareLaunchArgument("robot_y", default_value="0.0", description="JAKA base Y offset in world coordinates."))
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            description="JAKA robot model used by this simulation.",
            choices=["jaka_Zu12"],
            default_value="jaka_Zu12",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="aubo_gazebo",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="aubo_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="aubo_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="aubo_ros2.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="false",
            description=(
                "Launch the Gazebo GUI. Keep false for headless/remote "
                "environments."
            ),
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
