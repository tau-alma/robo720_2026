import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    controller_name = LaunchConfiguration("controller_name")

    controller_launch_argument = DeclareLaunchArgument(
        "controller_name",
        default_value="pf_velocity_controller",
        description="Which controller to use (velocity or torque).",
        choices=["pf_velocity_controller", "pf_effort_controller"]
    )

    # Initialize controller arguments
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("ex6"), "config", "franka_controllers.yaml"]
    )

    # World file
    world_file = os.path.join(
        get_package_share_directory("ex6"), "world", "obstacle_world.sdf"
    )

    # RViz config file
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ex6"), "config", "config.rviz"]
    )

    # Find the robot description file
    robot_description_xacro = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("franka_description"),
                 "urdf",
                 "effort_fr3_arm.urdf.xacro"]
            ),
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
            " ",
            "effort_commands:=true",
        ]
    )

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True,
                    "robot_description": ParameterValue(robot_description_xacro, value_type=str)
        }],
        remappings=[("robot_description", "robot_description")]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    pf_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller_name, "--controller-manager", "/controller_manager"],
    )

    # GZ nodes
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_xacro,
            "-name",
            "fr3_arm",
            "-allow_renaming",
            "true",
        ],
    )

    # launch arguments for Gazebo {args, world (- v 1 = log level)}
    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": f" -r -v 1 {world_file}"}.items(),
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config
        ]
    )

    # Bridge the static object poses from Gazebo to ROS
    pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="pose_bridge",
        arguments=[
            "/world/obstacle_world/pose/info@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
        ],
        output="screen",
    )

    # Bridge scene info to ROS with a custom bridge
    scene_bridge = Node(
        package="ros_ign_scene_bridge",
        executable="scene_bridge",
        output="screen",
    )

    nodes_to_start = [
        controller_launch_argument,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        pf_controller_spawner,
        gz_spawn_entity,
        gz_launch_description,
        rviz_node,
        pose_bridge,
        scene_bridge,
        SetEnvironmentVariable('IGN_GAZEBO_VERBOSE', '1'),
    ]

    return LaunchDescription(nodes_to_start)