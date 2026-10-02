import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Initialize controller arguments
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("ex4"), "config", "franka_controllers.yaml"]
    )

    # World file
    world_file = os.path.join(
        get_package_share_directory("franka_description"), "world", "sensor_world.sdf"
    )

    # RViz config file
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("franka_description"), "config", "config.rviz"]
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
    robot_description = {"robot_description": robot_description_xacro}

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
        remappings=[("robot_description", "robot_description")]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_velocity_controller", "--controller-manager", "/controller_manager"],
    )

    trajectory_publisher_node = Node(
        package="goal_publishers",
        executable="trajectory_publisher",
        output="both",
        parameters=[
            robot_description,
            initial_joint_controllers,
        ]
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

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        joint_velocity_controller_spawner,
        trajectory_publisher_node,
        gz_spawn_entity,
        gz_launch_description,
        rviz_node,
        SetEnvironmentVariable('IGN_GAZEBO_VERBOSE', '1'),
    ]

    return LaunchDescription(nodes_to_start)