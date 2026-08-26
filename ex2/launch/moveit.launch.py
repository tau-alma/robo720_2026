import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

# We use two separate functions to evaluate a launch argument immediately with Opaque function
def launch_setup(context, *args, **kwargs):
    package_name_str = LaunchConfiguration("package_name").perform(context)

    nodes_to_start = []

    moveit_config = (
        MoveItConfigsBuilder("franka", package_name=package_name_str)
        .robot_description(file_path="config/fr3.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        # Using mimic joint causes naming "issues" which floods terminal with error messages.
        # These do not affect planning or simulation in any way so we suppress them with log level setting.
        arguments=[
            "--ros-args",
            "--log-level", "moveit_robot_model.robot_model:=fatal",
        ],
    )
    nodes_to_start.append(run_move_group_node)

    rviz_config = PathJoinSubstitution(
        [FindPackageShare(package_name_str), "config", "moveit.rviz"]
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # Same naming "issue" as above
        arguments=["-d", rviz_config,
                   "--ros-args",
                   "--log-level", "moveit_robot_model.robot_model:=fatal",],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True}
        ],
    )
    nodes_to_start.append(rviz_node)

    # Static TF (might be redundant)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base"],
    )
    nodes_to_start.append(static_tf)

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description,
                    {"use_sim_time": True},],
    )
    nodes_to_start.append(robot_state_publisher)

    ###################################################################
    # If we were not using Gazebo ROS2 control plugin, here we would  #
    # launch a controller_manager node separately, before controllers #
    ###################################################################

    # Controller nodes (joint state broadcaster is part of ros2 control)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager", # same flag as -c below
            "/controller_manager",
        ],
    )
    nodes_to_start.append(joint_state_broadcaster_spawner)

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_arm_controller", "-c", "/controller_manager"],
    )
    nodes_to_start.append(arm_controller_spawner)

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "-c", "/controller_manager"],
    )
    nodes_to_start.append(hand_controller_spawner)

    # World file for Gazebo
    world_file = os.path.join(
        get_package_share_directory("franka_description"), "world", "sensor_world.sdf"
    )

    # We need the robot description as string to spawn it in Gazebo,
    # hence we cannot use moveit_config.robot_description which is a dict
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(package_name_str),
                 "config",
                 "fr3.urdf.xacro"]
            ),
            " ",
            "sim_ignition:=true",
        ],
    )

    # Gazebo nodes
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "fr3_arm",
            "-allow_renaming",
            "true",
        ],
    )
    nodes_to_start.append(gz_spawn_entity)

    # Gazebo launch arguments
    # -r starts simulation running
    # -v 1 sets verbosity level 1 (error messages)
    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": f" -r -v 1 {world_file}"}.items(),
    )
    nodes_to_start.append(gz_launch_description)

    # Bridge Gazebo topics to ROS
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",  # bridge Gazebo simulation time to ROS
        ],
    )
    nodes_to_start.append(ros_gz_bridge)

    return nodes_to_start

def generate_launch_description():
    package_name_launch_arg = DeclareLaunchArgument(
        "package_name",
        default_value="franka_gripper_moveit",
        description="Choose package to load MoveIt configuration from.",
    )

    return LaunchDescription([
        package_name_launch_arg,
        OpaqueFunction(function=launch_setup),
    ])