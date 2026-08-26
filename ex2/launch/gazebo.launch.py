import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ROS2 control config
    joint_state_broadcaster_config = PathJoinSubstitution(
        [FindPackageShare("franka_description"), "config", "joint_state_broadcaster.yaml"]
    )

    # World file for Gazebo
    world_file = os.path.join(
        get_package_share_directory("franka_description"), "world", "sensor_world.sdf"
    )

    # RViz config file
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ex2"), "config", "default_config.rviz"]
    )

    # Find robot description file
    robot_description_content = Command(
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
            joint_state_broadcaster_config,
            " ",
            "effort_commands:=true",
        ],
    )

    # ROS nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{
            "use_sim_time": True,
            "robot_description": ParameterValue(robot_description_content, value_type=str),
        }],
        remappings=[("robot_description", "robot_description")],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
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

    # Gazebo launch arguments
    # -r starts running
    # -v 1 sets verbosity level 1 (error messages)
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

    # PlotJuggler node
    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        output="screen",
        arguments=[]
    )

    # Bridge Gazebo topics to ROS
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            gz_spawn_entity,
            gz_launch_description,
            rviz_node,
            plotjuggler_node,
            ros_gz_bridge,
        ]
    )