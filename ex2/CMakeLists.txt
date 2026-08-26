cmake_minimum_required(VERSION 3.8)
project(ex2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(urdf REQUIRED)
find_package(pluginlib REQUIRED)

# Eigen + KDL
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(kdl_parser REQUIRED)

# Gazebo
find_package(ros_gz REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)

# Custom package for FR3
find_package(franka_description REQUIRED)

# Install the project launch files
install(
  DIRECTORY
    launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()