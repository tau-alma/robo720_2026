cmake_minimum_required(VERSION 3.8)
project(goal_publishers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Build tool
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ROS
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)

# Eigen + KDL
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(kdl_parser REQUIRED)

# Custom packages for FR3
find_package(franka_kdl REQUIRED)

# Add executable for trajectory publisher node
add_executable(trajectory_publisher src/trajectory_publisher.cpp)

# Header file
target_include_directories(trajectory_publisher PRIVATE
  include
)

ament_target_dependencies(trajectory_publisher
  rclcpp
  sensor_msgs
  trajectory_msgs
  kdl_parser
  urdf
  franka_kdl
)

# Python executables
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/goal_pose_publisher.py
  scripts/pose_trajectory_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install executable
install(TARGETS
  trajectory_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()