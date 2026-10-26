cmake_minimum_required(VERSION 3.8)
project(ex5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Build tools
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ROS
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(urdf REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)

# Eigen + KDL
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(kdl_parser REQUIRED)

# Gazebo
find_package(ros_gz REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)

# Custom packages
find_package(franka_description REQUIRED)
find_package(franka_kdl REQUIRED)

# Make the source files executable
add_library(${PROJECT_NAME} SHARED
  src/visual_controller.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  rclcpp
  rclcpp_lifecycle
  tf2_ros
  realtime_tools
  urdf
  std_msgs
  control_msgs
  kdl_parser
  franka_kdl
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the header files
install(DIRECTORY include/
  DESTINATION include
)

# Install the controller plugin
install(FILES controllers_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(controller_interface controllers_plugin.xml)

# Install the project files
install(
  DIRECTORY
    config launch model world
  DESTINATION share/${PROJECT_NAME}
)

# Python executables
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/pose_estimate.py
  scripts/tri_wave.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_dependencies(
  controller_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()