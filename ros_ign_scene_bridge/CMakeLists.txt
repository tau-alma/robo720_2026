cmake_minimum_required(VERSION 3.8)
project(ros_ign_scene_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)

find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

add_executable(scene_bridge src/scene_bridge.cpp)
ament_target_dependencies(scene_bridge rclcpp custom_msgs)
target_link_libraries(scene_bridge
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8
)

# Install executable
install(TARGETS
  scene_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()