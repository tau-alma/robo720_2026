cmake_minimum_required(VERSION 3.5)

project(franka_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install the URDF and Xacro files
install(DIRECTORY urdf meshes config world
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
