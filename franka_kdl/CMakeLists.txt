cmake_minimum_required(VERSION 3.8)
project(franka_kdl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Build tool
find_package(ament_cmake REQUIRED)

# Eigen + KDL
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)


# Custom package for Franka Panda
find_package(franka_description REQUIRED)

# Add source files
add_library(${PROJECT_NAME} SHARED
  src/solver.cpp
)

# Header files
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  orocos_kdl_vendor
  Eigen3
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  orocos_kdl_vendor
  Eigen3
)

ament_package()