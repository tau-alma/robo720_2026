cmake_minimum_required(VERSION 3.12)
project(ex1)

set(CMAKE_CXX_STANDARD 17)

# Eigen
find_package(Eigen3 REQUIRED)

# KDL
find_package(orocos_kdl REQUIRED)

# Python for matplotlib-cpp
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

add_executable(ex1 src/main.cpp
    src/kinematics.cpp)

target_include_directories(ex1 PRIVATE
    include
    /usr/local/include/matplotlibcpp
    ${EIGEN3_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
)

target_link_libraries(ex1
    ${orocos_kdl_LIBRARIES}
    ${Python3_LIBRARIES}
    Eigen3::Eigen
)