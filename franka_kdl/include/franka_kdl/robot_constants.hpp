#ifndef ROBO720_2026_FRANKA_KDL__ROBOT_CONSTANTS_HPP_
#define ROBO720_2026_FRANKA_KDL__ROBOT_CONSTANTS_HPP_

#include <cstddef>
#include <Eigen/Dense>

constexpr std::size_t NUM_TASK = 6;
constexpr std::size_t NUM_JOINTS = 7;

using Vector6d = Eigen::Matrix<double, NUM_TASK, 1>;
using Vector7d = Eigen::Matrix<double, NUM_JOINTS, 1>;

#endif  // ROBO720_2026_FRANKA_KDL__ROBOT_CONSTANTS_HPP_