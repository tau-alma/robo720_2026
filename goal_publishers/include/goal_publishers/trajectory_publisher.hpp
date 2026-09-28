#ifndef ROBO720_2026_GOAL_PUBLISHERS__TRAJECTORY_PUBLISHER_HPP_
#define ROBO720_2026_GOAL_PUBLISHERS__TRAJECTORY_PUBLISHER_HPP_

#include "franka_kdl/solver.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>


/**
 * Class to publish different trajectories (e.g. line, circle, ...)
 */
class TrajectoryPublisher : public rclcpp::Node
{
    public:
        TrajectoryPublisher();

    private:
        std::string robot_description_;
        std::string root_link_;
        std::string tip_link_;
        KDL::Tree tree_;
        KDL::Chain chain_;
        std::unique_ptr<Solver> solver_;

        KDL::JntArray q_;
        KDL::JntArray q_cmd_;
        KDL::JntArray q_dot_cmd_;

        std::string trajectory_type_;

        // Joint state subscriber
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
        void joint_state_callback(const sensor_msgs::msg::JointState& msg);

        // Publisher
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr trajectory_point_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
        trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg_;
        geometry_msgs::msg::PoseStamped goal_pose_msg_;
        void timer_callback();

        // Move the end effector in a straight line along y axis.
        void line_trajectory(KDL::Vector& tgt_pos, KDL::Twist& tgt_vel, double t);
        // Move the end effector in a circle trajectory in yz plane.
        void circle_trajectory(KDL::Vector& tgt_pos, KDL::Twist& tgt_vel, double t);
};

#endif  // ROBO720_2026_GOAL_PUBLISHERS__TRAJECTORY_PUBLISHER_HPP_