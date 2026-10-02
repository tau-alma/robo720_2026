#ifndef ROBO720_2026_EX4__TASK_SPACE_KINEMATIC_CONTROLLER_HPP_
#define ROBO720_2026_EX4__TASK_SPACE_KINEMATIC_CONTROLLER_HPP_

#include "franka_kdl/robot_constants.hpp"
#include "franka_kdl/solver.hpp"

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ex4 {

class TaskSpaceKinematicController : public controller_interface::ControllerInterface {
    public:
        // Controller interface
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::return_type update(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;
        CallbackReturn on_init() override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    private:
        // Non-RT (realtime) subscriber
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;
        
        // Buffer for RT update() loop (store a shared_ptr instead of the message itself -> less copying)
        realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::PoseStamped>> rt_goal_pose_buffer_;

        // Name of the following controller in the controller chain
        std::string following_controller_;

        std::string robot_description_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        Vector7d position_interface_vals_;

        std::vector<std::string> joint_names_;
        std::string root_link_;
        std::string tip_link_;

        KDL::Tree tree_;
        KDL::Chain chain_;
        std::unique_ptr<Solver> solver_;

        // Eigen variables
        Vector7d q_;
        Vector7d qd_; // desired joint positions
        Vector6d xe_; // pose error in task space
        Vector6d xd_dot_; // desired end effector velocity
        Vector7d q_dot_cmd_; // velocity commands
        Eigen::Matrix<double, NUM_TASK, NUM_JOINTS> J_;
        Eigen::Matrix<double, NUM_JOINTS, NUM_TASK> J_pinv_; // Jacobian pseudo-inverse

        // KDL variables
        KDL::JntArray q_kdl_;
        KDL::JntArray qd_kdl_;
        KDL::Frame x_kdl_;
        KDL::Twist xe_kdl_;
        KDL::Jacobian J_kdl_;

        double elapsed_time_{0.0};

        std::vector<double> p_gains_;
        Vector6d Kp_;

        // Subscriber callback 
        void goal_pose_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg);

        // Read joint states from the hardware state interface
        void updateJointStates();

        // Debug print functions to reduce clutter in the controller code
        void kdl_chain_error_print();
        void kdl_chain_success_print();
};

}  // namespace ex4

#endif  // ROBO720_2026_EX4__TASK_SPACE_KINEMATIC_CONTROLLER_HPP_