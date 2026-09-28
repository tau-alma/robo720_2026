#ifndef ROBO720_2026_EX3__JOINT_CONTROLLER_HPP_
#define ROBO720_2026_EX3__JOINT_CONTROLLER_HPP_

#include "franka_kdl/robot_constants.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ex3 {

/**
 * The joint position controller for two types of movement of the manipulator.
 */
class JointController : public controller_interface::ControllerInterface {
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
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr joint_trajectory_subscriber_;
        
        // Buffer for RT update() loop (store a shared_ptr instead of the message itself -> less copying)
        realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint>> rt_trajectory_buffer_;

        std::string robot_description_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        Vector7d position_interface_vals_;

        std::vector<std::string> joint_names_;
        std::string root_link_;
        std::string tip_link_;

        KDL::Tree tree_;
        KDL::Chain chain_;

        Vector7d q_;
        Vector7d initial_q_;
        Vector7d q_dot_cmd_;
        double elapsed_time_{0.0};

        std::vector<double> p_gains_;

        // For writing the parameter on non-RT thread and reading it in RT update() loop
        std::atomic<bool> use_feedforward_{false};
        rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

        // Subscriber callback 
        void joint_trajectory_callback(const std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint> msg);

        // Read joint states from the hardware state interface
        void updateJointStates();

        // Debug print functions to reduce clutter in the controller code
        void kdl_chain_error_print();
        void kdl_chain_success_print();
};

}  // namespace ex3

#endif  // ROBO720_2026_EX3__JOINT_CONTROLLER_HPP_