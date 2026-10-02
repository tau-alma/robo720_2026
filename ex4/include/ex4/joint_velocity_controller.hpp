#ifndef ROBO720_2026_EX4__JOINT_VELOCITY_CONTROLLER_HPP_
#define ROBO720_2026_EX4__JOINT_VELOCITY_CONTROLLER_HPP_

#include "franka_kdl/robot_constants.hpp"
#include "franka_kdl/solver.hpp"

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ex4 {

class JointVelocityController : public controller_interface::ChainableControllerInterface {
    public:
        // Controller interface
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        // We implement this low-level controller as chainable controller
        controller_interface::return_type update_and_write_commands( // <- old update() method
            const rclcpp::Time& time, const rclcpp::Duration& period) override;
        CallbackReturn on_init() override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        // New method required by ChainableControllerInterface
        bool on_set_chained_mode(bool chained_mode) override;

    private:
        // More new methods required by ChainableControllerInterface
        std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
        controller_interface::return_type update_reference_from_subscribers() override;

        // Non-RT (realtime) subscriber
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr joint_cmd_subscriber_;
        
        // Buffer for RT update_and_write_commands() loop (store a shared_ptr instead of the message itself -> less copying)
        realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint>> rt_buffer_;

        // Read from .yaml config file
        std::string robot_description_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;
        std::vector<std::string> reference_interface_types_;

        // Constructed from joint names and interface types
        std::vector<std::string> command_interface_names_;
        std::vector<std::string> reference_interface_names_;

        Vector7d position_interface_vals_;
        Vector7d velocity_interface_vals_;

        std::vector<std::string> joint_names_;
        std::string root_link_;
        std::string tip_link_;

        KDL::Tree tree_;
        KDL::Chain chain_;
        std::unique_ptr<Solver> solver_;

        // Eigen variables
        Eigen::Matrix<double, NUM_JOINTS, NUM_JOINTS> M_;
        Vector7d G_;
        Vector7d C_;
        Vector7d q_;
        Vector7d q_dot_;
        Vector7d qd_dot_; // desired joint velocities
        Vector7d e_dot_; // velocity error
        Vector7d u_; // desired joint accelerations
        Vector7d tau_; // torque command

        // KDL variables
        KDL::JntSpaceInertiaMatrix M_kdl_;
        KDL::JntArray G_kdl_;
        KDL::JntArray C_kdl_;
        KDL::JntArray q_kdl_;
        KDL::JntArray q_dot_kdl_;

        double elapsed_time_{0.0};

        std::vector<double> d_gains_;
        Vector7d Kd_;

        // Subscriber callback 
        void joint_cmd_callback(const std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint> msg);

        // Read joint states from the hardware state interface
        void updateJointStates();

        // Debug print functions to reduce clutter in the controller code
        void kdl_chain_error_print();
        void kdl_chain_success_print();
};

}  // namespace ex4

#endif  // ROBO720_2026_EX4__JOINT_VELOCITY_CONTROLLER_HPP_