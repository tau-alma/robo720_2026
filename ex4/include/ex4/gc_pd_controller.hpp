#ifndef ROBO720_2026_EX4__GC_PD_CONTROLLER_HPP_
#define ROBO720_2026_EX4__GC_PD_CONTROLLER_HPP_

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

#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ex4 {

class GCPDController : public controller_interface::ControllerInterface {
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

        std::string robot_description_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        Vector7d position_interface_vals_;
        Vector7d velocity_interface_vals_;

        std::vector<std::string> joint_names_;
        std::string root_link_;
        std::string tip_link_;

        KDL::Tree tree_;
        KDL::Chain chain_;
        std::unique_ptr<Solver> solver_;

        // Eigen variables
        Vector7d G_;
        Vector7d q_;
        Vector7d q_dot_;
        Vector7d qd_; // desired joint positions
        Vector7d e_; // position error
        Vector7d tau_; // torque command

        // KDL variables
        KDL::JntArray G_kdl_;
        KDL::JntArray q_kdl_;
        KDL::JntArray q_dot_kdl_;
        KDL::JntArray qd_kdl_;

        double elapsed_time_{0.0};

        std::vector<double> p_gains_;
        std::vector<double> d_gains_;
        Vector7d Kp_;
        Vector7d Kd_;

        // Subscriber callback 
        void goal_pose_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg);

        // Read joint states from the hardware state interface
        void updateJointStates();

        // Debug print functions to reduce clutter in the controller code
        void kdl_chain_error_print();
        void kdl_chain_success_print();
};

}  // namespace ex4

#endif  // ROBO720_2026_EX4__GC_PD_CONTROLLER_HPP_