#ifndef ROBO720_2026_EX5__VISUAL_CONTROLLER_HPP_
#define ROBO720_2026_EX5__VISUAL_CONTROLLER_HPP_

#include "franka_kdl/robot_constants.hpp"
#include "franka_kdl/solver.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ex5 {

class VisualController : public controller_interface::ControllerInterface {
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
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr marker_pose_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr trajectory_subscriber_;
        
        // Buffer for RT update() loop (store a shared_ptr instead of the message itself -> less copying)
        realtime_tools::RealtimeBuffer<std::shared_ptr<KDL::Frame>> rt_marker_pose_buffer_;
        realtime_tools::RealtimeBuffer<std::shared_ptr<KDL::Frame>> rt_trajectory_buffer_;

        // Flags
        std::atomic<bool> pose_detected_ {false}; // updated for each received marker_pose message, false if no detection
        std::atomic<bool> marker_found_{false}; // true when the marker is found for the first time

        // Marker TF broadcaster for RViz debug
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        std::string robot_description_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        Vector7d position_interface_vals_;
        Vector7d velocity_interface_vals_;

        std::vector<std::string> joint_names_;
        std::string root_link_;
        std::string tip_link_;
        std::string optical_link_;

        KDL::Tree tree_;
        KDL::Chain chain_;
        KDL::Chain chain_optical_;
        std::unique_ptr<Solver> solver_;
        std::unique_ptr<Solver> solver_optical_;

        // Eigen variables
        Eigen::Matrix<double, NUM_JOINTS, NUM_JOINTS> M_;
        Vector7d G_;
        Vector7d C_;
        Eigen::Matrix<double, NUM_TASK, NUM_JOINTS> J_;
        Eigen::Matrix<double, NUM_JOINTS, NUM_TASK> J_pinv_;
        Vector7d q_;
        Vector7d q_dot_;
        Vector7d q_dot_cmd_; // command velocity
        Vector7d e_dot_;
        Vector6d xe_;
        Vector6d xd_dot_;
        Vector7d e_; // position error
        Vector7d u_;
        Vector7d tau_; // torque command

        // KDL variables
        KDL::JntSpaceInertiaMatrix M_kdl_;
        KDL::JntArray G_kdl_;
        KDL::JntArray C_kdl_;
        KDL::Jacobian J_kdl_;
        KDL::JntArray q_kdl_;
        KDL::JntArray q_dot_kdl_;
        KDL::Frame T_base_cam_;
        KDL::Frame T_cam_marker_;
        KDL::Frame T_marker_goal_;
        KDL::Frame x_kdl_;
        KDL::Frame xd_kdl_;
        KDL::Twist xe_kdl_;

        double elapsed_time_{0.0};

        std::vector<double> p_gains_;
        std::vector<double> d_gains_;
        Vector6d Kp_;
        Vector7d Kd_;

        // Subscriber callback 
        void marker_pose_callback(const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg);
        void trajectory_callback(const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg);

        // Marker TF publisher
        void publish_marker_tf();

        // Read joint states from the hardware state interface
        void updateJointStates();

        // Debug print functions to reduce clutter in the controller code
        void kdl_chain_error_print(const std::string& tip_link);
        void kdl_chain_success_print(const std::string& tip_link, const KDL::Chain& chain);
};

}  // namespace ex5

#endif  // ROBO720_2026_EX5__VISUAL_CONTROLLER_HPP_