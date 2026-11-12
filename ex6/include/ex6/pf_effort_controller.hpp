#ifndef ROBO720_2026_EX6__PF_EFFORT_CONTROLLER_HPP_
#define ROBO720_2026_EX6__PF_EFFORT_CONTROLLER_HPP_

#include "franka_kdl/robot_constants.hpp"
#include "franka_kdl/solver.hpp"

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <custom_msgs/msg/obstacle_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <urdf/model.h>

#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ex6 {

class PFEffortController : public controller_interface::ControllerInterface {
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
        // Subscriber for obstacle initial positions and shape
        rclcpp::Subscription<custom_msgs::msg::ObstacleArray>::SharedPtr init_obstacle_subscriber_;
        // Subscriber for obstacle positions
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_obj_pose_;
        
        // Buffer for RT update() loop (store a shared_ptr instead of the message itself -> less copying)
        realtime_tools::RealtimeBuffer<std::shared_ptr<KDL::Frame>> rt_goal_pose_buffer_;
        realtime_tools::RealtimeBuffer<std::vector<custom_msgs::msg::Obstacle>> rt_obstacles_;

        // Marker publisher for RViz visualization
        rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

        std::string robot_description_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        Vector7d position_interface_vals_;
        Vector7d velocity_interface_vals_;

        // Joint limits and variables for computing repulsive force (get from yaml file)
        std::vector<double> joint_mins_;
        std::vector<double> joint_maxs_;
        double k_rep_;
        double Q_joint_;
        double Q_ee_;

        // For storing the obstacle data
        std::unordered_map<std::string, custom_msgs::msg::Obstacle> obstacle_map_;
        
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
        Eigen::Matrix<double, NUM_TASK, NUM_JOINTS> J_;
        Eigen::Matrix<double, NUM_TASK, NUM_JOINTS> J_dot_;
        Eigen::Matrix<double, NUM_JOINTS, NUM_TASK> J_pinv_;
        Vector7d q_;
        Vector7d q_dot_;
        Vector6d xe_;  // task space error
        Vector6d x_dot_;
        Vector6d xd_dot_; // task space desired velocity
        Vector7d u_;
        Vector7d tau_; // torque command
        Vector7d tau_att_;
        Vector7d tau_rep_;

        // KDL variables
        KDL::JntSpaceInertiaMatrix M_kdl_;
        KDL::JntArray G_kdl_;
        KDL::JntArray C_kdl_;
        KDL::Jacobian J_kdl_;
        KDL::Jacobian J_dot_kdl_;
        KDL::JntArray q_kdl_;
        KDL::JntArray q_dot_kdl_;
        KDL::Frame x_kdl_;
        KDL::Frame xd_kdl_;
        KDL::Twist xe_kdl_;

        double elapsed_time_{0.0};

        std::vector<double> p_gains_;
        std::vector<double> d_gains_;
        std::vector<double> b_damping_;
        Vector6d Kp_;
        Vector6d Kd_;
        Vector7d B_; // damping

        // Subscriber callbacks
        void goal_pose_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg);
        void init_obstacle_callback(const custom_msgs::msg::ObstacleArray msg);
        void object_pose_callback(const tf2_msgs::msg::TFMessage& msg);

        // Publishers
        void publish_markers();

        // Read joint states from the hardware state interface
        void updateJointStates();

        // Debug print functions to reduce clutter in the controller code
        void kdl_chain_error_print();
        void kdl_chain_success_print();
};

}  // namespace ex6

#endif  // ROBO720_2026_EX6__PF_EFFORT_CONTROLLER_HPP_