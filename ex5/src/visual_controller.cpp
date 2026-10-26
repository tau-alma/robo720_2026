#include "ex5/visual_controller.hpp"
#include "franka_kdl/robot_constants.hpp"
#include "franka_kdl/solver.hpp"

#include <cstddef>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <urdf/model.h>

#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>


namespace ex5 {


controller_interface::return_type VisualController::update(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& period) {
    // Update joint states from hardware state interface
    updateJointStates();

    const double dt = period.seconds();
    elapsed_time_ = elapsed_time_ + dt;

    // Save current state to KDL variables for solvers
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        q_kdl_(i) = position_interface_vals_(i);
        q_dot_kdl_(i) = velocity_interface_vals_(i);
    }

    // Get the current EE pose
    solver_->computeFK(q_kdl_, x_kdl_);

    if (!marker_found_) {  // Follow wave trajectory
        xd_kdl_ = *rt_trajectory_buffer_.readFromRT()->get();
    }
    else {  // Track the marker
        // Read the marker pose
        T_cam_marker_ = *rt_marker_pose_buffer_.readFromRT()->get();

        // Desired pose in base frame (update only if the detection changed)
        if (pose_detected_) {
            solver_optical_->computeFK(q_kdl_, T_base_cam_); // camera pose
            xd_kdl_ = T_base_cam_ * T_cam_marker_ * T_marker_goal_;
        }
    }

    // Get the error in task space
    xe_kdl_ = KDL::diff(x_kdl_, xd_kdl_);

    // Get dynamics parameters and Jacobian
    solver_->compute_dyn_params(q_kdl_, q_dot_kdl_, M_kdl_, G_kdl_, C_kdl_);
    solver_->compute_jac(q_kdl_, J_kdl_);

    // Copy data to Eigen variables for matrix computations
    M_ = M_kdl_.data;
    G_ = G_kdl_.data;
    C_ = C_kdl_.data;
    J_ = J_kdl_.data;
    q_ = q_kdl_.data;
    q_dot_ = q_dot_kdl_.data;

    xe_(0) = xe_kdl_.vel.x(); xe_(1) = xe_kdl_.vel.y(); xe_(2) = xe_kdl_.vel.z();
    xe_(3) = xe_kdl_.rot.x(); xe_(4) = xe_kdl_.rot.y(); xe_(5) = xe_kdl_.rot.z();

    // Jacobian pseudo-inverse
    solver_->get_damped_pseudo_inverse(J_, J_pinv_);

    // Desired end effector velocity
    xd_dot_ = Kp_.cwiseProduct(xe_);

    // Joint velocity command
    q_dot_cmd_ = J_pinv_ * xd_dot_;

    // Joint space PD + computed torque
    e_dot_ = q_dot_cmd_ - q_dot_;
    u_ = Kd_.cwiseProduct(e_dot_);
    tau_ = M_ * u_ + G_ + C_;

    // Send torque commands to the hardware command interface
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        command_interfaces_[i].set_value(tau_(i));
    }

    if (true) {
        publish_marker_tf();
    }

    return controller_interface::return_type::OK;
}

CallbackReturn VisualController::on_init() {
    // Get parameters from franka_controllers.yaml
    try {
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ = auto_declare<std::vector<std::string>>(
            "command_interfaces", command_interface_types_);
        state_interface_types_ = auto_declare<std::vector<std::string>>(
            "state_interfaces", state_interface_types_);

        root_link_ = auto_declare<std::string>("root_link", root_link_);
        tip_link_ = auto_declare<std::string>("tip_link", tip_link_);

        optical_link_ = auto_declare<std::string>("optical_link", optical_link_);

        p_gains_ = auto_declare<std::vector<double>>("p_gains", p_gains_);
        d_gains_ = auto_declare<std::vector<double>>("d_gains", d_gains_);
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            Kd_(i) = d_gains_.at(i);
        }
        for (std::size_t i = 0; i < NUM_TASK; ++i) {
            Kp_(i) = p_gains_.at(i);
        }
    } 
    catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage (on_init) with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Robot initialization done.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn VisualController::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_node()->get_logger(), "Robot configuration started");

    // Get the robot description parameter (effort_fr3_arm.urdf.xacro parsed into one URDF as a string)
    // Due to AsyncParametersClient, this has to be in on_configure() method.
    auto parameters_client =
        std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
    parameters_client->wait_for_service();
    auto future = parameters_client->get_parameters({"robot_description"});
    auto result = future.get();

    // Check if robot description was retrieved succesfully
    if (!result.empty()) {
        robot_description_ = result[0].value_to_string();
    }
    else {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
        return CallbackReturn::ERROR;
    }

    // Parse URDF string into URDF object
    urdf::Model urdf;
    if (!urdf.initString(robot_description_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf file");
        return CallbackReturn::ERROR;
    }
    else {
        RCLCPP_INFO(get_node()->get_logger(), "Found robot_description");
    }

    // Construct a KDL tree object from the URDF
    if (!kdl_parser::treeFromUrdfModel(urdf, tree_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct a KDL tree.");
        return CallbackReturn::ERROR;
    }
    else {
        RCLCPP_INFO(get_node()->get_logger(), "Constructed a KDL tree.");
    }

    // Get the KDL chain from KDL tree (for KDL Solver objects)
    if (!tree_.getChain(root_link_, tip_link_, chain_)) {
        kdl_chain_error_print(tip_link_);

        return CallbackReturn::ERROR;
    }
    else {
        kdl_chain_success_print(tip_link_, chain_);
    }

    // Get the KDL chain from base to optical link
    if (!tree_.getChain(root_link_, optical_link_, chain_optical_)) {
        kdl_chain_error_print(optical_link_);

        return CallbackReturn::ERROR;
    }
    else {
        kdl_chain_success_print(optical_link_, chain_optical_);
    }

    // Create a Solver instance
    solver_.reset(new Solver(chain_));
    solver_optical_.reset(new Solver(chain_optical_));

    // Create a subscription for the joint command
    marker_pose_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "marker_pose", rclcpp::SystemDefaultsQoS(), std::bind(&VisualController::marker_pose_callback,
        this, std::placeholders::_1));

    // subscriber for task-space commands [x y z r p y]
    trajectory_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "command", rclcpp::SystemDefaultsQoS(), std::bind(&VisualController::trajectory_callback,
        this, std::placeholders::_1));

    // Transform broadcaster for debugging
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

    RCLCPP_INFO(get_node()->get_logger(), "Robot configuration done.");
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
VisualController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & jname : joint_names_) {
        for (const auto & interface_type : command_interface_types_) {
            config.names.push_back(jname + "/" + interface_type);
        }
    }
    return config;
}

controller_interface::InterfaceConfiguration
VisualController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & jname : joint_names_) {
        for (const auto & interface_type : state_interface_types_) {
            config.names.push_back(jname + "/" + interface_type);
        }
    }
    return config;
}

CallbackReturn VisualController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
    updateJointStates();
    elapsed_time_ = 0.0;

    M_kdl_.resize(NUM_JOINTS);
    G_kdl_.resize(NUM_JOINTS);
    C_kdl_.resize(NUM_JOINTS);
    J_kdl_.resize(NUM_JOINTS);
    q_kdl_.resize(NUM_JOINTS);
    q_dot_kdl_.resize(NUM_JOINTS);

    T_base_cam_ = KDL::Frame::Identity();
    T_cam_marker_ = KDL::Frame::Identity();
    T_marker_goal_ = KDL::Frame(KDL::Rotation::EulerZYX(0.0, 0.0, M_PI), KDL::Vector(0.0, 0.0, 0.3));

    return CallbackReturn::SUCCESS;
}

void VisualController::updateJointStates() {
    // Pre-check array size to avoid bounds checking in loop
    if (state_interfaces_.size() != state_interface_types_.size() * NUM_JOINTS) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid number of state interfaces");
        return;
    }

    // Get the current joint positions
    auto* interfaces = state_interfaces_.data();
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        // Access interfaces directly with pointer arithmetic
        const auto& position_interface = interfaces[2*i];
        const auto& velocity_interface = interfaces[2*i + 1];

        // Interface name comparison
        const auto& pos_name = position_interface.get_interface_name();
        if (pos_name != "position") {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected position interface, but got %s",
                         pos_name.c_str());
            return;
        }
        const auto& vel_name = velocity_interface.get_interface_name();
        if (vel_name != "velocity") {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected velocity interface, but got %s",
                         vel_name.c_str());
            return;
        }
        
        // Direct value assignment
        position_interface_vals_(i) = position_interface.get_value();
        velocity_interface_vals_(i) = velocity_interface.get_value();
    }
}

void VisualController::trajectory_callback(const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) {
    auto xd = std::make_shared<KDL::Frame>();
    xd->p = KDL::Vector(msg->data.at(0), msg->data.at(1), msg->data.at(2));
    xd->M = KDL::Rotation::RPY(msg->data.at(3), msg->data.at(4), msg->data.at(5));

    rt_trajectory_buffer_.writeFromNonRT(xd);
}

// Marker pose in camera optical frame
void VisualController::marker_pose_callback(const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) {
    if (msg->data.size() == 0) {
        pose_detected_.store(false);
    }
    else {
        double xx, xy, xz, yx, yy, yz, zx, zy, zz, tx, ty, tz;
        xx = msg->data[0], xy = msg->data[1];  xz = msg->data[2];
        yx = msg->data[3], yy = msg->data[4];  yz = msg->data[5];
        zx = msg->data[6], zy = msg->data[7];  zz = msg->data[8];
        tx = msg->data[9], ty = msg->data[10]; tz = msg->data[11];

        auto T_cam_marker = std::make_shared<KDL::Frame>(
            KDL::Frame(KDL::Rotation(xx, xy, xz, yx, yy, yz, zx, zy, zz), KDL::Vector(tx, ty, tz)));
        rt_marker_pose_buffer_.writeFromNonRT(T_cam_marker);
        pose_detected_.store(true);
        marker_found_.store(true);
    }
}

void VisualController::publish_marker_tf() {
    geometry_msgs::msg::TransformStamped t;

    KDL::Frame f = xd_kdl_;

    t.header.stamp = get_node()->get_clock()->now();
    t.header.frame_id = "fr3_link0";
    t.child_frame_id = "aruco_marker";

    t.transform.translation.x = f.p.x();
    t.transform.translation.y = f.p.y();
    t.transform.translation.z = f.p.z();

    double qx, qy, qz, qw;
    f.M.GetQuaternion(qx, qy, qz, qw);
    t.transform.rotation.x = qx;
    t.transform.rotation.y = qy;
    t.transform.rotation.z = qz;
    t.transform.rotation.w = qw;

    tf_broadcaster_->sendTransform(t);
}

// Debug print functions to reduce clutter in controller code
void VisualController::kdl_chain_error_print(const std::string& tip_link) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to get KDL chain from tree: ");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  " << root_link_ << " --> " << tip_link);
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << tree_.getNrOfJoints() << " joints");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << tree_.getNrOfSegments() << " segments");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  The segments are:");

    KDL::SegmentMap segment_map = tree_.getSegments();
    KDL::SegmentMap::iterator it;

    for (it = segment_map.begin(); it != segment_map.end(); it++) {
        RCLCPP_ERROR(get_node()->get_logger(), "    %s", std::string((*it).first).c_str());
    }
}

void VisualController::kdl_chain_success_print(const std::string& tip_link, const KDL::Chain& chain) {
    RCLCPP_INFO(get_node()->get_logger(), "Got kdl chain");

    // debug: print kdl tree and kdl chain
    RCLCPP_INFO(get_node()->get_logger(), "  %s --> %s", root_link_.c_str(), tip_link.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Tree has %d joints", tree_.getNrOfJoints());
    RCLCPP_INFO(get_node()->get_logger(), "  Tree has %d segments", tree_.getNrOfSegments());
    RCLCPP_INFO(get_node()->get_logger(), "  The kdl_tree_ segments are:");

    // Print the segments of the KDL tree
    KDL::SegmentMap segment_map = tree_.getSegments();
    KDL::SegmentMap::iterator it;
    for (it = segment_map.begin(); it != segment_map.end(); it++)
    {
    RCLCPP_INFO(get_node()->get_logger(), "    %s", std::string((*it).first).c_str());
    }
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d joints", chain.getNrOfJoints());
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d segments", chain.getNrOfSegments());
    RCLCPP_INFO(get_node()->get_logger(), "  The kdl_chain_ segments are:");
    for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
        const KDL::Segment& segment = chain.getSegment(i);
        RCLCPP_INFO(get_node()->get_logger(), "    %s", segment.getName().c_str());
    }
}

}  // namespace ex5
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ex5::VisualController,
                       controller_interface::ControllerInterface)