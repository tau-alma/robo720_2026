#include "ex6/pf_effort_controller.hpp"
#include "franka_kdl/robot_constants.hpp"
#include "franka_kdl/solver.hpp"

#include <algorithm>
#include <cstddef>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <urdf/model.h>
#include <custom_msgs/msg/obstacle_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Geometry>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>


namespace ex6 {


controller_interface::return_type PFEffortController::update(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& period) {
    // Update joint states from hardware state interface
    updateJointStates();

    elapsed_time_ = elapsed_time_ + period.seconds();

    // Save current state to KDL variables for solvers
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        q_kdl_(i) = position_interface_vals_(i);
        q_dot_kdl_(i) = velocity_interface_vals_(i);
    }

    // Current EE pose
    solver_->computeFK(q_kdl_, x_kdl_);

    // Read the goal pose and convert it into KDL Frame
    xd_kdl_ = *rt_goal_pose_buffer_.readFromRT()->get();

    // Error in task space
    xe_kdl_ = KDL::diff(x_kdl_, xd_kdl_);

    // Get dynamics parameters and Jacobian
    solver_->compute_dyn_params(q_kdl_, q_dot_kdl_, M_kdl_, G_kdl_, C_kdl_);
    solver_->compute_jac(q_kdl_, J_kdl_);
    solver_->compute_jac_dot(q_kdl_, q_dot_kdl_, J_dot_kdl_);

    // Copy data to Eigen variables for matrix computations
    M_ = M_kdl_.data;
    G_ = G_kdl_.data;
    C_ = C_kdl_.data;
    J_ = J_kdl_.data;
    J_dot_ = J_dot_kdl_.data;
    q_ = q_kdl_.data;
    q_dot_ = q_dot_kdl_.data;
    
    xe_(0) = xe_kdl_.vel.x(); xe_(1) = xe_kdl_.vel.y(); xe_(2) = xe_kdl_.vel.z();
    xe_(3) = xe_kdl_.rot.x(); xe_(4) = xe_kdl_.rot.y(); xe_(5) = xe_kdl_.rot.z();

    // Jacobian pseudo-inverse
    solver_->get_damped_pseudo_inverse(J_, J_pinv_);

    // End effector velocity
    x_dot_ = J_ * q_dot_;

    // Attractive torque
    u_ = J_pinv_ * (Kp_.cwiseProduct(xe_) - Kd_.cwiseProduct(x_dot_) - J_dot_ * q_dot_);
    tau_att_ = M_ * u_ + C_ + G_;

    // Repulsive velocity
    // Joint limits
    tau_rep_.setZero();
    double F, D_min, D_max, dD, K;
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        D_min = pow(q_(i) - joint_mins_.at(i), 2);
        D_max = pow(q_(i) - joint_maxs_.at(i), 2);

        if (D_min <= Q_joint_) {
            dD = 2 * (q_(i) - joint_mins_.at(i));
            K = k_rep_ * (1/D_min - 1/Q_joint_) * (1/pow(D_min, 2));
        }
        else if (D_max <= Q_joint_) {
            dD = 2 * (q_(i) - joint_maxs_.at(i));
            K = k_rep_ * (1/D_max - 1/Q_joint_) * (1/pow(D_max, 2));
        }
        else {
            dD = 0.0;
            K = 0.0;
        }

        F = K * dD;
        tau_rep_(i) += F;
    }
    // Obstacles in task space
    const auto& obstacles = *rt_obstacles_.readFromRT();

    // Get the segment numbers of actuated joints
    std::vector<unsigned int> joint_to_seg;  // joint_to_seg[j] = segment index for joint j's child link
    for (unsigned int s = 0; s < chain_.getNrOfSegments(); ++s) {
        if (chain_.getSegment(s).getJoint().getType() != KDL::Joint::None) {
            joint_to_seg.push_back(s);
        }
    }
    // Add end effector manually, because it is after KDL::Joint::None
    joint_to_seg.push_back(-1);

    for (const auto& obs : obstacles){
        Vector7d F_vec = Vector7d::Zero();
        Eigen::Vector3d p_obs(obs.position.x, obs.position.y, obs.position.z);

        // Repulsive force for each joint's frame
        for (std::size_t i = 0; i < joint_to_seg.size(); ++i) {
            int seg_nr = joint_to_seg.at(i);
            KDL::Vector joint_cart_pos;
            KDL::Jacobian jac_kdl(NUM_JOINTS);
            solver_->computeFK(q_kdl_, joint_cart_pos, seg_nr);
            solver_->compute_jac(q_kdl_, jac_kdl, seg_nr);
            Eigen::Vector3d p(joint_cart_pos.x(), joint_cart_pos.y(), joint_cart_pos.z());
            Eigen::MatrixXd jac = jac_kdl.data;

            Eigen::Vector3d p_err = p - p_obs;

            // Spherical obstacle
            if (obs.shape_type == custom_msgs::msg::Obstacle::SPHERE) {
                // D is not mathematically accurate squared distance function,
                // but we can imagine r^2 here as an addition to Q*:
                // D <= Q*
                // p_e^T * p_e - r^2 <= Q*
                // p_e^T * p_e <= Q* + r^2,
                // where p_e^T * p_e is squared distance to sphere center
                double D = p_err.transpose() * p_err - (obs.radius * obs.radius);
                if (D <= Q_ee_) {
                    Vector7d dD_vec = 2 * p_err.transpose() * jac.topRows(3);
                    K = k_rep_ * (1/D - 1/Q_ee_) * (1/(D * D));
                    F_vec += K * dD_vec;
                }
            }
            // Box obstacle (such as wall)
            else if (obs.shape_type == custom_msgs::msg::Obstacle::BOX) {
                // Rotate the error vector to box frame (base frame and world frame orientations are identical
                // so we can use the box's orientation in world frame)
                Eigen::Quaterniond box_quat(obs.orientation.w, obs.orientation.x, obs.orientation.y, obs.orientation.z);
                // Normalize the quaternion just in case
                box_quat.normalize();

                // Convert into rotation matrix
                Eigen::Matrix3d box_rot = box_quat.toRotationMatrix();

                // p_a = R_ab * p_b, p_b = in world frame, R_ab = rotation from box frame to world frame
                p_err = box_rot.transpose() * p_err;

                // Box dimensions vector
                Eigen::Vector3d box_r(obs.box_size.at(0)/2, obs.box_size.at(1)/2, obs.box_size.at(2)/2);

                // Clamped point on box surface from box center towards end effector
                Eigen::Vector3d p_clamped = p_err.cwiseMax(-box_r).cwiseMin(box_r);

                // Vector from closest surface point EE position
                Eigen::Vector3d d_local = p_err - p_clamped;

                // Rotate back to base/world frame
                d_local = box_rot * d_local;

                // Squared distance to box surface
                double D = d_local.transpose() * d_local;
                if (D <= Q_ee_) {
                    Vector7d dD_vec = 2 * d_local.transpose() * jac.topRows(3);
                    K = k_rep_ * (1/D - 1/Q_ee_) * (1/(D * D));
                    F_vec += K * dD_vec;
                }
            }
        }

        tau_rep_ += F_vec;
    }

    // Add damping to repulsive force
    tau_rep_ -= B_.cwiseProduct(q_dot_);

    // Compute torque command
    tau_ = tau_att_ + tau_rep_;

    // Send torque commands to the hardware command interface
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        command_interfaces_[i].set_value(tau_(i));
    }

    publish_markers();

    return controller_interface::return_type::OK;
}

CallbackReturn PFEffortController::on_init() {
    // Get parameters from franka_controllers.yaml
    try {
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ = auto_declare<std::vector<std::string>>(
            "command_interfaces", command_interface_types_);
        state_interface_types_ = auto_declare<std::vector<std::string>>(
            "state_interfaces", state_interface_types_);

        root_link_ = auto_declare<std::string>("root_link", root_link_);
        tip_link_ = auto_declare<std::string>("tip_link", tip_link_);

        p_gains_ = auto_declare<std::vector<double>>("p_gains", p_gains_);
        d_gains_ = auto_declare<std::vector<double>>("d_gains", d_gains_);
        b_damping_ = auto_declare<std::vector<double>>("b_damping", b_damping_);
        for (std::size_t i = 0; i < NUM_TASK; ++i) {
            Kp_(i) = p_gains_.at(i);
            Kd_(i) = d_gains_.at(i);
        }
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            B_(i) = b_damping_.at(i);
        }

        k_rep_ = auto_declare<double>("k_repulsive", k_rep_);
        Q_joint_ = pow(auto_declare<double>("Q_joint", Q_joint_), 2);
        Q_ee_ = pow(auto_declare<double>("Q_ee", Q_ee_), 2);
    } 
    catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage (on_init) with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Robot initialization done.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PFEffortController::on_configure(
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

    // Get the joint angle limits from the URDF
    for (std::size_t i = 0; i < NUM_JOINTS; i++) {
        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
        if (!joint_urdf) {
            RCLCPP_ERROR(get_node()->get_logger(), "Could not find joint '%s' in urdf", joint_names_[i].c_str());
            return CallbackReturn::ERROR;
        }
        joint_mins_.push_back(joint_urdf->limits->lower);
        joint_maxs_.push_back(joint_urdf->limits->upper);
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
        kdl_chain_error_print();

        return CallbackReturn::ERROR;
    }
    else {
        kdl_chain_success_print();
    }

    // Create a Solver instance
    solver_.reset(new Solver(chain_));

    // Create a subscription for the joint command
    goal_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", rclcpp::SystemDefaultsQoS(), std::bind(&PFEffortController::goal_pose_callback,
        this, std::placeholders::_1));

    init_obstacle_subscriber_ = get_node()->create_subscription<custom_msgs::msg::ObstacleArray>(
        "obstacles", rclcpp::QoS(1).transient_local(), std::bind(&PFEffortController::init_obstacle_callback,
        this, std::placeholders::_1));

    sub_obj_pose_  = get_node()->create_subscription<tf2_msgs::msg::TFMessage>(
        "world/obstacle_world/pose/info", rclcpp::SystemDefaultsQoS(),
        std::bind(&PFEffortController::object_pose_callback, this, std::placeholders::_1));

    // Publisher
    pub_markers_ = get_node()->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 1000);

    RCLCPP_INFO(get_node()->get_logger(), "Robot configuration done.");
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PFEffortController::command_interface_configuration() const {
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
PFEffortController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & jname : joint_names_) {
        for (const auto & interface_type : state_interface_types_) {
            config.names.push_back(jname + "/" + interface_type);
        }
    }
    return config;
}

CallbackReturn PFEffortController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
    updateJointStates();
    elapsed_time_ = 0.0;

    M_kdl_.resize(NUM_JOINTS);
    G_kdl_.resize(NUM_JOINTS);
    C_kdl_.resize(NUM_JOINTS);
    J_kdl_.resize(NUM_JOINTS);
    J_dot_kdl_.resize(NUM_JOINTS);
    q_kdl_.resize(NUM_JOINTS);
    q_dot_kdl_.resize(NUM_JOINTS);

    // Initialize realtime buffer
    auto xd = std::make_shared<KDL::Frame>();
    xd->p = KDL::Vector(0.5, 0.0, 1.6);
    xd->M = KDL::Rotation::EulerZYX(0.0, 0.0, M_PI);
    rt_goal_pose_buffer_.writeFromNonRT(xd);

    // Activate publisher
    pub_markers_->on_activate();

    return CallbackReturn::SUCCESS;
}

void PFEffortController::updateJointStates() {
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

// Joint command subscriber callback
void PFEffortController::goal_pose_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) {
    auto xd = std::make_shared<KDL::Frame>();

    xd->p = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    xd->M = KDL::Rotation::Quaternion(
        msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

    rt_goal_pose_buffer_.writeFromNonRT(xd);
}

void PFEffortController::init_obstacle_callback(const custom_msgs::msg::ObstacleArray msg) {
    obstacle_map_.clear(); // In case a new scene was loaded, otherwise no new messages will be published
    std::vector<custom_msgs::msg::Obstacle> obs_vec;
    for (auto obs : msg.obstacles) {
        obstacle_map_.emplace(obs.name, obs);
        obs_vec.push_back(obs);
    }

    rt_obstacles_.writeFromNonRT(obs_vec);
}

void PFEffortController::object_pose_callback(const tf2_msgs::msg::TFMessage& msg) {
    for (geometry_msgs::msg::TransformStamped t : msg.transforms) {
        auto it = obstacle_map_.find(t.child_frame_id);
        if (it != obstacle_map_.end()) {
            it->second.position.x = t.transform.translation.x;
            it->second.position.y = t.transform.translation.y;
            it->second.position.z = t.transform.translation.z;
            it->second.orientation = t.transform.rotation;
        }
    }

    // Build a fresh snapshot vector to write to rt buffer
    std::vector<custom_msgs::msg::Obstacle> snapshot;
    snapshot.reserve(obstacle_map_.size());
    for (const auto& [name, obs] : obstacle_map_) {
        snapshot.push_back(obs);
    }

    rt_obstacles_.writeFromNonRT(snapshot);
}

void PFEffortController::publish_markers() {
    visualization_msgs::msg::MarkerArray msg;

    const auto& obstacles = *rt_obstacles_.readFromRT();
    int i = 0;
    for (const auto& obs : obstacles){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base";
        marker.header.stamp = get_node()->get_clock()->now();
        marker.ns = "";
        marker.id = i++;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = obs.position;
        marker.pose.orientation = obs.orientation;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

        if (obs.shape_type == custom_msgs::msg::Obstacle::SPHERE) {
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.scale.x = 2 * obs.radius;
            marker.scale.y = 2 * obs.radius;
            marker.scale.z = 2 * obs.radius;
        }

        else if (obs.shape_type == custom_msgs::msg::Obstacle::BOX) {
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = obs.box_size.at(0);
            marker.scale.y = obs.box_size.at(1);
            marker.scale.z = obs.box_size.at(2);
        }

        // Push back to marker array
        msg.markers.push_back(marker);
    }

    pub_markers_->publish(msg);
}

// Debug print functions to reduce clutter in controller code
void PFEffortController::kdl_chain_error_print() {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to get KDL chain from tree: ");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  " << root_link_ << " --> " << tip_link_);
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << tree_.getNrOfJoints() << " joints");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << tree_.getNrOfSegments() << " segments");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  The segments are:");

    KDL::SegmentMap segment_map = tree_.getSegments();
    KDL::SegmentMap::iterator it;

    for (it = segment_map.begin(); it != segment_map.end(); it++) {
        RCLCPP_ERROR(get_node()->get_logger(), "    %s", std::string((*it).first).c_str());
    }
}

void PFEffortController::kdl_chain_success_print() {
    RCLCPP_INFO(get_node()->get_logger(), "Got kdl chain");

    // debug: print kdl tree and kdl chain
    RCLCPP_INFO(get_node()->get_logger(), "  %s --> %s", root_link_.c_str(), tip_link_.c_str());
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
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d joints", chain_.getNrOfJoints());
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d segments", chain_.getNrOfSegments());
    RCLCPP_INFO(get_node()->get_logger(), "  The kdl_chain_ segments are:");
    for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++) {
        const KDL::Segment& segment = chain_.getSegment(i);
        RCLCPP_INFO(get_node()->get_logger(), "    %s", segment.getName().c_str());
    }
}

}  // namespace ex6
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ex6::PFEffortController,
                       controller_interface::ControllerInterface)