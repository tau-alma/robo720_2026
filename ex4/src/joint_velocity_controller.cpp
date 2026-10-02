#include "ex4/joint_velocity_controller.hpp"
#include "franka_kdl/robot_constants.hpp"
#include "franka_kdl/solver.hpp"

#include <cstddef>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/chainable_controller_interface.hpp>
#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>


namespace ex4 {


controller_interface::return_type JointVelocityController::update_and_write_commands(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& period) {
    // Update joint states from hardware state interface
    updateJointStates();

    elapsed_time_ = elapsed_time_ + period.seconds();

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        // Save current state to KDL variables for solvers
        q_kdl_(i) = position_interface_vals_(i);
        q_dot_kdl_(i) = velocity_interface_vals_(i);

        // Read values from reference interface to the Eigen variable
        qd_dot_(i) = reference_interfaces_.at(i);
    }

    // Get dynamics parameters
    solver_->compute_dyn_params(q_kdl_, q_dot_kdl_, M_kdl_, G_kdl_, C_kdl_);

    // Copy data to Eigen variables for matrix computations
    M_ = M_kdl_.data;
    G_ = G_kdl_.data;
    C_ = C_kdl_.data;
    q_dot_ = q_dot_kdl_.data;

    // Compute torque command
    e_dot_ = qd_dot_ - q_dot_;
    u_ = Kd_.cwiseProduct(e_dot_);
    tau_ = M_ * u_ + G_ + C_;

    // Send torque commands to the hardware command interface
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        command_interfaces_[i].set_value(tau_(i));
    }

    return controller_interface::return_type::OK;
}

CallbackReturn JointVelocityController::on_init() {
    // Get parameters from franka_controllers.yaml
    try {
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ = auto_declare<std::vector<std::string>>(
            "command_interfaces", command_interface_types_);
        state_interface_types_ = auto_declare<std::vector<std::string>>(
            "state_interfaces", state_interface_types_);
        reference_interface_types_ = auto_declare<std::vector<std::string>>(
            "reference_interfaces", reference_interface_types_);

        root_link_ = auto_declare<std::string>("root_link", root_link_);
        tip_link_ = auto_declare<std::string>("tip_link", tip_link_);

        d_gains_ = auto_declare<std::vector<double>>("d_gains", d_gains_);
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            Kd_(i) = d_gains_.at(i);
        }
    } 
    catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage (on_init) with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Robot initialization done.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityController::on_configure(
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
        kdl_chain_error_print();

        return CallbackReturn::ERROR;
    }
    else {
        kdl_chain_success_print();
    }

    // Create a Solver instance
    solver_.reset(new Solver(chain_));

    // Create a subscription for the joint command
    joint_cmd_subscriber_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
        "trajectory_point", rclcpp::SystemDefaultsQoS(), std::bind(&JointVelocityController::joint_cmd_callback,
        this, std::placeholders::_1));

    // Modified from ros2_control_demos: example 12
    reference_interfaces_.resize(reference_interface_types_.size() * NUM_JOINTS,
        std::numeric_limits<double>::quiet_NaN());

    RCLCPP_INFO(get_node()->get_logger(), "Robot configuration done.");
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointVelocityController::command_interface_configuration() const {
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
JointVelocityController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & jname : joint_names_) {
        for (const auto & interface_type : state_interface_types_) {
            config.names.push_back(jname + "/" + interface_type);
        }
    }
    return config;
}

CallbackReturn JointVelocityController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
    updateJointStates();
    elapsed_time_ = 0.0;

    M_kdl_.resize(NUM_JOINTS);
    G_kdl_.resize(NUM_JOINTS);
    C_kdl_.resize(NUM_JOINTS);
    q_kdl_.resize(NUM_JOINTS);
    q_dot_kdl_.resize(NUM_JOINTS);

    // Initialize realtime buffer
    auto msg = std::make_shared<trajectory_msgs::msg::JointTrajectoryPoint>();
    msg->velocities.resize(NUM_JOINTS);
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        msg->velocities.at(i) = 0.0;
    }
    rt_buffer_.writeFromNonRT(msg);

    // From ros2_control_demos: example 12
    std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), std::numeric_limits<double>::quiet_NaN());

    return CallbackReturn::SUCCESS;
}

bool JointVelocityController::on_set_chained_mode(bool /*chained_mode*/) { return true; }

std::vector<hardware_interface::CommandInterface>
JointVelocityController::on_export_reference_interfaces() {
    std::vector<hardware_interface::CommandInterface> reference_interfaces;

    // Creates new claimable command interfaces namespaced with the controller name
    // reference interface = command interface to command this controller
    int i = 0;
    for (const auto & jname : joint_names_) {
        for (const auto & interface_type : reference_interface_types_) {
            std::string name = (jname + "/" + interface_type);
            reference_interfaces.push_back(hardware_interface::CommandInterface(
                get_node()->get_name(), name, &reference_interfaces_.at(i)));
            i++;
        }
    }

    return reference_interfaces;
}

controller_interface::return_type JointVelocityController::update_reference_from_subscribers() {
    // When no preceding controller has claimed the reference interface,
    // get joint commands from subscriber
    auto joint_cmd = *rt_buffer_.readFromRT();
    
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        reference_interfaces_.at(i) = joint_cmd->velocities.at(i);
    }

    return controller_interface::return_type::OK;
}

void JointVelocityController::updateJointStates() {
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
void JointVelocityController::joint_cmd_callback(const std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint> msg) {
    rt_buffer_.writeFromNonRT(msg);
}

// Debug print functions to reduce clutter in controller code
void JointVelocityController::kdl_chain_error_print() {
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

void JointVelocityController::kdl_chain_success_print() {
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

}  // namespace ex4
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ex4::JointVelocityController,
                       controller_interface::ChainableControllerInterface)