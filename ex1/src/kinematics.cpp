#include "kinematics.hpp"

Kinematics::Kinematics(const float &L1, const float &L2, const float &L3)
    : L1_(L1), L2_(L2), L3_(L3)
{
}

Eigen::Vector2f Kinematics::compute_ee_pos(const float &q1, const float &q2, const float& q3)
{
    float x1 = cos(q1) * L1_; float y1 = sin(q1) * L1_;
    float x2 = x1 + cos(q1 + q2) * L2_; float y2 = y1 + sin(q1 + q2) * L2_;
    float x3 = x2 + cos(q1 + q2 + q3) * L3_; float y3 = y2 + sin(q1 + q2 + q3) * L3_;
    return Eigen::Vector2f(x3, y3);
}

Eigen::Matrix3f Kinematics::compute_fk_eigen(const float &q1, const float &q2, const float& q3)
{
    // Construct transformation matrices from each joint to next one
    Eigen::Matrix3f T_base_1, T_1_2, T_2_3, T_3_ee;
    T_base_1(0, 0) = cos(q1); T_base_1(0, 1) = -sin(q1); T_base_1(0, 2) = 0.0;
    T_base_1(1, 0) = sin(q1); T_base_1(1, 1) = cos(q1);  T_base_1(1, 2) = 0.0;
    T_base_1(2, 0) = 0.0;     T_base_1(2, 1) = 0.0;      T_base_1(2, 2) = 1.0;

    T_1_2(0, 0) = cos(q2); T_1_2(0, 1) = -sin(q2); T_1_2(0, 2) = L1_;
    T_1_2(1, 0) = sin(q2); T_1_2(1, 1) = cos(q2);  T_1_2(1, 2) = 0.0;
    T_1_2(2, 0) = 0.0;     T_1_2(2, 1) = 0.0;      T_1_2(2, 2) = 1.0;

    T_2_3(0, 0) = cos(q3); T_2_3(0, 1) = -sin(q3); T_2_3(0, 2) = L2_;
    T_2_3(1, 0) = sin(q3); T_2_3(1, 1) = cos(q3);  T_2_3(1, 2) = 0.0;
    T_2_3(2, 0) = 0.0;     T_2_3(2, 1) = 0.0;      T_2_3(2, 2) = 1.0;

    T_3_ee(0, 0) = 1.0; T_3_ee(0, 1) = 0.0; T_3_ee(0, 2) = L3_;
    T_3_ee(1, 0) = 0.0; T_3_ee(1, 1) = 1.0; T_3_ee(1, 2) = 0.0;
    T_3_ee(2, 0) = 0.0; T_3_ee(2, 1) = 0.0; T_3_ee(2, 2) = 1.0;

    Eigen::Matrix3f T_base_ee = T_base_1 * T_1_2 * T_2_3 * T_3_ee;
    return T_base_ee;
}

void Kinematics::construct_kdl_chain()
{
    // Construct Joint objects
    KDL::Joint joint_1(KDL::Joint::RotZ);
    KDL::Joint joint_2(KDL::Joint::RotZ);
    KDL::Joint joint_3(KDL::Joint::RotZ);
    //KDL::Joint joint_ee(KDL::Joint::None);

    // Construct Frame objects
    KDL::Frame frame_1_2(KDL::Vector(L1_, 0.0, 0.0));
    KDL::Frame frame_2_3(KDL::Vector(L2_, 0.0, 0.0));
    KDL::Frame frame_3_ee(KDL::Vector(L3_, 0.0, 0.0));

    // Construct Segment objects (notice difference compared to Eigen implementation
    // on which joints are grouped with which link lengths!)
    KDL::Segment segment_1(joint_1, frame_1_2);
    KDL::Segment segment_2(joint_2, frame_2_3);
    KDL::Segment segment_3(joint_3, frame_3_ee);
    //KDL::Segment segment_ee(joint_ee, frame_3_ee);

    // Add segments to the Chain object
    chain_.addSegment(segment_1);
    chain_.addSegment(segment_2);
    chain_.addSegment(segment_3);
    //chain_.addSegment(segment_ee);
}

KDL::Frame Kinematics::compute_fk_kdl(const float &q1, const float &q2, const float& q3)
{
    // Initialize JntArray object and populate it
    KDL::JntArray joint_values(chain_.getNrOfJoints());
    joint_values(0) = q1; joint_values(1) = q2; joint_values(2) = q3;

    // Construct FK solver object
    KDL::ChainFkSolverPos_recursive fk_solver(chain_);

    // Initialize Frame object for FK solution
    KDL::Frame ee_pose;

    // Solve FK
    fk_solver.JntToCart(joint_values, ee_pose);

    return ee_pose;
}

KDL::JntArray Kinematics::compute_ik_kdl(const float &q1_init, const float &q2_init, const float& q3_init, const KDL::Frame &target_pose)
{
    // Initialize JntArray object and populate it with initial values
    KDL::JntArray joint_values_init(chain_.getNrOfJoints());
    joint_values_init(0) = q1_init; joint_values_init(1) = q2_init; joint_values_init(2) = q3_init;

    // Initialize JntArray object for the IK solution
    KDL::JntArray joint_values_tgt(chain_.getNrOfJoints());

    // Construct IK solver object
    KDL::ChainIkSolverPos_LMA ik_solver(chain_);

    // Solve IK
    ik_solver.CartToJnt(joint_values_init, target_pose, joint_values_tgt);

    return joint_values_tgt;
}

KDL::Jacobian Kinematics::compute_jac_kdl(const float &q1, const float &q2, const float& q3,
                                          const int& segment_n)
{
    // Initialize JntArray object and populate it
    KDL::JntArray joint_values(chain_.getNrOfJoints());
    joint_values(0) = q1; joint_values(1) = q2; joint_values(2) = q3;

    // Initialize Jacobian object for solution
    KDL::Jacobian jac(chain_.getNrOfJoints());

    // Construct Jacobian solver object
    KDL::ChainJntToJacSolver jac_solver(chain_);

    // Solve Jacobian
    jac_solver.JntToJac(joint_values, jac, segment_n);

    return jac;
}
