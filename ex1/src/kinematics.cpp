#include "kinematics.hpp"

Kinematics::Kinematics(const float &L1, const float &L2, const float &L3)
    : L1_(L1), L2_(L2), L3_(L3)
{
}

Eigen::Vector2f Kinematics::compute_ee_pos(const float &q1, const float &q2, const float& q3)
{
    /**
     * TODO: Compute end effector position using trigonometrics
     */

    return Eigen::Vector2f();
}

Eigen::Matrix3f Kinematics::compute_fk_eigen(const float &q1, const float &q2, const float& q3)
{
    // Construct transformation matrices from each joint to next one
    Eigen::Matrix3f T_base_1, T_1_2, T_2_3, T_3_ee;

    /**
     * TODO: Populate the transformation matrices from joint to joint,
     * then chain them to get the transformation from 'base' to 'end effector'.
     */

    Eigen::Matrix3f T_base_ee;

    return T_base_ee;
}

void Kinematics::construct_kdl_chain()
{
    /**
     * TODO: Construct KDL Chain object
     */
}

KDL::Frame Kinematics::compute_fk_kdl(const float &q1, const float &q2, const float& q3)
{
    /**
     * TODO: Construct forward kinematic solver object,
     * and solve end effector pose given the joint values
     */

    return KDL::Frame();
}

KDL::JntArray Kinematics::compute_ik_kdl(const float &q1_init, const float &q2_init, const float& q3_init, const KDL::Frame &target_pose)
{
    /**
     * TODO: Construct inverse kinematics solver object,
     * and solve joint positions given the desired pose
     * and initial joint values.
     */

    return KDL::JntArray();
}

KDL::Jacobian Kinematics::compute_jac_kdl(const float &q1, const float &q2, const float& q3,
                                          const int& segment_n)
{
    /**
     * TODO: Construct Jacobian solver object, and
     * solve the Jacobian given the joint values.
     */

    return KDL::Jacobian();
}
