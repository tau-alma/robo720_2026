#include <iostream>
#include <cmath>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>

/**
 * Class which includes methods for:
 * - calculation of end effector position with Eigen using trigonometry
 * - calculation of forward kinematics with Eigen using transformation matrices
 * - construction of KDL chain object
 * - calculation of forward kinematics with KDL
 * - calculation of inverse kinematics with KDL
 * - calculation of Jacobian with KDL
 */
class Kinematics {
    public:
    Kinematics(const float& L1, const float& L2, const float& L3);

    /**
     * @brief Compute the end effector position at given joint positions.
     * 
     * @param[in] q1 Position of the first joint (rad).
     * @param[in] q2 Position of the second joint (rad or m).
     * @param[in] q3 Position of the third joint (rad).
     * 
     * @return End effector position as a 2D vector.
     */
    Eigen::Vector2f compute_ee_pos(const float& q1, const float& q2, const float& q3);

    /**
     * @brief Compute the forward kinematics at given joint positions.
     * 
     * @param[in] q1 Position of the first joint (rad).
     * @param[in] q2 Position of the second joint (rad or m).
     * @param[in] q3 Position of the third joint (rad).
     * 
     * @return 3x3 transformation matrix from the base joint to the end effector.
     */
    Eigen::Matrix3f compute_fk_eigen(const float& q1, const float& q2, const float& q3);

    /**
     * @brief Construct a KDL Chain object which is used in later methods.
     */
    void construct_kdl_chain();

    /**
     * @brief Compute the forward kinematics at given joint positions.
     * 
     * You need to create at least the following:
     * 
     * - KDL JntArray object
     * 
     * - some KDL FkSolver object
     * 
     * @param[in] q1 Position of the first joint (rad).
     * @param[in] q2 Position of the second joint (rad or m).
     * @param[in] q3 Position of the third joint (rad).
     * 
     * @return 4x4 Frame object for the transformation matrix from the base joint to the end effector.
     */
    KDL::Frame compute_fk_kdl(const float& q1, const float& q2, const float& q3);

    /**
     * @brief Compute the inverse kinematics for given end effector position.
     * 
     * You need to create at least the following:
     * 
     * - KDL JntArray object
     * 
     * - some KDL IkSolver object
     * 
     * @param[in] q1_init Initial position of the first joint (rad).
     * @param[in] q2_init Initial position of the second joint (rad or m).
     * @param[in] q3_init Initial position of the third joint (rad).
     * @param[in] target_pose Target end effector pose.
     * 
     * @return 2D JntArray object for joint positions in which the end effector is in target pose.
     */
    KDL::JntArray compute_ik_kdl(const float& q1_init, const float& q2_init, const float& q3_init, const KDL::Frame& target_pose);

    /**
     * @brief Compute the Jacobian matrix for given joint positions.
     * 
     * You need to create at least the following:
     * 
     * - KDL JntArray object
     * 
     * - some KDL JacSolver object
     * 
     * @param[in] q1 Position of the first joint (rad).
     * @param[in] q2 Position of the second joint (rad or m)
     * @param[in] q3 Position of the third joint (rad).
     * @param[in] segment_n Segment of the manipulator to compute the Jacobian for
     * 
     * @return Manipulator Jacobian.
     */
    KDL::Jacobian compute_jac_kdl(const float& q1, const float& q2, const float& q3, const int& segment_n);

    private:
    float L1_;
    float L2_;
    float L3_;

    KDL::Chain chain_;
};