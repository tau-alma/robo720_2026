#include "franka_kdl/solver.hpp"
#include "franka_kdl/robot_constants.hpp"

#include <cmath>
#include <cstddef>
#include <iostream>

#include <kdl/frames.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>


Solver::Solver(const KDL::Chain& chain)
: chain_(chain), g_(0.0, 0.0, -9.8) { // Default gravity vector in Gazebo
    ik_solver_.reset(new KDL::ChainIkSolverPos_LMA(chain));
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain));
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(chain));
    dyn_param_solver_.reset(new KDL::ChainDynParam(chain_, g_));
    jac_solver_.reset(new KDL::ChainJntToJacSolver(chain));
    jac_dot_solver_.reset(new KDL::ChainJntToJacDotSolver(chain));
}

int Solver::computeIK(const KDL::JntArray& q_init, const KDL::Frame& target_pose, KDL::JntArray& result) {
    int ret = ik_solver_->CartToJnt(q_init, target_pose, result);

    return ret;
}

int Solver::computeFK(const KDL::JntArray& q, KDL::Vector& pos, const int& seg_nr) {
    KDL::Frame result;
    // store the end-effector position in result
    int ret = fk_solver_->JntToCart(q, result, seg_nr);
    pos = result.p;

    return ret;
}

int Solver::computeFK(const KDL::JntArray& q, KDL::Frame& pose, const int& seg_nr) {
    // store the end-effector pose
    int ret = fk_solver_->JntToCart(q, pose, seg_nr);

    return ret;
}

int Solver::computeIKvel(const KDL::JntArray& q_init, const KDL::Twist& vel, KDL::JntArray& q_dot) {
    int ret = ik_vel_solver_->CartToJnt(q_init, vel, q_dot);

    return ret;
}

int Solver::compute_mass_matrix(const KDL::JntArray& q, KDL::JntSpaceInertiaMatrix& M) {
    int ret = dyn_param_solver_->JntToMass(q, M);
    return ret;
}

int Solver::compute_gravity_vector(const KDL::JntArray& q, KDL::JntArray& G) {
    int ret = dyn_param_solver_->JntToGravity(q, G);
    return ret;
}

int Solver::compute_coriolis_vector(const KDL::JntArray& q, const KDL::JntArray& q_dot, KDL::JntArray& C) {
    int ret = dyn_param_solver_->JntToCoriolis(q, q_dot, C);
    return ret;
}

int Solver::compute_dyn_params(const KDL::JntArray& q, const KDL::JntArray& q_dot,
                               KDL::JntSpaceInertiaMatrix& M, KDL::JntArray& G, KDL::JntArray& C) {
    int ret = compute_mass_matrix(q, M);
    if (ret < 0) {
        std::cout << "Mass matrix computation failed with error code: " << ret << std::endl;
        return ret;
    }
    ret = compute_gravity_vector(q, G);
    if (ret < 0) {
        std::cout << "Gravity vector computation failed with error code: " << ret << std::endl;
        return ret;
    }
    ret = compute_coriolis_vector(q, q_dot, C);
    if (ret < 0) {
        std::cout << "Coriolis vector computation failed with error code: " << ret << std::endl;
    }
    return ret;
}

int Solver::compute_jac(const KDL::JntArray& q, KDL::Jacobian& jac, const int& seg_nr) {
    int ret = jac_solver_->JntToJac(q, jac, seg_nr);

    return ret;
}

void Solver::get_damped_pseudo_inverse(const Eigen::Matrix<double, 6, NUM_JOINTS>& jac,
                                       Eigen::Matrix<double, NUM_JOINTS, 6>& jac_pinv) {
    const Eigen::MatrixXd I6 = Eigen::MatrixXd::Identity(6, 6);

    const Eigen::MatrixXd jacT = jac.transpose(); // -1x6
    const Eigen::MatrixXd jacjacT = jac * jacT; // 6x6
    const Eigen::MatrixXd jacjacT_damped = jacjacT + 1e-6 * I6;

    const Eigen::MatrixXd jacjacT_damped_inv = jacjacT_damped.ldlt().solve(I6);

    jac_pinv = jacT * jacjacT_damped_inv;
}

void Solver::kdl_debug_print(const KDL::JntArray& jnt_array) {
    std::cout << "KDL JntArray" << std::endl << std::endl;

    for (unsigned int i = 0; i < jnt_array.rows(); i++) {
        std::cout << i << ": " << jnt_array(i) << std::endl;
    }
}