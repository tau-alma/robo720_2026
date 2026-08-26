/*
Main file for the exercise. You need to make your modifications to file kinematics.cpp.
*/

#include "kinematics.hpp"

#include "matplotlibcpp/matplotlibcpp.h"
#include <eigen3/Eigen/Eigenvalues>

namespace plt = matplotlibcpp;

const float L1 = 1.0f;
const float L2 = 1.0f;
const float L3 = 0.1f;

void visualize_manipulator(float q1, float q2, float q3) {
    // Link positions
    float x1 = cos(q1) * L1; float y1 = sin(q1) * L1;
    float x2 = x1 + cos(q1 + q2) * L2; float y2 = y1 + sin(q1 + q2) * L2;
    float x3 = x2 + cos(q1 + q2 + q3) * L3; float y3 = y2 + sin(q1 + q2 + q3) * L3;

    plt::plot({0.0, x1, x2, x3}, {0.0, y1, y2, y3}, "bo-");
    plt::plot({0.0}, {0.0}, "rs");
}

void visualize_ee(float x_ee, float y_ee, float r11, float r12, float r21, float r22, bool no_rot = false) {
    if (!no_rot) {
        float s = 0.1;
        plt::plot({x_ee, x_ee + s*r11}, {y_ee, y_ee + s*r21}, "r-"); // x-axis
        plt::plot({x_ee, x_ee + s*r12}, {y_ee, y_ee + s*r22}, "g-"); // y-axis
    }
    // EE pos (no rotation)
    else {
        plt::plot({x_ee}, {y_ee}, "go");
    }
}

void visualize_jacobian(float x_ee, float y_ee, KDL::Jacobian jac){
    // Manipulability matrix eigenvectors and eigenvalues
    Eigen::MatrixXd jac_eigen = jac.data;
    Eigen::Matrix2d manipulability_mat = (jac_eigen.topRows(2) * jac_eigen.topRows(2).transpose());
    Eigen::EigenSolver<Eigen::Matrix2d> eig_solver(manipulability_mat);
    Eigen::Vector2d eigenvalues = eig_solver.eigenvalues().real();
    Eigen::Matrix2d eigenvectors = eig_solver.eigenvectors().real();

    int n_points = 100;
    double scale = 0.2;
    double a = std::sqrt(eigenvalues(0));
    double b = std::sqrt(eigenvalues(1));
    std::vector<double> ellipse_x, ellipse_y;

    for (int i = 0; i <= n_points; ++i) {
        double theta = 2 * M_PI * i / n_points;
        Eigen::Vector2d local(a * std::cos(theta), b * std::sin(theta));  // scale unit circle with eigenvalues
        Eigen::Vector2d world = eigenvectors * local; // rotate into eigenvector frame
        ellipse_x.push_back(x_ee + scale * world(0));
        ellipse_y.push_back(y_ee + scale * world(1));
    }
    plt::plot(ellipse_x, ellipse_y, "m-");
}

int main() {
    float q1 = M_PI / 6;
    float q2 = M_PI_4;
    float q3 = -M_PI / 8;

    // Kinematics object
    Kinematics kin(L1, L2, L3);

    // 1. End effector pose through trigonometry
    Eigen::Vector2f ee_pos = kin.compute_ee_pos(q1, q2, q3);

    plt::figure();
    visualize_manipulator(q1, q2, q3);
    visualize_ee(ee_pos(0), ee_pos(1), 0, 0, 0, 0, true);
    plt::title("End effector position through trigonometry");
    plt::axis("equal");
    plt::grid(true);
    plt::show();

    // 2. End effector pose through transformation matrices
    Eigen::Matrix3f ee_pose_eigen = kin.compute_fk_eigen(q1, q2, q3);

    plt::figure();
    visualize_manipulator(q1, q2, q3);
    visualize_ee(ee_pose_eigen(0,2), ee_pose_eigen(1,2), ee_pose_eigen(0,0), ee_pose_eigen(0,1), ee_pose_eigen(1,0), ee_pose_eigen(1,1));
    plt::title("End effector pose through transformation matrices using Eigen");
    plt::axis("equal");
    plt::grid(true);
    plt::show();

    // 3. Construct KDL Chain object
    kin.construct_kdl_chain();

    // 4. End effector pose through KDL FK solver
    KDL::Frame ee_pose_kdl = kin.compute_fk_kdl(q1, q2, q3);

    plt::figure();
    visualize_manipulator(q1, q2, q3);
    visualize_ee(ee_pose_kdl(0,3), ee_pose_kdl(1,3),ee_pose_kdl(0,0), ee_pose_kdl(0,1), ee_pose_kdl(1,0), ee_pose_kdl(1,1));
    plt::title("End effector pose through forward kinematics solver using KDL");
    plt::axis("equal");
    plt::grid(true);
    plt::show();

    // 5. Joint positions through KDL IK solver
    KDL::Frame tgt_pose = ee_pose_kdl;
    KDL::JntArray tgt_joint = kin.compute_ik_kdl(0.0, 0.0, 0.0, tgt_pose);
    
    plt::figure();
    visualize_manipulator(tgt_joint(0), tgt_joint(1), tgt_joint(2));
    visualize_ee(tgt_pose(0,3), tgt_pose(1,3), tgt_pose(0,0), tgt_pose(0,1), tgt_pose(1,0), tgt_pose(1,1));
    plt::title("Joint positions through inverse kinematics solver using KDL");
    plt::axis("equal");
    plt::grid(true);
    plt::show();

    // 6. Jacobian through KDL Jacobian solver
    KDL::Jacobian jac = kin.compute_jac_kdl(q1, q2, q3, -1);

    plt::figure();
    visualize_manipulator(q1, q2, q3);
    visualize_jacobian(ee_pos(0), ee_pos(1), jac);
    plt::title("Manipulability ellipse using Jacobian");
    plt::axis("equal");
    plt::grid(true);

    // Other configuration
    q1 = M_PI_4; q2 = 3 * M_PI_4; q3 = -M_PI / 8;
    ee_pos = kin.compute_ee_pos(q1, q2, q3);
    jac = kin.compute_jac_kdl(q1, q2, q3, -1);

    plt::figure();
    visualize_manipulator(q1, q2, q3);
    visualize_jacobian(ee_pos(0), ee_pos(1), jac);
    plt::title("Manipulability ellipse using Jacobian");
    plt::axis("equal");
    plt::grid(true);

    // Singularity configuration
    q1 = M_PI_4; q2 = 0.0; q3 = 0.0;
    ee_pos = kin.compute_ee_pos(q1, q2, q3);
    jac = kin.compute_jac_kdl(q1, q2, q3, -1);

    plt::figure();
    visualize_manipulator(q1, q2, q3);
    visualize_jacobian(ee_pos(0), ee_pos(1), jac);
    plt::title("Manipulability ellipse in singularity using Jacobian");
    plt::axis("equal");
    plt::grid(true);

    plt::show();

    // 7. Jacobian in all of the manipulator's frames
    q1 = M_PI_4; q2 = 3 * M_PI_4; q3 = -M_PI / 8;
    ee_pos = kin.compute_ee_pos(q1, q2, q3);
    std::vector<KDL::Jacobian> jacs;
    for (int i = 0; i < 4; ++i) {
        jac = kin.compute_jac_kdl(q1, q2, q3, i);
        jacs.push_back(jac);
    }

    // Frames attached to joints in task space
    float x0 = 0.0; float y0 = 0.0;
    float x1 = cos(q1) * L1; float y1 = sin(q1) * L1;
    float x2 = x1 + cos(q1 + q2) * L2; float y2 = y1 + sin(q1 + q2) * L2;
    float x3 = x2 + cos(q1 + q2 + q3) * L3; float y3 = y2 + sin(q1 + q2 + q3) * L3;
    std::vector<float> x_coords = {x0, x1, x2, x3};
    std::vector<float> y_coords = {y0, y1, y2, y3};

    plt::figure();
    visualize_manipulator(q1, q2, q3);
    for (int i = 0; i < 4; ++i) {
        visualize_jacobian(x_coords.at(i), y_coords.at(i), jacs.at(i));
    }
    plt::title("Manipulability ellipse for each joint using Jacobian");
    plt::axis("equal");
    plt::grid(true);

    plt::show();

    return 0;
}