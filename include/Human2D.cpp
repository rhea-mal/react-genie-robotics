#include "Human2D.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

namespace Optitrack {

    Human2D::Human2D(const std::vector<std::string>& link_names) {
        for (const auto& name : link_names) {
            m_initial_poses[name] = Eigen::Affine3d::Identity();
            m_rotations[name] = Eigen::Matrix3d::Identity();
        }
    }

    Eigen::Matrix3d reOrthogonalizeRotationMatrix(const Eigen::Matrix3d& rotationMatrix) {
        // Perform Singular Value Decomposition (SVD)
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(rotationMatrix, Eigen::ComputeFullU | Eigen::ComputeFullV);

        // Reconstruct the rotation matrix using the left singular vectors and right singular vectors
        Eigen::Matrix3d orthoRotationMatrix = svd.matrixU() * svd.matrixV().transpose();

        return orthoRotationMatrix;
    }

    void Human2D::calibratePose(const std::vector<Eigen::Affine3d>& current_pose, const int n_samples, const bool reset) {
        std::string name = "right_hand";
        if (reset) {
            m_initial_poses[name] = Eigen::Affine3d::Identity();
        }
        for (size_t i = 0; i < current_pose.size(); ++i) {
            m_initial_poses[name].translation() += (1. / n_samples) * current_pose[i].translation();
            m_initial_poses[name].linear() += (1. / n_samples) * current_pose[i].linear();
        }
        m_initial_poses[name].linear() = reOrthogonalizeRotationMatrix(m_initial_poses[name].linear());
    }

    std::vector<Eigen::Affine3d> Human2D::relativePose(const std::vector<Eigen::Affine3d>& current_poses) {
        std::vector<Eigen::Affine3d> relative_pose;
        std::string name = "right_hand";

        std::cout << "Optitrack zero pose: \n" << m_initial_poses[name].linear() << "\n" << std::endl;

        for (size_t i = 0; i < current_poses.size(); ++i) {
            Eigen::Affine3d transform_diff = Eigen::Affine3d::Identity();
            transform_diff.linear() = current_poses[i].linear() * m_initial_poses[name].linear().transpose();  // rotate from current to goal orientation
            transform_diff.linear() = m_rotations[name] * transform_diff.linear() * m_rotations[name].transpose();
            transform_diff.translation() = m_rotations[name] * (current_poses[i].translation() - m_initial_poses[name].translation());
            relative_pose.push_back(transform_diff);
        }

        return relative_pose;
    }

    void Human2D::setRotationReference(const Eigen::Matrix3d& R) {
        m_rotations["right_hand"] = R;
    }

    Eigen::Affine3d Human2D::getInitialPose() {
        return m_initial_poses["right_hand"];
    }

} // namespace Optitrack
