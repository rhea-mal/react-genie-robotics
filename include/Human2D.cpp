#include "Human2D.h"
#include <iostream>

using namespace std;
using namespace Eigen;

namespace Optitrack {

Matrix3d reOrthogonalizeRotationMatrix(const Matrix3d& rotationMatrix) {
    JacobiSVD<Matrix3d> svd(rotationMatrix, ComputeFullU | ComputeFullV);
    Matrix3d orthoRotationMatrix = svd.matrixU() * svd.matrixV().transpose();
    return orthoRotationMatrix;
}

Human2D::Human2D(const std::vector<std::string>& link_names) {
    for (auto name : link_names) {
        if (name == "left_hand" || name == "right_hand") {
            m_initial_poses[name] = Affine3d::Identity();
        }
    }
    m_filter = std::make_unique<Sai2Common::ButterworthFilter>(0.1);
}

void Human2D::calibratePose(const std::vector<std::string>& link_names,
                          const std::vector<Affine3d>& current_poses,
                          const int n_samples,
                          const bool reset) {
    std::vector<std::string> hands_only{"left_hand", "right_hand"};
    if (reset) {
        for (auto name : hands_only) {
            m_initial_poses[name] = Affine3d::Identity();
        }
    } 

    for (int i = 0; i < hands_only.size(); ++i) {
        std::string name = hands_only[i];
        auto it = std::find(link_names.begin(), link_names.end(), name);
        if (it != link_names.end()) {
            int index = std::distance(link_names.begin(), it);
            m_initial_poses[name].translation() += (1. / n_samples) * current_poses[index].translation();
            m_initial_poses[name].linear() += (1. / n_samples) * current_poses[index].linear();
            m_initial_poses[name].linear() = reOrthogonalizeRotationMatrix(m_initial_poses[name].linear());
        }
    }
}

std::vector<Affine3d> Human2D::relativePose(const std::vector<std::string>& link_names,
                                          const std::vector<Affine3d>& current_poses) {
    std::vector<Affine3d> relative_pose;
    std::vector<std::string> hands_only{"left_hand", "right_hand"};

    for (int i = 0; i < hands_only.size(); ++i) {
        std::string name = hands_only[i];
        auto it = std::find(link_names.begin(), link_names.end(), name);
        if (it != link_names.end()) {
            int index = std::distance(link_names.begin(), it);
            Affine3d transform_diff = Affine3d::Identity();
            transform_diff.linear() = current_poses[index].linear() * m_initial_poses[name].linear().transpose();
            transform_diff.linear() = m_rotations[name] * transform_diff.linear() * m_rotations[name].transpose();
            transform_diff.translation() = m_rotations[name] * (current_poses[index].translation() - m_initial_poses[name].translation());
            relative_pose.push_back(transform_diff);
        }
    }
    return relative_pose;
}

Vector3d Human2D::computeCentroid(const Affine3d& left_hand_pose, const Affine3d& right_hand_pose) {
    return 0.5 * (left_hand_pose.translation() + right_hand_pose.translation());
}

Matrix3d Human2D::computeOrientationDifference(const Affine3d& left_hand_pose, const Affine3d& right_hand_pose) {
    return left_hand_pose.linear() * right_hand_pose.linear().transpose();
}

double Human2D::computeOrientationRelativeToUser(const Affine3d& left_hand_pose, const Affine3d& right_hand_pose, Vector3d forward_vector) {
    Vector3d direction = left_hand_pose.translation() - right_hand_pose.translation();
    direction.normalize();
    forward_vector.normalize();
    return acos(direction.dot(forward_vector));
}

void Human2D::setRotationReference(const std::string& name, const Matrix3d& R) {
    if (name == "left_hand" || name == "right_hand") {
        m_rotations[name] = R;
    }
}

void Human2D::setMultiRotationReference(const std::vector<std::string>& link_names,
                                      const std::vector<Matrix3d>& R) {
    std::vector<std::string> hands_only{"left_hand", "right_hand"};

    for (int i = 0; i < hands_only.size(); ++i) {
        auto it = std::find(link_names.begin(), link_names.end(), hands_only[i]);
        if (it != link_names.end()) {
            int index = std::distance(link_names.begin(), it);
            setRotationReference(hands_only[i], R[index]);
        }
    }
}

Affine3d Human2D::getInitialPose(const std::string& name) {
    if (name == "left_hand" || name == "right_hand") {
        return m_initial_poses[name];
    }
    return Affine3d::Identity();
}

std::vector<Affine3d> Human2D::getMultiInitialPose(const std::vector<std::string>& link_names) {
    std::vector<Affine3d> poses;
    std::vector<std::string> hands_only{"left_hand", "right_hand"};

    for (auto name : hands_only) {
        auto it = std::find(link_names.begin(), link_names.end(), name);
        if (it != link_names.end()) {
            poses.push_back(m_initial_poses[name]);
        }
    }
    return poses;
}

void Human2D::setInitialPose(const std::string& name, const Affine3d& pose) {
    if (name == "left_hand" || name == "right_hand") {
        m_initial_poses[name] = pose;
    }
}

void Human2D::setMultiInitialPose(const std::vector<std::string>& link_names,
                                const std::vector<Affine3d>& poses) {
    std::vector<std::string> hands_only{"left_hand", "right_hand"};

    for (int i = 0; i < hands_only.size(); ++i) {
        auto it = std::find(link_names.begin(), link_names.end(), hands_only[i]);
        if (it != link_names.end()) {
            int index = std::distance(link_names.begin(), it);
            setInitialPose(hands_only[i], poses[index]);
        }
    }
}

}  // namespace Optitrack
