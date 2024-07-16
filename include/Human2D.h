#ifndef HUMAN2D_H
#define HUMAN2D_H

#include <vector>
#include <unordered_map>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <string>

namespace Optitrack {
    class Human2D {
    public:
        Human2D(const std::vector<std::string>& link_names);
        std::vector<Eigen::Affine3d> relativePose(const std::vector<Eigen::Affine3d>& current_poses);
        void setRotationReference(const Eigen::Matrix3d& R);
        void calibratePose(const std::vector<Eigen::Affine3d>& current_pose, const int n_samples, const bool reset);
        Eigen::Affine3d getInitialPose();

    private:
        std::unordered_map<std::string, Eigen::Affine3d> m_initial_poses;
        std::unordered_map<std::string, Eigen::Matrix3d> m_rotations;
    };

    Eigen::Matrix3d reOrthogonalizeRotationMatrix(const Eigen::Matrix3d& rotationMatrix);
}

#endif // HUMAN2D_H
