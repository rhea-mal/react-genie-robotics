/**
 * @file Human2D.h
 * @brief Class to handle human motion inputs 
 * @version 0.1
 * @date 2024-04-18
 * 
 */

#pragma once

#include <map>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "Sai2Model.h"
#include "filters/ButterworthFilter.h"

using namespace Eigen;

namespace Optitrack {

class Human2D {
    public:
        Human2D(const std::vector<std::string>& link_names);

        // zero-pose calibration
        void calibratePose(const std::vector<std::string>& link_names,
                           const std::vector<Affine3d>& current_poses,
                           const int n_samples,
                           const bool reset = 0);

        // get relative pose from the calibrated starting positions 
        std::vector<Affine3d> relativePose(const std::vector<std::string>& link_names,
                                           const std::vector<Affine3d>& current_poses);

        // compute the position of the end effector as the centroid of the left and right hands
        Vector3d computeCentroid(const Affine3d& left_hand_pose, const Affine3d& right_hand_pose);

        // compute the orientation as the difference between the left and right hands
        Matrix3d computeOrientationDifference(const Affine3d& left_hand_pose, const Affine3d& right_hand_pose);

        // compute the orientation with respect to the forward vector (user facing direction)
        double computeOrientationRelativeToUser(const Affine3d& left_hand_pose, const Affine3d& right_hand_pose, Vector3d forward_vector);

        /*
            Getters and setters 
        */

        /*
            Set rotation matrix to rotate from optitrack marker frame to robot link frame 
        */
        void setRotationReference(const std::string& name, const Matrix3d& R);

        void setMultiRotationReference(const std::vector<std::string>& link_names,
                                       const std::vector<Matrix3d>& R);
        
        /*
            Get and set poses
        */
        Affine3d getInitialPose(const std::string& name);

        std::vector<Affine3d> getMultiInitialPose(const std::vector<std::string>& link_names);

        void setInitialPose(const std::string& name, const Affine3d& pose);

        void setMultiInitialPose(const std::vector<std::string>& link_names,
                                 const std::vector<Affine3d>& poses);

    private:
        // starting pose for human limbs 
        std::map<std::string, Affine3d> m_initial_poses;

        // rotation matrices to rotate from optitrack frame to robot frame 
        std::map<std::string, Matrix3d> m_rotations;

        // low-pass filter for inputs 
        std::unique_ptr<Sai2Common::ButterworthFilter> m_filter;
};

}  // namespace Optitrack
