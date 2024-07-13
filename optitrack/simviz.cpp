/**
 * @file simviz.cpp
 * @brief Simulation and visualization of panda robot 
 * 
 */

#include <math.h>
#include <signal.h>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <typeinfo>
#include <random>

#include "Sai2Graphics.h"
#include "Sai2Model.h"
#include "Sai2Simulation.h"
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "logger/Logger.h"

bool fSimulationRunning = true;
void sighandler(int){fSimulationRunning = false;}

#include "redis_keys.h"

using namespace Eigen;
using namespace std;

// mutex and globals
VectorXd panda_ui_torques;

mutex mutex_torques, mutex_update;

// specify urdf and robots 
static const string panda_name = "mmp_panda";

static const string camera_name = "camera_fixed";

// simulation thread
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim);

// Function to compute orientation difference between two positions
Matrix3d computeOrientationDifference(const Vector3d& left_hand_pos, const Vector3d& right_hand_pos) {
    Vector3d direction = (left_hand_pos - right_hand_pos).normalized();
    Vector3d forward_vector(1.0, 0.0, 0.0); // Assuming forward vector in the X direction
    Matrix3d rotation_matrix = AngleAxisd(acos(direction.dot(forward_vector)), forward_vector.cross(direction).normalized()).toRotationMatrix();
    return rotation_matrix;
}


// Main control loop
int main() {
    // Initialize Redis client
    RedisClient redis_client;
    redis_client.connect();

    // Initialize robot model and control
    auto robot = make_shared<RobotModel>();
    auto joint_task = make_shared<RobotControl>();
    auto pose_task = make_shared<RobotControl>();
    MatrixXd N_prec;
    VectorXd robot_control_torques;

    // Initialize human model
    Human2D human({"left_hand", "right_hand"});

    // Timer for control loop
    Timer timer;
    timer.setLoopFrequency(120); // 120 Hz
    timer.initializeTimer(1e6);  // 1 second timeout

    bool runloop = true;
    enum State { RESET, CALIBRATION, TRACKING } state = RESET;
    int n_samples = 0;
    const int n_calibration_samples = 100;

    while (runloop) {
        timer.waitForNextLoop();
        const double time = timer.elapsedSimTime();

        // Update robot model
        robot->setQ(redis_client.getEigen(PANDA_JOINT_ANGLES_KEY));
        robot->setDq(redis_client.getEigen(PANDA_JOINT_VELOCITIES_KEY));
        robot->updateModel();

        // Read human data
        Vector3d right_hand_pos = redis_client.getEigen(RIGHT_HAND_POS);
        Vector3d left_hand_pos = redis_client.getEigen(LEFT_HAND_POS);

        // Construct current poses
        Affine3d right_hand_pose = Affine3d::Identity();
        right_hand_pose.translation() = right_hand_pos;

        Affine3d left_hand_pose = Affine3d::Identity();
        left_hand_pose.translation() = left_hand_pos;

        std::vector<Affine3d> current_link_poses = {right_hand_pose, left_hand_pose};

        if (state == RESET) {
            joint_task->setGoalPosition(robot->q());
            N_prec.setIdentity();
            joint_task->updateTaskModel(N_prec);
            robot_control_torques = joint_task->computeTorques();

            if (joint_task->goalPositionReached(1e-2)) {
                if (redis_client.getInt(USER_READY_KEY) == 1) {
                    state = CALIBRATION;
                    n_samples = 0;
                    continue;
                }
            }
        } else if (state == CALIBRATION) {
            human.calibratePose({"left_hand", "right_hand"}, current_link_poses, n_calibration_samples, n_samples == 0);

            if (n_samples >= n_calibration_samples) {
                state = TRACKING;
                n_samples = 0;
                continue;
            } else {
                n_samples++;
            }
        } else if (state == TRACKING) {
            N_prec.setIdentity();
            pose_task->updateTaskModel(N_prec);
            joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

            auto relative_poses = human.relativePose({"left_hand", "right_hand"}, current_link_poses);
            Vector3d centroid = human.computeCentroid(relative_poses[0], relative_poses[1]);
            Matrix3d orientation = computeOrientationDifference(left_hand_pos, right_hand_pos);

            pose_task->setGoalPosition(centroid);
            pose_task->setGoalOrientation(orientation);
            robot_control_torques = pose_task->computeTorques();
            robot_control_torques += joint_task->computeTorques();
        }

        // Send torques to robot
        redis_client.setEigen(PANDA_JOINT_TORQUES_COMMANDED_KEY, robot_control_torques);
    }

    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();
    redis_client.setEigen(PANDA_JOINT_TORQUES_COMMANDED_KEY, VectorXd::Zero(robot_control_torques.size()));

    return 0;
}