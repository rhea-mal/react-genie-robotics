/**
 * @file controller.cpp
 * @brief Controller file
 */

#include <Sai2Model.h>
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <signal.h>
#include "redis_keys.h"
#include "../include/Human2D.h"

using namespace std;
using namespace Eigen;
using namespace Sai2Primitives;
using namespace Optitrack;

bool runloop = false;
void sighandler(int){runloop = false;}

// specify urdf and robots
const string robot_file = "./resources/model/mmp_panda.urdf";

enum State {
    RESET = 0,
    CALIBRATION,
    TRACKING,
    TEST
};

// this is OT R transpose only
Eigen::Matrix3d quaternionToRotationMatrix(const VectorXd& quat)
{
    Eigen::Quaterniond q;
    q.x() = quat[0];
    q.y() = quat[1];
    q.z() = quat[2];
    q.w() = quat[3];
    return q.toRotationMatrix();
}

void control(std::shared_ptr<Human2D> human,
             std::shared_ptr<Sai2Model::Sai2Model> robot);

int main() {
    // start redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load robots, read current state and update the model
    auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_file, false);
    robot->setQ(redis_client.getEigen(PANDA_JOINT_ANGLES_KEY));
    robot->setDq(redis_client.getEigen(PANDA_JOINT_VELOCITIES_KEY));
    robot->updateModel();

    // Initialize Human2D class
    std::vector<std::string> link_names = {"right_hand", "left_hand"};
    auto human = std::make_shared<Human2D>(link_names);

    // prepare controller
    int dof = robot->dof();
    VectorXd control_torques = VectorXd::Zero(dof);  // panda + gripper torques 
    redis_client.setInt(USER_READY_KEY, 0);

    // start control thread
    thread control_thread(control, human, robot);
    control_thread.join();

    return 0;
}

//------------------------------------------------------------------------------
void control(std::shared_ptr<Human2D> human,
             std::shared_ptr<Sai2Model::Sai2Model> robot) {

    // create redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    // update robot model and initialize control vectors
    robot->setQ(redis_client.getEigen(PANDA_JOINT_ANGLES_KEY));
    robot->setDq(redis_client.getEigen(PANDA_JOINT_VELOCITIES_KEY));
    robot->updateModel();
    int dof = robot->dof();
    MatrixXd N_prec = MatrixXd::Identity(dof, dof);
   
    // create tasks
    const string control_link = "end-effector";
    const Vector3d control_point = Vector3d(0.0, 0.0, -0.05);
    Affine3d compliant_frame = Affine3d::Identity();
    compliant_frame.translation() = control_point;
    auto pose_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, control_link, compliant_frame);
    pose_task->disableVelocitySaturation();
    pose_task->disableInternalOtg();
    pose_task->setPosControlGains(1500, 150, 0);
    pose_task->setOriControlGains(3000, 150, 0);

    auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
    joint_task->disableVelocitySaturation();
    joint_task->disableInternalOtg();
    VectorXd q_desired = robot->q();
    joint_task->setGains(400, 40, 0);
    joint_task->setGoalPosition(q_desired);

    // initialize
    int state = RESET;
    const int n_calibration_samples = 1000;  // 1 second of samples
    int n_samples = 0;
    VectorXd robot_control_torques = VectorXd::Zero(dof);

    // create a loop timer
    runloop = true;
    double control_freq = 1000;
    Sai2Common::LoopTimer timer(control_freq, 1e6);

    while (runloop) {
        timer.waitForNextLoop();
        const double time = timer.elapsedSimTime();

        // update robot model
        robot->setQ(redis_client.getEigen(PANDA_JOINT_ANGLES_KEY));
        robot->setDq(redis_client.getEigen(PANDA_JOINT_VELOCITIES_KEY));
        robot->updateModel();

        // read human data
        std::vector<std::string> link_names = {"right_hand", "left_hand"};
        std::vector<Affine3d> current_link_poses;
        for (const auto& link_name : link_names) {
            Eigen::Vector3d current_position = redis_client.getEigen(OPTI_POS_PREFIX + link_name);
            Eigen::VectorXd quaternion_matrix = redis_client.getEigen(OPTI_ORI_PREFIX + link_name);

            if (quaternion_matrix.size() != 4) {
                std::cerr << "Error: Quaternion retrieved from Redis does not have 4 elements." << std::endl;
                continue;
            }

            Eigen::Affine3d current_pose = Eigen::Affine3d::Identity();
            current_pose.translation() = current_position;
            current_pose.linear() = quaternionToRotationMatrix(quaternion_matrix);

            current_link_poses.push_back(current_pose);
        }

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
            human->calibratePose(link_names, current_link_poses, n_calibration_samples, true);

            if (n_samples > n_calibration_samples) {
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

            auto relative_poses = human->relativePose(link_names, current_link_poses);
            Vector3d centroid = human->computeCentroid(relative_poses[0], relative_poses[1]);

            pose_task->setGoalPosition(centroid);
            pose_task->setGoalOrientation(Matrix3d::Identity());
            robot_control_torques = pose_task->computeTorques();
            robot_control_torques += joint_task->computeTorques();

        }

        // send torques to robot
        redis_client.setEigen(PANDA_JOINT_TORQUES_COMMANDED_KEY, robot_control_torques);
    }

    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();
    redis_client.setEigen(PANDA_JOINT_TORQUES_COMMANDED_KEY, 0 * robot_control_torques);
}