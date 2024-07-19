/**
 * @file controller.cpp
 * @brief Controller file
 */

#include <Sai2Model.h>
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
//#include "include/Human.h"
#include "/Users/rheamalhotra/Desktop/robotics/react-genie-robotics/include/Human.h"


#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;
using namespace Sai2Primitives;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

#include "redis_keys.h"

// States 
enum State {
    CALIBRATION = 0,
    POSTURE,
    TRACKING
};

int main() {
    // Location of URDF files specifying world and robot information
    const string robot_file = "/Users/rheamalhotra/Desktop/robotics/react-genie-robotics/optitrack/model/mmp_panda.urdf";

    // Initial state 
    int state = CALIBRATION;
    string controller_status = "1";
    
    // Start redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    // Set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // Load robots, read current state and update the model
    auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_file, false);
    robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
    robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
    robot->updateModel();

    // Prepare controller
    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);  // panda + gripper torques 
    MatrixXd N_prec = MatrixXd::Identity(dof, dof);

    // Arm task 
    const string control_link = "link7";
    const Vector3d control_point = Vector3d(0, 0, 0.07);
    Affine3d compliant_frame = Affine3d::Identity();
    compliant_frame.translation() = control_point;
    auto pose_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, control_link, compliant_frame);
    pose_task->setPosControlGains(400, 40, 0);
    pose_task->setOriControlGains(400, 40, 0);

    Vector3d ee_pos = robot->position(control_link, control_point);
    Matrix3d ee_ori = robot->rotation(control_link);

    // Base partial joint task 
    MatrixXd base_selection_matrix = MatrixXd::Zero(3, robot->dof());
    base_selection_matrix(0, 0) = 1;
    base_selection_matrix(1, 1) = 1;
    base_selection_matrix(2, 2) = 1;
    auto base_task = std::make_shared<Sai2Primitives::JointTask>(robot, base_selection_matrix);
    base_task->setGains(400, 40, 0);

    Vector3d base_pose = robot->q().head(3);

    // Joint task
    auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
    joint_task->setGains(400, 40, 0);

    VectorXd q_desired(dof);
    q_desired.tail(7) << -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
    q_desired.tail(7) *= M_PI / 180.0;
    q_desired.head(3) << 0, 0, 0;
    joint_task->setGoalPosition(q_desired);

    bool arm_driven_control = true;
    if (arm_driven_control) {
        base_task->setGains(0, 40, 0);
    }

    // Initialize Human class for tracking
    vector<string> link_names = {"right_hand", "left_hand", "center_hips"};
    auto human = std::make_shared<Optitrack::Human>(link_names);
    Matrix3d R_realsense_to_sai = AngleAxisd(-M_PI / 2, Vector3d::UnitZ()).toRotationMatrix() * AngleAxisd(-M_PI / 2, Vector3d::UnitX()).toRotationMatrix();
    human->setMultiRotationReference(link_names, {R_realsense_to_sai, R_realsense_to_sai, R_realsense_to_sai});

    // Create a loop timer
    runloop = true;
    double control_freq = 1000;
    Sai2Common::LoopTimer timer(control_freq, 1e6);

    // Calibration variables
    int calibration_samples = 100;
    vector<Affine3d> current_poses(3, Affine3d::Identity());

    while (runloop) {
        timer.waitForNextLoop();
        const double time = timer.elapsedSimTime();

        // Update robot 
        robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
        robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
        robot->updateModel();

        if (state == CALIBRATION) {
            // Collect poses for calibration
            current_poses[0].translation() = redis_client.getEigen(RIGHT_HAND_POS);
            current_poses[1].translation() = redis_client.getEigen(LEFT_HAND_POS);
            current_poses[2].translation() = redis_client.getEigen(CENTER_HIPS_POS);

            human->calibratePose(link_names, current_poses, calibration_samples, false);

            if (--calibration_samples <= 0) {
                state = POSTURE;
            }
        } else if (state == POSTURE) {
            // Update task model 
            N_prec.setIdentity();
            joint_task->updateTaskModel(N_prec);

            command_torques = joint_task->computeTorques();

            if ((robot->q() - q_desired).norm() < 1e-2) {
                cout << "Posture To Tracking" << endl;
                pose_task->reInitializeTask();
                base_task->reInitializeTask();
                joint_task->reInitializeTask();

                ee_pos = robot->position(control_link, control_point);
                ee_ori = robot->rotation(control_link);
                base_pose = robot->q().head(3);

                state = TRACKING;
            }
        } else if (state == TRACKING) {
            // Update goal positions and orientations based on human tracking

            // Update current poses from Redis
            current_poses[0].translation() = redis_client.getEigen(RIGHT_HAND_POS);
            current_poses[1].translation() = redis_client.getEigen(LEFT_HAND_POS);
            current_poses[2].translation() = redis_client.getEigen(CENTER_HIPS_POS);

            auto relative_poses = human->relativePose(link_names, current_poses);

            // Set task goals
            pose_task->setGoalPosition(relative_poses[0].translation());
            pose_task->setGoalOrientation(ee_ori); // Keep orientation constant
            base_task->setGoalPosition(relative_poses[2].translation());

            // Update task models
            if (arm_driven_control) {
                N_prec.setIdentity();
                pose_task->updateTaskModel(N_prec);
                base_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
                joint_task->updateTaskModel(base_task->getTaskAndPreviousNullspace());
            } else {
                N_prec.setIdentity();
                base_task->updateTaskModel(N_prec);
                pose_task->updateTaskModel(base_task->getTaskAndPreviousNullspace());
                joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
            }

            command_torques = pose_task->computeTorques() + base_task->computeTorques() + joint_task->computeTorques();
        }

        // Execute redis write callback
        redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    }

    timer.stop();
    cout << "\nSimulation loop timer stats:\n";
    timer.printInfoPostRun();
    redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

    return 0;
}
