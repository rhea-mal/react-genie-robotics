/**
 * @file panda_controller.cpp
 * @brief Basic Panda Controller to map left and right hand positions to the end-effector.
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
#include "/Users/rheamalhotra/Desktop/robotics/react-genie-robotics/include/Human2D.h"

using namespace std;
using namespace Eigen;
using namespace Sai2Primitives;
using namespace Sai2Common;
using namespace Optitrack;

bool fSimulationRunning = true;
void sighandler(int){fSimulationRunning = false;}

VectorXd panda_ui_torques;

enum State {
    RESET = 0,
    CALIBRATION,
    TRACKING,
};

int main() {
    const string robot_file = "/Users/rheamalhotra/Desktop/robotics/react-genie-robotics/optitrack/model/mmp_panda.urdf";

    // Start Redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    // Set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // Load robot model
    auto robot = make_shared<Sai2Model::Sai2Model>(robot_file, false);
    robot->setQ(redis_client.getEigen(PANDA_JOINT_ANGLES_KEY));
    robot->setDq(redis_client.getEigen(PANDA_JOINT_VELOCITIES_KEY));
    robot->updateModel();

    // Prepare controller
    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    MatrixXd N_prec = MatrixXd::Identity(dof, dof);

    // End-effector task
    const string control_link = "end-effector";
    const Vector3d control_point = Vector3d(0.0, 0.0, -0.05);
    Affine3d compliant_frame = Affine3d::Identity();
    compliant_frame.translation() = control_point;

    auto pose_task = make_shared<MotionForceTask>(robot, control_link, compliant_frame);
    pose_task->disableVelocitySaturation();
    pose_task->disableInternalOtg();

    // joint task
    MatrixXd joint_selection_matrix = MatrixXd::Zero(7, robot->dof());
    joint_selection_matrix(0, 3) = 1;
    joint_selection_matrix(1, 4) = 1;
    joint_selection_matrix(2, 5) = 1;
    joint_selection_matrix(3, 6) = 1;
    joint_selection_matrix(4, 7) = 1;
    joint_selection_matrix(5, 8) = 1;
    joint_selection_matrix(6, 9) = 1;
    auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot, joint_selection_matrix);
    joint_task->disableVelocitySaturation();
    joint_task->disableInternalOtg();
    VectorXd q_desired(7);

    // Mobile base task
    MatrixXd base_selection_matrix = MatrixXd::Zero(3, robot->dof());
    base_selection_matrix(0, 0) = 1;
    base_selection_matrix(1, 1) = 1;
    base_selection_matrix(2, 2) = 1;
    auto base_task = make_shared<JointTask>(robot, base_selection_matrix);
    base_task->disableVelocitySaturation();
    base_task->disableInternalOtg();
    base_task->setGains(100, 10, 0);

    // Create a loop timer
    fSimulationRunning = true;
    double control_freq = 1000;
    LoopTimer timer(control_freq, 1e6);

    q_desired << -166.0, -101.0, -39.0, -148.0, 78.0, 95.0, -53.0; // Desired Null Space Arm Posture
    q_desired *= M_PI / 180.0;

    // Set task Gains
    pose_task->setPosControlGains(1500, 150, 0);
    pose_task->setOriControlGains(3000, 150, 0);
    base_task->setGains(800, 80, 0);

    Vector3d wait_pos(3.0, 0.0, 0.8);
    Matrix3d wait_rot;
    wait_rot << 0, -1,  0,
                0,  0, -1,
                1,  0,  0;    

    // Current EE values
    Vector3d x;
    Vector3d dx;
    Matrix3d R;

    // Curr robot joint values
    VectorXd q(dof);
    VectorXd dq(dof);

    // Initialize Human2D
    vector<string> complete_link_names = {"left_hand", "right_hand"};
    vector<Affine3d> current_link_poses(complete_link_names.size(), Affine3d::Identity());
    auto human = make_shared<Optitrack::Human2D>(complete_link_names);

    // Initialize state
    State state = RESET;
    bool first_loop = true;
    int n_samples = 0;
    const int n_calibration_samples = 100; // Example value, adjust as needed

    while (fSimulationRunning) {
        timer.waitForNextLoop();
        const double time = timer.elapsedSimTime();

        // Update robot model
        robot->setQ(redis_client.getEigen(PANDA_JOINT_ANGLES_KEY));
        robot->setDq(redis_client.getEigen(PANDA_JOINT_VELOCITIES_KEY));
        robot->updateModel();

        // Update EE values
        x << robot->positionInWorld(control_link, control_point);
        dx << robot->Jv(control_link, control_point) * robot->dq();
        R << robot->rotationInWorld(control_link);

        q << robot->q();
        dq << robot->dq();

        // Read hand positions
        Vector3d right_hand_pos = redis_client.getEigen(RIGHT_HAND_POS);
        Vector3d left_hand_pos = redis_client.getEigen(LEFT_HAND_POS);

        // State machine logic
        switch (state) {
            case RESET:
                // Reset logic
                cout << "RESET state" << endl;
                // Perform reset actions here
                state = CALIBRATION;
                break;

            case CALIBRATION:
                // Calibration logic
                cout << "CALIBRATION state" << endl;
                human->calibratePose(complete_link_names, current_link_poses, n_calibration_samples, first_loop);
                if (first_loop) {
                    first_loop = false;
                }

                if (n_samples > n_calibration_samples) {
                    state = TRACKING;
                    n_samples = 0;

                    // Publish the starting poses in OptiTrack frame
                    auto initial_poses = human->getMultiInitialPose(complete_link_names);
                    for (int i = 0; i < initial_poses.size(); ++i) {
                        cout << "Link: " << complete_link_names[i] << "\n";
                        cout << "Starting position: \n" << initial_poses[i].translation().transpose() << "\n";
                        cout << "Starting rotation: \n" << initial_poses[i].linear() << "\n";
                        cout << "--- \n";  
                    }
                } else {
                    n_samples++;
                }
                break;

            case TRACKING:
                // Tracking logic
                cout << "TRACKING state" << endl;

                // Compute the centroid of the hand positions for the end-effector target
                Vector3d ee_pos_goal = 0.5 * (right_hand_pos + left_hand_pos);

                // Set the end-effector task goal
                pose_task->setGoalPosition(ee_pos_goal);

                // Read hip position for the mobile base target
                Vector3d hip_pos = redis_client.getEigen(HIPS_POS);
                Vector3d base_pos_goal = hip_pos;

                // Set the mobile base task goal
                base_task->setGoalPosition(base_pos_goal);

                // Compute control torques
                command_torques.setZero();
                N_prec.setIdentity();
                base_task->updateTaskModel(N_prec);
                command_torques += base_task->computeTorques();
                pose_task->updateTaskModel(base_task->getTaskAndPreviousNullspace());
                command_torques += pose_task->computeTorques();
                break;
        }

        // Send torques to robot
        redis_client.setEigen(PANDA_JOINT_TORQUES_COMMANDED_KEY, command_torques);
    }

    // Stop timer and clean up
    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();
    redis_client.setEigen(PANDA_JOINT_TORQUES_COMMANDED_KEY, VectorXd::Zero(command_torques.size()));

    return 0;
}
