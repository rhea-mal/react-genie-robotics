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

// globals
VectorXd nominal_posture;
VectorXd control_torques;
mutex mutex_update;
mutex mutex_torques;

enum State {
    RESET = 0,
    CALIBRATION,
    TRACKING,
};

const string robot_file = "/Users/rheamalhotra/Desktop/robotics/react-genie-robotics/optitrack/model/mmp_panda.urdf";

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

Matrix3d R_realsense_to_sai = AngleAxisd(- M_PI / 2, Vector3d::UnitZ()).toRotationMatrix() * AngleAxisd(-M_PI / 2, Vector3d::UnitX()).toRotationMatrix();

int main() {
    // Start Redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    // Set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // Load robot model
    auto robot = make_shared<Sai2Model::Sai2Model>(robot_file, false);
    nominal_posture = robot->q();

    // Prepare controller
    int dof = robot->dof();
    control_torques = VectorXd::Zero(dof);

    // Base task = main task
    base_task->reInitializeTask();
    base_task->updateTaskModel(N_prec);
    base_task->setGoalPosition(curr_base_pos_goal);
    base_task->setGoalVelocity(curr_base_vel_goal);
    control_torques += base_task->computeTorques();


    redis_client.setInt(USER_READY_KEY, 0);
    MatrixXd N_prec = MatrixXd::Identity(dof, dof);
    robot->updateModel();

    std::vector<std::string> link_names = {"end-effector"};
    auto human = std::make_shared<Optitrack::Human2D>(link_names);
    human->setRotationReference(R_realsense_to_sai);
    redis_client.setInt(USER_READY_KEY, 0);

    // start controller thread
    thread control_thread(control, human, robot);

    control_thread.join();

    return 0;
}

void control(std::shared_ptr<Optitrack::Human2D> human,
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

    Eigen::Affine3d control_frame = Eigen::Affine3d::Identity();
    control_frame.translation() = Vector3d(0, 0, 0);

    auto task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, "end-effector", control_frame);
    task->disableInternalOtg();
    task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
    task->setPosControlGains(400, 40, 0);

    auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
    joint_task->disableVelocitySaturation();
    joint_task->disableInternalOtg();
    VectorXd q_desired = robot->q();
    joint_task->setGains(400, 40, 0);
    joint_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
    joint_task->setGoalPosition(nominal_posture);

    // initialize
    int state = CALIBRATION;

    bool first_loop = true;
    const int n_calibration_samples = 1000;  // 1 second of samples
    int n_samples = 0;
    VectorXd robot_control_torques = VectorXd::Zero(dof);

    Eigen::Affine3d end_effector_initial_pose = Eigen::Affine3d::Identity();

    // create a loop timer
    bool runloop = true;
    double control_freq = 1000;
    Sai2Common::LoopTimer timer(control_freq, 1e6);

    while (runloop) {
        timer.waitForNextLoop();
        const double time = timer.elapsedSimTime();

        // update robot model
        robot->setQ(redis_client.getEigen(PANDA_JOINT_ANGLES_KEY));
        robot->setDq(redis_client.getEigen(PANDA_JOINT_VELOCITIES_KEY));
        robot->updateModel();

        // Get the position from Redis
        Eigen::Vector3d end_effector_position = redis_client.getEigen("sai2::realsense::right_hand");

        Eigen::Affine3d current_pose = Eigen::Affine3d::Identity();
        current_pose.translation() = end_effector_position;

        if (state == RESET) {
            // start robot at default configuration and hold the posture
            joint_task->setGoalPosition(nominal_posture);
            N_prec.setIdentity();
            joint_task->updateTaskModel(N_prec);
            robot_control_torques = joint_task->computeTorques();

            if (joint_task->goalPositionReached(1e-2)) {
                if (redis_client.getInt(USER_READY_KEY) == 1) {
                    state = CALIBRATION;
                    first_loop = true;
                    n_samples = 0;
                    continue;
                }
            }
        } else if (state == CALIBRATION) {
            human->calibratePose({current_pose}, n_calibration_samples, first_loop);
            if (first_loop) {
                first_loop = false;
            }

            if (n_samples > n_calibration_samples) {
                state = TRACKING;
                n_samples = 0;
                end_effector_initial_pose = human->getInitialPose();
                std::cout << "end_effector_initial_pose: " << end_effector_initial_pose.translation().transpose() << std::endl;
                continue;
            } else {
                n_samples++;
            }

        } else if (state == TRACKING) {
            // update model
            N_prec.setIdentity();
            task->updateTaskModel(N_prec);
            // Controller
            control_torques = VectorXd::Zero(dof);

            // get relative pose
            std::vector<Eigen::Affine3d> current_poses = {current_pose};
            std::vector<Eigen::Affine3d> relative_poses = human->relativePose(current_poses);
            Eigen::Affine3d relative_pose = relative_poses[0];

            // Print relative_pose and control_torques
            std::cout << "relative_pose translation: " << relative_pose.translation().transpose() << std::endl;
            std::cout << "relative_pose rotation: \n" << relative_pose.rotation() << std::endl;

            // set task goals and compute control torques
            Eigen::Vector3d goal_position = end_effector_initial_pose.translation() + relative_pose.translation();
            task->setGoalPosition(goal_position);

            control_torques = task->computeTorques() + robot->coriolisForce() + joint_task->computeTorques();

            std::cout << "control_torques: " << control_torques.transpose() << std::endl;
        }

        // execute redis write callback
        redis_client.setEigen(PANDA_JOINT_TORQUES_COMMANDED_KEY, control_torques);
    }

    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();
    redis_client.setEigen(PANDA_JOINT_TORQUES_COMMANDED_KEY, 0 * control_torques);  // back to floating
}
