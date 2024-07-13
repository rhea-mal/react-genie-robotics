/**
 * @file simviz.cpp
 * @brief Simulation and visualization of panda robot with 1 DOF gripper 
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
VectorXd ui_torques;
VectorXd toro_ui_torques;

mutex mutex_torques, mutex_update;

// specify urdf and robots 
static const string robot_name = "mmp_panda";
static const string toro_name = "toro";

static const string camera_name = "camera_fixed";


// dynamic objects information
const vector<std::string> object_names = {"cup", "ball"};
vector<Affine3d> object_poses;
vector<VectorXd> object_velocities;
const int n_objects = object_names.size();

// simulation thread
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim);

int main() {
	Sai2Model::URDF_FOLDERS["CS225A_URDF_FOLDER"] = string(CS225A_URDF_FOLDER);
	
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/mmp_panda/mmp_panda.urdf";
	static const string toro_file = string(CS225A_URDF_FOLDER) + "/toro/toro.urdf";

	static const string world_file = string(MMP_PANDA_FOLDER) + "/world_mmp_panda.urdf";
	std::cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	auto redis_client = Sai2Common::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_file, camera_name, false);
	graphics->setBackgroundColor(66.0/255, 135.0/255, 245.0/255);  // set blue background 	
	// graphics->showLinkFrame(true, robot_name, "link7", 0.15);  // can add frames for different links
	// graphics->getCamera(camera_name)->setClippingPlanes(0.1, 50);  // set the near and far clipping planes 

	graphics->addUIForceInteraction(robot_name);
	graphics->addUIForceInteraction(toro_name);

	// load robots
	auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_file, false);
	auto toro = std::make_shared<Sai2Model::Sai2Model>(toro_file, false);

	robot->updateModel();
	toro->updateModel();

	ui_torques = VectorXd::Zero(robot->dof());
	toro_ui_torques = VectorXd::Zero(toro->dof());

	// load simulation world
	auto sim = std::make_shared<Sai2Simulation::Sai2Simulation>(world_file, false);
	sim->setJointPositions(robot_name, robot->q());
	sim->setJointVelocities(robot_name, robot->dq());

	sim->setJointPositions(toro_name, toro->q());
	sim->setJointVelocities(toro_name, toro->dq());



	// fill in object information 
	for (int i = 0; i < n_objects; ++i) {
		object_poses.push_back(sim->getObjectPose(object_names[i]));
		object_velocities.push_back(sim->getObjectVelocity(object_names[i]));
	}

    // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(0.0);

    // set co-efficient of friction
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);

	/*------- Set up visualization -------*/
	// init redis client values 
	redis_client.setEigen(JOINT_ANGLES_KEY, robot->q()); 
	redis_client.setEigen(JOINT_VELOCITIES_KEY, robot->dq()); 
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * robot->q());

	redis_client.setEigen(TORO_JOINT_ANGLES_KEY, toro->q()); 
	redis_client.setEigen(TORO_JOINT_VELOCITIES_KEY, toro->dq()); 
	redis_client.setEigen(TORO_JOINT_TORQUES_COMMANDED_KEY, 0 * toro->q());

	redis_client.setBool(THROW_BALL_KEY, false);
	redis_client.setBool(BALL_HIT_KEY, false);

	// start simulation thread
	thread sim_thread(simulation, sim);

	// while window is open:
	while (graphics->isWindowOpen() && fSimulationRunning) {
        graphics->updateRobotGraphics(robot_name, redis_client.getEigen(JOINT_ANGLES_KEY));
		graphics->updateRobotGraphics(toro_name, redis_client.getEigen(TORO_JOINT_ANGLES_KEY));
		{
			lock_guard<mutex> lock(mutex_update);
			for (int i = 0; i < n_objects; ++i) {
				graphics->updateObjectGraphics(object_names[i], object_poses[i]);
			}
		}

		
		
		Vector3d vert_new(0.0, 0.0, 50.0);

		VectorXd rob_pos(10);
		rob_pos << redis_client.getEigen(JOINT_ANGLES_KEY);

		Vector3d pos_new(1.0, 1.0, 5.5);
		Vector3d look_new(rob_pos(0), rob_pos(1), 0.0);


		//graphics->setCameraPose(camera_name, pos_new, vert_new, look_new);
		//graphics->render(camera_name);


		graphics->renderGraphicsWorld();
		{
			lock_guard<mutex> lock(mutex_torques);
			
			ui_torques = graphics->getUITorques(robot_name);
		}
	}

    // stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim) {
	// fSimulationRunning = true;

    // create redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

	// create a timer
	double sim_freq =2000;
	Sai2Common::LoopTimer timer(sim_freq);

	sim->setTimestep(1.0 / sim_freq);
    sim->enableGravityCompensation(true);

	sim->enableJointLimits(robot_name);
	sim->enableJointLimits(toro_name);

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		VectorXd control_torques = redis_client.getEigen(JOINT_TORQUES_COMMANDED_KEY);
		VectorXd toro_control_torques = redis_client.getEigen(TORO_JOINT_TORQUES_COMMANDED_KEY);
		{
			lock_guard<mutex> lock(mutex_torques);
			sim->setJointTorques(robot_name, control_torques + ui_torques);
			sim->setJointTorques(toro_name, toro_control_torques + toro_ui_torques);
		}
		sim->integrate();
        redis_client.setEigen(JOINT_ANGLES_KEY, sim->getJointPositions(robot_name));
        redis_client.setEigen(JOINT_VELOCITIES_KEY, sim->getJointVelocities(robot_name));

		redis_client.setEigen(TORO_JOINT_ANGLES_KEY, sim->getJointPositions(toro_name));
        redis_client.setEigen(TORO_JOINT_VELOCITIES_KEY, sim->getJointVelocities(toro_name));

		auto ball = sim->getDynamicWorld()->getBaseNode("ball");
		
		// Ball throwing
		if (redis_client.getBool(THROW_BALL_KEY)){
			chai3d::cVector3d om(0.0, 0.0, 0.0);

			double x_vel = (rand() % 400 - 200)*0.01;
			double y_vel = (rand() % 400 - 200)*0.01;
			double z_vel = (rand() % 200 + 350)*0.01;

			ball->m_dynamicJoints[0]->setVel(x_vel);
			ball->m_dynamicJoints[1]->setVel(y_vel);
			ball->m_dynamicJoints[2]->setVel(z_vel);
			ball->m_dynamicJoints[3]->setVelSpherical(om);

			ball->m_dynamicJoints[0]->setPos(-2.0);
			ball->m_dynamicJoints[1]->setPos(0.0);
			ball->m_dynamicJoints[2]->setPos(0.3);

			redis_client.setBool(THROW_BALL_KEY, false);
			redis_client.setBool(BALL_HIT_KEY, true);
			cout << "ball thrown" << endl;
		}
		
		redis_client.setEigen(BALL_INIT_POS, Vector3d(ball->m_dynamicJoints[0]->getPos(),
														ball->m_dynamicJoints[1]->getPos(),
														ball->m_dynamicJoints[2]->getPos()));
		redis_client.setEigen(BALL_INIT_VEL, Vector3d(ball->m_dynamicJoints[0]->getVel(),
														ball->m_dynamicJoints[1]->getVel(),
														ball->m_dynamicJoints[2]->getVel()));
	


		// update object information 
		{
			lock_guard<mutex> lock(mutex_update);
			for (int i = 0; i < n_objects; ++i) {
				object_poses[i] = sim->getObjectPose(object_names[i]);
				object_velocities[i] = sim->getObjectVelocity(object_names[i]);
			}
		}
	}
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
}