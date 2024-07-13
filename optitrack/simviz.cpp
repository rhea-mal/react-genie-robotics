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

int main() {	
	static const string panda_file = "./resources/model/mmp_panda.urdf";

	static const string world_file = "./resources/world/world_volley.urdf";
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
	graphics->getCamera(camera_name)->setClippingPlanes(0.1, 2000);  // set the near and far clipping planes 

	//graphics->addUIForceInteraction(panda_name);

	// load robots
	auto panda = std::make_shared<Sai2Model::Sai2Model>(panda_file, false);

	panda->updateModel();
	panda_ui_torques = VectorXd::Zero(panda->dof());

	// load simulation world
	auto sim = std::make_shared<Sai2Simulation::Sai2Simulation>(world_file, false);
	sim->setJointPositions(panda_name, panda->q());
	sim->setJointVelocities(panda_name, panda->dq());
    // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(1.0);

    // set co-efficient of friction
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);

	/*------- Set up visualization -------*/
	// init redis client values 
	redis_client.setEigen(PANDA_JOINT_ANGLES_KEY, panda->q()); 
	redis_client.setEigen(PANDA_JOINT_VELOCITIES_KEY, panda->dq()); 
	redis_client.setEigen(PANDA_JOINT_TORQUES_COMMANDED_KEY, 0 * panda->q());

	bool conmove = true;
	thread sim_thread(simulation, sim);

	// while window is open:
	while (graphics->isWindowOpen() && fSimulationRunning) {
        graphics->updateRobotGraphics(panda_name, redis_client.getEigen(PANDA_JOINT_ANGLES_KEY));
		{
			lock_guard<mutex> lock(mutex_update);
		}
		graphics->renderGraphicsWorld();
		{
			lock_guard<mutex> lock(mutex_torques);
			panda_ui_torques = graphics->getUITorques(panda_name);
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

	sim->enableJointLimits(panda_name);
	//sim->enableJointLimits(toro_name);

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		VectorXd panda_control_torques = redis_client.getEigen(PANDA_JOINT_TORQUES_COMMANDED_KEY);
		{
			lock_guard<mutex> lock(mutex_torques);
			sim->setJointTorques(panda_name, panda_control_torques + panda_ui_torques);
		}
		sim->integrate();
        redis_client.setEigen(PANDA_JOINT_ANGLES_KEY, sim->getJointPositions(panda_name));
        redis_client.setEigen(PANDA_JOINT_VELOCITIES_KEY, sim->getJointVelocities(panda_name));

		// update object information 
		{
			lock_guard<mutex> lock(mutex_update);
		}
	}
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
}
