/**
 * @file controller.cpp
 * @brief Controller file
 * 
 */

#include <Sai2Model.h>
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

using namespace std;
using namespace Eigen;
using namespace Sai2Primitives;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

#include "redis_keys.h"

// States 
enum State {
	INIT = 0, 
	WAIT,
	CALC_BALL_POS,
	MOVE_TO_BALL_POS,
	TRANS_TO_HIT,
	HIT_BALL,
	RETURN,
	TEST
};


// Trajectory Generator function, generates smooth trajectory from pos and vel at t0 to a desired pos and vel at t1
VectorXd trajectory(VectorXd x0, VectorXd v0, double t0, VectorXd x1, VectorXd v1, double t1, double time) {
  return  ((pow(t0, 3)*x1 - pow(t1, 3)*x0 + t0*pow(t1, 3)*v0 - pow(t0, 3)*t1*v1 + 3*t0*pow(t1, 2)*x0 - 3*pow(t0, 2)*t1*x1 - pow(t0, 2)*pow(t1, 2)*v0 + pow(t0, 2)*pow(t1, 2)*v1) +
		   (pow(t0, 3)*v1 - pow(t1, 3)*v0 - t0*pow(t1, 2)*v0 + 2*pow(t0, 2)*t1*v0 - 2*t0*pow(t1, 2)*v1 + pow(t0, 2)*t1*v1 - 6*t0*t1*x0 + 6*t0*t1*x1)*time + 
		   (3*t0*x0 - 3*t0*x1 + 3*t1*x0 - 3*t1*x1 - pow(t0, 2)*v0 - 2*pow(t0, 2)*v1 + 2*pow(t1, 2)*v0 + pow(t1, 2)*v1 - t0*t1*v0 + t0*t1*v1)*pow(time, 2) -
		   (2*x0 - 2*x1 - t0*v0 - t0*v1 + t1*v0 + t1*v1)*pow(time, 3))/pow((t0 - t1), 3);
};

// Trajectory Generator function velocity
VectorXd d_trajectory(VectorXd x0, VectorXd v0, double t0, VectorXd x1, VectorXd v1, double t1, double time) {
  return  ((pow(t0, 3)*v1 - pow(t1, 3)*v0 - t0*pow(t1, 2)*v0 + 2*pow(t0, 2)*t1*v0 - 2*t0*pow(t1, 2)*v1 + pow(t0, 2)*t1*v1 - 6*t0*t1*x0 + 6*t0*t1*x1) + 
		   2*(3*t0*x0 - 3*t0*x1 + 3*t1*x0 - 3*t1*x1 - pow(t0, 2)*v0 - 2*pow(t0, 2)*v1 + 2*pow(t1, 2)*v0 + pow(t1, 2)*v1 - t0*t1*v0 + t0*t1*v1)*pow(time, 1) -
		   3*(2*x0 - 2*x1 - t0*v0 - t0*v1 + t1*v0 + t1*v1)*pow(time, 2))/pow((t0 - t1), 3);
};


Vector3d interpAngularVelocity(Quaterniond q1, Quaterniond q2, double t, double dt) {
    double theta = acos(q1.dot(q2));

    Quaterniond qInterpolated = q1.slerp(t, q2);
    Quaterniond q1_conj = q1.conjugate();

    // Compute the derivative of the slerp interpolation
    Quaterniond qDot((q2.coeffs() * sin(t * theta) - q1.coeffs() * sin((1 - t) * theta) * dt) * (theta / sin(theta)));

    // Compute the angular velocity
    Quaterniond qDotInterpolated = qDot * qInterpolated.conjugate();
    Vector3d angularVelocity = 2.0 * Vector3d(qDotInterpolated.x(), qDotInterpolated.y(), qDotInterpolated.z());

    return angularVelocity;
}


int main() {
	// Location of URDF files specifying world and robot information
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/mmp_panda/mmp_panda.urdf";

	// initial state 
	int state = INIT;
	string controller_status = "1";
	
	// start redis client
	auto redis_client = Sai2Common::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_file, false);
	robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
	robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);  // panda + gripper torques 
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// Main task 
	const string control_link = "end-effector";
	const Vector3d control_point = Vector3d(0.0, 0.0, -0.05);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, control_link, compliant_frame);
	pose_task->disableVelocitySaturation();
	pose_task->disableInternalOtg();
	
	// Base task 
	MatrixXd base_selection_matrix = MatrixXd::Zero(3, robot->dof());
	base_selection_matrix(0, 0) = 1;
	base_selection_matrix(1, 1) = 1;
	base_selection_matrix(2, 2) = 1;
	auto base_task = std::make_shared<Sai2Primitives::JointTask>(robot, base_selection_matrix);
	base_task->disableVelocitySaturation();
	base_task->disableInternalOtg();

	Vector3d base_pose = robot->q().head(3);

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

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	Sai2Common::LoopTimer timer(control_freq, 1e6);

	// Create data collection file
	ofstream file_1;
    file_1.open("base_pos.txt");

	

	// TUNABLE PARAMS
	double rest_time = 0.0;					// time between reaching goal position and starting swing
	double transistion_time = 0.5;
	double swing_time_scaling = 0.25;		// scale value for how long a swing should take
	double start_hit_offset = 0.4;			// how far the EE should move during a swing
	double follow_through_scaling = 0.003;	// Scales time to continue swing past desired impact time
	double power_scaling = -8.0;
	double hit_height = 0.9;				// height at which to hit ball
	double min_time_to_reach_ball = 0.5;	// minimum time given for base to reach ball position
	double base_offset = start_hit_offset;				

	q_desired << -166.0, -101.0, -39.0, -148.0, 78.0, 95.0, -53.0; // Desired Null Space Arm Posture
	q_desired *= M_PI / 180.0;	


	// Set task Gains
	pose_task->setPosControlGains(1500, 150, 0);
	pose_task->setOriControlGains(3000, 150, 0);
	base_task->setGains(800, 80, 0);

	Vector3d wait_pos(-4.0, 2.0, 0.8);
	Matrix3d wait_rot;
	wait_rot << 0, -1,  0,
				0,  0, -1,
				1,  0,  0;

	// World Values
	double ball_radius = 0.1;
	double g = -3.0;
	Vector2d ball_start_pos(-2.0, 0.0);

	// Ball Values
	Vector3d ball_impact_pos;
	Vector3d ball_impact_vel;
	Vector3d ball_init_pos;
	double swing_time;
	double time_at_hit;
	double follow_through;
	int num_hits = 0;
	

	// State Transition Variables
	Vector3d vel_at_end_state;
	Vector3d pos_at_end_state;
	Matrix3d rot_at_end_state;
	VectorXd q_at_end_state(dof);
	VectorXd dq_at_end_state(dof);
	double time_at_end_state;

	// Current EE values
	Vector3d x;
	Vector3d dx;
	Matrix3d R;

	// Curr robot joint valies
	VectorXd q(dof);
	VectorXd dq(dof);

	// Current Base, and EE goals
	Vector3d curr_base_pos_goal;
	Vector3d curr_base_vel_goal;
	Vector3d curr_ee_pos_goal;
	Vector3d curr_ee_vel_goal;
	Matrix3d curr_ee_ori_goal;
	Vector3d curr_ee_ang_goal;


	while (runloop) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// update robot 
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		robot->updateModel();

		// Update EE values
		x << robot->positionInWorld(control_link, control_point);
		dx << robot->Jv(control_link, control_point) *  robot->dq();
		R << robot->rotationInWorld(control_link);

		q << robot->q();
		dq << robot->dq();
		
		// For recording ball position
		ball_init_pos << redis_client.getEigen(BALL_INIT_POS);	
		
		// Throw a ball if current ball position is too low
		if (ball_init_pos(2) < 0.3 && time > 3){
			cout << "Return Error is " << (ball_init_pos({0, 1}) - ball_start_pos).norm() << " m" << endl;
			cout << "Reset" << endl;
			//num_hits = 0;
			redis_client.setBool(THROW_BALL_KEY, true);
		}
		
		// Initialize state transistion variablez
		if (state == INIT) {
			pos_at_end_state = x;
			vel_at_end_state = dx;
			rot_at_end_state = R;
			time_at_end_state = time;
			q_at_end_state = q;
			dq_at_end_state = dq;
			state = WAIT;
		} 
		
		
		if (state == WAIT) {
			
			// Move the EE to the desired wait pos/rot and stay still
			curr_ee_pos_goal = wait_pos;
			curr_ee_vel_goal = Vector3d(0.0, 0.0, 0.0);

			curr_ee_ori_goal = wait_rot;
			curr_ee_ang_goal = Vector3d(0.0, 0.0, 0.0);

			curr_base_pos_goal << wait_pos({0, 1}), 0.0;
			curr_base_vel_goal << Vector3d(0.0, 0.0, 0.0);

			if (redis_client.getBool(BALL_HIT_KEY)){
				pos_at_end_state = x;
				vel_at_end_state = dx;
				rot_at_end_state = R;
				time_at_end_state = time;
				q_at_end_state = q;
				dq_at_end_state = dq;
				state = CALC_BALL_POS;
			}
		}

		if (state == CALC_BALL_POS) {

			// Record ball position and velocity at time of hit
			Vector3d ball_init_pos(redis_client.getEigen(BALL_INIT_POS));				
			Vector3d ball_init_vel(redis_client.getEigen(BALL_INIT_VEL));
			double vz = ball_init_vel[2];

			// Protects against imaginary roots (traj apex never reaches desired height)
			if (pow(vz, 2) < 2*g*(ball_init_pos[2] - hit_height)){
				cout << "not possible to hit at desired height" << endl;
				state = WAIT;
			} else {
				// Calculate ball trajectory
				double time_to_hit = (-vz - sqrt(pow(vz, 2) - 2*g*(ball_init_pos[2] - hit_height)))/g;

				time_at_hit = time + time_to_hit;

				double x_hit = ball_init_pos[0] + ball_init_vel[0]*time_to_hit;
				double y_hit = ball_init_pos[1] + ball_init_vel[1]*time_to_hit;
				double z_hit = hit_height;

				ball_impact_pos << x_hit, y_hit, z_hit;
				ball_impact_vel << ball_init_vel[0], ball_init_vel[1], vz+(g*time_to_hit);

				//ball_impact_pos += ball_impact_vel.normalized() * ball_radius;

				// Scale swing time by ball velocity (higher velocity=longer swing time)
				swing_time = pow(ball_impact_vel.norm(), 1.0/power_scaling)*swing_time_scaling;
				follow_through = pow(ball_impact_vel.norm(), 1.0/-power_scaling)*follow_through_scaling;

				// Protects against traj apex that occurs too soon
				if (time_to_hit < (min_time_to_reach_ball + rest_time + transistion_time + rest_time + swing_time)){
					cout << "not possible to hit at desired height" << endl;
					state = WAIT;
				} else {
					//cout << "Ball impact in " << time_to_hit << " sec" << endl;
					state = MOVE_TO_BALL_POS;
				}
			}
			redis_client.setBool(BALL_HIT_KEY, false);
		}

		if (state == MOVE_TO_BALL_POS) {
			// Generate smooth traj from start position to goal positions over proper time frame
			double t0 = time_at_end_state;
			Vector3d x0 = q_at_end_state({0, 1, 2});
			Vector3d v0 = dq_at_end_state({0, 1, 2}); 

			double t1 = time_at_hit - swing_time - transistion_time - rest_time;
			Vector3d x1;
			x1 << ball_impact_pos({0, 1}) + base_offset*ball_impact_vel.normalized()({0, 1}), atan2(ball_impact_vel(1), ball_impact_vel(0));
			Vector3d v1 = Vector3d(0.0, 0.0, 0.0);

			VectorXd traj_vector(3);
			traj_vector = trajectory(x0, v0, t0, x1, v1, t1, time);

			VectorXd d_traj_vector(3);
			d_traj_vector = d_trajectory(x0, v0, t0, x1, v1, t1, time);

			curr_base_pos_goal = traj_vector;
			curr_base_vel_goal = d_traj_vector;

			if (time > t1){
				pos_at_end_state = x;
				vel_at_end_state = dx;
				rot_at_end_state = R;
				time_at_end_state = time;
				q_at_end_state = q;
				dq_at_end_state = dq;
				state = TRANS_TO_HIT;
			}
		}

		if (state == TRANS_TO_HIT) {

			// Calculate desired EE orientation (normal to ball velocity) as Quaternion
			Vector3d xAxis = -ball_impact_vel.normalized();
			Vector3d yAxis = xAxis.cross(Vector3d(xAxis(0), xAxis(1), 0.0)).normalized();
			Vector3d zAxis = xAxis.cross(yAxis);

			Matrix3d rotationMatrix;
			rotationMatrix.col(0) = xAxis;
			rotationMatrix.col(1) = yAxis;
			rotationMatrix.col(2) = zAxis;

			Quaterniond goal_ori = Quaterniond(rotationMatrix);
			Quaterniond quad_rot_at_end_state = Quaterniond(rot_at_end_state);

			//curr_ee_ori_goal = rotationMatrix;

			// Calculate desired EE position (ball impact pos offset by ball's velocity)
			Vector3d start_hit_pos = ball_impact_pos + start_hit_offset*ball_impact_vel.normalized();
			
			// Generate smooth traj from start position to goal positions over proper time frame
			double t0 = time_at_end_state;
			VectorXd x0(4);
			x0 << pos_at_end_state, 0.0;
			VectorXd v0(4); 
			v0 << vel_at_end_state, 0.0;

			double t1 = time_at_hit - swing_time - rest_time;
			VectorXd x1(4) ;
			x1 << start_hit_pos, 1.0;
			VectorXd v1(4);
			v1 << 0.0, 0.0, 0.0, 0.0;

			if (time < t1){
				VectorXd traj_vector(4);
				traj_vector = trajectory(x0, v0, t0, x1, v1, t1, time);

				VectorXd d_traj_vector(4);
				d_traj_vector = d_trajectory(x0, v0, t0, x1, v1, t1, time);

				curr_ee_pos_goal = traj_vector({0, 1, 2});
				curr_ee_vel_goal = d_traj_vector({0, 1, 2});

				curr_ee_ori_goal = quad_rot_at_end_state.slerp(traj_vector(3), goal_ori).toRotationMatrix();
				curr_ee_ang_goal = interpAngularVelocity(quad_rot_at_end_state, goal_ori, traj_vector(3), d_traj_vector(3));
			} else{

				Vector3d ball_init_pos(redis_client.getEigen(BALL_INIT_POS));				
				Vector3d ball_init_vel(redis_client.getEigen(BALL_INIT_VEL));
				double vz = ball_init_vel[2];

				// Calculate ball trajectory
				double time_to_hit = (-vz - sqrt(pow(vz, 2) - 2*g*(ball_init_pos[2] - hit_height)))/g;

				time_at_hit = time + time_to_hit;
			}



			if (time > time_at_hit - swing_time){
				pos_at_end_state = x;
				vel_at_end_state = dx;
				rot_at_end_state = R;
				time_at_end_state = time;
				q_at_end_state = q;
				dq_at_end_state = dq;
				state = HIT_BALL;
				//cout << "pos reached" << endl;
			}
		}


		if (state == HIT_BALL) {


			Vector3d ball_init_pos(redis_client.getEigen(BALL_INIT_POS));				
			Vector3d ball_init_vel(redis_client.getEigen(BALL_INIT_VEL));
			double vz = ball_init_vel[2];

			if (vz < 0){
				double time_to_hit = (-vz - sqrt(pow(vz, 2) - 2*g*(ball_init_pos[2] - hit_height)))/g;
				time_at_hit = time + time_to_hit;
			}



			double t0 = time_at_end_state;
			Vector3d x0 = pos_at_end_state;
			Vector3d v0 = vel_at_end_state;

			double t1 = time_at_hit;
			Vector3d x1 = ball_impact_pos;
			Vector3d v1 = -ball_impact_vel;

			if (time - time_at_hit < follow_through){
				curr_ee_pos_goal = trajectory(x0, v0, t0, x1, v1, t1, time);
				curr_ee_vel_goal = d_trajectory(x0, v0, t0, x1, v1, t1, time);
			} else{
				curr_ee_vel_goal = Vector3d(0.0, 0.0, 0.0);
				//curr_ee_vel_goal = -ball_impact_vel*0.5;
			}

			curr_base_pos_goal = q_at_end_state({0, 1, 2});
			curr_base_vel_goal = Vector3d(0.0, 0.0, 0.0);

			//curr_ee_ori_goal = rot_at_end_state;
			curr_ee_ang_goal = Vector3d(0.0, 0.0, 0.0);

			if (time - time_at_hit > 0.5){
				//cout << "hit finished" << endl;
				pos_at_end_state = x;
				rot_at_end_state = R;
				vel_at_end_state = dx;
				time_at_end_state = time;
				q_at_end_state = q;
				dq_at_end_state = dq;
				
				state = RETURN;

				// For catch with self
				//redis_client.setBool(BALL_HIT_KEY, true);
				num_hits += 1;
				cout << "Trial " << num_hits << endl;
				//cout << num_hits << " Hits in a row!!!" << endl;
			}
			
		}

		if (state == RETURN) {

			Quaterniond goal_ori = Quaterniond(wait_rot);
			Quaterniond quad_rot_at_end_state = Quaterniond(rot_at_end_state);

			// Generate smooth traj over proper time frame
			double t0 = time_at_end_state;
			VectorXd x0(7);
			x0 << pos_at_end_state, 0.0, q_at_end_state({0, 1, 2});
			VectorXd v0(7); 
			v0 << vel_at_end_state, 0.0, dq_at_end_state({0, 1, 2});

			double t1 = time_at_end_state + 3;
			VectorXd x1(7) ;
			x1 << wait_pos, 1.0, wait_pos({0, 1}), 0.0;
			VectorXd v1(7);
			v1 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

			VectorXd traj_vector(7);
			VectorXd d_traj_vector(7);
			traj_vector = trajectory(x0, v0, t0, x1, v1, t1, time);
			d_traj_vector = d_trajectory(x0, v0, t0, x1, v1, t1, time);

			curr_ee_pos_goal = traj_vector({0, 1, 2});
			curr_ee_vel_goal = d_traj_vector({0, 1, 2});

			//curr_ee_ori_goal = quad_rot_at_end_state.slerp(traj_vector(3), goal_ori).toRotationMatrix();
			//curr_ee_ang_goal = interpAngularVelocity(quad_rot_at_end_state, goal_ori, traj_vector(3), d_traj_vector(3));

			curr_base_pos_goal << traj_vector({4, 5, 6});
			curr_base_vel_goal << d_traj_vector({4, 5, 6});

			if (redis_client.getBool(BALL_HIT_KEY)){
				pos_at_end_state = x;
				vel_at_end_state = dx;
				time_at_end_state = time;
				rot_at_end_state = R;
				q_at_end_state = q;
				dq_at_end_state = dq;
				state = CALC_BALL_POS;
			}

			if (time > t1){
				pos_at_end_state = x;
				vel_at_end_state = dx;
				rot_at_end_state = R;
				time_at_end_state = time;
				q_at_end_state = q;
				dq_at_end_state = dq;
				state = WAIT;
			}	
		}

		// Controller
		command_torques = VectorXd::Zero(dof);
		
		// Base task = main task
		base_task->reInitializeTask();
		base_task->updateTaskModel(N_prec);
		base_task->setGoalPosition(curr_base_pos_goal);
		base_task->setGoalVelocity(curr_base_vel_goal);
		command_torques += base_task->computeTorques();

		// Pose task = main arm control
		if (state == MOVE_TO_BALL_POS){
			joint_task->reInitializeTask();
			joint_task->updateTaskModel(base_task->getTaskAndPreviousNullspace());
			joint_task->setGoalPosition(q_desired);
			joint_task->setGains(800, 80, 0);
			command_torques += joint_task->computeTorques();
		} else{
			pose_task->reInitializeTask();
			pose_task->updateTaskModel(base_task->getTaskAndPreviousNullspace());

			pose_task->setGoalPosition(curr_ee_pos_goal);
			pose_task->setGoalLinearVelocity(curr_ee_vel_goal);

			pose_task->setGoalOrientation(curr_ee_ori_goal);
			pose_task->setGoalAngularVelocity(curr_ee_ang_goal);

			command_torques += pose_task->computeTorques();

			// Joint task = null space posture control for pose task
			joint_task->reInitializeTask();
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
			joint_task->setGoalPosition(q_desired);
			joint_task->setGains(80, 10, 0);
			command_torques += joint_task->computeTorques();
		}

		// Record data
		file_1 << x[0] << "\t" << x[1] << "\t" << x[2] <<  "\t" << 
				  curr_ee_pos_goal[0] << "\t" << curr_ee_pos_goal[1] << "\t" << curr_ee_pos_goal[2] << "\t" <<
				  ball_impact_pos[0] << "\t" << ball_impact_pos[1] << "\t" << ball_impact_pos[2] << "\t" <<
				  ball_impact_vel[0] << "\t" << ball_impact_vel[1] << "\t" << ball_impact_vel[2] << "\t" << 
				  ball_init_pos[0] << "\t" << ball_init_pos[1] << "\t" << ball_init_pos[2] << "\t" << 
				  robot->q()(0) << "\t" << robot->q()(1) << "\t" << 
				  curr_base_pos_goal[0] << "\t" << curr_base_pos_goal[1] << "\t" <<
				  pose_task->getOrientationError().norm() << "\t" << 
				  time << "\t" << time_at_hit << "\t" << swing_time << "\t" << rest_time << "\t" << num_hits << "\t" <<
				  command_torques(0) << "\t" << command_torques(1) << "\t" << command_torques(2) << "\t" << 
				  command_torques(3) << "\t" << command_torques(4) << "\t" << command_torques(5) << "\t" <<
				  command_torques(6) << "\t" << command_torques(7) << "\t" << command_torques(8) << "\t" <<
				  command_torques(9) << endl;

		// execute redis write callback
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		if (num_hits >= 101){
			runloop = false;
		}
	}

	file_1.close();
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}

