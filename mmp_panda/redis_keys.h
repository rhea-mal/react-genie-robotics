/**
 * @file redis_keys.h
 * @author William Chong (williamchong@stanford.edu)
 * @brief 
 * @version 0.1
 * @date 2022-04-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

const std::string JOINT_ANGLES_KEY = "sai2::sim::mmp_panda::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::sim::mmp_panda::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::mmp_panda::actuators::fgc";
const std::string CONTROLLER_RUNNING_KEY = "sai2::sim::mmp_panda::controller";

const std::string TORO_JOINT_ANGLES_KEY = "sai2::sim::toro::sensors::q";
const std::string TORO_JOINT_VELOCITIES_KEY = "sai2::sim::toro::sensors::dq";
const std::string TORO_JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::toro::actuators::fgc";
const std::string TORO_CONTROLLER_RUNNING_KEY = "sai2::sim::toro::controller";

// Tells simvis to throw another ball
const std::string THROW_BALL_KEY = "test1";
// Tells controller that ball has been hit
const std::string BALL_HIT_KEY = "test2";

const std::string BALL_INIT_POS = "test3";
const std::string BALL_INIT_VEL = "test4";



