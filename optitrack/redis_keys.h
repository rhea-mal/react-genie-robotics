/**
 * @file redis_keys.h
 * @brief 
 * @version 0.1
 * @date 2022-04-30
 * 
 */

const std::string PANDA_JOINT_ANGLES_KEY = "sai2::sim::mmp_panda::sensors::q";
const std::string PANDA_JOINT_VELOCITIES_KEY = "sai2::sim::mmp_panda::sensors::dq";
const std::string PANDA_JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::mmp_panda::actuators::fgc";

const std::string USER_READY_KEY = "sai2::optitrack::user_ready";

const std::string RIGHT_HAND_POS = "sai2::optitrack::rigid_body_pos::1";
//"sai2::realsense::right_hand";
const std::string LEFT_HAND_POS = "sai2::realsense::left_hand";
const std::string CENTER_HIPS_POS = "sai2::realsense::center_hips";

const std::string REPLAY_RIGHT_HAND_POS = "sai2::sim::mmp_panda::right_hand::pos";
const std::string REPLAY_LEFT_HAND_POS = "sai2::sim::mmp_panda::left_hand::pos";
