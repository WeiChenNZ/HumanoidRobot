#include "MemoryManager.h"

using namespace std;
using namespace Eigen;

MemoryManager::MemoryManager(void)
{
    initSharedMemory(false);
    init();
    connect();
}

void MemoryManager::initSharedMemory(bool init)
{
    //Thread state
    THREAD_STATE = make_unique<SharedMemory>("BRUCE","THREAD_STATE", init);
    THREAD_STATE->addBlock("simulation", MatrixXd::Zero(1,1));
    THREAD_STATE->addBlock("bear", MatrixXd::Zero(1,1));
    THREAD_STATE->addBlock("dxl", MatrixXd::Zero(1,1));
    THREAD_STATE->addBlock("sense", MatrixXd::Zero(1,1));
    THREAD_STATE->addBlock("estimation", MatrixXd::Zero(1,1));
    THREAD_STATE->addBlock("low_level", MatrixXd::Zero(1,1));
    THREAD_STATE->addBlock("high_level", MatrixXd::Zero(1,1));
    THREAD_STATE->addBlock("top_level", MatrixXd::Zero(1,1));

    //Simulator state
    SIMULATOR_STATE = make_unique<SharedMemory>("BRUCE", "SIMULATOR_STATE", init);
    SIMULATOR_STATE->addBlock("time_stamp", MatrixXd::Zero(1,1));

    //Sense State
    SENSE_STATE = make_unique<SharedMemory>("BRUCE", "SENSE_STATE", init);
    SENSE_STATE->addBlock("time_stamp", MatrixXd::Zero(1,1));
    SENSE_STATE->addBlock("imu_acceleration", MatrixXd::Zero(1,3));
    SENSE_STATE->addBlock("imu_ang_rate", MatrixXd::Zero(1,3));
    SENSE_STATE->addBlock("foot_contacts", MatrixXd::Zero(1,4));

    //Gamepad State
    GAMEPAD_STATE = make_unique<SharedMemory>("BRUCE", "GAMEPAD_STATE", init);
    GAMEPAD_STATE->addBlock("U", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("D", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("L", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("R", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("A", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("B", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("X", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("Y", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("LZ", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("LS", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("LS2", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("LSP", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("LSM", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("RZ", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("RS", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("RS2", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("RSP", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("RSM", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("ST", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("BK", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("ALT", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("FN", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("LX", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("LY", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("RX", MatrixXd::Zero(1,1));
    GAMEPAD_STATE->addBlock("RY", MatrixXd::Zero(1,1));

    //Leg State
    LEG_STATE = make_unique<SharedMemory>("BRUCE", "LEG_STATE", init);
    LEG_STATE->addBlock("time_stamp", MatrixXd::Zero(1,1));
    LEG_STATE->addBlock("joint_positions", MatrixXd::Zero(1,10));
    LEG_STATE->addBlock("joint_velocities", MatrixXd::Zero(1,10));
    LEG_STATE->addBlock("joint_torques", MatrixXd::Zero(1,10));
    LEG_STATE->addBlock("temperature", MatrixXd::Zero(1,1));
    LEG_STATE->addBlock("voltage", MatrixXd::Zero(1,1));

    //Leg Command
    LEG_COMMAND = make_unique<SharedMemory>("BRUCE", "LEG_COMMAND", init);
    LEG_COMMAND->addBlock("time_stamp", MatrixXd::Zero(1,1));
    LEG_COMMAND->addBlock("goal_torques", MatrixXd::Zero(1,10));
    LEG_COMMAND->addBlock("goal_positions", MatrixXd::Zero(1,10));
    LEG_COMMAND->addBlock("goal_velocities", MatrixXd::Zero(1,10));
    LEG_COMMAND->addBlock("BEAR_mode", MatrixXd::Zero(1,1));
    LEG_COMMAND->addBlock("BEAR_enable", MatrixXd::Zero(1,1));
    LEG_COMMAND->addBlock("damping", MatrixXd::Zero(1,1));

    //Arm State
    ARM_STATE = make_unique<SharedMemory>("BRUCE", "ARM_STATE", init);
    ARM_STATE->addBlock("time_stamp", MatrixXd::Zero(1,1));
    ARM_STATE->addBlock("joint_positions", MatrixXd::Zero(1,6));
    ARM_STATE->addBlock("joint_velocities", MatrixXd::Zero(1,6));

    //Arm Command
    ARM_COMMAND = make_unique<SharedMemory>("BRUCE", "ARM_COMMAND", init);
    ARM_COMMAND->addBlock("time_stamp", MatrixXd::Zero(1,1));
    ARM_COMMAND->addBlock("goal_positions", MatrixXd::Zero(1,6));
    ARM_COMMAND->addBlock("goal_velocities", MatrixXd::Zero(1,6));
    ARM_COMMAND->addBlock("DXL_mode", MatrixXd::Zero(1,1));
    ARM_COMMAND->addBlock("DXL_enable", MatrixXd::Zero(1,1));

    //Estimator State
    ESTIMATOR_STATE = make_unique<SharedMemory>("BRUCE", "ESTIMATOR_STATE", init);
    ESTIMATOR_STATE->addBlock("time_stamp", MatrixXd::Zero(1,1));
    ESTIMATOR_STATE->addBlock("body_position", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("body_velocity", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("body_acceleration", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("body_rot_matrix", MatrixXd::Identity(3,3));
    ESTIMATOR_STATE->addBlock("body_euler_ang", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("body_yaw_ang", MatrixXd::Zero(1,1));
    ESTIMATOR_STATE->addBlock("body_ang_rate", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("com_position", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("com_velocity", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("ang_momentum", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("H_matrix", MatrixXd::Zero(16,16));
    ESTIMATOR_STATE->addBlock("CG_vector", MatrixXd::Zero(1,16));
    ESTIMATOR_STATE->addBlock("AG_matrix", MatrixXd::Zero(6,16));
    ESTIMATOR_STATE->addBlock("dAGdq_vector", MatrixXd::Zero(1,6));
    ESTIMATOR_STATE->addBlock("foot_contacts", MatrixXd::Zero(1,4));

    ESTIMATOR_STATE->addBlock("right_foot_rot_matrix", MatrixXd::Zero(3,3));
    ESTIMATOR_STATE->addBlock("right_foot_ang_rate", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("right_foot_Jw", MatrixXd::Zero(3,16));
    ESTIMATOR_STATE->addBlock("right_foot_dJwdq", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("right_foot_position", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("right_foot_velocity", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("right_toe_position", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("right_toe_velocity", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("right_toe_Jv", MatrixXd::Zero(3,16));
    ESTIMATOR_STATE->addBlock("right_toe_dJvdq", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("right_heel_position", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("right_heel_velocity", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("right_heel_Jv", MatrixXd::Zero(3,16));
    ESTIMATOR_STATE->addBlock("right_heel_dJvdq", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("right_ankle_position", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("right_ankle_velocity", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("right_ankle_Jv", MatrixXd::Zero(3,16));
    ESTIMATOR_STATE->addBlock("right_ankle_dJvdq", MatrixXd::Zero(1,3));

    ESTIMATOR_STATE->addBlock("left_foot_rot_matrix", MatrixXd::Zero(3,3));
    ESTIMATOR_STATE->addBlock("left_foot_ang_rate", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("left_foot_Jw", MatrixXd::Zero(3,16));
    ESTIMATOR_STATE->addBlock("left_foot_dJwdq", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("left_foot_position", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("left_foot_velocity", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("left_toe_position", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("left_toe_velocity", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("left_toe_Jv", MatrixXd::Zero(3,16));
    ESTIMATOR_STATE->addBlock("left_toe_dJvdq", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("left_heel_position", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("left_heel_velocity", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("left_heel_Jv", MatrixXd::Zero(3,16));
    ESTIMATOR_STATE->addBlock("left_heel_dJvdq", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("left_ankle_position", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("left_ankle_velocity", MatrixXd::Zero(1,3));
    ESTIMATOR_STATE->addBlock("left_ankle_Jv", MatrixXd::Zero(3,16));
    ESTIMATOR_STATE->addBlock("left_ankle_dJvdq", MatrixXd::Zero(1,3));

    //Estimator Command
    ESTIMATOR_COMMAND = make_unique<SharedMemory>("BRUCE", "ESTIMATOR_COMMAND", init);
    ESTIMATOR_COMMAND->addBlock("restart", MatrixXd::Zero(1,1));

    //Planner Command
    PLANNER_COMMAND = make_unique<SharedMemory>("BRUCE", "PLANNER_COMMAND", init);
    PLANNER_COMMAND->addBlock("time_stamp", MatrixXd::Zero(1,1));
    PLANNER_COMMAND->addBlock("mode", MatrixXd::Zero(1,1));
    PLANNER_COMMAND->addBlock("phase", MatrixXd::Zero(1,1));
    PLANNER_COMMAND->addBlock("body_position", MatrixXd::Zero(1,3));
    PLANNER_COMMAND->addBlock("body_velocity", MatrixXd::Zero(1,3));
    PLANNER_COMMAND->addBlock("body_rot_matrix", MatrixXd::Identity(3,3));
    PLANNER_COMMAND->addBlock("body_ang_rate", MatrixXd::Zero(1,3));
    PLANNER_COMMAND->addBlock("com_position", MatrixXd::Zero(1,3));
    PLANNER_COMMAND->addBlock("com_velocity", MatrixXd::Zero(1,3));

    PLANNER_COMMAND->addBlock("right_foot_phase", MatrixXd::Zero(1,1));
    PLANNER_COMMAND->addBlock("right_foot_position", MatrixXd::Zero(1,3));
    PLANNER_COMMAND->addBlock("right_foot_velocity", MatrixXd::Zero(1,3));
    PLANNER_COMMAND->addBlock("right_foot_rot_matrix", MatrixXd::Zero(3,3));
    PLANNER_COMMAND->addBlock("right_foot_ang_rate", MatrixXd::Zero(1,3));

    PLANNER_COMMAND->addBlock("left_foot_phase", MatrixXd::Zero(1,1));
    PLANNER_COMMAND->addBlock("left_foot_position", MatrixXd::Zero(1,3));
    PLANNER_COMMAND->addBlock("left_foot_velocity", MatrixXd::Zero(1,3));
    PLANNER_COMMAND->addBlock("left_foot_rot_matrix", MatrixXd::Zero(3,3));
    PLANNER_COMMAND->addBlock("left_foot_ang_rate", MatrixXd::Zero(1,3));

    //User Command
    USER_COMMAND = make_unique<SharedMemory>("BRUCE", "USER_COMMAND", init);
    USER_COMMAND->addBlock("time_stamp", MatrixXd::Zero(1,1));
    USER_COMMAND->addBlock("mode", MatrixXd::Zero(1,1));
    USER_COMMAND->addBlock("com_xy_velocity", MatrixXd::Zero(1,2));
    USER_COMMAND->addBlock("yaw_rate", MatrixXd::Zero(1,1));
    USER_COMMAND->addBlock("com_position_change_scaled", MatrixXd::Zero(1,3));
    USER_COMMAND->addBlock("body_euler_angle_change", MatrixXd::Zero(1,3));
    USER_COMMAND->addBlock("right_foot_yaw_angle_change", MatrixXd::Zero(1,1));
    USER_COMMAND->addBlock("left_foot_yaw_angle_change", MatrixXd::Zero(1,1));
    USER_COMMAND->addBlock("foot_clearance", MatrixXd::Zero(1,1));
    USER_COMMAND->addBlock("cooling_speed", MatrixXd::Zero(1,1));
    USER_COMMAND->addBlock("dcm_offset_compensation", MatrixXd::Zero(1,2));
    USER_COMMAND->addBlock("com_offset_compensation", MatrixXd::Zero(1,1));


}

void MemoryManager::init()
{
    THREAD_STATE->setInitTrue();
    SIMULATOR_STATE->setInitTrue();
    SENSE_STATE->setInitTrue();
    GAMEPAD_STATE->setInitTrue();
    LEG_STATE->setInitTrue();
    LEG_COMMAND->setInitTrue();
    ARM_STATE->setInitTrue();
    ARM_COMMAND->setInitTrue();
    ESTIMATOR_STATE->setInitTrue();
    ESTIMATOR_COMMAND->setInitTrue();
    PLANNER_COMMAND->setInitTrue();
    USER_COMMAND->setInitTrue();
}

void MemoryManager::connect()
{
    THREAD_STATE->connectSegment();
    SIMULATOR_STATE->connectSegment();
    SENSE_STATE->connectSegment();
    GAMEPAD_STATE->connectSegment();
    LEG_STATE->connectSegment();
    LEG_COMMAND->connectSegment();
    ARM_STATE->connectSegment();
    ARM_COMMAND->connectSegment();
    ESTIMATOR_STATE->connectSegment();
    ESTIMATOR_COMMAND->connectSegment();
    PLANNER_COMMAND->connectSegment();
    USER_COMMAND->connectSegment();
}

