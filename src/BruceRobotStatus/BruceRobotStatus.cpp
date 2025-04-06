#include "BruceRobotStatus.h"
#include "MemoryManager.h"
#include "MathTools.h"
#include <chrono>
#include <thread>

using namespace Eigen;
using namespace std;

void BruceRobotStatus::updateRobotStatus(void)
{
    unordered_map<string, MatrixXd> estimationData = MemoryManager::getInstance().ESTIMATOR_STATE->getVal();

    //time
    timeStamp = estimationData["time_stamp"](0,0);   

    //body
    bodyPositionInWorld = estimationData["body_position"];
    bodyVelocityInWorld = estimationData["body_velocity"];
    bodyAccelerationInWorld =  estimationData["body_acceleration"];
    bodyRotInWorld =  estimationData["body_rot_matrix"];
    bodyOmegaInBody = estimationData["body_ang_rate"];
    yaw = estimationData["body_yaw_ang"](0,0);
    RotOfYaw = MathTools::Rz(yaw);

    //center of mass
    comPositionInWorld = estimationData["com_position"];
    comVelocityInWorld = estimationData["com_velocity"];
    comAngularMomentumInWorld = estimationData["ang_momentum"];

    //dynamics
    H = estimationData["H_matrix"];
    CG = estimationData["CG_vector"];
    AG = estimationData["AG_matrix"];
    dAGdq = estimationData["dAGdq_vector"];

    //foot contacts
    footContacts = estimationData["foot_contacts"];

    //right foot
    RotRightFootInWorld = estimationData["right_foot_rot_matrix"];
    omegaRightFoot = estimationData["right_foot_ang_rate"];
    JwRightFoot = estimationData["right_foot_Jw"];
    dJwdqRightFoot = estimationData["right_foot_dJwdq"];
    positionRightFootInWorld = estimationData["right_foot_position"];
    velocityRightFootInWorld = estimationData["right_foot_velocity"];
    positionRightToeInWorld = estimationData["right_toe_position"];
    velocityRightToeInWorld = estimationData["right_toe_velocity"];
    JvRightToeInWorld = estimationData["right_toe_Jv"];
    dJvdqRightToeInWorld = estimationData["right_toe_dJvdq"];
    positionRightHeelInWorld = estimationData["right_heel_position"];
    velocityRightHeelInWorld = estimationData["right_heel_velocity"];
    JvRightHeelInWorld = estimationData["right_heel_Jv"];
    dJvdqRightHeelInWorld = estimationData["right_heel_dJvdq"];
    positionRightAnkleInWorld = estimationData["right_ankle_position"];
    velocityRightAnkleInWorld = estimationData["right_ankle_velocity"];
    JvRightAnkleInWorld = estimationData["right_ankle_Jv"];
    dJvdqRightAnkleInWorld = estimationData["right_ankle_dJvdq"];

    //left foot
    RotLeftFootInWorld = estimationData["left_foot_rot_matrix"];
    omegaLeftFoot = estimationData["left_foot_ang_rate"];
    JwLeftFoot = estimationData["left_foot_Jw"];
    dJwdqLeftFoot = estimationData["left_foot_dJwdq"];
    positionLeftFootInWorld = estimationData["left_foot_position"];
    velocityLeftFootInWorld = estimationData["left_foot_velocity"];
    positionLeftToeInWorld = estimationData["left_toe_position"];
    velocityLeftToeInWorld = estimationData["left_toe_velocity"];
    JvLeftToeInWorld = estimationData["left_toe_Jv"];
    dJvdqLeftToeInWorld = estimationData["left_toe_dJvdq"];
    positionLeftHeelInWorld = estimationData["left_heel_position"];
    velocityLeftHeelInWorld = estimationData["left_heel_velocity"];
    JvLeftHeelInWorld = estimationData["left_heel_Jv"];
    dJvdqLeftHeelInWorld = estimationData["left_heel_dJvdq"];
    positionLeftAnkleInWorld = estimationData["left_ankle_position"];
    velocityLeftAnkleInWorld = estimationData["left_ankle_velocity"];
    JvLeftAnkleInWorld = estimationData["left_ankle_Jv"];
    dJvdqLeftAnkleInWorld = estimationData["left_ankle_dJvdq"];

}   

void BruceRobotStatus::updateRobotDCMstatus()
{
    unordered_map<string, MatrixXd> estimationData = MemoryManager::getInstance().ESTIMATOR_STATE->getVal();

    yaw = estimationData["body_yaw_ang"](0,0);
    RotOfYaw = MathTools::Rz(1e-6 + yaw);

    bodyPositionInWorld = estimationData["body_position"];
    bodyRotInWorld =  estimationData["body_rot_matrix"];

    comPositionInWorld = estimationData["com_position"];
    comVelocityInWorld = estimationData["com_velocity"];

    footContacts = estimationData["foot_contacts"];

    positionRightToeInWorld = estimationData["right_toe_position"];
    positionRightHeelInWorld = estimationData["right_heel_position"];
    positionRightFootInWorld = estimationData["right_foot_position"];
    positionRightAnkleInWorld = estimationData["right_ankle_position"];

    positionLeftToeInWorld = estimationData["left_toe_position"];
    positionLeftHeelInWorld = estimationData["left_heel_position"];
    positionLeftFootInWorld = estimationData["left_foot_position"];
    positionLeftAnkleInWorld = estimationData["left_ankle_position"];
}

void BruceRobotStatus::updateSenseStatus()
{
    unordered_map<string, MatrixXd> senseData = MemoryManager::getInstance().SENSE_STATE->getVal();

    imuAccel = senseData["imu_acceleration"];
    imuOmega = senseData["imu_ang_rate"];
    footContacts = senseData["foot_contacts"];
}

void BruceRobotStatus::updatePlanStatus()
{
    unordered_map<string, MatrixXd> planData = MemoryManager::getInstance().PLANNER_COMMAND->getVal();

    mode = int(planData["mode"](0,0));
    phase = int(planData["phase"](0,0));

    bodyPositionInWorldDest = planData["body_position"];
    bodyVelocityInWorldDest = planData["body_velocity"];
    bodyRotInWorldDest =  planData["body_rot_matrix"];
    bodyOmegaInBodyDest = planData["body_ang_rate"];
    comPositionInWorldDest = planData["com_position"];
    comVelocityInWorldDest = planData["com_velocity"];

    footPhaseRight = int(planData["right_foot_phase"](0,0));
    positionRightFootInWorldDest = planData["right_foot_position"];
    velocityRightFootInWorldDest = planData["right_foot_velocity"];
    RotRightFootInWorldDest = planData["right_foot_rot_matrix"];
    omegaRightFootDest = planData["right_foot_ang_rate"];

    footPhaseLeft = int(planData["left_foot_phase"](0,0));
    positionLeftFootInWorldDest = planData["left_foot_position"];
    velocityLeftFootInWorldDest = planData["left_foot_velocity"];
    RotLeftFootInWorldDest = planData["left_foot_rot_matrix"];
    omegaLeftFootDest = planData["left_foot_ang_rate"];
}

void BruceRobotStatus::updateInputStatus()
{
    unordered_map<string, MatrixXd> inputData = MemoryManager::getInstance().USER_COMMAND->getVal();

    modeCmd = int(inputData["mode"](0,0));
    xyVelocityCmd = inputData["com_xy_velocity"];
    yawRateCmd = inputData["yaw_rate"](0,0);
    comPositionChangedCmd = inputData["com_position_change_scaled"];
    eulerAngleChangeCmd = inputData["body_euler_angle_change"];
    RotChange = MathTools::Rx(eulerAngleChangeCmd(0,0)) *
                MathTools::Ry(eulerAngleChangeCmd(1,0)) *
                MathTools::Rz(eulerAngleChangeCmd(2,0));
    yawAngleChangeRightCmd = inputData["right_foot_yaw_angle_change"](0,0);
    yawAngleChangeLeftCmd = inputData["left_foot_yaw_angle_change"](0,0);
    footClearnceCmd = inputData["foot_clearance"](0,0);
    coolingSpeedCmd = inputData["cooling_speed"](0,0);
    dcmOffsetCompensationCmd = inputData["dcm_offset_compensation"];
    comOffsetCompensationCmd = inputData["com_offset_compensation"](0,0);
}

void BruceRobotStatus::updateLegStatus(bool torque)
{
    unordered_map<string, MatrixXd> legData = MemoryManager::getInstance().LEG_STATE->getVal();
    MatrixXd q = legData["joint_positions"];
    MatrixXd dq = legData["joint_velocities"];

    //right leg
    joints["HIP_YAW_R"]["q"] = q(0,0);
    joints["HIP_ROLL_R"]["q"] = q(1,0);
    joints["HIP_PITCH_R"]["q"] = q(2,0);
    joints["KNEE_PITCH_R"]["q"] = q(3,0);
    joints["ANKLE_PITCH_R"]["q"] = q(4,0);

    joints["HIP_YAW_R"]["dq"] = dq(0,0);
    joints["HIP_ROLL_R"]["dq"] = dq(1,0);
    joints["HIP_PITCH_R"]["dq"] = dq(2,0);
    joints["KNEE_PITCH_R"]["dq"] = dq(3,0);
    joints["ANKLE_PITCH_R"]["dq"] = dq(4,0);

    //left leg
    joints["HIP_YAW_L"]["q"] = q(5,0);
    joints["HIP_ROLL_L"]["q"] = q(6,0);
    joints["HIP_PITCH_L"]["q"] = q(7,0);
    joints["KNEE_PITCH_L"]["q"] = q(8,0);
    joints["ANKLE_PITCH_L"]["q"] = q(9,0);

    joints["HIP_YAW_L"]["dq"] = dq(5,0);
    joints["HIP_ROLL_L"]["dq"] = dq(6,0);
    joints["HIP_PITCH_L"]["dq"] = dq(7,0);
    joints["KNEE_PITCH_L"]["dq"] = dq(8,0);
    joints["ANKLE_PITCH_L"]["dq"] = dq(9,0);

    if(torque)
    {
        MatrixXd torques = legData["joint_torques"];
        joints["HIP_YAW_R"]["torque"] = torques(0,0);
        joints["HIP_ROLL_R"]["torque"] = torques(1,0);
        joints["HIP_PITCH_R"]["torque"] = torques(2,0);
        joints["KNEE_PITCH_R"]["torque"] = torques(3,0);
        joints["ANKLE_PITCH_R"]["torque"] = torques(4,0);
        joints["HIP_YAW_L"]["torque"] = torques(5,0);
        joints["HIP_ROLL_L"]["torque"] = torques(6,0);
        joints["HIP_PITCH_L"]["torque"] = torques(7,0);
        joints["KNEE_PITCH_L"]["torque"] = torques(8,0);
        joints["ANKLE_PITCH_L"]["torque"] = torques(9,0);
    }
}

void BruceRobotStatus::setCommandLegPositions()
{
    MatrixXd BearEnable(1,1);
    BearEnable << 1.;

    MatrixXd BearMode(1,1);
    BearMode << BearModes::position;

    MatrixXd goalPositions(10,1);
    goalPositions << joints["HIP_YAW_R"]["q_goal"],
                   joints["HIP_ROLL_R"]["q_goal"],
                   joints["HIP_PITCH_R"]["q_goal"],
                   joints["KNEE_PITCH_R"]["q_goal"],
                   joints["ANKLE_PITCH_R"]["q_goal"],
                   joints["HIP_YAW_L"]["q_goal"],
                   joints["HIP_ROLL_L"]["q_goal"],
                   joints["HIP_PITCH_L"]["q_goal"],
                   joints["KNEE_PITCH_L"]["q_goal"],
                   joints["ANKLE_PITCH_L"]["q_goal"];
    
    unordered_map<string, MatrixXd> commands;
    commands["BEAR_enable"] = BearEnable;
    commands["BEAR_mode"] = BearMode;
    commands["goal_torques"] = goalPositions;

    MemoryManager::getInstance().LEG_COMMAND->setVal(commands);
}


void BruceRobotStatus::setCommandLegTorques()
{
    MatrixXd BearEnable(1,1);
    BearEnable << 1.;

    MatrixXd BearMode(1,1);
    BearMode << BearModes::torque;

    MatrixXd goalTorques(10,1);
    goalTorques << joints["HIP_YAW_R"]["tau_goal"],
                   joints["HIP_ROLL_R"]["tau_goal"],
                   joints["HIP_PITCH_R"]["tau_goal"],
                   joints["KNEE_PITCH_R"]["tau_goal"],
                   joints["ANKLE_PITCH_R"]["tau_goal"],
                   joints["HIP_YAW_L"]["tau_goal"],
                   joints["HIP_ROLL_L"]["tau_goal"],
                   joints["HIP_PITCH_L"]["tau_goal"],
                   joints["KNEE_PITCH_L"]["tau_goal"],
                   joints["ANKLE_PITCH_L"]["tau_goal"];
    
    unordered_map<string, MatrixXd> commands;
    commands["BEAR_enable"] = BearEnable;
    commands["BEAR_mode"] = BearMode;
    commands["goal_torques"] = goalTorques;

    MemoryManager::getInstance().LEG_COMMAND->setVal(commands);
}

void BruceRobotStatus::setCommandLegValues()
{
    MatrixXd BearEnable(1,1);
    BearEnable << 1.;

    MatrixXd BearMode(1,1);
    BearMode << BearModes::force;

    MatrixXd goalTorques(10,1);
    goalTorques << joints["HIP_YAW_R"]["tau_goal"],
                   joints["HIP_ROLL_R"]["tau_goal"],
                   joints["HIP_PITCH_R"]["tau_goal"],
                   joints["KNEE_PITCH_R"]["tau_goal"],
                   joints["ANKLE_PITCH_R"]["tau_goal"],
                   joints["HIP_YAW_L"]["tau_goal"],
                   joints["HIP_ROLL_L"]["tau_goal"],
                   joints["HIP_PITCH_L"]["tau_goal"],
                   joints["KNEE_PITCH_L"]["tau_goal"],
                   joints["ANKLE_PITCH_L"]["tau_goal"];

    MatrixXd goalPositions(10,1);
    goalPositions << joints["HIP_YAW_R"]["q_goal"],
                     joints["HIP_ROLL_R"]["q_goal"],
                     joints["HIP_PITCH_R"]["q_goal"],
                     joints["KNEE_PITCH_R"]["q_goal"],
                     joints["ANKLE_PITCH_R"]["q_goal"],
                     joints["HIP_YAW_L"]["q_goal"],
                     joints["HIP_ROLL_L"]["q_goal"],
                     joints["HIP_PITCH_L"]["q_goal"],
                     joints["KNEE_PITCH_L"]["q_goal"],
                     joints["ANKLE_PITCH_L"]["q_goal"];

    MatrixXd goalVelocities(10,1);
    goalVelocities << joints["HIP_YAW_R"]["dq_goal"],
                      joints["HIP_ROLL_R"]["dq_goal"],
                      joints["HIP_PITCH_R"]["dq_goal"],
                      joints["KNEE_PITCH_R"]["dq_goal"],
                      joints["ANKLE_PITCH_R"]["dq_goal"],
                      joints["HIP_YAW_L"]["dq_goal"],
                      joints["HIP_ROLL_L"]["dq_goal"],
                      joints["HIP_PITCH_L"]["dq_goal"],
                      joints["KNEE_PITCH_L"]["dq_goal"],
                      joints["ANKLE_PITCH_L"]["dq_goal"];
    
    unordered_map<string, MatrixXd> commands;
    commands["BEAR_enable"] = BearEnable;
    commands["BEAR_mode"] = BearMode;
    commands["goal_torques"] = goalTorques;
    commands["goal_positions"] = goalPositions;
    commands["goal_velocities"] = goalVelocities;
    MemoryManager::getInstance().LEG_COMMAND->setVal(commands);
}

void BruceRobotStatus::updateArmStatus()
{
    unordered_map<string, MatrixXd> armData = MemoryManager::getInstance().ARM_STATE->getVal();
    MatrixXd q = armData["joint_positions"];
    MatrixXd dq = armData["joint_velocities"];

    //right arm
    joints["SHOULDER_PITCH_R"]["q"] = q(0,0);
    joints["SHOULDER_ROLL_R"]["q"] = q(1,0);
    joints["ELBOW_YAW_R"]["q"] = q(2,0);

    joints["SHOULDER_PITCH_R"]["dq"] = dq(0,0);
    joints["SHOULDER_ROLL_R"]["dq"] = dq(1,0);
    joints["ELBOW_YAW_R"]["dq"] = dq(2,0);

    //left arm
    joints["SHOULDER_PITCH_L"]["q"] = q(3,0);
    joints["SHOULDER_ROLL_L"]["q"] = q(4,0);
    joints["ELBOW_YAW_L"]["q"] = q(5,0);

    joints["SHOULDER_PITCH_L"]["dq"] = dq(3,0);
    joints["SHOULDER_ROLL_L"]["dq"] = dq(4,0);
    joints["ELBOW_YAW_L"]["dq"] = dq(5,0);
}

void BruceRobotStatus::setCommandArmPositions()
{
    MatrixXd DXLEnable(1,1);
    DXLEnable << 1.;

    MatrixXd DXLMode(1,1);
    DXLMode << DXLModes::dxl_position;

    MatrixXd goalPositions(6,1);
    goalPositions << joints["SHOULDER_PITCH_R"]["q_goal"],
                     joints["SHOULDER_ROLL_R"]["q_goal"],
                     joints["ELBOW_YAW_R"]["q_goal"],
                     joints["SHOULDER_PITCH_L"]["q_goal"],
                     joints["SHOULDER_ROLL_L"]["q_goal"],
                     joints["ELBOW_YAW_L"]["q_goal"];
    
    unordered_map<string, MatrixXd> commands;
    commands["DXL_enable"] = DXLEnable;
    commands["DXL_mode"] = DXLMode;
    commands["goal_positions"] = goalPositions;
    
    MemoryManager::getInstance().ARM_COMMAND->setVal(commands);
}

void BruceRobotStatus::updateGamepadStatus()
{
    unordered_map<string, MatrixXd> gamepadData = MemoryManager::getInstance().GAMEPAD_STATE->getVal();

    gamepad["U"] = gamepadData["U"](0,0);
    gamepad["D"] = gamepadData["D"](0,0);
    gamepad["L"] = gamepadData["L"](0,0);
    gamepad["R"] = gamepadData["R"](0,0);
    gamepad["A"] = gamepadData["A"](0,0);
    gamepad["B"] = gamepadData["B"](0,0);
    gamepad["X"] = gamepadData["X"](0,0);
    gamepad["Y"] = gamepadData["Y"](0,0);
    gamepad["LZ"] = gamepadData["LZ"](0,0);
    gamepad["LS"] = gamepadData["LS"](0,0);
    gamepad["LS2"] = gamepadData["LS2"](0,0);
    gamepad["LSP"] = gamepadData["LSP"](0,0);
    gamepad["LSM"] = gamepadData["LSM"](0,0);
    gamepad["RZ"] = gamepadData["RZ"](0,0);
    gamepad["RS"] = gamepadData["RS"](0,0);
    gamepad["RS2"] = gamepadData["RS2"](0,0);
    gamepad["RSP"] = gamepadData["RSP"](0,0);
    gamepad["RSM"] = gamepadData["RSM"](0,0);
    gamepad["ST"] = gamepadData["ST"](0,0);
    gamepad["BK"] = gamepadData["BK"](0,0);
    gamepad["ALT"] = gamepadData["ALT"](0,0);
    gamepad["FN"] = gamepadData["FN"](0,0);
    gamepad["LX"] = gamepadData["LX"](0,0);
    gamepad["LY"] = gamepadData["LY"](0,0);
    gamepad["RX"] = gamepadData["RX"](0,0);
    gamepad["RY"] = gamepadData["RY"](0,0);
}

void BruceRobotStatus::stopRobot()
{
    MatrixXd BearCommand(1,1);
    BearCommand << 0.;

    unordered_map<string, MatrixXd> BearCommands;
    BearCommands["BEAR_enable"] = BearCommand;
    MemoryManager::getInstance().LEG_COMMAND->setVal(BearCommands);

    MatrixXd DXLCommand(1,1);
    DXLCommand << 0.;

    unordered_map<string, MatrixXd> DXLCommands;
    DXLCommands["DXL_enable"] = DXLCommand;
    MemoryManager::getInstance().ARM_COMMAND->setVal(DXLCommands);
}

void BruceRobotStatus::dampingRobot()
{
    MatrixXd BearCommand(1,1);
    BearCommand << 1.;
    MatrixXd damping(1,1);
    damping << 1.;

    unordered_map<string, MatrixXd> BearCommands;
    BearCommands["BEAR_enable"] = BearCommand;
    BearCommands["damping"] = damping;
    MemoryManager::getInstance().LEG_COMMAND->setVal(BearCommands);
}

bool BruceRobotStatus::isDamping()
{
    unordered_map<string, MatrixXd> legData = MemoryManager::getInstance().LEG_STATE->getVal();
    if(legData["damping"](0,0) == 0.0)
        return false;
    else
        return true;
}

double BruceRobotStatus::getTime()
{
    if(1)//(SIMULATION)  //need to implement a simulation global flag
    {
        unordered_map<string, MatrixXd> simData = MemoryManager::getInstance().SIMULATOR_STATE->getVal();
        return simData["time_stamp"](0,0);
    }
    else
    {
        auto now = chrono::system_clock::now();
        return chrono::duration<double>(now.time_since_epoch()).count();
    }
}

void BruceRobotStatus::sleep(double dt)
{
    if(1) //(SIMULATION) //need to implement a simulation global flag
    {
        double t0 = BruceRobotStatus::getTime();
        while(BruceRobotStatus::getTime() - t0 < dt)
        {
            //do nothing
        }
    }
    else
    {
        this_thread::sleep_for(std::chrono::duration<double>(dt));
    }
}

bool BruceRobotStatus::threadError(double dt)
{
    // not to be implemented yet
}

void BruceRobotStatus::stopThreading()
{
    // not to be implemented yet
}
