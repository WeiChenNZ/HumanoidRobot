#include "BruceRobotStatus.h"
#include "MemoryManager.h"
#include "MathTools.h"

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

void BruceRobotStatus::updateLegStatus()
{
    unordered_map<string, MatrixXd> legData = MemoryManager::getInstance().LEG_STATE->getVal();
    MatrixXd q = legData["joint_positions"];
    MatrixXd dq = legData["joint_velocities"];

    //right leg
    joints["HIP_YAW_R"]["q"] = q(0,0);

}
