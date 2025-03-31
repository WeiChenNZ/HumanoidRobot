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
