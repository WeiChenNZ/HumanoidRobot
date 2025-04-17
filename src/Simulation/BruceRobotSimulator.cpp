#include "BruceRobotSimulator.h"
#include "MemoryManager.h"


using namespace Eigen;
using namespace std;

BruceRobotSimulator::BruceRobotSimulator(std::shared_ptr<KinematicsInterface> kinematics_, std::shared_ptr<DynamicsInterface> dynamics_)
: kinematics(kinematics_), dynamics(dynamics_)
{
    numLegs = 2;
    numArms = 2;
    numJointsPerLeg = 5;
    numJointsPerArm = 3;
    numJoints = numLegs*numJointsPerLeg + numArms*numJointsPerArm;
    numContactSensors = 4;

    legP = MatrixXd::Zero(numJointsPerLeg, 1);
    legP << 265, 150, 80, 80, 30;

    legI = MatrixXd::Zero(numJointsPerLeg, 1);
    
    legD = MatrixXd::Zero(numJointsPerLeg, 1);
    legD << 1., 2.3, 0.8, 0.8, 0.003;

    armP = MatrixXd::Zero(numJointsPerArm, 1);
    armP << 1.6, 1.6, 1.6;

    armI = MatrixXd::Zero(numJointsPerArm, 1);
    
    armD = MatrixXd::Zero(numJointsPerArm, 1);
    armD << 0.03, 0.03, 0.03;

    P.resize(2*numJointsPerLeg + 2*numJointsPerArm, 1);
    I.resize(2*numJointsPerLeg + 2*numJointsPerArm, 1);
    D.resize(2*numJointsPerLeg + 2*numJointsPerArm, 1);
    P << legP, legP, armP, armP;
    I << legI, legI, armI, armI;
    D << legD, legD, armD, armD;

    simulationFrequency = 1000; //Hz
    simulationMode = SimModes::POSITION;  //0 torque ,   2 position

    threadRunningFlag = false;
}

void BruceRobotSimulator::initializeSimulator(void)
{
    gazeboBridge = make_shared<GazeboBridge>("bruce", numJoints, numContactSensors);
    gazeboBridge->setStepSize(1./simulationFrequency);
    gazeboBridge->setOperatingMode(simulationMode);
    gazeboBridge->setAllPositionPidGains(P, I, D);
    // R = -1.5707963267948966, 
    //     1.5861961113210856,
    //     0.5174901606659535, 
    //     -0.9879447774660488, 
    //     0.47045461680009526
    // L = -1.5707963267948966, 
    //     1.5553965422687077,
    //     0.5174901606659535, 
    //     -0.9879447774660488, 
    //     0.47045461680009526

    //get data from origin python program
    //should be updated when I've implemented IK 
    double lr1 = -1.5707963267948966 + M_PI_2;
    double lr2 = 1.5861961113210856 - M_PI_2;
    double lr3 = 0.5174901606659535;
    double lr4 = -0.9879447774660488;
    double lr5 = 0.47045461680009526;

    double ll1 = -1.5707963267948966 + M_PI_2;
    double ll2 = 1.5553965422687077 - M_PI_2;
    double ll3 = 0.5174901606659535;
    double ll4 = -0.9879447774660488;
    double ll5 = 0.47045461680009526;

    //arm positon
    double ar1 = -0.7, ar2 = 1.3, ar3 = 2.0;
    double al1 = 0.7, al2 = -1.3, al3 = -2.0;

    MatrixXd initPos(numJoints, 1);
    initPos << lr1,lr2,lr3,lr4,lr5,ll1,ll2,ll3,ll4,ll5,ar1,ar2,ar3,al1,al2,al3;

    gazeboBridge->resetSimulation(initPos);

}

void BruceRobotSimulator::writePosition(Eigen::MatrixXd legPos, Eigen::MatrixXd armPos)
{
    MatrixXd goalPos(1, numJoints);
    goalPos << legPos(0,0) + M_PI_2, legPos(1,0) - M_PI_2, legPos(2,0), legPos(3,0), legPos(4,0),
               legPos(5,0) + M_PI_2, legPos(6,0) - M_PI_2, legPos(7,0), legPos(8,0), legPos(9,0),
               armPos(0,0), armPos(1,0), armPos(2,0),
               armPos(3,0), armPos(4,0), armPos(5,0);

    if(simulationMode != SimModes::POSITION)   
    {
        simulationMode = SimModes::POSITION;
        gazeboBridge->setOperatingMode(simulationMode);
    }

    gazeboBridge->setCommandPosition(goalPos);
}

void BruceRobotSimulator::writeTorque(Eigen::MatrixXd legTor, Eigen::MatrixXd armTor)
{
    MatrixXd gaolTor(numJoints,1);
    gaolTor << legTor(0,0), legTor(1,0), legTor(2,0), legTor(3,0), legTor(4,0),
               legTor(5,0), legTor(6,0), legTor(7,0), legTor(8,0), legTor(9,0),
               armTor(0,0), armTor(1,0), armTor(2,0),
               armTor(3,0), armTor(4,0), armTor(5,0);

    if(simulationMode != SimModes::TORQUE)
    {
        simulationMode = SimModes::TORQUE;
        gazeboBridge->setOperatingMode(simulationMode);
    }        

    gazeboBridge->setCommandTorque(gaolTor);
}


void BruceRobotSimulator::run(void)
{
    threadRunningFlag = true;
    t = thread(&BruceRobotSimulator::mainLoop, this);
}

void BruceRobotSimulator::mainLoop(void)
{
    MemoryManager::getInstance().init();
    MemoryManager::getInstance().connect();

    /////////////////////////////////
    //Bruce Robot setup
    //TODO

    /////////////////////////////////

    initializeSimulator();
    int i = 0;
    while(threadRunningFlag)
    {

        updateSensorInfo();
        // if(SIMULATION) //need to add new features to support this flag
        {
            updateEstimation();
        }

        unordered_map<string, MatrixXd> legCommand = MemoryManager::getInstance().LEG_COMMAND->getVal();
        unordered_map<string, MatrixXd> armCommand = MemoryManager::getInstance().ARM_COMMAND->getVal();

        if(legCommand["BEAR_enable"](0,0) == 1.0)
        {
            if(legCommand["BEAR_mode"](0,0) == 2)
            {
                //position mode
                writePosition(legCommand["goal_positions"], armCommand["goal_positions"]);
            }
            else if(legCommand["BEAR_mode"](0,0) == 0 || legCommand["BEAR_mode"](0,0) == 3)
            {
                //torque mode
                writeTorque(legCommand["goal_torques"], 
                            getArmGoalTorques(armCommand["goal_positions"], armCommand["goal_velocities"]));
            }
        }
        
        gazeboBridge->stepSimulation();
        cout<<"Sim thread is running at "<<i++<<" circles"<<endl;
        // sleep(1);
    }
}



MatrixXd BruceRobotSimulator::getArmGoalTorques(MatrixXd pos, MatrixXd vel)
{
    MatrixXd armGoalTorque = MatrixXd::Zero(6,1);
    for(int i = 0; i < 6; i++)
    {
        //PD controller
        armGoalTorque(i,0) = armP(i % numJointsPerArm,0)*(pos(i,0) - qArm(i,0)) +
                             armD(i % numJointsPerArm,0)*(vel(i,0) - dqArm(i,0));
    }

    return armGoalTorque;
}

void BruceRobotSimulator::updateSensorInfo(void)
{
    //get sim time
    MemoryManager::getInstance().SIMULATOR_STATE->setVal({{"time_stamp", gazeboBridge->getCurrentTime()}});

    //get joint status
    MatrixXd q = gazeboBridge->getCurrentPosition();
    MatrixXd dq = gazeboBridge->getCurrentVelocity();
    MatrixXd tau = gazeboBridge->getCurrentForce();

    qLeg = MatrixXd::Zero(10,0);
    qLeg << q(0,0) - M_PI_2, q(1,0) + M_PI_2, q(2,0), q(3,0), q(4,0),
            q(5,0) - M_PI_2, q(6,0) + M_PI_2, q(7,0), q(8,0), q(9,0);
    
    qArm = q.block(10,0,6,1);
    dqLeg = dq.block(0,0,10,1);
    dqArm = dq.block(10,0,6,1);

    unordered_map<string, MatrixXd> legData{{"joint_positions", qLeg},
                                            {"joint_velocities", dqLeg},
                                            {"joint_torques", tau.block(0,0,10,1)}};

    unordered_map<string, MatrixXd> armData{{"joint_positions", qArm},
                                            {"joint_velocities", dqArm}};

    MemoryManager::getInstance().LEG_STATE->setVal(legData);
    MemoryManager::getInstance().ARM_STATE->setVal(armData); 

    //get IMU status
    rotMat = gazeboBridge->getBodyRotMat();
    accel = gazeboBridge->getImuAcceleration();
    omega = rotMat.transpose() * gazeboBridge->getImuAngularRate();
    footContacts = gazeboBridge->getFootContacts();

    unordered_map<string, MatrixXd> senseData{{"imu_acceleration",accel},
                                              {"imu_ang_rate", omega},
                                              {"foot_contacts", footContacts}};
    
    MemoryManager::getInstance().SENSE_STATE->setVal(senseData);
}

void BruceRobotSimulator::updateEstimation(void)
{
    vector<double> posAndVel{qLeg(0,0),qLeg(1,0),qLeg(2,0),qLeg(3,0),qLeg(4,0),
                             qLeg(5,0),qLeg(6,0),qLeg(7,0),qLeg(8,0),qLeg(9,0),
                             dqLeg(0,0),dqLeg(1,0),dqLeg(2,0),dqLeg(3,0),dqLeg(4,0),
                             dqLeg(5,0),dqLeg(6,0),dqLeg(7,0),dqLeg(8,0),dqLeg(9,0)};
    // kinematics->forwardKinematicsLeg()
    MatrixXd Rwb = rotMat;
    MatrixXd wbb = omega;
    MatrixXd pwb = gazeboBridge->getBodyPosition();
    MatrixXd vwb = gazeboBridge->getBodyVelocity();
    MatrixXd awb = Rwb * accel;
    MatrixXd vbb = Rwb * vwb;
    double yawAngle = atan2(Rwb(1,0), Rwb(0,0));

    //forward kinematics
    unordered_map<string, MatrixXd> kinResult = kinematics->forwardKinematicsLeg(posAndVel);
    
    //inverse kinematics
    unordered_map<string, MatrixXd> fkInput;
    fkInput["Rotation_Mat"] = Rwb;
    fkInput["Body_Position"] = pwb;
    fkInput["Body_w"] = wbb;
    fkInput["Body_v"] = vbb;

    fkInput["b_right_toe_position"] = kinResult["b_right_toe_position"];
    fkInput["b_right_toe_J"] = kinResult["b_right_toe_J"];
    fkInput["b_right_toe_dJ"] = kinResult["b_right_toe_dJ"];
    fkInput["b_right_heel_position"] = kinResult["b_right_heel_position"];
    fkInput["b_right_heel_J"] = kinResult["b_right_heel_J"];
    fkInput["b_right_heel_dJ"] = kinResult["b_right_heel_dJ"];
    fkInput["b_right_ankle_position"] = kinResult["b_right_ankle_position"];
    fkInput["b_right_ankle_J"] = kinResult["b_right_ankle_J"];
    fkInput["b_right_ankle_dJ"] = kinResult["b_right_ankle_dJ"];
    fkInput["b_right_foot_rot"] = kinResult["b_right_foot_rot"];
    fkInput["b_right_foot_Jw"] = kinResult["b_right_foot_Jw"];
    fkInput["b_right_foot_dJw"] = kinResult["b_right_foot_dJw"];
    
    fkInput["b_left_toe_position"] = kinResult["b_left_toe_position"];
    fkInput["b_left_toe_J"] = kinResult["b_left_toe_J"];
    fkInput["b_left_toe_dJ"] = kinResult["b_left_toe_dJ"];
    fkInput["b_left_heel_position"] = kinResult["b_left_heel_position"];
    fkInput["b_left_heel_J"] = kinResult["b_left_heel_J"];
    fkInput["b_left_heel_dJ"] = kinResult["b_left_heel_dJ"];
    fkInput["b_left_ankle_position"] = kinResult["b_left_ankle_position"];
    fkInput["b_left_ankle_J"] = kinResult["b_left_ankle_J"];
    fkInput["b_left_ankle_dJ"] = kinResult["b_left_ankle_dJ"];
    fkInput["b_left_foot_rot"] = kinResult["b_left_foot_rot"];
    fkInput["b_left_foot_Jw"] = kinResult["b_left_foot_Jw"];
    fkInput["b_left_foot_dJw"] = kinResult["b_left_foot_dJw"];

    fkInput["rd1"] = MatrixXd::Constant(1,1,dqLeg(0,0));
    fkInput["rd2"] = MatrixXd::Constant(1,1,dqLeg(1,0));
    fkInput["rd3"] = MatrixXd::Constant(1,1,dqLeg(2,0));
    fkInput["rd4"] = MatrixXd::Constant(1,1,dqLeg(3,0));
    fkInput["rd5"] = MatrixXd::Constant(1,1,dqLeg(4,0));

    fkInput["ld1"] = MatrixXd::Constant(1,1,dqLeg(5,0));
    fkInput["ld2"] = MatrixXd::Constant(1,1,dqLeg(6,0));
    fkInput["ld3"] = MatrixXd::Constant(1,1,dqLeg(7,0));
    fkInput["ld4"] = MatrixXd::Constant(1,1,dqLeg(8,0));
    fkInput["ld5"] = MatrixXd::Constant(1,1,dqLeg(9,0));

    unordered_map<string, MatrixXd>fkResult = kinematics->forwardKinematicsRobot(fkInput);

    //inverse dynamics
    vector<double> q(posAndVel.begin(), posAndVel.begin() + 10);
    vector<double> dq(posAndVel.begin()+ 10, posAndVel.begin() + 20);
    dynamics->updateInversDynamics(Rwb,pwb,wbb,vbb,q,dq);

    //update estimation data
    unordered_map<string, MatrixXd> estimationData;
    estimationData["time_stamp"] = gazeboBridge->getCurrentTime();
    estimationData["body_position"] = pwb; //3x1
    estimationData["body_velocity"] = vwb; //3x1
    estimationData["body_acceleration"] = awb; //3x1
    estimationData["body_rot_matrix"] = Rwb; //3x3
    estimationData["body_ang_rate"] = wbb; //3x1
    estimationData["body_yaw_ang"] = MatrixXd::Constant(1,1,yawAngle); //1x1
    estimationData["com_position"] = dynamics->getPcom();
    estimationData["com_velocity"] = dynamics->getVcom();
    estimationData["ang_momentum"] = dynamics->getKG();
    estimationData["H_matrix"] = dynamics->getH();
    estimationData["CG_vector"] = dynamics->getCG();
    estimationData["AG_matrix"] = dynamics->getAG();
    estimationData["dAGdq_vector"] = dynamics->getdAGdq();
    estimationData["foot_contacts"] = footContacts;

    estimationData["right_foot_rot_matrix"] = fkResult["right_foot_rot_matrix"];
    estimationData["right_foot_ang_rate"] = fkResult["right_foot_ang_rate"];
    estimationData["right_foot_Jw"] = fkResult["right_foot_Jw"];
    estimationData["right_foot_dJwdq"] = fkResult["right_foot_dJwdq"];
    estimationData["right_foot_position"] = fkResult["right_foot_position"];
    estimationData["right_foot_velocity"] = fkResult["right_foot_velocity"];
    estimationData["right_toe_position"] = fkResult["right_toe_position"];
    estimationData["right_toe_velocity"] = fkResult["right_toe_velocity"];
    estimationData["right_toe_Jv"] = fkResult["right_toe_Jv"];
    estimationData["right_toe_dJvdq"] = fkResult["right_toe_dJvdq"];
    estimationData["right_heel_position"] = fkResult["right_heel_position"];
    estimationData["right_heel_velocity"] = fkResult["right_heel_velocity"];
    estimationData["right_heel_Jv"] = fkResult["right_heel_Jv"];
    estimationData["right_heel_dJvdq"] = fkResult["right_heel_dJvdq"];
    estimationData["right_ankle_position"] = fkResult["right_ankle_position"];
    estimationData["right_ankle_velocity"] = fkResult["right_ankle_velocity"];
    estimationData["right_ankle_Jv"] = fkResult["right_ankle_Jv"];
    estimationData["right_ankle_dJvdq"] = fkResult["right_ankle_dJvdq"];

    estimationData["left_foot_rot_matrix"] = fkResult["left_foot_rot_matrix"];
    estimationData["left_foot_ang_rate"] = fkResult["left_foot_ang_rate"];
    estimationData["left_foot_Jw"] = fkResult["left_foot_Jw"];
    estimationData["left_foot_dJwdq"] = fkResult["left_foot_dJwdq"];
    estimationData["left_foot_position"] = fkResult["left_foot_position"];
    estimationData["left_foot_velocity"] = fkResult["left_foot_velocity"];
    estimationData["left_toe_position"] = fkResult["left_toe_position"];
    estimationData["left_toe_velocity"] = fkResult["left_toe_velocity"];
    estimationData["left_toe_Jv"] = fkResult["left_toe_Jv"];
    estimationData["left_toe_dJvdq"] = fkResult["left_toe_dJvdq"];
    estimationData["left_heel_position"] = fkResult["left_heel_position"];
    estimationData["left_heel_velocity"] = fkResult["left_heel_velocity"];
    estimationData["left_heel_Jv"] = fkResult["left_heel_Jv"];
    estimationData["left_heel_dJvdq"] = fkResult["left_heel_dJvdq"];
    estimationData["left_ankle_position"] = fkResult["left_ankle_position"];
    estimationData["left_ankle_velocity"] = fkResult["left_ankle_velocity"];
    estimationData["left_ankle_Jv"] = fkResult["left_ankle_Jv"];
    estimationData["left_ankle_dJvdq"] = fkResult["left_ankle_dJvdq"];

    MemoryManager::getInstance().ESTIMATOR_STATE->setVal(estimationData);
}

