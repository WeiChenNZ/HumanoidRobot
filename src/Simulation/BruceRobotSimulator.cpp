#include "BruceRobotSimulator.h"


using namespace Eigen;
using namespace std;

BruceRobotSimulator::BruceRobotSimulator(void)
{
    numLegs = 2;
    numArms = 2;
    numJointsPerLeg = 5;
    numJointsPerArm = 3;
    numJoints = numLegs*numJointsPerLeg + numArms*numJointsPerArm;
    numContactSensors = 4;

    legP = MatrixXd::Zero(1,numJointsPerLeg);
    legP << 265, 150, 80, 80, 30;

    legI = MatrixXd::Zero(1,numJointsPerLeg);
    
    legD = MatrixXd::Zero(1,numJointsPerLeg);
    legD << 1., 2.3, 0.8, 0.8, 0.003;

    armP = MatrixXd::Zero(1,numJointsPerArm);
    armP << 1.6, 1.6, 1.6;

    armI = MatrixXd::Zero(1,numJointsPerArm);
    
    armD = MatrixXd::Zero(1,numJointsPerArm);
    armD << 0.03, 0.03, 0.03;

    P.resize(1, 2*numJointsPerLeg + 2*numJointsPerArm);
    I.resize(1, 2*numJointsPerLeg + 2*numJointsPerArm);
    D.resize(1, 2*numJointsPerLeg + 2*numJointsPerArm);
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

    MatrixXd initPos(1, numJoints);
    initPos << lr1,lr2,lr3,lr4,lr5,ll1,ll2,ll3,ll4,ll5,ar1,ar2,ar3,al1,al2,al3;

    gazeboBridge->resetSimulation(initPos);

}

void BruceRobotSimulator::writePosition(Eigen::MatrixXd legPos, Eigen::MatrixXd armPos)
{
    MatrixXd goalPos(1, numJoints);
    goalPos << legPos(0,0) + M_PI_2, legPos(0,1) - M_PI_2, legPos(0,2), legPos(0,3), legPos(0,4),
               legPos(0,5) + M_PI_2, legPos(0,6) - M_PI_2, legPos(0,7), legPos(0,8), legPos(0,9),
               armPos(0,0), armPos(0,1), armPos(0,2),
               armPos(0,3), armPos(0,4), armPos(0,5);

    if(simulationMode != SimModes::POSITION)   
    {
        simulationMode = SimModes::POSITION;
        gazeboBridge->setOperatingMode(simulationMode);
    }

    gazeboBridge->setCommandPosition(goalPos);
}

void BruceRobotSimulator::writeTorque(Eigen::MatrixXd legTor, Eigen::MatrixXd armTor)
{
    MatrixXd gaolTor(1, numJoints);
    gaolTor << legTor(0,0), legTor(0,1), legTor(0,2), legTor(0,3), legTor(0,4),
               legTor(0,5), legTor(0,6), legTor(0,7), legTor(0,8), legTor(0,9),
               armTor(0,0), armTor(0,1), armTor(0,2),
               armTor(0,3), armTor(0,4), armTor(0,5);

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
    int i = 0;
    MatrixXd jointsPos;// = gazeboBridge->getJointPosition();

    while(threadRunningFlag)
    {

        jointsPos = gazeboBridge->getJointPosition();
        jointsPos(0,10) += M_PI/180;
        gazeboBridge->setCommandPosition(jointsPos);


        // cout<<"joints pos = "<<jointsPos<<endl;

        i++;
        cout<<"thread is running at "<<i<<" circle"<<endl;
        gazeboBridge->stepSimulation();

        sleep(1);
    }
}

