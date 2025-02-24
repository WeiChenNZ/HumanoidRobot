#include "BruceRobotKinematics.h"


using namespace std;
using namespace Eigen;

Kinematics::Kinematics(std::shared_ptr<RobotModel> robotModel_):robotMode(robotModel_)
{
    numJointsPerLeg = robotMode->getNumJointsPerLeg();
}



std::vector<KinematicsDataType> Kinematics::inverseKinematicsFoot(Eigen::MatrixXd, Eigen::MatrixXd, LEG)
{

}


std::vector<KinematicsDataType> Kinematics::inverseKinematicsAnkle(Eigen::MatrixXd, Eigen::MatrixXd, LEG)
{

}


std::vector<KinematicsDataType> Kinematics::forwardKinematicsLeg(std::vector<double> jointsPosVel)
{
    if(jointsPosVel.size() != numJointsPerLeg*2*2) //positin and velocity
    {
        cerr<<"Invalid leg configuration!"<<endl;
        // return;
    }

    //q--joint_position, dq--joint_velocity
    vector<double> qRight, qLeft, dqRight, dqLeft;
    for(int i = 0; i < numJointsPerLeg; i++)
    {
        qRight.push_back(jointsPosVel[i]);
        qLeft.push_back(jointsPosVel[i+numJointsPerLeg]);
        dqRight.push_back(jointsPosVel[i+numJointsPerLeg*2]);
        dqLeft.push_back(jointsPosVel[i+numJointsPerLeg*3]);
    }

    //TODO
    //calculate DH model, derivative
}


std::vector<KinematicsDataType> Kinematics::forwardKinematicsRobot(std::vector<KinematicsDataType>)
{

}