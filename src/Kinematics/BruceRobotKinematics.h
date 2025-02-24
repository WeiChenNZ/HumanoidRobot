#pragma once

#include "KinematicsInterface.h"
#include <memory>
#include "RobotModel.h"

class Kinematics: public KinematicsInterface{

    public:

        Kinematics(std::shared_ptr<RobotModel>);

        std::vector<KinematicsDataType> inverseKinematicsFoot(Eigen::MatrixXd, Eigen::MatrixXd, LEG); 
        std::vector<KinematicsDataType> inverseKinematicsAnkle(Eigen::MatrixXd, Eigen::MatrixXd, LEG); 
        std::vector<KinematicsDataType> forwardKinematicsLeg(std::vector<double>);
        std::vector<KinematicsDataType> forwardKinematicsRobot(std::vector<KinematicsDataType>);


    private:
        std::shared_ptr<RobotModel> robotMode;
        int numJointsPerLeg;
};