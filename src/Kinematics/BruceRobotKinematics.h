#pragma once

#include "KinematicsInterface.h"
#include <memory>
#include "RobotModel.h"
#include <unordered_map>

class Kinematics: public KinematicsInterface{

    public:

        Kinematics(std::shared_ptr<RobotModel>);

        std::vector<double> inverseKinematicsFoot(Eigen::MatrixXd, Eigen::MatrixXd, LEG); 
        std::vector<double> inverseKinematicsAnkle(Eigen::MatrixXd, Eigen::MatrixXd, LEG); 
        std::unordered_map<std::string, Eigen::MatrixXd> forwardKinematicsLeg(std::vector<double>);
        std::unordered_map<std::string, Eigen::MatrixXd> forwardKinematicsRobot(std::unordered_map<std::string, Eigen::MatrixXd>);


    private:
        std::shared_ptr<RobotModel> robotMode;
        //here I just assign fixed data in these parameters
        //later I will set these data automatically based on robot URDF model
        int numJointsPerLeg;
        double hx = 0.029216;
        double hy = 0.075856;
        double hz = 0.039765;
        double d2 = 0.0;
        double a3 = 0.204949;
        double a4 = 0.199881;
        double a5 = 0.024;
        double at = 0.05;
        double ah = 0.035;
        double a6 = ah;
};