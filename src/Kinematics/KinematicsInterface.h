#pragma once

#include <tuple>
#include "Eigen/Core"
#include <variant>


enum LEG{
    RIGHT_LEG = 1,
    LEFT_LEG = -1
};

using KinematicsDataType = std::variant<double, int, Eigen::MatrixXd>;

class KinematicsInterface{

    public:
        virtual std::vector<KinematicsDataType> inverseKinematicsFoot(Eigen::MatrixXd, Eigen::MatrixXd, LEG) = 0; 
        virtual std::vector<KinematicsDataType> inverseKinematicsAnkle(Eigen::MatrixXd, Eigen::MatrixXd, LEG) = 0; 
        virtual std::vector<KinematicsDataType> forwardKinematicsLeg(std::vector<double>) = 0;
        virtual std::vector<KinematicsDataType> forwardKinematicsRobot(std::vector<KinematicsDataType>) = 0;
};