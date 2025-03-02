#pragma once

#include <tuple>
#include "Eigen/Core"
#include <variant>


enum LEG{
    RIGHT_LEG = 1,
    LEFT_LEG = -1
};


class KinematicsInterface{

    public:
        virtual std::vector<double> inverseKinematicsFoot(Eigen::MatrixXd, Eigen::MatrixXd, LEG) = 0; 
        virtual std::vector<double> inverseKinematicsAnkle(Eigen::MatrixXd, Eigen::MatrixXd, LEG) = 0; 
        virtual std::unordered_map<std::string, Eigen::MatrixXd> forwardKinematicsLeg(std::vector<double>) = 0;
        virtual std::unordered_map<std::string, Eigen::MatrixXd> forwardKinematicsRobot(std::unordered_map<std::string, Eigen::MatrixXd>) = 0;
};