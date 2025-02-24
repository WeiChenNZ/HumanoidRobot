#pragma once
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <Eigen/Core>

class RobotModel{

    public:
        RobotModel(std::string, std::string);

        Eigen::MatrixXd getMatrixFromJson(std::string);
        Eigen::MatrixXd getVectorFromJson(std::string);


        int getNumJointsPerLeg(){return numJointsPerLeg;}

    private:
        urdf::ModelInterfaceSharedPtr model;
        nlohmann::json parameters;

        int numJointsPerLeg;
        std::vector<std::string> rightLegLinks;
        std::vector<std::string> rightLegJoints;
        std::vector<std::string> leftLegLinks;
        std::vector<std::string> leftLegJoints;
        std::vector<std::string> rightArmLinks;
        std::vector<std::string> rightArmJoints;
        std::vector<std::string> leftArmLinks;
        std::vector<std::string> leftArmJoints;

};