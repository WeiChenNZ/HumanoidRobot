#pragma once
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <Eigen/Core>

class RobotModel{

    public:
        RobotModel(std::string, std::string);
        Eigen::MatrixXd getMatrix(std::string);



    private:
        urdf::ModelInterfaceSharedPtr model;
        nlohmann::json parameters;

};