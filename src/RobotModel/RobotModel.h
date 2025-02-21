#pragma once
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <iostream>
#include <nlohmann/json.hpp>

class RobotModel{

    public:
        RobotModel(std::string, std::string);


    private:
        // urdf::ModelInterface model;
        urdf::ModelInterfaceSharedPtr model;


};