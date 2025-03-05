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

        void setSpatialInertials(void);
        void setRelativePosOfJointsLeg(void);


        int getNumJointsPerLeg(){return numJointsPerLeg;}
        std::vector<Eigen::Vector3d> getRelativePosOfJointsLeg(void);
        std::vector<Eigen::MatrixXd> getSpatialInertials(void);



    private:
        urdf::ModelInterfaceSharedPtr model;
        nlohmann::json parameters;

        int numJointsPerLeg;
        std::string baseLinkName;
        std::vector<std::string> rightLegLinks;
        std::vector<std::string> rightLegJoints;
        std::vector<std::string> leftLegLinks;
        std::vector<std::string> leftLegJoints;
        std::vector<std::string> rightArmLinks;
        std::vector<std::string> rightArmJoints;
        std::vector<std::string> leftArmLinks;
        std::vector<std::string> leftArmJoints;

        std::vector<Eigen::MatrixXd> spatialInertials;
        std::vector<Eigen::Vector3d> relativePosOfJoints;

};