#include "RobotModel.h"
#include <fstream>
#include "Eigen/Geometry"
#include "MathTools.h"


using namespace std;
using namespace Eigen;

RobotModel::RobotModel(string urdfFileName, string parameterFileName)
{
    ifstream parameterFile(parameterFileName);

    model = urdf::parseURDFFile(urdfFileName);

    if(!model || !parameterFile.is_open())
    {
        cerr<<"failed to open configuration files"<<endl;
        return;
    }

    //set file contents to local virable
    parameterFile >> parameters; 
    baseLinkName = parameters["Base_Link"].get<string>();
    rightArmLinks = parameters["Right_Arm_Links"].get<vector<string>>();
    rightArmJoints = parameters["Right_Arm_Joints"].get<vector<string>>();
    leftArmLinks = parameters["Left_Arm_Links"].get<vector<string>>();
    leftArmJoints = parameters["Left_Arm_Joints"].get<vector<string>>();
    rightLegLinks = parameters["Right_Leg_Links"].get<vector<string>>();
    rightLegJoints = parameters["Right_Leg_Joints"].get<vector<string>>();
    leftLegLinks = parameters["Left_Leg_Links"].get<vector<string>>();
    leftLegJoints = parameters["Left_Leg_Joints"].get<vector<string>>();

    //assume joints on each legs are the same, if the real robot contains different joints per leg
    //this line should be modified
    numJointsPerLeg = leftLegJoints.size() == rightLegJoints.size() ? leftLegJoints.size() : 0;
    if(!numJointsPerLeg) throw runtime_error("error number of joints per leg does not match!\n");

    setSpatialInertials();
    setRelativePosOfJointsLeg();

}

void RobotModel::setSpatialInertials(void)
{
    MatrixXd spatialInertial(6,6);
    MatrixXd I3 = MatrixXd::Identity(3,3);

    //base link
    {
        string linkName = baseLinkName;
        double x =  model->links_[linkName]->inertial->origin.rotation.x;
        double y =  model->links_[linkName]->inertial->origin.rotation.y;
        double z =  model->links_[linkName]->inertial->origin.rotation.z;
        double w =  model->links_[linkName]->inertial->origin.rotation.w;
        Quaterniond q(w, x,y,z);
        MatrixXd R = q.toRotationMatrix();

        double px = model->links_[linkName]->inertial->origin.position.x;
        double py = model->links_[linkName]->inertial->origin.position.y;
        double pz = model->links_[linkName]->inertial->origin.position.z;
        Vector3d t(px,py,pz);

        double ixx = model->links_[linkName]->inertial->ixx;
        double ixy = model->links_[linkName]->inertial->ixy;
        double ixz = model->links_[linkName]->inertial->ixz;
        double iyy = model->links_[linkName]->inertial->iyy;
        double iyz = model->links_[linkName]->inertial->iyz;
        double izz = model->links_[linkName]->inertial->izz;

        MatrixXd I = MatrixXd::Zero(3,3);
        I << ixx, ixy, ixz,
             ixy, iyy, iyz,
             ixz, iyz, izz;
        
        double m = model->links_[linkName]->inertial->mass;

        //calculate Spatial Intertial based on algorithm:
        // Ga = Ad(Tba).T * Gb * Ad(Tba)
        spatialInertial.block(0,0,3,3) = R*I*R.transpose() - MathTools::skewMatrix(t)*R*m*I3*R.transpose()*MathTools::skewMatrix(t);
        spatialInertial.block(0,3,3,3) = MathTools::skewMatrix(t)*R*m*I3*R.transpose();
        spatialInertial.block(3,0,3,3) = -R*m*I3*R.transpose()*MathTools::skewMatrix(t);
        spatialInertial.block(3,3,3,3) = R*m*I3*R.transpose();
 
        spatialInertials.push_back(spatialInertial);
    }

    //right leg
    for(auto linkName: rightLegLinks)
    {
        double x =  model->links_[linkName]->inertial->origin.rotation.x;
        double y =  model->links_[linkName]->inertial->origin.rotation.y;
        double z =  model->links_[linkName]->inertial->origin.rotation.z;
        double w =  model->links_[linkName]->inertial->origin.rotation.w;
        Quaterniond q(w, x,y,z);
        MatrixXd R = q.toRotationMatrix();

        double px = model->links_[linkName]->inertial->origin.position.x;
        double py = model->links_[linkName]->inertial->origin.position.y;
        double pz = model->links_[linkName]->inertial->origin.position.z;
        Vector3d t(px,py,pz);

        double ixx = model->links_[linkName]->inertial->ixx;
        double ixy = model->links_[linkName]->inertial->ixy;
        double ixz = model->links_[linkName]->inertial->ixz;
        double iyy = model->links_[linkName]->inertial->iyy;
        double iyz = model->links_[linkName]->inertial->iyz;
        double izz = model->links_[linkName]->inertial->izz;

        MatrixXd I = MatrixXd::Zero(3,3);
        I << ixx, ixy, ixz,
             ixy, iyy, iyz,
             ixz, iyz, izz;
        
        double m = model->links_[linkName]->inertial->mass;

        //calculate Spatial Intertial based on algorithm:
        // Ga = Ad(Tba).T * Gb * Ad(Tba)
        spatialInertial.block(0,0,3,3) = R*I*R.transpose() - MathTools::skewMatrix(t)*R*m*I3*R.transpose()*MathTools::skewMatrix(t);
        spatialInertial.block(0,3,3,3) = MathTools::skewMatrix(t)*R*m*I3*R.transpose();
        spatialInertial.block(3,0,3,3) = -R*m*I3*R.transpose()*MathTools::skewMatrix(t);
        spatialInertial.block(3,3,3,3) = R*m*I3*R.transpose();
 
        spatialInertials.push_back(spatialInertial);
    }
    //left Leg    
    for(auto linkName: leftLegLinks)
    {
        double x =  model->links_[linkName]->inertial->origin.rotation.x;
        double y =  model->links_[linkName]->inertial->origin.rotation.y;
        double z =  model->links_[linkName]->inertial->origin.rotation.z;
        double w =  model->links_[linkName]->inertial->origin.rotation.w;
        Quaterniond q(w, x,y,z);
        MatrixXd R = q.toRotationMatrix();

        double px = model->links_[linkName]->inertial->origin.position.x;
        double py = model->links_[linkName]->inertial->origin.position.y;
        double pz = model->links_[linkName]->inertial->origin.position.z;
        Vector3d t(px,py,pz);

        double ixx = model->links_[linkName]->inertial->ixx;
        double ixy = model->links_[linkName]->inertial->ixy;
        double ixz = model->links_[linkName]->inertial->ixz;
        double iyy = model->links_[linkName]->inertial->iyy;
        double iyz = model->links_[linkName]->inertial->iyz;
        double izz = model->links_[linkName]->inertial->izz;

        MatrixXd I = MatrixXd::Zero(3,3);
        I << ixx, ixy, ixz,
             ixy, iyy, iyz,
             ixz, iyz, izz;
        
        double m = model->links_[linkName]->inertial->mass;

        //calculate Spatial Intertial based on algorithm:
        // Ga = Ad(Tba).T * Gb * Ad(Tba)
        spatialInertial.block(0,0,3,3) = R*I*R.transpose() - MathTools::skewMatrix(t)*R*m*I3*R.transpose()*MathTools::skewMatrix(t);
        spatialInertial.block(0,3,3,3) = MathTools::skewMatrix(t)*R*m*I3*R.transpose();
        spatialInertial.block(3,0,3,3) = -R*m*I3*R.transpose()*MathTools::skewMatrix(t);
        spatialInertial.block(3,3,3,3) = R*m*I3*R.transpose();
 
        spatialInertials.push_back(spatialInertial);
    }
}

void RobotModel::setRelativePosOfJointsLeg(void)
{
    Vector3d pos;
    for(auto jointNameLeg:rightLegJoints)
    {
        double x = model->joints_[jointNameLeg]->parent_to_joint_origin_transform.position.x;
        double y = model->joints_[jointNameLeg]->parent_to_joint_origin_transform.position.y;
        double z = model->joints_[jointNameLeg]->parent_to_joint_origin_transform.position.z;
        pos << x,y,z;
        relativePosOfJoints.push_back(pos);
    }
    for(auto jointNameLeg:leftLegJoints)
    {
        double x = model->joints_[jointNameLeg]->parent_to_joint_origin_transform.position.x;
        double y = model->joints_[jointNameLeg]->parent_to_joint_origin_transform.position.y;
        double z = model->joints_[jointNameLeg]->parent_to_joint_origin_transform.position.z;
        pos << x,y,z;
        relativePosOfJoints.push_back(pos);
    }
}

MatrixXd RobotModel::getMatrixFromJson(std::string key)
{
    auto matrixData = parameters[key].get<vector<vector<double>>>();

    int rows = matrixData.size();
    int cols = matrixData[0].size();
    MatrixXd mat(rows, cols);

    for(int i = 0; i < rows; i++)
        for(int j = 0; j < cols; j++)
            mat(i,j) = matrixData[i][j];
    
    return mat;
}

MatrixXd RobotModel::getVectorFromJson(std::string key)
{
    auto matrixData = parameters[key].get<vector<double>>();

    int len = matrixData.size();

    MatrixXd mat(1, len);

    for(int i = 0; i < len; i++)
            mat(0,i) = matrixData[i];
    
    return mat;
}

std::vector<Eigen::Vector3d> RobotModel::getRelativePosOfJointsLeg(void)
{
    return relativePosOfJoints;
}

std::vector<Eigen::MatrixXd> RobotModel::getSpatialInertials(void)
{
    return spatialInertials;
}