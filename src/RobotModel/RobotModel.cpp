#include "RobotModel.h"
#include <fstream>


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

    
    urdf::Pose joint = model->joints_[rightLegJoints[0]]->parent_to_joint_origin_transform;
    double r,p,y;
    joint.rotation.getRPY(r,p,y);

    int a = 0;

    // cout<<rightArmLinks<<endl;

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