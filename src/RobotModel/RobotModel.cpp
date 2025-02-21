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

}

MatrixXd RobotModel::getMatrix(std::string key)
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