#include "RobotModel.h"

using namespace std;
using namespace Eigen;

int main()
{
    string urdfFileName = URDF_FILE_PATH;
    string parameterFileName = PARAMETER_FILE_PATH;

    RobotModel rm(urdfFileName, parameterFileName);

    cout<<"vector = "<<rm.getVectorFromJson("vec4")<<endl;

    
    // cout<<"joints number = "<<rm.model->joints_.size()<<endl;

    // cout<<"param name  = "<<rm.parameters["name"]<<endl;


    return 0;
}

