#include "RobotModel.h"

using namespace std;

int main()
{
    string urdfFileName = URDF_FILE_PATH;
    string parameterFileName = PARAMETER_FILE_PATH;

    RobotModel rm(urdfFileName, parameterFileName);

    return 0;
}

