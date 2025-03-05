#include "RobotModel.h"

using namespace std;
using namespace Eigen;

int main()
{
    string urdfFileName = URDF_FILE_PATH;
    string parameterFileName = PARAMETER_FILE_PATH;

    RobotModel rm(urdfFileName, parameterFileName);

    // auto relPos = rm.getRelativePosOfJointsLeg();

    // int i = 0;
    // for(auto a:relPos)
    // {
    //     cout<<"r_"<<i<<"_"<<i+1<<" = "<<a[0]<<" "<<a[1]<<" "<<a[2]<<endl;
    //     i++;
    // }

    int i = 0;
    auto inertials = rm.getSpatialInertials();
    for (auto a:inertials)
    {
        // cout<<"body "<<i<<"'s Spatial Inertial is:"<<endl
        //     <<a<<endl;
        // i++;
    }

    

    
    // cout<<"joints number = "<<rm.model->joints_.size()<<endl;

    // cout<<"param name  = "<<rm.parameters["name"]<<endl;


    return 0;
}

