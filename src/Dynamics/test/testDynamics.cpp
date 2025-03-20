
#include "BruceDynamics.h"

using namespace std;

int main()
{
    shared_ptr<RobotModel> robotModel = make_shared<RobotModel>(URDF_FILE_PATH, PARAMETER_FILE_PATH);
    unique_ptr<DynamicsInterface> dIF = make_unique<BruceInverseDynamics>(robotModel);
    return 0;
}