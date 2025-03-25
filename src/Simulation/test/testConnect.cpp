#include "GazeboBridge.h"
#include "BruceRobotSimulator.h"
#include "RobotModel.h"
#include "BruceDynamics.h"
#include "BruceRobotKinematics.h"

using namespace std;

int main()
{
    shared_ptr<RobotModel> robotModel = make_shared<RobotModel>(URDF_FILE_PATH, PARAMETER_FILE_PATH);
    shared_ptr<DynamicsInterface> DI = make_shared<BruceInverseDynamics>(robotModel);
    shared_ptr<KinematicsInterface> KI = make_shared<Kinematics>(robotModel);
    BruceRobotSimulator brs(KI, DI);
    //brs.initializeSimulator();
    brs.run();

    // sleep(3);
    int a = 0;
    while(1);
    return 0;
}