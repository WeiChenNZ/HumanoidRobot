
#include "BruceRobotStatus.h"
#include "Eigen/Core"

using namespace Eigen;
int main()
{
    BruceRobotStatus brs;
    brs.comPositionInWorld = MatrixXd::Zero(3,1);
    return 0;
}