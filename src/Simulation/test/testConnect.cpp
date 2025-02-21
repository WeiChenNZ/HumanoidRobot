#include "gazeboBridge.h"
#include "BruceRobotSimulator.h"


int main()
{
    // GazeboBridge gb("bruce", 5);
    BruceRobotSimulator brs;
    brs.initializeSimulator();
    brs.run();

    // sleep(3);
    int a = 0;
    while(1);
    return 0;
}