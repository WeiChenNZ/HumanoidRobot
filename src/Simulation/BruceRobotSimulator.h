#pragma once

#include "GazeboBridge.h"
#include <atomic>
#include <thread>

enum SimModes
{
    TORQUE = 0,
    POSITION = 2
};

class BruceRobotSimulator{

    public:
        BruceRobotSimulator(void);
        void initializeSimulator(void);

        void run(void);
        void stop(void){threadRunningFlag = false;}
        void mainLoop(void);

        void writePosition(Eigen::MatrixXd, Eigen::MatrixXd);
        void writeTorque(Eigen::MatrixXd, Eigen::MatrixXd);

        ~BruceRobotSimulator()
        {
            stop();
            if(t.joinable())
            {
                t.join();
                std::cout<<"Simulation Thread Stopped!"<<std::endl;
            }
        }

    private:
        int numLegs;
        int numJointsPerLeg;
        int numArms;
        int numJointsPerArm;
        int numJoints;
        int numContactSensors;
        Eigen::MatrixXd legP;
        Eigen::MatrixXd legI;
        Eigen::MatrixXd legD;
        Eigen::MatrixXd armP;
        Eigen::MatrixXd armI;
        Eigen::MatrixXd armD;
        Eigen::MatrixXd P;
        Eigen::MatrixXd I;
        Eigen::MatrixXd D;

        std::shared_ptr<GazeboBridge> gazeboBridge = nullptr;
        int simulationFrequency;
        int simulationMode;

        std::thread t;
        std::atomic<bool> threadRunningFlag;


};