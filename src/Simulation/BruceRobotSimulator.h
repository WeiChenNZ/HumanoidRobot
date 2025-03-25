#pragma once

#include "GazeboBridge.h"
#include <atomic>
#include <thread>
#include "KinematicsInterface.h"
#include "DynamicsInterface.h"

enum SimModes
{
    TORQUE = 0,
    POSITION = 2
};

class BruceRobotSimulator{

    public:
        BruceRobotSimulator(std::shared_ptr<KinematicsInterface> kinematics_, std::shared_ptr<DynamicsInterface> dynamics_);
        void initializeSimulator(void);

        void run(void);
        void stop(void){threadRunningFlag = false;}
        void mainLoop(void);

        void writePosition(Eigen::MatrixXd, Eigen::MatrixXd);
        void writeTorque(Eigen::MatrixXd, Eigen::MatrixXd);

        Eigen::MatrixXd getArmGoalTorques(Eigen::MatrixXd, Eigen::MatrixXd);
        void updateSensorInfo(void);

        //when robot is running in the simulaton mode
        //we do not use the sensor estimation methods
        //instead, we calculate estimation data based on kinematics and dynamics
        void updateEstimation(void); //calculate_robot_model

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
        Eigen::MatrixXd qArm;
        Eigen::MatrixXd dqArm;
        Eigen::MatrixXd qLeg;
        Eigen::MatrixXd dqLeg;
        Eigen::MatrixXd rotMat;
        Eigen::MatrixXd accel; //3x1
        Eigen::MatrixXd omega; //3x1
        Eigen::MatrixXd footContacts;

        std::shared_ptr<GazeboBridge> gazeboBridge = nullptr;
        int simulationFrequency;
        int simulationMode;

        std::thread t;
        std::atomic<bool> threadRunningFlag;

        std::shared_ptr<KinematicsInterface> kinematics;
        std::shared_ptr<DynamicsInterface> dynamics;

};