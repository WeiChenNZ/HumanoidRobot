#pragma once

#include "SharedMemory.h"
#include <string>
#include <memory>

class GazeboBridge{

    public:
        GazeboBridge(std::string name_, int numJoints_, int numContactSensors_ = 1):
        robotName(name_),numJoints(numJoints_),numContactSensors(numContactSensors_)
        {
            worldName = "world";
            dir = "/tmp/";
            timeout = 10;

            initSharedMemory();
            initGazeboClient();
        }

        void initSharedMemory(void);
        void initGazeboClient(void);

        //read methods
        Eigen::MatrixXd getCurrentPosition(void){return JOINT_STATE->getVal()["position"];}
        Eigen::MatrixXd getCurrentVelocity(void){return JOINT_STATE->getVal()["velocity"];}
        Eigen::MatrixXd getCurrentForce(void){return JOINT_STATE->getVal()["force"];}
        Eigen::MatrixXd getCurrentTime(void){return JOINT_STATE->getVal()["time"];}
        Eigen::MatrixXd getBodyPosition(void){return BODY_POSE->getVal()["position"];}
        Eigen::MatrixXd getBodyQuaternion(void){return BODY_POSE->getVal()["quaternion"];}
        Eigen::MatrixXd getBodyRotMat(void);
        Eigen::MatrixXd getBodyEulerAngles(void){return BODY_POSE->getVal()["euler_angles"];}
        Eigen::MatrixXd getBodyVelocity(void){return BODY_POSE->getVal()["velocity"];}
        Eigen::MatrixXd getImuAcceleration(void){return IMU_STATE->getVal()["accel"];}
        Eigen::MatrixXd getImuAngularRate(void){return BODY_POSE->getVal()["ang_rate"];}
        Eigen::MatrixXd getFootContacts(void){return BODY_POSE->getVal()["on"];}
        Eigen::MatrixXd getJointPosition(void){return JOINT_POSITION_COMMAND->getVal()["data"];}
        
        //write method
        void setCommandTorque(Eigen::MatrixXd);
        void setCommandPosition(Eigen::MatrixXd);
        void pausePhysics(void);
        void unpausePhysics(void);
        void stepSimulation(void);
        void resetSimulation(Eigen::MatrixXd);
        void setRealTimeUpdateRate(double);
        void setStepSize(double);
        void setAllPositionPidGains(Eigen::MatrixXd,Eigen::MatrixXd,Eigen::MatrixXd);
        void setJointPositionPidGains(int,double,double,double);
        void setOperatingMode(int);
        void setJointLimits(Eigen::MatrixXd, Eigen::MatrixXd);
        void setTorqueLimits(Eigen::MatrixXd, Eigen::MatrixXd);
        void setBodyForce(Eigen::MatrixXd);
        void setBodyTorque(Eigen::MatrixXd);



    private:
        std::string robotName;
        std::string worldName;
        std::string dir;
        int numJoints;
        int numContactSensors;
        int timeout;
        int worldSocket;
        int robotSocket;
        char recvBuff[1024];

        std::unique_ptr<SharedMemory> WORLD_PARAMETER = nullptr;
        std::unique_ptr<SharedMemory> MODEL_PARAMETER = nullptr;
        std::unique_ptr<SharedMemory> JOINT_STATE = nullptr;
        std::unique_ptr<SharedMemory> JOINT_TORQUE_COMMAND = nullptr;
        std::unique_ptr<SharedMemory> POSITION_PID_GAIN = nullptr;
        std::unique_ptr<SharedMemory> JOINT_POSITION_COMMAND = nullptr;
        std::unique_ptr<SharedMemory> JOINT_LIMIT = nullptr;
        std::unique_ptr<SharedMemory> TORQUE_LIMIT = nullptr;
        std::unique_ptr<SharedMemory> BODY_POSE = nullptr;
        std::unique_ptr<SharedMemory> IMU_STATE = nullptr;
        std::unique_ptr<SharedMemory> LIMB_CONTACT = nullptr;
        std::unique_ptr<SharedMemory> BODY_FORCE = nullptr;
        std::unique_ptr<SharedMemory> BODY_TORQUE = nullptr;

};