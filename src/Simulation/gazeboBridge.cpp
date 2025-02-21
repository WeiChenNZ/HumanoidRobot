#include "gazeboBridge.h"
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <Eigen/Geometry>  // For Quaternion

using namespace std;
using namespace Eigen;

void GazeboBridge::initSharedMemory(void)
{
    WORLD_PARAMETER = make_unique<SharedMemory>("GAZ", "WORLD_PARAMS", false);
    WORLD_PARAMETER->addBlock("data", WorldParameters());

    MODEL_PARAMETER = make_unique<SharedMemory>(robotName, "MODEL_PARAMS", false);
    MODEL_PARAMETER->addBlock("data", ModelParameters());

    JOINT_STATE = make_unique<SharedMemory>(robotName, "STATUS", false);
    JOINT_STATE->addBlock("time", MatrixXd::Zero(1, 1));
    JOINT_STATE->addBlock("position", MatrixXd::Zero(1, numJoints));
    JOINT_STATE->addBlock("velocity", MatrixXd::Zero(1, numJoints));
    JOINT_STATE->addBlock("force", MatrixXd::Zero(1, numJoints));

    JOINT_TORQUE_COMMAND = make_unique<SharedMemory>(robotName, "FORCE_COMMS", false);
    JOINT_TORQUE_COMMAND->addBlock("data", MatrixXd::Zero(1, numJoints));

    POSITION_PID_GAIN = make_unique<SharedMemory>(robotName, "PID_GAINS", false);
    POSITION_PID_GAIN->addBlock("data", MatrixXd::Zero(1, 3*numJoints));

    JOINT_POSITION_COMMAND = make_unique<SharedMemory>(robotName, "POS_COMMS", false);
    JOINT_POSITION_COMMAND->addBlock("data", MatrixXd::Zero(1, numJoints));

    JOINT_LIMIT = make_unique<SharedMemory>(robotName, "JOINT_LIMITS", false);
    JOINT_LIMIT->addBlock("data", MatrixXd::Zero(1, 2*numJoints));

    TORQUE_LIMIT = make_unique<SharedMemory>(robotName, "EFFORT_LIMITS", false);
    TORQUE_LIMIT->addBlock("data", MatrixXd::Zero(1, 2*numJoints));

    BODY_POSE = make_unique<SharedMemory>(robotName, "BODY_POSE", false);
    BODY_POSE->addBlock("time", MatrixXd::Zero(1, 1));
    BODY_POSE->addBlock("position", MatrixXd::Zero(1, 3));
    BODY_POSE->addBlock("quaternion", MatrixXd::Zero(1, 4));
    BODY_POSE->addBlock("euler_angles", MatrixXd::Zero(1, 3));
    BODY_POSE->addBlock("velocity", MatrixXd::Zero(1, 3));
    
    IMU_STATE = make_unique<SharedMemory>(robotName, "IMU_STATES", false);
    IMU_STATE->addBlock("time", MatrixXd::Zero(1, 1));
    IMU_STATE->addBlock("accel", MatrixXd::Zero(1, 3));
    IMU_STATE->addBlock("ang_rate", MatrixXd::Zero(1, 3));

    LIMB_CONTACT = make_unique<SharedMemory>(robotName, "LIMB_CONTACTS", false);
    LIMB_CONTACT->addBlock("on", MatrixXd::Zero(1, numContactSensors));

    BODY_FORCE = make_unique<SharedMemory>(robotName, "BODY_FORCE", false);
    BODY_FORCE->addBlock("force", MatrixXd::Zero(1, 3));

    BODY_TORQUE = make_unique<SharedMemory>(robotName, "BODY_TORQUE", false);
    BODY_TORQUE->addBlock("torque", MatrixXd::Zero(1, 3));

    try
    {
        cout<<"enter into try block"<<endl;
        //if shared memory exist, just need to link them
        WORLD_PARAMETER->connectSegment(WORLD);
        MODEL_PARAMETER->connectSegment(MODEL);
        JOINT_STATE->connectSegment();
        JOINT_TORQUE_COMMAND->connectSegment();
        POSITION_PID_GAIN->connectSegment();
        JOINT_POSITION_COMMAND->connectSegment();
        JOINT_LIMIT->connectSegment();
        TORQUE_LIMIT->connectSegment();
        BODY_POSE->connectSegment();
        IMU_STATE->connectSegment();
        LIMB_CONTACT->connectSegment();
        BODY_FORCE->connectSegment();
        BODY_TORQUE->connectSegment();
    }
    catch(const std::exception& e)
    {
        cout<<"enter into catch block"<<endl;
        //if shard memory does not exist, then create new
        WORLD_PARAMETER->setInitTrue();
        MODEL_PARAMETER->setInitTrue();
        JOINT_STATE->setInitTrue();
        JOINT_TORQUE_COMMAND->setInitTrue();
        POSITION_PID_GAIN->setInitTrue();
        JOINT_POSITION_COMMAND->setInitTrue();
        JOINT_LIMIT->setInitTrue();
        TORQUE_LIMIT->setInitTrue();
        BODY_POSE->setInitTrue();
        IMU_STATE->setInitTrue();
        LIMB_CONTACT->setInitTrue();
        BODY_FORCE->setInitTrue();
        BODY_TORQUE->setInitTrue();

        WORLD_PARAMETER->connectSegment(WORLD);
        MODEL_PARAMETER->connectSegment(MODEL);
        JOINT_STATE->connectSegment();
        JOINT_TORQUE_COMMAND->connectSegment();
        POSITION_PID_GAIN->connectSegment();
        JOINT_POSITION_COMMAND->connectSegment();
        JOINT_LIMIT->connectSegment();
        TORQUE_LIMIT->connectSegment();
        BODY_POSE->connectSegment();
        IMU_STATE->connectSegment();
        LIMB_CONTACT->connectSegment();
        BODY_FORCE->connectSegment();
        BODY_TORQUE->connectSegment();
    }
    

}

void GazeboBridge::initGazeboClient(void)
{
    bool connected = false, worldConnected = false, robotConnected = false;
    sockaddr_un worldAddr{}, robotAddr{};

    worldAddr.sun_family = AF_UNIX;
    robotAddr.sun_family = AF_UNIX;
    string worldAddress = dir + worldName;
    string robotAddress = dir + robotName;
    strncpy(worldAddr.sun_path, worldAddress.c_str(), worldAddress.length());
    strncpy(robotAddr.sun_path, robotAddress.c_str(), robotAddress.length());

    int timeoutCnt = 0;
    while(!connected && timeoutCnt < timeout)
    {
        //world
        worldSocket = socket(AF_UNIX, SOCK_STREAM, 0);
        if(worldSocket == -1)
        {
            cout<<"world connection socket open error!"<<endl;
            worldConnected = false;
        }
        else worldConnected = true;

        if(connect(worldSocket, reinterpret_cast<struct sockaddr*>(&worldAddr), sizeof(worldAddr)) == -1)
        {
            cout<<"world connection connecting error!"<<endl;
            worldConnected = false;
        }
        else worldConnected = true;

        //robot
        robotSocket = socket(AF_UNIX, SOCK_STREAM, 0);
        if(robotSocket == -1)
        {
            cout<<"robot connection socket open error!"<<endl;
            robotConnected = false;
        }
        else robotConnected = true;
        
        if(connect(robotSocket, reinterpret_cast<struct sockaddr*>(&robotAddr), sizeof(robotAddr)) == -1)
        {
            cout<<"robot connection connecting error!"<<endl;
            robotConnected = false;
        }
        else robotConnected = true;

        if(robotConnected && worldConnected) 
            connected = true;
        else
        {
            timeoutCnt++;
            connected = false;
            cout<<"Gazebo connecting error at time "<<timeoutCnt<<" retry..."<<endl;
            sleep(1);
        }
    }

    if(connected) cout<<"Gazebo connecting successful !"<<endl;
    else cout<<"Gazebo connecting fails !"<<endl;
}

Eigen::MatrixXd GazeboBridge::getBodyRotMat(void)
{
    Eigen::MatrixXd bq = getBodyQuaternion();
    Eigen::Quaterniond q(bq(0), bq(1), bq(2), bq(3));
    Eigen::MatrixXd rotationMatrix = q.toRotationMatrix();
    return rotationMatrix;
}

void GazeboBridge::setCommandTorque(Eigen::MatrixXd force)
{
    unordered_map<string, MatrixXd> data = {{"data", force}};
    JOINT_TORQUE_COMMAND->setVal(data);
}

void GazeboBridge::setCommandPosition(Eigen::MatrixXd position)
{
    unordered_map<string, MatrixXd> data = {{"data", position}};
    JOINT_POSITION_COMMAND->setVal(data);
}

void GazeboBridge::pausePhysics(void)
{
    const char* cmd = "pause_physics";
    send(worldSocket, cmd, strlen(cmd), 0);
    recv(worldSocket, recvBuff, sizeof(recvBuff), 0);
}

void GazeboBridge::unpausePhysics(void)
{
    const char* cmd = "unpause_physics";
    send(worldSocket, cmd, strlen(cmd), 0);
    recv(worldSocket, recvBuff, sizeof(recvBuff), 0);    
}

void GazeboBridge::stepSimulation(void)
{
    const char* cmd = "step_simulation";
    send(worldSocket, cmd, strlen(cmd), 0);
    recv(worldSocket, recvBuff, sizeof(recvBuff), 0); 
}

void GazeboBridge::resetSimulation(Eigen::MatrixXd initPos)
{
    if(initPos != MatrixXd::Zero(1, numJoints))
    {
        setCommandPosition(initPos);
    }
    else
    {
        setCommandPosition(MatrixXd::Zero(1, numJoints));
    }
    const char* cmd = "reset_simulation";
    int num = send(worldSocket, cmd, strlen(cmd), 0);
    int rv = recv(worldSocket, recvBuff, sizeof(recvBuff), 0); 
    int a = 0;
}

void GazeboBridge::setRealTimeUpdateRate(double rate)
{
    unordered_map<string, WorldParameters> data = WORLD_PARAMETER->getValWorld();
    data["data"].real_time_update_rate = rate;
    WORLD_PARAMETER->setVal(data);

    const char* cmd = "update_world_parameters";
    send(worldSocket, cmd, strlen(cmd), 0);
    recv(worldSocket, recvBuff, sizeof(recvBuff), 0);     
}

void GazeboBridge::setStepSize(double stepSize)
{
    unordered_map<string, WorldParameters> data = WORLD_PARAMETER->getValWorld();
    data["data"].step_size = stepSize;
    WORLD_PARAMETER->setVal(data);

    const char* cmd = "update_world_parameters";
    send(worldSocket, cmd, strlen(cmd), 0);
    recv(worldSocket, recvBuff, sizeof(recvBuff), 0);       
}

void GazeboBridge::setAllPositionPidGains(Eigen::MatrixXd p,Eigen::MatrixXd i,Eigen::MatrixXd d)
{
    MatrixXd pid(1, 3*numJoints); //3*16 on bruce
    for(int j = 0; j < p.size(); j++)
    {
        pid(0, 3*j)     = p(0, j);
        pid(0, 3*j + 1) = i(0, j);
        pid(0, 3*j + 2) = d(0, j);
    }
    unordered_map<string, MatrixXd> data = {{"data", pid}};
    POSITION_PID_GAIN->setVal(data);

    const char* cmd = "set_position_pid_gains";
    send(robotSocket, cmd, strlen(cmd), 0);
    recv(robotSocket, recvBuff, sizeof(recvBuff), 0);         
}

void GazeboBridge::setJointPositionPidGains(int index,double p,double i,double d)
{
    unordered_map<string, MatrixXd> data = POSITION_PID_GAIN->getVal();
    data["data"](0, 3*index)     = p;
    data["data"](0, 3*index + 1) = i;
    data["data"](0, 3*index + 2) = d;
    POSITION_PID_GAIN->setVal(data);

    const char* cmd = "set_position_pid_gains";
    send(robotSocket, cmd, strlen(cmd), 0);
    recv(robotSocket, recvBuff, sizeof(recvBuff), 0);       
}

void GazeboBridge::setOperatingMode(int mode)
{
    unordered_map<string, ModelParameters> data = MODEL_PARAMETER->getValModel();
    data["data"].operating_mode = mode;
    MODEL_PARAMETER->setVal(data);

    const char* cmd = "update_model_parameters";
    send(robotSocket, cmd, strlen(cmd), 0);
    recv(robotSocket, recvBuff, sizeof(recvBuff), 0);       
}

void GazeboBridge::setJointLimits(MatrixXd lower, MatrixXd upper)
{
    unordered_map<string, MatrixXd> data = {{"data", MatrixXd::Zero(1, 2*numJoints)}};
    for(int i = 0; i < lower.size(); i++)
    {
        data["data"](0, 2*i)     = lower(0,i);
        data["data"](0, 2*i + 1) = upper(0, i);
    }
    JOINT_LIMIT->setVal(data);

    const char* cmd = "set_joint_limits";
    send(robotSocket, cmd, strlen(cmd), 0);
    recv(robotSocket, recvBuff, sizeof(recvBuff), 0);     
}

void GazeboBridge::setTorqueLimits(MatrixXd lower, MatrixXd upper)
{
    unordered_map<string, MatrixXd> data = {{"data", MatrixXd::Zero(1, 2*numJoints)}};
    for(int i = 0; i < lower.size(); i++)
    {
        data["data"](0, 2*i)     = lower(0,i);
        data["data"](0, 2*i + 1) = upper(0, i);
    }
    TORQUE_LIMIT->setVal(data);

    const char* cmd = "set_effort_limits";
    send(robotSocket, cmd, strlen(cmd), 0);
    recv(robotSocket, recvBuff, sizeof(recvBuff), 0);   
}

void GazeboBridge::setBodyForce(Eigen::MatrixXd force)
{
    unordered_map<string, MatrixXd> data = {{"force", force}};
    BODY_FORCE->setVal(data);

    const char* cmd = "set_body_force";
    send(robotSocket, cmd, strlen(cmd), 0);
    recv(robotSocket, recvBuff, sizeof(recvBuff), 0);      
}

void GazeboBridge::setBodyTorque(Eigen::MatrixXd torque)
{
    unordered_map<string, MatrixXd> data = {{"force", torque}};
    BODY_TORQUE->setVal(data);

    const char* cmd = "set_body_torque";
    send(robotSocket, cmd, strlen(cmd), 0);
    recv(robotSocket, recvBuff, sizeof(recvBuff), 0);      
}