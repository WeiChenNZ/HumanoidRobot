cmake_minimum_required(VERSION 3.10)

add_executable(Gaztest ./test/testConnect.cpp GazeboBridge.cpp BruceRobotSimulator.cpp ../Shared_Memory/SharedMemory.cpp
                       ../Kinematics/BruceRobotKinematics.cpp ../Dynamics/BruceDynamics.cpp ../MemoryManager/MemoryManager.cpp 
                       ../RobotModel/RobotModel.cpp)

target_link_libraries(Gaztest urdfdom_model nlohmann_json::nlohmann_json)