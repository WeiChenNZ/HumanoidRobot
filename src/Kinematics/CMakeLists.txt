cmake_minimum_required(VERSION 3.10)

project(Kinematics)



add_executable(TestKin test/testKinematics.cpp BruceRobotKinematics.cpp ../RobotModel/RobotModel.cpp)
target_link_libraries(TestKin urdfdom_model nlohmann_json::nlohmann_json)

