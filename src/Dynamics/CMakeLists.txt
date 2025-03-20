cmake_minimum_required(VERSION 3.10)

project(Dynamics)

add_executable(TestDyn test/testDynamics.cpp BruceDynamics.cpp ../RobotModel/RobotModel.cpp)
target_link_libraries(TestDyn urdfdom_model nlohmann_json::nlohmann_json)