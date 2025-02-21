cmake_minimum_required(VERSION 3.10)

project(RobotModel)

find_package(urdfdom REQUIRED)
find_package(nlohmann_json REQUIRED)


add_executable(RobotTest test/testModel.cpp RobotModel.cpp)
target_link_libraries(RobotTest urdfdom_model nlohmann_json::nlohmann_json)