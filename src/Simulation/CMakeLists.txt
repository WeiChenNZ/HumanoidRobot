cmake_minimum_required(VERSION 3.10)

add_executable(Gaztest ./test/testConnect.cpp GazeboBridge.cpp BruceRobotSimulator.cpp ../Shared_Memory/SharedMemory.cpp)