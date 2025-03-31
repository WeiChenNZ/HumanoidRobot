cmake_minimum_required(VERSION 3.10)

project(BruceRobotStatus)


add_executable(testBruceStatus test/testBruceStatus.cpp BruceRobotStatus.cpp ../Shared_Memory/SharedMemory.cpp
               ../MemoryManager/MemoryManager.cpp)