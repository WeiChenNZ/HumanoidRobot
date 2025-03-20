cmake_minimum_required(VERSION 3.10)

project(MemoryManager)

add_executable(MemoryManager test/testMM.cpp MemoryManager.cpp ../Shared_Memory/SharedMemory.cpp)