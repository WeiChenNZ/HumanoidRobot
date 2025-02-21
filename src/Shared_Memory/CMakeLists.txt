cmake_minimum_required(VERSION 3.10)


add_executable(ShmTest SharedMemory.cpp test/shm_test.cpp)
add_executable(ParamTest SharedMemory.cpp test/test_params.cpp)