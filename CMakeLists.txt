cmake_minimum_required(VERSION 3.10)

project(HumanoidRobot)

set(CMAKE_CXX_STANDARD 20) 
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_BUILD_TYPE Debug)

#set(CMAKE_BUILD_TYPE Release)

# Find all .cpp source files 
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Find all directories containing header files and add them as include directories
file(GLOB_RECURSE HEADER_DIRS RELATIVE ${CMAKE_SOURCE_DIR} "src/*.h")

# Extract unique directories from found header files
set(INCLUDE_DIRS "")
foreach(HEADER_FILE ${HEADER_DIRS})
    get_filename_component(HEADER_PATH ${HEADER_FILE} DIRECTORY)
    list(APPEND INCLUDE_DIRS ${HEADER_PATH})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS) 
 
# Add include directories recursively
foreach(DIR ${INCLUDE_DIRS}) 
    include_directories(${CMAKE_SOURCE_DIR}/${DIR})
endforeach()
 
message(++++++++++++++++++++++)
message(${INCLUDE_DIRS})

# Find and include Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_subdirectory(src/Shared_Memory)
add_subdirectory(src/Simulation)
add_subdirectory(src/RobotModel)

# Create executable
#add_executable(main ${SOURCES})