#pragma once

#include "Eigen/Core"
#include <iostream>
#include <fcntl.h>     // O_* constants
#include <sys/mman.h>  // shm_open, mmap
#include <sys/stat.h>  // mode constants
#include <unistd.h>    // ftruncate, close
#include <unordered_map>
#include <string>
#include <semaphore.h> // C++20 semaphores
#include <atomic>
#include <variant>
#include <tuple>
#include <span>



enum WRITE_RANGE
{
    SEG_ONLY,
    SEG_ALL
};

enum SegType
{
    DATA,
    MODEL,
    WORLD
};

struct ModelParameters{
    int operating_mode;
    double state_update_rate;

    size_t rows(){return 1;}
    size_t cols(){return 1;}
    size_t size(){return 1;}
};

struct WorldParameters{
    double step_size;
    double real_time_update_rate;

    size_t rows(){return 1;}
    size_t cols(){return 1;}
    size_t size(){return 1;}    
};

using ValueType = std::variant<std::string, Eigen::MatrixXd, size_t, std::pair<int,int>, ModelParameters, WorldParameters>;


//in order to support the two Parameter structures above, I need to add more methods to deal with them
//the best way should be use an interface, maybe I can reconstruct it later
class SharedMemory{

    public:
        SharedMemory(std::string robotName_, std::string segName_, bool init)
        {
            robotName = robotName_;
            segmentName = segName_;
            initialize = init;
            blocks = {};
            memoryAddr = nullptr;
            semaphore = nullptr;
            memorySize = 0;
        }

        void setInitTrue(void){initialize = true;}
        
        void addBlock(std::string name, Eigen::MatrixXd data);
        void addBlock(std::string name, ModelParameters data);
        void addBlock(std::string name, WorldParameters data);

        void updateSegment(void);
        void updateSegmentModel(void);
        void updateSegmentWorld(void);

        void connectSegment(SegType st = DATA);

        void setVal(std::unordered_map<std::string, Eigen::MatrixXd>, WRITE_RANGE wr = SEG_ALL);
        void setVal(std::unordered_map<std::string, ModelParameters>, WRITE_RANGE wr = SEG_ALL);
        void setVal(std::unordered_map<std::string, WorldParameters>, WRITE_RANGE wr = SEG_ALL);
        
        std::unordered_map<std::string, Eigen::MatrixXd> getVal(void);
        std::unordered_map<std::string, ModelParameters> getValModel(void);
        std::unordered_map<std::string, WorldParameters> getValWorld(void);

        //tool method, convert linear memory into segments
        template <typename T>
        std::span<T> getMemorySegment() {
            size_t num_elements = memorySize / sizeof(T);
            return std::span<T>(static_cast<T*>(memoryAddr), num_elements);
        }
        


    private:
        std::string robotName;
        std::string segmentName;
        bool initialize;
        std::vector<std::unordered_map<std::string,ValueType>> blocks;
        void* memoryAddr;
        sem_t* semaphore;
        size_t memorySize;
        Eigen::MatrixXd memData;
        ModelParameters memDataModel;
        WorldParameters memDataWorld;
        int shmFd;
};

