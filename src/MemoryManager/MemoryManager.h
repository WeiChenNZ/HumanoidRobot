#pragma once

#include "SharedMemory.h"
#include <memory>

class MemoryManager{

    public:
        static MemoryManager& getInstance()
        {
            static MemoryManager instance;
            return instance;
        }

        void init(void);
        void connect(void);

        //disable copy
        MemoryManager(const MemoryManager&) = delete;
        MemoryManager& operator=(const MemoryManager&) = delete;

        std::unique_ptr<SharedMemory> THREAD_STATE = nullptr;
        std::unique_ptr<SharedMemory> SIMULATOR_STATE = nullptr;
        std::unique_ptr<SharedMemory> SENSE_STATE = nullptr;
        std::unique_ptr<SharedMemory> GAMEPAD_STATE = nullptr;
        std::unique_ptr<SharedMemory> LEG_STATE = nullptr;
        std::unique_ptr<SharedMemory> LEG_COMMAND = nullptr;
        std::unique_ptr<SharedMemory> ARM_STATE = nullptr;
        std::unique_ptr<SharedMemory> ARM_COMMAND = nullptr;
        std::unique_ptr<SharedMemory> ESTIMATOR_STATE = nullptr;
        std::unique_ptr<SharedMemory> ESTIMATOR_COMMAND = nullptr;
        std::unique_ptr<SharedMemory> PLANNER_COMMAND = nullptr;
        std::unique_ptr<SharedMemory> USER_COMMAND = nullptr;




    private:
        //singleton mode
        MemoryManager(void);
        void initSharedMemory(bool init);



};