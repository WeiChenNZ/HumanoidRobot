#pragma once

#include <chrono>
#include <functional>
#include <thread>
#include <iostream>
#include <atomic>


class PeriodicTask{
    
    public:
        //period -- ms
        PeriodicTask(int period, std::function<void()> task)
        :period_(period), task_(task), stopFlag_(false){}

        void start(void) 
        {
            if (thread_.joinable()) 
            {
                std::cout << "Thread already running. Cannot start again."<<std::endl;
                return;
            }
            stopFlag_ = false; 
            thread_ = std::thread(&PeriodicTask::run, this);
        }

        void stop(void) 
        {
            stopFlag_ = true;
            if(thread_.joinable())
            {
                thread_.join();
                std::cout<<"Periodic task has stopped!"<<std::endl;
            }
        }

        ~PeriodicTask()
        {
            stop();
            std::cout<<"Periodic task has stopped in the deconstructor!"<<std::endl;
        }

    private:
        std::chrono::milliseconds period_;
        std::function<void()> task_;
        std::atomic<bool> stopFlag_;
        std::thread thread_;

        void run(void)
        {
            auto nextTime = std::chrono::steady_clock().now();
            while(!stopFlag_)
            {
                auto startTime = std::chrono::steady_clock().now();
                task_();
                auto endTime = std::chrono::steady_clock().now();
                auto duration = endTime - startTime;

                if(duration > period_)
                {
                    //if the task runs slower and out of the period, then print warning message
                    std::cerr << "[WARNING] Task overran by "
                              << std::chrono::duration_cast<std::chrono::milliseconds>(duration - period_).count()
                              << " ms"<<std::endl;
                              
                    nextTime = endTime;
                }
                else
                {
                    nextTime += period_;
                    std::this_thread::sleep_until(nextTime);
                }
            }
            std::cout<<"run function terminates!"<<std::endl;
        }
};