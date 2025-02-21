#include "SharedMemory.h"
#include <memory>
#include <thread>

using namespace Eigen;
using namespace std;

void writeFunc(shared_ptr<SharedMemory> s)
{
    static double i = 0;
    while(1)
    {
        i++;
        MatrixXd d33(3,3);
        d33<<1+i,2+i,3+i,4+i,5+i,6+i,7+i,8+i,9+i;
        MatrixXd d22(2,2);
        d22<<1+i,2+i,3+i,4+i;
        unordered_map<string, MatrixXd> val;
        val["block1"] = d33;
        val["block2"] = d22;
        s->setVal(val,SEG_ALL);
        sleep(1);
    }
}

void readFunc(shared_ptr<SharedMemory> s)
{
    unordered_map<string, MatrixXd> val;
    while(1)
    {
        val = s->getVal();

        cout<<"Element 1 = "<<endl<<val["block1"]<<endl
            <<"Element 2 = "<<endl<<val["block2"]<<endl;

        sleep(1);
    }
}

int main()
{

    shared_ptr<SharedMemory> shmModel = make_shared<SharedMemory>("Wayne", "_Model",1);
    shared_ptr<SharedMemory> shmWorld = make_shared<SharedMemory>("Wayne", "_World",1);
    
    ModelParameters mp;
    WorldParameters wp;

    shmModel->addBlock("block1", mp);
    shmWorld->addBlock("block1", wp);

    shmModel->connectSegment(MODEL);
    shmWorld->connectSegment(WORLD);

    unordered_map<string, ModelParameters> valModel, resultModel;
    unordered_map<string, WorldParameters> valWorld, resultWorld;

    mp.operating_mode = 33;
    mp.state_update_rate = 123.456;

    wp.step_size = 99.1;
    wp.real_time_update_rate = 100.5;

    valModel["block1"] = mp;
    valWorld["block1"] = wp;

    shmModel->setVal(valModel, SEG_ALL);
    shmWorld->setVal(valWorld, SEG_ALL);

    resultModel = shmModel->getValModel();
    resultWorld = shmWorld->getValWorld();

    cout<<"Model Params 1 = "<<resultModel["block1"].operating_mode<<endl
        <<"Model Params 2 = "<<resultModel["block1"].state_update_rate<<endl;

    cout<<"World Params 1 = "<<resultWorld["block1"].step_size<<endl
        <<"World Params 2 = "<<resultWorld["block1"].real_time_update_rate<<endl;

    // thread t2(readFunc, shm);
    // thread t1(writeFunc, shm);
    

    // t1.join();
    // t2.join();

    return 0;
}