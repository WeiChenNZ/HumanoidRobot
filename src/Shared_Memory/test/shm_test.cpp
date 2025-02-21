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


    shared_ptr<SharedMemory> shm = make_shared<SharedMemory>("Wayne", "_Seg1",1);
    MatrixXd data(3,3), data2(2,2);
    shm->addBlock("block1", MatrixXd::Zero(3,3));
    shm->addBlock("block2", MatrixXd::Zero(2,2));
    shm->connectSegment();

    thread t2(readFunc, shm);
    thread t1(writeFunc, shm);
    

    t1.join();
    t2.join();

    return 0;
}