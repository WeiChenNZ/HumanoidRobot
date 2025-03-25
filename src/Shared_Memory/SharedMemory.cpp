#include "SharedMemory.h"

using namespace std;
using namespace Eigen;

void SharedMemory::addBlock(string name, MatrixXd data)
{
    unordered_map<string, ValueType> block;
    block["name"] = name;
    block["data"] = data;
    block["size"] = data.rows()*data.cols()*sizeof(double);
    block["shape"] = make_pair(data.rows(), data.cols());
    block["midx"] = memorySize;

    memorySize += get<size_t>(block["size"]);
    blocks.push_back(block);
}

void SharedMemory::addBlock(std::string name, ModelParameters data)
{
    unordered_map<string, ValueType> block;
    block["name"] = name;
    block["data"] = data;
    block["size"] = data.rows()*data.cols()*sizeof(ModelParameters);
    block["shape"] = make_pair(data.rows(), data.cols());
    block["midx"] = memorySize;

    memorySize += get<size_t>(block["size"]);
    blocks.push_back(block);    
}

void SharedMemory::addBlock(std::string name, WorldParameters data)
{
    unordered_map<string, ValueType> block;
    block["name"] = name;
    block["data"] = data;
    block["size"] = data.rows()*data.cols()*sizeof(WorldParameters);
    block["shape"] = make_pair(data.rows(), data.cols());
    block["midx"] = memorySize;

    memorySize += get<size_t>(block["size"]);
    blocks.push_back(block);    
}

void SharedMemory::updateSegment()
{
    MatrixXd temp = MatrixXd::Zero(0,1);
    for(auto element:blocks)
    {
        //get data and reshape it into a column vector
        MatrixXd mat = get<MatrixXd>(element["data"]);
        MatrixXd reshaped(mat.size(),1);
        
        for(int i =0; i < mat.rows(); i++)
            for(int j = 0; j < mat.cols(); j++)
            {
                reshaped(i*mat.cols() + j) = mat(i,j);
            }
        //append all data into a column vector
        MatrixXd mid(temp.rows()+ reshaped.rows(),1);
        mid << temp , reshaped;

        //prepare for the next loop
        temp.resize(mid.size(),1);
        temp = mid;
    }

    //store the final data
    memData.resize(temp.size(),1);
    memData = temp;
}

void SharedMemory::updateSegmentModel()
{
    memDataModel = get<ModelParameters>(blocks[0]["data"]);
}

void SharedMemory::updateSegmentWorld()
{
    memDataWorld = get<WorldParameters>(blocks[0]["data"]);
}

void SharedMemory::connectSegment(SegType st)
{
    if(st == DATA)
        updateSegment();
    else if(st == MODEL)
        updateSegmentModel();
    else if(st == WORLD)
        updateSegmentWorld();

    if(memorySize == 0) throw std::runtime_error("can not allocate 0 length memory!");

    string pathName = robotName + "_" + segmentName;
    
    if(initialize)
    {
        //first unlock if shared memory exist
        shm_unlink((pathName + "_mem").c_str());
        sem_unlink((pathName + "_lock").c_str());

        //second link 
        shmFd = shm_open((pathName + "_mem").c_str(), O_CREAT | O_RDWR, 0666);
        //set lenght
        ftruncate(shmFd, memorySize);
        
        semaphore = sem_open((pathName + "_lock").c_str(), O_CREAT, 0666, 1);
        initialize = false;
    }
    else
    {
        shmFd = shm_open((pathName + "_mem").c_str(), O_RDWR, 0666);
        semaphore = sem_open((pathName + "_lock").c_str(), 0);
    }

    if(shmFd == -1 || semaphore == nullptr)
    {
        throw runtime_error("create shared memory error !\n");
    }

    memoryAddr = mmap(nullptr, memorySize, PROT_READ | PROT_WRITE, MAP_SHARED, shmFd, 0);
    sem_post(semaphore);
}

void SharedMemory::setVal(std::unordered_map<std::string, Eigen::MatrixXd> value, WRITE_RANGE wr)
{
    if(wr == SEG_ALL)
    {
        for(auto& element:blocks)
        {
            if(value.find(get<string>(element["name"])) != value.end())
            {
                //in order to deal with all row and column vector
                //we need to transfer column vector to row vector
                //in the other cases, do not transpose
                if(value[get<string>(element["name"])].rows() > 1 && value[get<string>(element["name"])].cols() == 1)
                {
                    //col vector Nx1
                    element["data"] = value[get<string>(element["name"])].transpose();
                }
                else
                {
                    //1xN or MxN or 1x1
                    element["data"] = value[get<string>(element["name"])];
                }
            }
        }
        updateSegment();

        //write process
        sem_wait(semaphore);//lock
        lseek(shmFd, 0, SEEK_SET); //set file pointer to the start
        write(shmFd, memData.data(), memorySize);
        sem_post(semaphore);
    }
    else if(wr == SEG_ONLY)
    {
        //updata all segments
        sem_wait(semaphore);
        span<double> memSeg = getMemorySegment<double>();

        for(auto& element:blocks)
        {
            if(value.find(get<string>(element["name"])) != value.end())
            {
                element["data"] = value[get<string>(element["name"])];
            }
            else
            {
                int idx0 = get<size_t>(element["midx"]) / sizeof(double);
                int idx1 = get<size_t>(element["size"]) / sizeof(double) + idx0;
                span<double>sub = memSeg.subspan(idx0, idx1 - idx0);
                MatrixXd d(get<pair<int,int>>(element["shape"]).first, get<pair<int,int>>(element["shape"]).second);

                //reshape
                for(int j = 0; j < d.rows(); j++)
                {
                    for(int k = 0; k < d.cols(); k++)
                    {
                        d(j,k) = sub[d.cols() + k];
                    }
                }
                element["data"] = d;//updata object's data structure
            }
        }
        updateSegment();

        //write process
        lseek(shmFd, 0, SEEK_SET);
        write(shmFd, memData.data(), memorySize);
        sem_post(semaphore);
    }
}

void SharedMemory::setVal(std::unordered_map<std::string, ModelParameters> value, WRITE_RANGE wr)
{
    for(auto& element:blocks)
    {
        if(value.find(get<string>(element["name"])) != value.end())
        {
            element["data"] = value[get<string>(element["name"])];
        }
    }
    updateSegmentModel();

    //write process
    sem_wait(semaphore);//lock
    lseek(shmFd, 0, SEEK_SET); //set file pointer to the start
    write(shmFd, &memDataModel, memorySize);
    sem_post(semaphore);
}

void SharedMemory::setVal(std::unordered_map<std::string, WorldParameters> value, WRITE_RANGE wr)
{
    for(auto& element:blocks)
    {
        if(value.find(get<string>(element["name"])) != value.end())
        {
            element["data"] = value[get<string>(element["name"])];
        }
    }
    updateSegmentWorld();

    //write process
    sem_wait(semaphore);//lock
    lseek(shmFd, 0, SEEK_SET); //set file pointer to the start
    write(shmFd, &memDataWorld, memorySize);
    sem_post(semaphore);
}


std::unordered_map<std::string, Eigen::MatrixXd> SharedMemory::getVal(void)
{
    sem_wait(semaphore);

    span<double> memSeg = getMemorySegment<double>();
    unordered_map<std::string, Eigen::MatrixXd> data;

    for(auto& element:blocks)
    {
        int idx0 = get<size_t>(element["midx"]) / sizeof(double);
        int idx1 = get<size_t>(element["size"]) / sizeof(double) + idx0;
        span<double>sub = memSeg.subspan(idx0, idx1 - idx0);
        MatrixXd d(get<pair<int,int>>(element["shape"]).first, get<pair<int,int>>(element["shape"]).second);

        //reshape
        for(int j = 0; j < d.rows(); j++)
            for(int k = 0; k < d.cols(); k++)
            {
                d(j,k) = sub[j*d.cols() + k];
            }
        element["data"] = d;//update objects own data structure
        data[get<string>(element["name"])] = d;
    }
    sem_post(semaphore);
    return data;
}

std::unordered_map<std::string, ModelParameters> SharedMemory::getValModel(void)
{
    sem_wait(semaphore);

    unordered_map<std::string, ModelParameters> data;

    ModelParameters* p = static_cast<ModelParameters*>(memoryAddr);

    for(auto& element:blocks)
    {
        element["data"] = *p;//update objects own data structure
        data[get<string>(element["name"])] = *p;
    }
    sem_post(semaphore);
    return data;
}

std::unordered_map<std::string, WorldParameters> SharedMemory::getValWorld(void)
{
    sem_wait(semaphore);

    span<double> memSeg = getMemorySegment<double>();
    unordered_map<std::string, WorldParameters> data;

    for(auto& element:blocks)
    {
        int idx0 = get<size_t>(element["midx"]) / sizeof(double);
        int idx1 = get<size_t>(element["size"]) / sizeof(double) + idx0;
        span<double>sub = memSeg.subspan(idx0, idx1 - idx0);
        WorldParameters d;

        d.step_size = sub[0];
        d.real_time_update_rate = sub[1];
        
        element["data"] = d;//update objects own data structure
        data[get<string>(element["name"])] = d;
    }
    sem_post(semaphore);
    return data;
}

