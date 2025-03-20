#include "MemoryManager.h"
#include <unordered_map>

using namespace std;
using namespace Eigen;
int main(void)
{
    MemoryManager& a = MemoryManager::getInstance();

    unordered_map<string, MatrixXd> data;
    MatrixXd pos = MatrixXd::Zero(1,3);
    pos <<1,2,3;
    data["body_position"] = pos;
    a.ESTIMATOR_STATE->setVal(data);

    cout<<a.ESTIMATOR_STATE->getVal()["body_position"]<<endl;
    return 0;
}