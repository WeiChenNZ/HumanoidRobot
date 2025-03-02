#pragma once
#include "Eigen/Core"

class MathTools{
    public:

        static Eigen::Matrix3d skewMatrix(Eigen::Vector3d v)
        {
            Eigen::Matrix3d skewMat;
            skewMat <<   0  , -v(2)  ,  v(1),
                       v(2) ,    0   , -v(0),
                      -v(1) ,  v(0)  ,    0 ;
            return skewMat;
        } 
};
