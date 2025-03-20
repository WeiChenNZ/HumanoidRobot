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

        static Eigen::MatrixXd twistHat(Eigen::MatrixXd V)
        {
            Eigen::Vector3d w = V.block(0,0,3,1);
            Eigen::Vector3d v = V.block(3,0,3,1);

            Eigen::Matrix3d wHat = skewMatrix(w);
            Eigen::Matrix3d vHat = skewMatrix(v);

            Eigen::MatrixXd result = Eigen::MatrixXd::Zero(6,6);
            result.block(0,0,3,3) = wHat;
            result.block(3,3,3,3) = wHat;
            result.block(3,0,3,3) = vHat;

            return result;
        }

        static Eigen::MatrixXd twistHatStar(Eigen::MatrixXd V)
        {
            Eigen::Vector3d w = V.block(0,0,3,1);
            Eigen::Vector3d v = V.block(3,0,3,1);

            Eigen::Matrix3d wHat = skewMatrix(w);
            Eigen::Matrix3d vHat = skewMatrix(v);

            Eigen::MatrixXd result = Eigen::MatrixXd::Zero(6,6);
            result.block(0,0,3,3) = wHat;
            result.block(3,3,3,3) = wHat;
            result.block(0,3,3,3) = vHat;

            return result;
        }
};
