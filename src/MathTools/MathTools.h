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

        static Eigen::MatrixXd Rx(double roll)
        {
            double c = cos(roll);
            double s = sin(roll);
            Eigen::MatrixXd R = Eigen::MatrixXd::Zero(3,3);
            R << 1., 0., 0.,
                 0., c,  -s,
                 0., s,  c;
            return R;
        }

        static Eigen::MatrixXd Ry(double pitch)
        {
            double c = cos(pitch);
            double s = sin(pitch);
            Eigen::MatrixXd R = Eigen::MatrixXd::Zero(3,3);
            R << c, 0., s,
                 0.,1., 0.,
                 -s,0., c;
            return R;
        }

        static Eigen::MatrixXd Rz(double yaw)
        {
            double c = cos(yaw);
            double s = sin(yaw);
            Eigen::MatrixXd R = Eigen::MatrixXd::Zero(3,3);
            R << c, -s, 0.,
                 s,  c, 0.,
                 0.,0., 1.;
            return R;
        }
};
