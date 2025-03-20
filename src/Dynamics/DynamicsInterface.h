#pragma once

#include "Eigen/Core"

class DynamicsInterface{
        
        virtual void updateInversDynamics(Eigen::MatrixXd R, Eigen::MatrixXd p, Eigen::MatrixXd w, Eigen::MatrixXd bv, std::vector<double> q, std::vector<double> dq) = 0;

        virtual Eigen::MatrixXd getH(void) = 0;
        virtual Eigen::MatrixXd getCG(void) = 0;
        virtual Eigen::MatrixXd getAG(void)  = 0;
        virtual Eigen::MatrixXd getdAGdq(void)  = 0;
        virtual Eigen::MatrixXd getPcom(void)  = 0;
        virtual Eigen::MatrixXd getVcom(void) = 0;
        virtual Eigen::MatrixXd getIG(void) = 0;
        virtual Eigen::MatrixXd getKG(void) = 0;
};