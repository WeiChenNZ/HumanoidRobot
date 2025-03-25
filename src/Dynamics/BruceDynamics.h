#pragma once

#include "RobotModel.h"
#include "Eigen/Core"
#include "DynamicsInterface.h"
#include <memory>

class BruceInverseDynamics: public DynamicsInterface{

    public:
        BruceInverseDynamics(std::shared_ptr<RobotModel> robotModel_):robotModel(robotModel_)
        {}

        void updateInversDynamics(Eigen::MatrixXd R, Eigen::MatrixXd p, Eigen::MatrixXd w, Eigen::MatrixXd bv, std::vector<double> q, std::vector<double> dq);

        Eigen::MatrixXd getH(void) {return H;}
        Eigen::MatrixXd getCG(void) {return CG;}
        Eigen::MatrixXd getAG(void) {return AG;}
        Eigen::MatrixXd getdAGdq(void) {return dAGdq;}
        Eigen::MatrixXd getPcom(void) {return pcom;}
        Eigen::MatrixXd getVcom(void) {return vcom;}
        Eigen::MatrixXd getKG(void) {return hG.block(0,0,3,1);} //hG[0:2]
        Eigen::MatrixXd getIG(void) {return hG.block(3,0,3,1);} //hG[3:5]


    private:
        std::shared_ptr<RobotModel> robotModel;

        //H(q)q'' + C(q,q')q' + G(q) = tau
        Eigen::MatrixXd H;
        Eigen::MatrixXd CG;

        //centroidal momentum
        Eigen::MatrixXd AG;
        Eigen::MatrixXd dAGdq;
        Eigen::MatrixXd pcom;
        Eigen::MatrixXd vcom;
        Eigen::MatrixXd hG;

};