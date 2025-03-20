#include "BruceDynamics.h"
#include "MathTools.h"

using namespace std;
using namespace Eigen;
void BruceInverseDynamics::updateInversDynamics(Eigen::MatrixXd R, Eigen::MatrixXd p, Eigen::MatrixXd w, Eigen::MatrixXd bv, std::vector<double> q, std::vector<double> dq)
{
    if(q.size() != 10 && dq.size() != 10) 
        throw std::runtime_error("Ivalid dynamics parameters");

    MatrixXd v_0 = MatrixXd::Zero(6,1);
    v_0 << w, bv;

    MatrixXd e3 = MatrixXd::Zero(6,1);
    e3 << 0.,0.,1.,0.,0.,0.;
    
    
    MatrixXd X0 = MatrixXd::Zero(6,6);

    //Ad(Tn_n-1).transpose
    //X1 = Ad(T1_0).t  
    //Fn-1 = Xn * Fn  : transfer wrench
    MatrixXd X1 = MatrixXd::Zero(6,6);
    MatrixXd X2 = MatrixXd::Zero(6,6);
    MatrixXd X3 = MatrixXd::Zero(6,6);
    MatrixXd X4 = MatrixXd::Zero(6,6);
    MatrixXd X5 = MatrixXd::Zero(6,6);
    MatrixXd X6 = MatrixXd::Zero(6,6);
    MatrixXd X7 = MatrixXd::Zero(6,6);
    MatrixXd X8 = MatrixXd::Zero(6,6);
    MatrixXd X9 = MatrixXd::Zero(6,6);
    MatrixXd X10 = MatrixXd::Zero(6,6);

    Matrix3d r_01_hat = MathTools::skewMatrix(robotModel->getRelativePosOfJointsLeg()[0]);
    Matrix3d r_12_hat = MathTools::skewMatrix(robotModel->getRelativePosOfJointsLeg()[1]);
    Matrix3d r_23_hat = MathTools::skewMatrix(robotModel->getRelativePosOfJointsLeg()[2]);
    Matrix3d r_34_hat = MathTools::skewMatrix(robotModel->getRelativePosOfJointsLeg()[3]);
    Matrix3d r_45_hat = MathTools::skewMatrix(robotModel->getRelativePosOfJointsLeg()[4]);
    Matrix3d r_06_hat = MathTools::skewMatrix(robotModel->getRelativePosOfJointsLeg()[5]);
    Matrix3d r_67_hat = MathTools::skewMatrix(robotModel->getRelativePosOfJointsLeg()[6]);
    Matrix3d r_78_hat = MathTools::skewMatrix(robotModel->getRelativePosOfJointsLeg()[7]);
    Matrix3d r_89_hat = MathTools::skewMatrix(robotModel->getRelativePosOfJointsLeg()[8]);
    Matrix3d r_910_hat = MathTools::skewMatrix(robotModel->getRelativePosOfJointsLeg()[9]);

    //formulate X matrics
    X0.block(0,0,3,3) = R;
    X0.block(3,3,3,3) = R;

    double c1 = cos(q[0]);
    double s1 = sin(q[0]);
    Matrix3d R1;
    R1 << c1, -s1, 0.,
          s1,  c1, 0.,
          0.,  0., 1.;
    X1.block(0,0,3,3) = R1;
    X1.block(3,3,3,3) = R1;
    X1.block(0,3,3,3) = r_01_hat * R1;

    double c2 = cos(q[1]);
    double s2 = sin(q[1]);
    Matrix3d R2;
    R2 << c2, -s2, 0.,
          0.,  0., 1.,
         -s2, -c2, 0.;
    X2.block(0,0,3,3) = R2;
    X2.block(3,3,3,3) = R2;
    X2.block(0,3,3,3) = r_12_hat * R2;

    double c3 = cos(q[2]);
    double s3 = sin(q[2]);
    Matrix3d R3;
    R3 << c3, -s3, 0.,
          0.,  0., -1,
          s3,  c3, 0.;
    X3.block(0,0,3,3) = R3;
    X3.block(3,3,3,3) = R3;
    X3.block(0,3,3,3) = r_23_hat * R3;

    double c4 = cos(q[3]);
    double s4 = sin(q[3]);
    Matrix3d R4;
    R4 << c4, -s4, 0.,
          s4,  c4, 0.,
          0.,  0., 1.;
    X4.block(0,0,3,3) = R4;
    X4.block(3,3,3,3) = R4;
    X4.block(0,3,3,3) = r_34_hat * R4;

    double c5 = cos(q[4]);
    double s5 = sin(q[4]);
    Matrix3d R5;
    R5 << c5, -s5, 0.,
          s5,  c5, 0.,
          0.,  0., 1.;
    X5.block(0,0,3,3) = R5;
    X5.block(3,3,3,3) = R5;
    X5.block(0,3,3,3) = r_45_hat * R5;

    double c6 = cos(q[5]);
    double s6 = sin(q[5]);
    Matrix3d R6;
    R6 << c6, -s6, 0.,
          s6,  c6, 0.,
          0.,  0., 1.;
    X6.block(0,0,3,3) = R6;
    X6.block(3,3,3,3) = R6;
    X6.block(0,3,3,3) = r_06_hat * R6;

    double c7 = cos(q[6]);
    double s7 = sin(q[6]);
    Matrix3d R7;
    R7 << c7, -s7, 0.,
          0.,  0., 1.,
         -s7, -c7, 0.;
    X7.block(0,0,3,3) = R7;
    X7.block(3,3,3,3) = R7;
    X7.block(0,3,3,3) = r_67_hat * R7;

    double c8 = cos(q[7]);
    double s8 = sin(q[7]);
    Matrix3d R8;
    R8 << c8, -s8, 0.,
          0.,  0., -1,
          s8,  c8, 0.;
    X8.block(0,0,3,3) = R8;
    X8.block(3,3,3,3) = R8;
    X8.block(0,3,3,3) = r_78_hat * R8;

    double c9 = cos(q[8]);
    double s9 = sin(q[8]);
    Matrix3d R9;
    R9 << c9, -s9, 0.,
          s9,  c9, 0.,
          0.,  0., 1.;
    X9.block(0,0,3,3) = R9;
    X9.block(3,3,3,3) = R9;
    X9.block(0,3,3,3) = r_89_hat * R9;

    double c10 = cos(q[9]);
    double s10 = sin(q[9]);
    Matrix3d R10;
    R10 <<c10, -s10, 0.,
          s10,  c10, 0.,
            0.,  0., 1.;
    X10.block(0,0,3,3) = R10;
    X10.block(3,3,3,3) = R10;
    X10.block(0,3,3,3) = r_910_hat * R10;

    //Ad(Tn_n-1)
    //X1T = Ad(T1_0)
    Matrix3d X0T = X0.transpose();
    Matrix3d X1T = X1.transpose();
    Matrix3d X2T = X2.transpose();
    Matrix3d X3T = X3.transpose();
    Matrix3d X4T = X4.transpose();
    Matrix3d X5T = X5.transpose();
    Matrix3d X6T = X6.transpose();
    Matrix3d X7T = X7.transpose();
    Matrix3d X8T = X8.transpose();
    Matrix3d X9T = X9.transpose();
    Matrix3d X10T = X10.transpose();

    //calculate C
    //set a0 = 0 <== G = 0, and q" = 0, so the dynamics : Cq' = tau
    //so Cq'[i] = tau[i]
    MatrixXd vHatstar_0 = MathTools::twistHatStar(v_0);
    MatrixXd Is_0 = robotModel->getSpatialInertials()[0];
    MatrixXd fv_0 = vHatstar_0 * Is_0 * v_0;
    MatrixXd f_0 = fv_0;

    //right leg
    MatrixXd vJ_1 = dq[0] * e3;
    MatrixXd v_1 = X1T * v_0 + vJ_1;
    MatrixXd vHat_1 = MathTools::twistHat(v_1);
    MatrixXd vHatstar_1 = MathTools::twistHatStar(v_1);
    MatrixXd av_1 = vHat_1 * vJ_1; 
    MatrixXd a_1 = av_1; //becuase a0 = 0
    MatrixXd Is_1 = robotModel->getSpatialInertials()[1];
    MatrixXd fv_1 = vHatstar_1 * Is_1 * v_1;
    MatrixXd f_1 = Is_1 * a_1 + fv_1;

    MatrixXd vJ_2 = dq[1] * e3;
    MatrixXd v_2 = X2T * v_1 + vJ_2;
    MatrixXd vHat_2 = MathTools::twistHat(v_2);
    MatrixXd vHatstar_2 = MathTools::twistHatStar(v_2);
    MatrixXd av_2 = vHat_2 * vJ_2; 
    MatrixXd a_2 = X2T * a_1 + av_2; 
    MatrixXd Is_2 = robotModel->getSpatialInertials()[2];
    MatrixXd fv_2 = vHatstar_2 * Is_2 * v_2;
    MatrixXd f_2 = Is_2 * a_2 + fv_2;

    MatrixXd vJ_3 = dq[2] * e3;
    MatrixXd v_3 = X3T * v_2 + vJ_3;
    MatrixXd vHat_3 = MathTools::twistHat(v_3);
    MatrixXd vHatstar_3 = MathTools::twistHatStar(v_3);
    MatrixXd av_3 = vHat_3 * vJ_3; 
    MatrixXd a_3 = X3T * a_2 + av_3; 
    MatrixXd Is_3 = robotModel->getSpatialInertials()[3];
    MatrixXd fv_3 = vHatstar_3 * Is_3 * v_3;
    MatrixXd f_3 = Is_3 * a_3 + fv_3;

    MatrixXd vJ_4 = dq[3] * e3;
    MatrixXd v_4 = X4T * v_3 + vJ_4;
    MatrixXd vHat_4 = MathTools::twistHat(v_4);
    MatrixXd vHatstar_4 = MathTools::twistHatStar(v_4);
    MatrixXd av_4 = vHat_4 * vJ_4; 
    MatrixXd a_4 = X4T * a_3 + av_4; 
    MatrixXd Is_4 = robotModel->getSpatialInertials()[4];
    MatrixXd fv_4 = vHatstar_4 * Is_4 * v_4;
    MatrixXd f_4 = Is_4 * a_4 + fv_4;

    MatrixXd vJ_5 = dq[4] * e3;
    MatrixXd v_5 = X5T * v_4 + vJ_5;
    MatrixXd vHat_5 = MathTools::twistHat(v_5);
    MatrixXd vHatstar_5 = MathTools::twistHatStar(v_5);
    MatrixXd av_5 = vHat_5 * vJ_5; 
    MatrixXd a_5 = X5T * a_4 + av_5; 
    MatrixXd Is_5 = robotModel->getSpatialInertials()[5];
    MatrixXd fv_5 = vHatstar_5 * Is_5 * v_5;
    MatrixXd f_5 = Is_5 * a_5 + fv_5;

    //left leg
    MatrixXd vJ_6 = dq[5] * e3;
    MatrixXd v_6 = X6T * v_0 + vJ_6;
    MatrixXd vHat_6 = MathTools::twistHat(v_6);
    MatrixXd vHatstar_6 = MathTools::twistHatStar(v_6);
    MatrixXd av_6 = vHat_6 * vJ_6; 
    MatrixXd a_6 = av_6; //becuase a0 = 0
    MatrixXd Is_6 = robotModel->getSpatialInertials()[6];
    MatrixXd fv_6 = vHatstar_6 * Is_6 * v_6;
    MatrixXd f_6 = Is_6 * a_6 + fv_6;

    MatrixXd vJ_7 = dq[6] * e3;
    MatrixXd v_7 = X7T * v_6 + vJ_7;
    MatrixXd vHat_7 = MathTools::twistHat(v_7);
    MatrixXd vHatstar_7 = MathTools::twistHatStar(v_7);
    MatrixXd av_7 = vHat_7 * vJ_7; 
    MatrixXd a_7 = X7T * a_6 + av_7; 
    MatrixXd Is_7 = robotModel->getSpatialInertials()[7];
    MatrixXd fv_7 = vHatstar_7 * Is_7 * v_7;
    MatrixXd f_7 = Is_7 * a_7 + fv_7;

    MatrixXd vJ_8 = dq[7] * e3;
    MatrixXd v_8 = X8T * v_7 + vJ_8;
    MatrixXd vHat_8 = MathTools::twistHat(v_8);
    MatrixXd vHatstar_8 = MathTools::twistHatStar(v_8);
    MatrixXd av_8 = vHat_8 * vJ_8; 
    MatrixXd a_8 = X8T * a_7 + av_8; 
    MatrixXd Is_8 = robotModel->getSpatialInertials()[8];
    MatrixXd fv_8 = vHatstar_8 * Is_8 * v_8;
    MatrixXd f_8 = Is_8 * a_8 + fv_8;

    MatrixXd vJ_9 = dq[8] * e3;
    MatrixXd v_9 = X9T * v_8 + vJ_9;
    MatrixXd vHat_9 = MathTools::twistHat(v_9);
    MatrixXd vHatstar_9 = MathTools::twistHatStar(v_9);
    MatrixXd av_9 = vHat_9 * vJ_9; 
    MatrixXd a_9 = X9T * a_8 + av_9; 
    MatrixXd Is_9 = robotModel->getSpatialInertials()[9];
    MatrixXd fv_9 = vHatstar_9 * Is_9 * v_9;
    MatrixXd f_9 = Is_9 * a_9 + fv_9;

    MatrixXd vJ_10 = dq[9] * e3;
    MatrixXd v_10 = X10T * v_9 + vJ_10;
    MatrixXd vHat_10 = MathTools::twistHat(v_10);
    MatrixXd vHatstar_10 = MathTools::twistHatStar(v_10);
    MatrixXd av_10 = vHat_10 * vJ_10; 
    MatrixXd a_10 = X10T * a_9 + av_10; 
    MatrixXd Is_10 = robotModel->getSpatialInertials()[10];
    MatrixXd fv_10 = vHatstar_10 * Is_10 * v_10;
    MatrixXd f_10 = Is_10 * a_10 + fv_10;   

    //calculate Cq'
    //backward recursive 
    MatrixXd C = MatrixXd::Zero(16,1);
    C(15,0) = f_10(2,0);
    f_9 += X10 * f_10;

    C(14,0) = f_9(2,0);
    f_8 += X9 * f_9;

    C(13,0) = f_8(2,0);
    f_7 += X8 * f_8;

    C(12,0) = f_7(2,0);
    f_6 += X7 * f_7;

    C(11,0) = f_6(2,0);
    f_0 += X6 * f_6;

    C(10,0) = f_5(2,0);
    f_4 += X5 * f_5;

    C(9,0) = f_4(2,0);
    f_3 += X4 * f_4;

    C(8,0) = f_3(2,0);
    f_2 += X3 * f_3;

    C(7,0) = f_2(2,0);
    f_1 += X2 * f_2;

    C(6,0) = f_1(2,0);
    f_0 += X1 * f_1;

    C.block(0,0,6,1) = f_0;

    //modify dynamics based on g
    MatrixXd spatial_a_w = MatrixXd::Zero(6,1);
    spatial_a_w << 0., 0., 0., 0., 0., 9.81;
    MatrixXd a_0 = X0T * spatial_a_w;
    f_0 = Is_0 * a_0 + fv_0;

    // right
    a_1 =  X1T * a_0 + av_1;
    f_1 = Is_1 * a_1 + fv_1;

    a_2 =  X2T * a_1 + av_2;
    f_2 = Is_2 * a_2 + fv_2;

    a_3 =  X3T * a_2 + av_3;
    f_3 = Is_3 * a_3 + fv_3;

    a_4 =  X4T * a_3 + av_4;
    f_4 = Is_4 * a_4 + fv_4;

    a_5 =  X5T * a_4 + av_5;
    f_5 = Is_5 * a_5 + fv_5;

    // left
    a_6 =  X6T * a_0 + av_6;
    f_6 = Is_6 * a_6 + fv_6;

    a_7 =  X7T * a_6 + av_7;
    f_7 = Is_7 * a_7 + fv_7;

    a_8 =  X8T * a_7 + av_8;
    f_8 = Is_8 * a_8 + fv_8;

    a_9 =  X9T * a_8 + av_9;
    f_9 = Is_9 * a_9 + fv_9;

    a_10 =  X10T * a_9 + av_10;
    f_10 = Is_10 * a_10 + fv_10;

    CG = MatrixXd::Zero(16,1);
    CG(15,0) = f_10(2,0);
    f_9 += X10 * f_10;

    CG(14,0) = f_9(2,0);
    f_8 += X9 * f_9;

    CG(13,0) = f_8(2,0);
    f_7 += X8 * f_8;

    CG(12,0) = f_7(2,0);
    f_6 += X7 * f_7;

    CG(11,0) = f_6(2,0);
    f_0 += X6 * f_6;

    CG(10,0) = f_5(2,0);
    f_4 += X5 * f_5;

    CG(9,0) = f_4(2,0);
    f_3 += X4 * f_4;

    CG(8,0) = f_3(2,0);
    f_2 += X3 * f_3;

    CG(7,0) = f_2(2,0);
    f_1 += X2 * f_2;

    CG(6,0) = f_1(2,0);
    f_0 += X1 * f_1;

    CG.block(0,0,6,1) = f_0;    

    //calculate H matrix
    //composite rigid body algorihm
    H = MatrixXd::Zero(16,16);
    MatrixXd Ic_0 = Is_0;
    MatrixXd Ic_1 = Is_1;
    MatrixXd Ic_2 = Is_2;
    MatrixXd Ic_3 = Is_3;
    MatrixXd Ic_4 = Is_4;
    MatrixXd Ic_5 = Is_5;
    MatrixXd Ic_6 = Is_6;
    MatrixXd Ic_7 = Is_7;
    MatrixXd Ic_8 = Is_8;
    MatrixXd Ic_9 = Is_9;
    MatrixXd Ic_10 = Is_10;

    //i = 10
    Ic_9 += X10 * Ic_10 * X10T;
    MatrixXd F = Ic_10 * e3;
    H(15,15) = Ic_10(2,2);
    //k = 10
    F = X10 * F;
    H(15,14) = F(2);
    H(14,15) = F(2);
    //k = 9
    F = X9 * F;
    H(15,13) = F(2);
    H(13,15) = F(2);
    //k = 8
    F = X8 * F;
    H(15,12) = F(2);
    H(12,15) = F(2);
    //k = 7
    F = X7 * F;
    H(15,11) = F(2);
    H(11,15) = F(2);
    //k = 6
    F = X6 * F;
    H.block(15,0,1,6) = F;
    H.block(0,15,6,1) = F.transpose();

    //i = 9
    Ic_8 += X9 * Ic_9 * X9T;
    F = Ic_9 * e3;
    H(14,14) = Ic_9(2,2);
    //k = 9
    F = X9 * F;
    H(14,13) = F(2);
    H(13,14) = F(2);
    //k = 8
    F = X8 * F;
    H(14,12) = F(2);
    H(12,14) = F(2);
    //k = 7
    F = X7 * F;
    H(14,11) = F(2);
    H(11,14) = F(2);
    //k = 6
    F = X6 * F;
    H.block(14,0,1,6) = F;
    H.block(0,14,6,1) = F.transpose();

    //i = 8
    Ic_7 += X8 * Ic_8 * X8T;
    F = Ic_8 * e3;
    H(13,13) = Ic_8(2,2);
    //k = 8
    F = X8 * F;
    H(13,12) = F(2);
    H(12,13) = F(2);
    //k = 7
    F = X7 * F;
    H(13,11) = F(2);
    H(11,13) = F(2);
    //k = 6
    F = X6 * F;
    H.block(13,0,1,6) = F;
    H.block(0,13,6,1) = F.transpose();

    //i = 7
    Ic_6 += X7 * Ic_7 * X7T;
    F = Ic_7 * e3;
    H(12,12) = Ic_7(2,2);
    //k = 7
    F = X7 * F;
    H(12,11) = F(2);
    H(11,12) = F(2);
    //k = 6
    F = X6 * F;
    H.block(12,0,1,6) = F;
    H.block(0,12,6,1) = F.transpose();

    //i = 6
    Ic_0 += X6 * Ic_6 * X6T;
    F = Ic_6 * e3;
    H(11,11) = Ic_6(2,2);
    //k = 6
    F = X6 * F;
    H.block(11,0,1,6) = F;
    H.block(0,11,6,1) = F.transpose();

    //i = 5
    Ic_4 += X5 * Ic_5 * X5T;
    F = Ic_5 * e3;
    H(10,10) = Ic_5(2,2);
    //k = 5
    F = X5 * F;
    H(10,9) = F(2);
    H(9,10) = F(2);
    //k = 4
    F = X4 * F;
    H(10,8) = F(2);
    H(8,10) = F(2);
    //k = 3
    F = X3 * F;
    H(10,7) = F(2);
    H(7,10) = F(2);
    //k = 2
    F = X2 * F;
    H(10,6) = F(2);
    H(6,10) = F(2);
    //k = 1
    F = X1 * F;
    H.block(10,0,1,6) = F;
    H.block(0,10,6,1) = F.transpose();

    //i = 4
    Ic_3 += X4 * Ic_4 * X4T;
    F = Ic_4 * e3;
    H(9,9) = Ic_4(2,2);
    //k = 4
    F = X4 * F;
    H(9,8) = F(2);
    H(8,9) = F(2);
    //k = 3
    F = X3 * F;
    H(9,7) = F(2);
    H(7,9) = F(2);
    //k = 2
    F = X2 * F;
    H(9,6) = F(2);
    H(6,9) = F(2);
    //k = 1
    F = X1 * F;
    H.block(9,0,1,6) = F;
    H.block(0,9,6,1) = F.transpose();

    //i = 3
    Ic_2 += X3 * Ic_3 * X3T;
    F = Ic_3 * e3;
    H(8,8) = Ic_3(2,2);
    //k = 3
    F = X3 * F;
    H(8,7) = F(2);
    H(7,8) = F(2);
    //k = 2
    F = X2 * F;
    H(8,6) = F(2);
    H(6,8) = F(2);
    //k = 1
    F = X1 * F;
    H.block(8,0,1,6) = F;
    H.block(0,8,6,1) = F.transpose();

    //i = 2
    Ic_1 += X2 * Ic_2 * X2T;
    F = Ic_2 * e3;
    H(7,7) = Ic_2(2,2);
    //k = 2
    F = X2 * F;
    H(7,6) = F(2);
    H(6,7) = F(2);
    //k = 1
    F = X1 * F;
    H.block(7,0,1,6) = F;
    H.block(0,7,6,1) = F.transpose();

    //i = 1
    Ic_0 += X1 * Ic_1 * X1T;
    F = Ic_1 * e3;
    H(6,6) = Ic_1(2,2);
    //k = 1
    F = X1 * F;
    H.block(6,0,1,6) = F;
    H.block(0,6,6,1) = F.transpose();

    //i = 0
    H.block(0,0,6,6) = Ic_0;

    //adjust for sparsity
    //avoiding sigular matrix
    H.block(0,0,6,16) += 1e-12 * MatrixXd::Ones(6,16);
    
    double totalMass = robotModel->getTotalMass();

    Vector3d bp_G(H(2,4)/totalMass, H(0,5)/totalMass, H(1,3)/totalMass);

    X0.block(0,3,3,3) = -R * MathTools::skewMatrix(bp_G);

    MatrixXd S6 = MatrixXd::Zero(6,16);
    S6.block(0,0,6,6) = MatrixXd::Identity(6,6);

    AG = X0 * S6 * H;
    dAGdq = X0 * C.block(0,0,6,1);

    pcom = p + R * bp_G;
    MatrixXd vBody = MatrixXd::Zero(16,1);
    vBody << v_0, dq[0],dq[1],dq[2],dq[3],dq[4],dq[5],dq[6],dq[7],dq[8],dq[9];
    hG = AG * vBody;
    vcom = hG.block(3,0,3,1)/totalMass;
}