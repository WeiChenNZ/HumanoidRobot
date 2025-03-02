#include "BruceRobotKinematics.h"
#include <tuple>
#include <math.h>
#include <Eigen/Dense>
#include "MathTools.h"


using namespace std;
using namespace Eigen;

Kinematics::Kinematics(std::shared_ptr<RobotModel> robotModel_):robotMode(robotModel_)
{
    numJointsPerLeg = robotMode->getNumJointsPerLeg();
}



std::vector<double> Kinematics::inverseKinematicsFoot(Eigen::MatrixXd position, Eigen::MatrixXd direction, LEG leg)
{
    double r11 = direction(0,0);
    double r21 = direction(1,0);
    double r31 = direction(2,0);
    double dx = position(0,0) - hx;
    double dy = position(1,0) + hy * static_cast<float>(leg);
    double dz = position(2,0) + hz;

    double k1 = dx * r31 - dz * r11;
    double k2 = dy * r31 - dz * r21;
    double t1 = atan2(-k1, k2);
    double c1 = cos(t1);
    double s1 = sin(t1);

    double dxc1_dys1 = dx * c1 + dy * s1;

    double t2 = atan2(-dz, dxc1_dys1);
    double c2 = cos(t2);
    double s2 = sin(t2);

    double c345 = r21 * c1 - r11 * s1;

    double s2n = abs(s2);

    double s345,k3;
    if(s2n > 1e-6)
    {
        s345 = r31 / s2;
        k3   = -dz / s2 - a5 * c345;
    }
    else
    {
        s345 = -(r11 * c1 + r21 * s1) / c2;
        k3   = dxc1_dys1 / c2 - a5 * c345;
    }

    double k4 = dy * c1 - dx * s1 - a5 * s345 - d2;
    double c4 = (k3*k3 + k4*k4 - a3*a3 - a4*a4) / 2. / a3 / a4;

    if(c4 > 1.)
    {
        c4 = 1.;
    }
    else if(c4 < -1.)
    {
        c4 = -1.;
    }

    double s4 = -sqrt(1 - c4*c4);
    double t4 = atan2(s4, c4);

    double a34 = a3 + a4 * c4;
    double as4 = a4 * s4;
    double t3  = atan2(a34 * k4 - as4 * k3, a34 * k3 + as4 * k4);
    double t5  = atan2(s345, c345) - t3 - t4;

    vector<double> result;
    result.push_back(t1);
    result.push_back(t2);
    result.push_back(t3);
    result.push_back(t4);
    result.push_back(t5);
    
    return result;
}


std::vector<double> Kinematics::inverseKinematicsAnkle(Eigen::MatrixXd position, Eigen::MatrixXd direction, LEG leg)
{
    double r11 = direction(0,0);
    double r21 = direction(1,0);
    double r31 = direction(2,0);
    double dx = position(0,0) - hx;
    double dy = position(1,0) + hy * static_cast<float>(leg);
    double dz = position(2,0) + hz;

    double k1 = dx * r31 - dz * r11;
    double k2 = dy * r31 - dz * r21;
    double t1 = atan2(-k1, k2);
    double c1 = cos(t1);
    double s1 = sin(t1);

    double dxc1_dys1 = dx * c1 + dy * s1;

    double t2 = atan2(-dz, dxc1_dys1);
    double c2 = cos(t2);
    double s2 = sin(t2);

    double c345 = r21 * c1 - r11 * s1;

    double s2n = abs(s2);

    double s345,k3;
    if(s2n > 1e-6)
    {
        s345 = r31 / s2;
        k3   = -dz / s2;
    }
    else
    {
        s345 = -(r11 * c1 + r21 * s1) / c2;
        k3   = dxc1_dys1 / c2;
    }

    double k4 = dy * c1 - dx * s1 - d2;
    double c4 = (k3*k3 + k4*k4 - a3*a3 - a4*a4) / 2. / a3 / a4;   

    if(c4 > 1.)
    {
        c4 = 1.;
    }
    else if(c4 < -1.)
    {
        c4 = -1.;
    } 

    double s4 = -sqrt(1 - c4*c4);
    double t4 = atan2(s4, c4);

    double a34 = a3 + a4 * c4;
    double as4 = a4 * s4;
    double t3  = atan2(a34 * k4 - as4 * k3, a34 * k3 + as4 * k4);
    double t5  = atan2(s345, c345) - t3 - t4;

    vector<double> result;
    result.push_back(t1);
    result.push_back(t2);
    result.push_back(t3);
    result.push_back(t4);
    result.push_back(t5);
    
    return result;
}


unordered_map<string, MatrixXd> Kinematics::forwardKinematicsLeg(vector<double> jointsPosVel)
{
    if(jointsPosVel.size() != numJointsPerLeg*2*2) //positin and velocity
    {
        cerr<<"Invalid leg configuration!"<<endl;
        throw std::runtime_error("joints configuration do not macth!\n");
    }

    MatrixXd dqr(5,1);
    dqr << jointsPosVel[10],jointsPosVel[11], jointsPosVel[12],jointsPosVel[13],jointsPosVel[14];
    MatrixXd dql(5,1);
    dql << jointsPosVel[15],jointsPosVel[16], jointsPosVel[17],jointsPosVel[18],jointsPosVel[19];

    //q--joint_position, dq--joint_velocity
    //we know Bruce robot has 5 joints each leg, so here I just use the calculated math formulas 
    //I will implement dynamical DH model calculation and derivation after I finish the original version
    double t1,t2,t3,t4,t5,td1,td2,td3,td4,td5;
    tie(t1, t2, t3, t4, t5) = tuple{jointsPosVel[0],jointsPosVel[1], jointsPosVel[2],jointsPosVel[3],jointsPosVel[4]};
    tie(td1, td2, td3, td4, td5) = tuple{jointsPosVel[10],jointsPosVel[11], jointsPosVel[12],jointsPosVel[13],jointsPosVel[14]};

    double s1 = sin(t1);
    double c1 = cos(t1);
    double s2 = sin(t2);
    double c2 = cos(t2);

    double s1s2 = s1 * s2;
    double c1s2 = c1 * s2;
    double c1c2 = c1 * c2;
    double s1c2 = s1 * c2;
    double t34  = t3 + t4;
    double t345 = t34 + t5;
    double s345 = sin(t345);
    double c345 = cos(t345);

    double a5s345 = a5 * s345;
    double a5c345 = a5 * c345;
    double a6s345 = a6 * s345;
    double a6c345 = a6 * c345;

    double a4s34 = a4 * sin(t34);
    double a4c34 = a4 * cos(t34);
    double a3s3  = a3 * sin(t3);
    double a3c3  = a3 * cos(t3);

    // Ankle
    double as34a   = a3s3 + a4s34;
    double ac34a   = a3c3 + a4c34;
    double d2as34a = d2 + as34a;

    double s2ac34a   =   s2 * ac34a;
    double s1c2ac34a = s1c2 * ac34a;
    double c1c2ac34a = c1c2 * ac34a;

    double c1c2ac34_s1d2as34a = c1c2ac34a - s1 * d2as34a;
    double s1c2ac34_c1d2as34a = s1c2ac34a + c1 * d2as34a;

    //position
    MatrixXd pra(3,1);
    pra <<  hx + c1c2ac34_s1d2as34a,
           -hy + s1c2ac34_c1d2as34a,
           -hz - s2ac34a;
    //Jacobian
    MatrixXd Jra(3,5);
    Jra << -s1c2ac34_c1d2as34a, -c1s2 * ac34a, -s1 * ac34a - c1c2 * as34a, -s1 * a4c34 - c1c2 * a4s34, 0. ,
            c1c2ac34_s1d2as34a, -s1s2 * ac34a,  c1 * ac34a - s1c2 * as34a,  c1 * a4c34 - s1c2 * a4s34, 0. ,
                            0.,   -c2 * ac34a,                 s2 * as34a,                 s2 * a4s34, 0. ;
    //velocity
    MatrixXd vra(3,1);
    vra = Jra * dqr;
    //Jacobian Derivative
    MatrixXd dJra = MatrixXd::Zero(3,5);
    dJra(0, 0) = -Jra(1, 0) * td1 - Jra(1, 1) * td2 - Jra(1, 2) * td3 - Jra(1, 3) * td4;
    dJra(1, 0) =  Jra(0, 0) * td1 + Jra(0, 1) * td2 + Jra(0, 2) * td3 + Jra(0, 3) * td4;

    double astd34a = as34a * td3 + a4s34 * td4;
    double s2ac4a  = s2 * a4c34;
    dJra(0, 1) = -Jra(1, 1) * td1 - c1c2ac34a * td2 + c1s2 * astd34a;
    dJra(1, 1) =  Jra(0, 1) * td1 - s1c2ac34a * td2 + s1s2 * astd34a;
    dJra(2, 1) =                      s2ac34a * td2 +   c2 * astd34a;

    double s1a4a = s1 * a4s34 - c1c2 * a4c34;
    double c1a4a = c1 * a4s34 + s1c2 * a4c34;
    double as34atd2 = as34a * td2;
    dJra(0, 2) = -Jra(1, 2) * td1 + c1s2 * as34atd2 - (Jra(1, 0) + s1 * d2) * td3 +  s1a4a * td4;
    dJra(1, 2) =  Jra(0, 2) * td1 + s1s2 * as34atd2 + (Jra(0, 0) + c1 * d2) * td3 -  c1a4a * td4;
    dJra(2, 2) =                      c2 * as34atd2 +               s2ac34a * td3 + s2ac4a * td4;

    double td34 = td3 + td4;
    double as4atd2 = a4s34 * td2;
    dJra(0, 3) = -Jra(1, 3) * td1 + c1s2 * as4atd2 +  s1a4a * td34;
    dJra(1, 3) =  Jra(0, 3) * td1 + s1s2 * as4atd2 -  c1a4a * td34;
    dJra(2, 3) =                      c2 * as4atd2 + s2ac4a * td34;

    //Toe
    double as56t   = a5s345 + a6c345 + (at - ah) * c345;
    double ac56t   = a5c345 - a6s345 - (at - ah) * s345;
    double as456t  = as56t  + a4s34;
    double ac456t  = ac56t  + a4c34;
    double as3456t = as456t + a3s3;
    double ac3456t = ac456t + a3c3;
    double d2as3456t = d2 + as3456t;

    double s2ac3456t   =   s2 * ac3456t;
    double s1c2ac3456t = s1c2 * ac3456t;
    double c1c2ac3456t = c1c2 * ac3456t;

    double c1c2ac3456_s1d2as3456t = c1c2ac3456t - s1 * d2as3456t;
    double s1c2ac3456_c1d2as3456t = s1c2ac3456t + c1 * d2as3456t;

    //position
    MatrixXd prt(3,1);
    prt << hx + c1c2ac3456_s1d2as3456t,
          -hy + s1c2ac3456_c1d2as3456t,
          -hz - s2ac3456t;
    //Jacobian
    MatrixXd Jrt(3,5);
    Jrt << -s1c2ac3456_c1d2as3456t, -c1s2 * ac3456t, -s1 * ac3456t - c1c2 * as3456t, -s1 * ac456t - c1c2 * as456t, -s1 * ac56t - c1c2 * as56t ,
            c1c2ac3456_s1d2as3456t, -s1s2 * ac3456t,  c1 * ac3456t - s1c2 * as3456t,  c1 * ac456t - s1c2 * as456t,  c1 * ac56t - s1c2 * as56t ,
                                0.,   -c2 * ac3456t,                   s2 * as3456t,                  s2 * as456t,                 s2 * as56t ;

    //velocity
    MatrixXd vrt(3,1);
    vrt = Jrt * dqr;
    //Jacobian Derivative
    MatrixXd dJrt = MatrixXd::Zero(3,5);
    dJrt(0, 0) = -Jrt(1, 0) * td1 - Jrt(1, 1) * td2 - Jrt(1, 2) * td3 - Jrt(1, 3) * td4 - Jrt(1, 4) * td5;
    dJrt(1, 0) =  Jrt(0, 0) * td1 + Jrt(0, 1) * td2 + Jrt(0, 2) * td3 + Jrt(0, 3) * td4 + Jrt(0, 4) * td5;

    double astd345t = as3456t * td3 + as456t * td4 + as56t * td5;
    double s2ac456t = s2 * ac456t;
    double s2ac56t  = s2 * ac56t;
    dJrt(0, 1) = -Jrt(1, 1) * td1 - c1c2ac3456t * td2 + c1s2 * astd345t;
    dJrt(1, 1) =  Jrt(0, 1) * td1 - s1c2ac3456t * td2 + s1s2 * astd345t;
    dJrt(2, 1) =                      s2ac3456t * td2 +   c2 * astd345t;

    double s1a456t = s1 * as456t - c1c2 * ac456t;
    double c1a456t = c1 * as456t + s1c2 * ac456t;
    double s1a56t  = s1 *  as56t - c1c2 *  ac56t;
    double c1a56t  = c1 *  as56t + s1c2 *  ac56t;
    double as3456ttd2 = as3456t * td2;
    double s1a56ttd5  =  s1a56t * td5;
    double c1a56ttd5  =  c1a56t * td5;
    double s2ac56ttd5 = s2ac56t * td5;
    dJrt(0, 2) = -Jrt(1, 2) * td1 + c1s2 * as3456ttd2 - (Jrt(1, 0) + s1 * d2) * td3 +  s1a456t * td4 +  s1a56ttd5;
    dJrt(1, 2) =  Jrt(0, 2) * td1 + s1s2 * as3456ttd2 + (Jrt(0, 0) + c1 * d2) * td3 -  c1a456t * td4 -  c1a56ttd5;
    dJrt(2, 2) =                      c2 * as3456ttd2 +             s2ac3456t * td3 + s2ac456t * td4 + s2ac56ttd5;

    double as456ttd2 = as456t * td2;
    dJrt(0, 3) = -Jrt(1, 3) * td1 + c1s2 * as456ttd2 +  s1a456t * td34 +  s1a56ttd5;
    dJrt(1, 3) =  Jrt(0, 3) * td1 + s1s2 * as456ttd2 -  c1a456t * td34 -  c1a56ttd5;
    dJrt(2, 3) =                      c2 * as456ttd2 + s2ac456t * td34 + s2ac56ttd5;

    double td345 = td34 + td5;
    double as56ttd2 = as56t * td2;
    dJrt(0, 4) = -Jrt(1, 4) * td1 + c1s2 * as56ttd2 +  s1a56t * td345;
    dJrt(1, 4) =  Jrt(0, 4) * td1 + s1s2 * as56ttd2 -  c1a56t * td345;
    dJrt(2, 4) =                      c2 * as56ttd2 + s2ac56t * td345;

    //Heel
    double as56h   = a5s345 - a6c345;
    double ac56h   = a5c345 + a6s345;
    double as456h  = as56h  + a4s34;
    double ac456h  = ac56h  + a4c34;
    double as3456h = as456h + a3s3;
    double ac3456h = ac456h + a3c3;
    double d2as3456h = d2 + as3456h;

    double s2ac3456h   =   s2 * ac3456h;
    double s1c2ac3456h = s1c2 * ac3456h;
    double c1c2ac3456h = c1c2 * ac3456h;

    double c1c2ac3456_s1d2as3456h = c1c2ac3456h - s1 * d2as3456h;
    double s1c2ac3456_c1d2as3456h = s1c2ac3456h + c1 * d2as3456h;

    //position
    MatrixXd prh(3,1);
    prh << hx + c1c2ac3456_s1d2as3456h,
          -hy + s1c2ac3456_c1d2as3456h,
          -hz - s2ac3456h;

    //Jacobian
    MatrixXd Jrh(3,5);
    Jrh << -s1c2ac3456_c1d2as3456h, -c1s2 * ac3456h, -s1 * ac3456h - c1c2 * as3456h, -s1 * ac456h - c1c2 * as456h, -s1 * ac56h - c1c2 * as56h ,
            c1c2ac3456_s1d2as3456h, -s1s2 * ac3456h,  c1 * ac3456h - s1c2 * as3456h,  c1 * ac456h - s1c2 * as456h,  c1 * ac56h - s1c2 * as56h ,
                                0.,   -c2 * ac3456h,                   s2 * as3456h,                  s2 * as456h,                 s2 * as56h ;
    
    //velocity
    MatrixXd vrh(3,1);
    vrh = Jrh * dqr;

    //Jacobian Derivative
    MatrixXd dJrh = MatrixXd::Zero(3,5);
    dJrh(0, 0) = -Jrh(1, 0) * td1 - Jrh(1, 1) * td2 - Jrh(1, 2) * td3 - Jrh(1, 3) * td4 - Jrh(1, 4) * td5;
    dJrh(1, 0) =  Jrh(0, 0) * td1 + Jrh(0, 1) * td2 + Jrh(0, 2) * td3 + Jrh(0, 3) * td4 + Jrh(0, 4) * td5;

    double astd345h = as3456h * td3 + as456h * td4 + as56h * td5;
    double s2ac456h = s2 * ac456h;
    double s2ac56h  = s2 *  ac56h;
    dJrh(0, 1) = -Jrh(1, 1) * td1 - c1c2ac3456h * td2 + c1s2 * astd345h;
    dJrh(1, 1) =  Jrh(0, 1) * td1 - s1c2ac3456h * td2 + s1s2 * astd345h;
    dJrh(2, 1) =                      s2ac3456h * td2 +   c2 * astd345h;

    double s1a456h = s1 * as456h - c1c2 * ac456h;
    double c1a456h = c1 * as456h + s1c2 * ac456h;
    double s1a56h  = s1 *  as56h - c1c2 *  ac56h;
    double c1a56h  = c1 *  as56h + s1c2 *  ac56h;
    double as3456htd2 = as3456h * td2;
    double s1a56htd5  =  s1a56h * td5;
    double c1a56htd5  =  c1a56h * td5;
    double s2ac56htd5 = s2ac56h * td5;
    dJrh(0, 2) = -Jrh(1, 2) * td1 + c1s2 * as3456htd2 - (Jrh(1, 0) + s1 * d2) * td3 +  s1a456h * td4 +  s1a56htd5;
    dJrh(1, 2) =  Jrh(0, 2) * td1 + s1s2 * as3456htd2 + (Jrh(0, 0) + c1 * d2) * td3 -  c1a456h * td4 -  c1a56htd5;
    dJrh(2, 2) =                      c2 * as3456htd2 +             s2ac3456h * td3 + s2ac456h * td4 + s2ac56htd5;

    double as456td2 = as456h * td2;
    dJrh(0, 3) = -Jrh(1, 3) * td1 + c1s2 * as456td2 +  s1a456h * td34 +  s1a56htd5;
    dJrh(1, 3) =  Jrh(0, 3) * td1 + s1s2 * as456td2 -  c1a456h * td34 -  c1a56htd5;
    dJrh(2, 3) =                      c2 * as456td2 + s2ac456h * td34 + s2ac56htd5;

    double as56td2 = as56h * td2;
    dJrh(0, 4) = -Jrh(1, 4) * td1 + c1s2 * as56td2 +  s1a56h * td345;
    dJrh(1, 4) =  Jrh(0, 4) * td1 + s1s2 * as56td2 -  c1a56h * td345;
    dJrh(2, 4) =                      c2 * as56td2 + s2ac56h * td345;

    //Middle of foot
    MatrixXd prm(3,1);
    prm = (prt + prh) / 2. ;
    MatrixXd vrm(3,1);
    vrm = (vrt + vrh) / 2. ;
    
    //foot orientation
    MatrixXd Rr(3,3);
    Rr << -c1c2 * s345 - s1 * c345, -c1s2, -c1c2 * c345 + s1 * s345 ,
          -s1c2 * s345 + c1 * c345, -s1s2, -s1c2 * c345 - c1 * s345 ,
                         s2 * s345,   -c2,                s2 * c345 ;
    //rotation Jacobian
    MatrixXd Jwr(3,3);
    Jwr << 0., -s1, c1s2, c1s2, c1s2 ,
           0.,  c1, s1s2, s1s2, s1s2 ,
           1.,  0.,   c2,   c2,   c2 ;
    //rotation Jacobian Derivative
    MatrixXd dJwr = MatrixXd::Zero(3,5);
    dJwr(0, 1) = -c1 * td1;
    dJwr(1, 1) = -s1 * td1;

    dJwr(0, 2) = -s1s2 * td1 + c1c2 * td2;
    dJwr(1, 2) =  c1s2 * td1 + s1c2 * td2;
    dJwr(2, 2) =                -s2 * td2;

    dJwr.block(0,3,3,1) = dJwr.block(0,2,3,1);
    dJwr.block(0,4,3,1) = dJwr.block(0,2,3,1);
    
    //////////////////////////////////////////////////////////////////////////////////////////////
    //LEFT FOOT
    //////////////////////////////////////////////////////////////////////////////////////////////
    tie(t1, t2, t3, t4, t5) = tuple{jointsPosVel[5],jointsPosVel[6], jointsPosVel[7],jointsPosVel[8],jointsPosVel[9]};
    tie(td1, td2, td3, td4, td5) = tuple{jointsPosVel[15],jointsPosVel[16], jointsPosVel[17],jointsPosVel[18],jointsPosVel[19]};

    s1 = sin(t1);
    c1 = cos(t1);
    s2 = sin(t2);
    c2 = cos(t2);

    s1s2 = s1 * s2;
    c1s2 = c1 * s2;
    c1c2 = c1 * c2;
    s1c2 = s1 * c2;
    t34  = t3 + t4;
    t345 = t34 + t5;
    s345 = sin(t345);
    c345 = cos(t345);

    a5s345 = a5 * s345;
    a5c345 = a5 * c345;
    a6s345 = a6 * s345;
    a6c345 = a6 * c345;

    a4s34 = a4 * sin(t34);
    a4c34 = a4 * cos(t34);
    a3s3  = a3 * sin(t3);
    a3c3  = a3 * cos(t3);

    //Ankle
    as34a = a3s3 + a4s34;
    ac34a = a3c3 + a4c34;
    d2as34a = d2 + as34a;

    s2ac34a   =   s2 * ac34a;
    s1c2ac34a = s1c2 * ac34a;
    c1c2ac34a = c1c2 * ac34a;

    c1c2ac34_s1d2as34a = c1c2ac34a - s1 * d2as34a;
    s1c2ac34_c1d2as34a = s1c2ac34a + c1 * d2as34a;

    //position
    MatrixXd pla(3,1);
    pla <<  hx + c1c2ac34_s1d2as34a,
            hy + s1c2ac34_c1d2as34a,
           -hz - s2ac34a;
    //Jacobian
    MatrixXd Jla(3,5);
    Jla << -s1c2ac34_c1d2as34a, -c1s2 * ac34a, -s1 * ac34a - c1c2 * as34a, -s1 * a4c34 - c1c2 * a4s34, 0. ,
            c1c2ac34_s1d2as34a, -s1s2 * ac34a,  c1 * ac34a - s1c2 * as34a,  c1 * a4c34 - s1c2 * a4s34, 0. ,
                            0.,   -c2 * ac34a,                 s2 * as34a,                 s2 * a4s34, 0. ;
    
    //velocity
    MatrixXd vla(3,1);
    vla = Jla * dql;

    //Jacobian Derivative
    MatrixXd dJla = MatrixXd::Zero(3,5);
    dJla(0, 0) = -Jla(1, 0) * td1 - Jla(1, 1) * td2 - Jla(1, 2) * td3 - Jla(1, 3) * td4;
    dJla(1, 0) =  Jla(0, 0) * td1 + Jla(0, 1) * td2 + Jla(0, 2) * td3 + Jla(0, 3) * td4;

    astd34a = as34a * td3 + a4s34 * td4;
    s2ac4a  = s2 * a4c34;
    dJla(0, 1) = -Jla(1, 1) * td1 - c1c2ac34a * td2 + c1s2 * astd34a;
    dJla(1, 1) =  Jla(0, 1) * td1 - s1c2ac34a * td2 + s1s2 * astd34a;
    dJla(2, 1) =                      s2ac34a * td2 +   c2 * astd34a;

    s1a4a = s1 * a4s34 - c1c2 * a4c34;
    c1a4a = c1 * a4s34 + s1c2 * a4c34;
    as34atd2 = as34a * td2;
    dJla(0, 2) = -Jla(1, 2) * td1 + c1s2 * as34atd2 - (Jla(1, 0) + s1 * d2) * td3 +  s1a4a * td4;
    dJla(1, 2) =  Jla(0, 2) * td1 + s1s2 * as34atd2 + (Jla(0, 0) + c1 * d2) * td3 -  c1a4a * td4;
    dJla(2, 2) =                      c2 * as34atd2 +               s2ac34a * td3 + s2ac4a * td4;

    td34 = td3 + td4;
    as4atd2 = a4s34 * td2;
    dJla(0, 3) = -Jla(1, 3) * td1 + c1s2 * as4atd2 +  s1a4a * td34;
    dJla(1, 3) =  Jla(0, 3) * td1 + s1s2 * as4atd2 -  c1a4a * td34;
    dJla(2, 3) =                      c2 * as4atd2 + s2ac4a * td34;

    //Toe
    as56t   = a5s345 + a6c345 + (at - ah) * c345;
    ac56t   = a5c345 - a6s345 - (at - ah) * s345;
    as456t  = as56t  + a4s34;
    ac456t  = ac56t  + a4c34;
    as3456t = as456t + a3s3;
    ac3456t = ac456t + a3c3;
    d2as3456t = d2 + as3456t;

    s2ac3456t   = s2   * ac3456t;
    s1c2ac3456t = s1c2 * ac3456t;
    c1c2ac3456t = c1c2 * ac3456t;

    c1c2ac3456_s1d2as3456t = c1c2ac3456t - s1 * d2as3456t;
    s1c2ac3456_c1d2as3456t = s1c2ac3456t + c1 * d2as3456t;

    //position
    MatrixXd plt(3,1);
    plt <<  hx + c1c2ac3456_s1d2as3456t,
            hy + s1c2ac3456_c1d2as3456t,
           -hz - s2ac3456t;
    //Jacobian
    MatrixXd Jlt(3,5);
    Jlt << -s1c2ac3456_c1d2as3456t, -c1s2 * ac3456t, -s1 * ac3456t - c1c2 * as3456t, -s1 * ac456t - c1c2 * as456t, -s1 * ac56t - c1c2 * as56t ,
            c1c2ac3456_s1d2as3456t, -s1s2 * ac3456t,  c1 * ac3456t - s1c2 * as3456t,  c1 * ac456t - s1c2 * as456t,  c1 * ac56t - s1c2 * as56t ,
                                0.,   -c2 * ac3456t,                   s2 * as3456t,                  s2 * as456t,                 s2 * as56t ;
    //velocity
    MatrixXd vlt(3,1);
    vlt = Jlt * dql;
    //Jacobian Derivative
    MatrixXd dJlt = MatrixXd::Zero(3,5);
    dJlt(0, 0) = -Jlt(1, 0) * td1 - Jlt(1, 1) * td2 - Jlt(1, 2) * td3 - Jlt(1, 3) * td4 - Jlt(1, 4) * td5;
    dJlt(1, 0) =  Jlt(0, 0) * td1 + Jlt(0, 1) * td2 + Jlt(0, 2) * td3 + Jlt(0, 3) * td4 + Jlt(0, 4) * td5;

    astd345t = as3456t * td3 + as456t * td4 + as56t * td5;
    s2ac456t = s2 * ac456t;
    s2ac56t  = s2 * ac56t;
    dJlt(0, 1) = -Jlt(1, 1) * td1 - c1c2ac3456t * td2 + c1s2 * astd345t;
    dJlt(1, 1) =  Jlt(0, 1) * td1 - s1c2ac3456t * td2 + s1s2 * astd345t;
    dJlt(2, 1) =                      s2ac3456t * td2 +   c2 * astd345t;

    s1a456t = s1 * as456t - c1c2 * ac456t;
    c1a456t = c1 * as456t + s1c2 * ac456t;
    s1a56t  = s1 *  as56t - c1c2 *  ac56t;
    c1a56t  = c1 *  as56t + s1c2 *  ac56t;
    as3456ttd2 = as3456t * td2;
    s1a56ttd5  =  s1a56t * td5;
    c1a56ttd5  =  c1a56t * td5;
    s2ac56ttd5 = s2ac56t * td5;
    dJlt(0, 2) = -Jlt(1, 2) * td1 + c1s2 * as3456ttd2 - (Jlt(1, 0) + s1 * d2) * td3 +  s1a456t * td4 +  s1a56ttd5;
    dJlt(1, 2) =  Jlt(0, 2) * td1 + s1s2 * as3456ttd2 + (Jlt(0, 0) + c1 * d2) * td3 -  c1a456t * td4 -  c1a56ttd5;
    dJlt(2, 2) =                      c2 * as3456ttd2 +             s2ac3456t * td3 + s2ac456t * td4 + s2ac56ttd5;

    td34 = td3 + td4;
    as456ttd2 = as456t * td2;
    dJlt(0, 3) = -Jlt(1, 3) * td1 + c1s2 * as456ttd2 +  s1a456t * td34 +  s1a56ttd5;
    dJlt(1, 3) =  Jlt(0, 3) * td1 + s1s2 * as456ttd2 -  c1a456t * td34 -  c1a56ttd5;
    dJlt(2, 3) =                      c2 * as456ttd2 + s2ac456t * td34 + s2ac56ttd5;

    td345 = td34 + td5;
    as56ttd2 = as56t * td2;
    dJlt(0, 4) = -Jlt(1, 4) * td1 + c1s2 * as56ttd2 +  s1a56t * td345;
    dJlt(1, 4) =  Jlt(0, 4) * td1 + s1s2 * as56ttd2 -  c1a56t * td345;
    dJlt(2, 4) =                      c2 * as56ttd2 + s2ac56t * td345;

    //Heel
    as56h   = a5s345 - a6c345;
    ac56h   = a5c345 + a6s345;
    as456h  = as56h  + a4s34;
    ac456h  = ac56h  + a4c34;
    as3456h = as456h + a3s3;
    ac3456h = ac456h + a3c3;
    d2as3456h = d2 + as3456h;

    s2ac3456h   = s2   * ac3456h;
    s1c2ac3456h = s1c2 * ac3456h;
    c1c2ac3456h = c1c2 * ac3456h;

    c1c2ac3456_s1d2as3456h = c1c2ac3456h - s1 * d2as3456h;
    s1c2ac3456_c1d2as3456h = s1c2ac3456h + c1 * d2as3456h;

    //position
    MatrixXd plh(3,1);
    plh << hx + c1c2ac3456_s1d2as3456h,
           hy + s1c2ac3456_c1d2as3456h,
          -hz - s2ac3456h;
    //Jacobian
    MatrixXd Jlh(3,5);
    Jlh << -s1c2ac3456_c1d2as3456h, -c1s2 * ac3456h, -s1 * ac3456h - c1c2 * as3456h, -s1 * ac456h - c1c2 * as456h, -s1 * ac56h - c1c2 * as56h ,
            c1c2ac3456_s1d2as3456h, -s1s2 * ac3456h,  c1 * ac3456h - s1c2 * as3456h,  c1 * ac456h - s1c2 * as456h,  c1 * ac56h - s1c2 * as56h ,
                                0.,   -c2 * ac3456h,                   s2 * as3456h,                  s2 * as456h,                 s2 * as56h ;
    //velocity
    MatrixXd vlh(3,1);
    vlh = Jlh * dql;
    //Jacobian Derivative
    MatrixXd dJlh = MatrixXd::Zero(3,5);
    dJlh(0, 0) = -Jlh(1, 0) * td1 - Jlh(1, 1) * td2 - Jlh(1, 2) * td3 - Jlh(1, 3) * td4 - Jlh(1, 4) * td5;
    dJlh(1, 0) =  Jlh(0, 0) * td1 + Jlh(0, 1) * td2 + Jlh(0, 2) * td3 + Jlh(0, 3) * td4 + Jlh(0, 4) * td5;

    astd345h = as3456h * td3 + as456h * td4 + as56h * td5;
    s2ac456h = s2 * ac456h;
    s2ac56h  = s2 *  ac56h;
    dJlh(0, 1) = -Jlh(1, 1) * td1 - c1c2ac3456h * td2 + c1s2 * astd345h;
    dJlh(1, 1) =  Jlh(0, 1) * td1 - s1c2ac3456h * td2 + s1s2 * astd345h;
    dJlh(2, 1) =                      s2ac3456h * td2 +   c2 * astd345h;

    s1a456h = s1 * as456h - c1c2 * ac456h;
    c1a456h = c1 * as456h + s1c2 * ac456h;
    s1a56h  = s1 *  as56h - c1c2 *  ac56h;
    c1a56h  = c1 *  as56h + s1c2 *  ac56h;
    as3456htd2 = as3456h * td2;
    s1a56htd5  =  s1a56h * td5;
    c1a56htd5  =  c1a56h * td5;
    s2ac56htd5 = s2ac56h * td5;
    dJlh(0, 2) = -Jlh(1, 2) * td1 + c1s2 * as3456htd2 - (Jlh(1, 0) + s1 * d2) * td3 +  s1a456h * td4 +  s1a56htd5;
    dJlh(1, 2) =  Jlh(0, 2) * td1 + s1s2 * as3456htd2 + (Jlh(0, 0) + c1 * d2) * td3 -  c1a456h * td4 -  c1a56htd5;
    dJlh(2, 2) =                      c2 * as3456htd2 +             s2ac3456h * td3 + s2ac456h * td4 + s2ac56htd5;

    as456td2 = as456h * td2;
    dJlh(0, 3) = -Jlh(1, 3) * td1 + c1s2 * as456td2 +  s1a456h * td34 +  s1a56htd5;
    dJlh(1, 3) =  Jlh(0, 3) * td1 + s1s2 * as456td2 -  c1a456h * td34 -  c1a56htd5;
    dJlh(2, 3) =                      c2 * as456td2 + s2ac456h * td34 + s2ac56htd5;

    as56td2 = as56h * td2;
    dJlh(0, 4) = -Jlh(1, 4) * td1 + c1s2 * as56td2 +  s1a56h * td345;
    dJlh(1, 4) =  Jlh(0, 4) * td1 + s1s2 * as56td2 -  c1a56h * td345;
    dJlh(2, 4) =                      c2 * as56td2 + s2ac56h * td345;

    //Middle of foot
    MatrixXd plm(3,1);
    plm = (plt + plh) / 2. ;
    MatrixXd vlm(3,1);
    vlm = (vlt + vlh) / 2. ;

    //orientation
    MatrixXd Rl(3,3);
    Rl << -c1c2 * s345 - s1 * c345, -c1s2, -c1c2 * c345 + s1 * s345 ,
          -s1c2 * s345 + c1 * c345, -s1s2, -s1c2 * c345 - c1 * s345 ,
                         s2 * s345,   -c2,                s2 * c345 ;
    //rotational Jacobian
    MatrixXd Jwl(3,5);
    Jwl << 0., -s1, c1s2, c1s2, c1s2 ,
           0.,  c1, s1s2, s1s2, s1s2 ,
           1.,  0.,   c2,   c2,   c2 ;
    //rotational Jacobian Derivative
    MatrixXd dJwl = MatrixXd::Zero(3,5);
    dJwl(0, 1) = -c1 * td1;
    dJwl(1, 1) = -s1 * td1;

    dJwl(0, 2) = -s1s2 * td1 + c1c2 * td2;
    dJwl(1, 2) =  c1s2 * td1 + s1c2 * td2;
    dJwl(2, 2) =                -s2 * td2;

    dJwl.block(0,3,3,1) = dJwl.block(0,2,3,1);
    dJwl.block(0,4,3,1) = dJwl.block(0,2,3,1);

    unordered_map<string, MatrixXd> result;
    //b_ prefix means these data relative to the body frame
    result["b_right_toe_position"] = prt;
    result["b_right_toe_velocity"] = vrt;
    result["b_right_toe_J"] = Jrt;
    result["b_right_toe_dJ"] = dJrt;
    result["b_right_heel_position"] = prh;
    result["b_right_heel_velocity"] = vrh;
    result["b_right_heel_J"] = Jrh;
    result["b_right_heel_dJ"] = dJrh;
    result["b_right_ankle_position"] = pra;
    result["b_right_ankle_velocity"] = vra;
    result["b_right_ankle_J"] = Jra;
    result["b_right_ankle_dJ"] = dJra;
    result["b_right_foot_position"] = prm;
    result["b_right_foot_velocity"] = vrm;
    result["b_right_foot_rot"] = Rr;
    result["b_right_foot_Jw"] = Jwr;
    result["b_right_foot_dJw"] = dJwr;
    result["b_left_toe_position"] = plt;
    result["b_left_toe_velocity"] = vlt;
    result["b_left_toe_J"] = Jlt;
    result["b_left_toe_dJ"] = dJlt;
    result["b_left_heel_position"] = plh;
    result["b_left_heel_velocity"] = vlh;
    result["b_left_heel_J"] = Jlh;
    result["b_left_heel_dJ"] = dJlh;
    result["b_left_ankle_position"] = pla;
    result["b_left_ankle_velocity"] = vla;
    result["b_left_ankle_J"] = Jla;
    result["b_left_ankle_dJ"] = dJla;
    result["b_left_foot_position"] = plm;
    result["b_left_foot_velocity"] = vlm;
    result["b_left_foot_rot"] = Rl;
    result["b_left_foot_Jw"] = Jwl;
    result["b_left_foot_dJw"] = dJwl;

    return result;
}

unordered_map<string, MatrixXd> Kinematics::forwardKinematicsRobot(unordered_map<string, MatrixXd> input)
{
    MatrixXd R(3,3);
    R << input["Rotation_Mat"];
    MatrixXd p = input["Body_Position"];

    MatrixXd prt = input["b_right_toe_position"];
    MatrixXd prh = input["b_right_heel_position"];
    MatrixXd pra = input["b_right_ankle_position"];
    MatrixXd plt = input["b_left_toe_position"];
    MatrixXd plh = input["b_left_heel_position"];
    MatrixXd pla = input["b_left_ankle_position"];
    //foot position in world fram
    MatrixXd crt = p + R * prt;
    MatrixXd crh = p + R * prh;
    MatrixXd cra = p + R * pra;
    MatrixXd clt = p + R * plt;
    MatrixXd clh = p + R * plh;
    MatrixXd cla = p + R * pla;
    MatrixXd crm = (crt + crh) / 2. ;
    MatrixXd clm = (clt + clh) / 2. ;

    
    MatrixXd Rr = input["b_right_foot_rot"];
    MatrixXd Rl = input["b_left_foot_rot"];
    //foot orientation in world frame
    MatrixXd wRr = R * Rr;
    MatrixXd wRl = R * Rl;

    MatrixXd wJrt = MatrixXd::Zero(3,16);
    MatrixXd wJrh = MatrixXd::Zero(3,16);
    MatrixXd wJra = MatrixXd::Zero(3,16);
    MatrixXd wJlt = MatrixXd::Zero(3,16);
    MatrixXd wJlh = MatrixXd::Zero(3,16);
    MatrixXd wJla = MatrixXd::Zero(3,16);

    MatrixXd Jrt = input["b_right_toe_J"];
    MatrixXd Jrh = input["b_right_heel_J"];
    MatrixXd Jra = input["b_right_ankle_J"];
    MatrixXd Jlt = input["b_left_toe_J"];
    MatrixXd Jlh = input["b_left_heel_J"];
    MatrixXd Jla = input["b_left_ankle_J"];

    //Jacobian in world frame
    Vector3d prt_v = prt;
    wJrt.block(0,0,3,3) = - R * MathTools::skewMatrix(prt_v);
    wJrt.block(0,3,3,3) = R;
    wJrt.block(0,6,3,5) = R * Jrt;

    Vector3d prh_v = prh;
    wJrh.block(0,0,3,3) = -R * MathTools::skewMatrix(prh_v);
    wJrh.block(0,3,3,3) = R;
    wJrh.block(0,6,3,5) = R * Jrh;

    Vector3d pra_v = pra;
    wJra.block(0,0,3,3) = -R * MathTools::skewMatrix(pra_v);
    wJra.block(0,3,3,3) = R;
    wJra.block(0,6,3,5) = R * Jra;

    Vector3d plt_v = plt;
    wJlt.block(0,0,3,3) = -R * MathTools::skewMatrix(plt_v);
    wJlt.block(0,3,3,3) = R;
    wJlt.block(0,11,3,5) = R * Jlt;

    Vector3d plh_v = plh;
    wJlh.block(0,0,3,3) = -R * MathTools::skewMatrix(plh_v);
    wJlh.block(0,3,3,3) = R;
    wJlh.block(0,11,3,5) = R * Jlh;

    Vector3d pla_v = pla;
    wJla.block(0,0,3,3) = -R * MathTools::skewMatrix(pla_v);
    wJla.block(0,3,3,3) = R;
    wJla.block(0,11,3,5) = R * Jla;

    //rotation Jacobian in world frame
    MatrixXd wJwr = MatrixXd::Zero(3,16);
    MatrixXd wJwl = MatrixXd::Zero(3,16);

    MatrixXd Jwr = input["b_right_foot_Jw"];
    MatrixXd Jwl = input["b_left_foot_Jw"];

    wJwr.block(0,0,3,3) = Rr.transpose();
    wJwr.block(0,6,3,5) = Rr.transpose() * Jwr;

    wJwl.block(0,0,3,3) = Rl.transpose();
    wJwl.block(0,11,3,5) = Rl.transpose() * Jwl;

    MatrixXd w = input["Body_w"];
    MatrixXd bv = input["Body_v"];
    MatrixXd dqr(5,1);
    dqr << input["rd1"], input["rd2"], input["rd3"], input["rd4"], input["rd5"];
    MatrixXd dql(5,1);
    dql << input["ld1"], input["ld2"], input["ld3"], input["ld4"], input["ld5"];

    MatrixXd dqAll;//16*1
    dqAll << w, bv, dqr, dql;

    //foot velocity in world frame
    MatrixXd dcrt = wJrt * dqAll;
    MatrixXd dcrh = wJrh * dqAll;
    MatrixXd dcra = wJra * dqAll;
    MatrixXd dclt = wJlt * dqAll;
    MatrixXd dclh = wJlh * dqAll;
    MatrixXd dcla = wJla * dqAll;
    MatrixXd dcrm = (dcrt + dcrh) / 2. ;
    MatrixXd dclm = (dclt + dclh) / 2. ;
    //foot angular rate in world frame
    MatrixXd wr = wJwr * dqAll;
    MatrixXd wl = wJwl * dqAll;

    //calculate dJ*dq
    Vector3d w_v = w;
    MatrixXd what = MathTools::skewMatrix(w_v);

    MatrixXd whatwhat = what * what;
    MatrixXd whatbv = what * bv;
    MatrixXd what2 = 2 * what;

    MatrixXd dJrt = input["b_right_toe_dJ"];
    MatrixXd wdJrtdq = R * (whatbv + whatwhat * prt + (what2 * Jrt + dJrt) * dqr);

    MatrixXd dJrh = input["b_right_heel_dJ"];
    MatrixXd wdJrhdq = R * (whatbv + whatwhat * prh + (what2 * Jrh + dJrh) * dqr);

    MatrixXd dJra = input["b_right_ankle_dJ"];
    MatrixXd wdJradq = R * (whatbv + whatwhat * pra + (what2 * Jra + dJra) * dqr);

    MatrixXd dJlt = input["b_left_toe_dJ"];
    MatrixXd wdJltdq = R * (whatbv + whatwhat * plt + (what2 * Jlt + dJlt) * dql);

    MatrixXd dJlh = input["b_left_heel_dJ"];
    MatrixXd wdJlhdq = R * (whatbv + whatwhat * plh + (what2 * Jlh + dJlh) * dql);

    MatrixXd dJla = input["b_left_ankle_dJ"];
    MatrixXd wdJladq = R * (whatbv + whatwhat * pla + (what2 * Jla + dJla) * dql);

    MatrixXd dJwr = input["b_right_foot_dJw"];
    MatrixXd wdJwrdq = Rr.transpose() * (what * Jwr + dJwr) * dqr;

    MatrixXd dJwl = input["b_left_foot_dJw"];
    MatrixXd wdJwldq = Rl.transpose() * (what * Jwl + dJwl) * dql;

    unordered_map<string, MatrixXd> result;

    result["right_toe_position"] = crt;
    result["right_toe_velocity"] = dcrt;
    result["right_toe_Jv"] = wJrt;
    result["right_toe_dJvdq"] = wdJrtdq;
    result["right_heel_position"] = crh;
    result["right_heel_velocity"] = dcrh;
    result["right_heel_Jv"] = wJrh;
    result["right_heel_dJvdq"] = wdJrhdq;
    result["right_ankle_position"] = cra;
    result["right_ankle_velocity"] = dcra;
    result["right_ankle_Jv"] = wJra;
    result["right_ankle_dJvdq"] = wdJradq;
    result["right_foot_position"] = crm;
    result["right_foot_velocity"] = dcrm;
    result["right_foot_rot_matrix"] = wRr;
    result["right_foot_ang_rate"] = wr;
    result["right_foot_Jw"] = wJwr;
    result["right_foot_dJwdq"] = wdJwrdq;

    result["left_toe_position"] = clt;
    result["left_toe_velocity"] = dclt;
    result["left_toe_Jv"] = wJlt;
    result["left_toe_dJvdq"] = wdJltdq;
    result["left_heel_position"] = clh;
    result["left_heel_velocity"] = dclh;
    result["left_heel_Jv"] = wJlh;
    result["left_heel_dJvdq"] = wdJlhdq;
    result["left_ankle_position"] = cla;
    result["left_ankle_velocity"] = dcla;
    result["left_ankle_Jv"] = wJla;
    result["left_ankle_dJvdq"] = wdJladq;
    result["left_foot_position"] = clm;
    result["left_foot_velocity"] = dclm;
    result["left_foot_rot_matrix"] = wRl;
    result["left_foot_ang_rate"] = wl;
    result["left_foot_Jw"] = wJwl;
    result["left_foot_dJwdq"] = wdJwldq;

    return result;
}
