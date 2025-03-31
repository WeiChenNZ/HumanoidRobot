#pragma once

#include "Eigen/Core"
#include <unordered_map>



class BruceRobotStatus{

    public:

        //public data so you can access them directly
        double timeStamp;

        //body
        Eigen::MatrixXd bodyPositionInWorld; //p_wb
        Eigen::MatrixXd bodyVelocityInWorld; //v_wb
        Eigen::MatrixXd bodyAccelerationInWorld; //a_wb
        Eigen::MatrixXd bodyRotInWorld; //R_wb
        Eigen::MatrixXd bodyOmegaInBody; //w_bb
        double          yaw; //yaw
        Eigen::MatrixXd RotOfYaw; //Rot of yaw

        //center of mass
        Eigen::MatrixXd comPositionInWorld; //p_wg
        Eigen::MatrixXd comVelocityInWorld; //p_wg
        Eigen::MatrixXd comAngularMomentumInWorld; //k_wg

        //dynamics
        Eigen::MatrixXd H; //H
        Eigen::MatrixXd CG; //C+G
        Eigen::MatrixXd AG; //AG
        Eigen::MatrixXd dAGdq; //dAGdq

        //foot contacts
        Eigen::MatrixXd footContacts; //foot_contacts

        //right foot
        Eigen::MatrixXd RotRightFootInWorld; //R_wf_r
        Eigen::MatrixXd omegaRightFoot; //w_ff_r
        Eigen::MatrixXd JwRightFoot; //Jw_ff_r
        Eigen::MatrixXd dJwdqRightFoot; //dJwdq_ff_r
        Eigen::MatrixXd positionRightFootInWorld; //p_wf_r
        Eigen::MatrixXd velocityRightFootInWorld; //v_wf_r
        Eigen::MatrixXd positionRightToeInWorld; //p_wt_r
        Eigen::MatrixXd velocityRightToeInWorld; //v_wt_r
        Eigen::MatrixXd JvRightToeInWorld; //Jv_wt_r
        Eigen::MatrixXd dJvdqRightToeInWorld; //dJvdq_wt_r
        Eigen::MatrixXd positionRightHeelInWorld; //p_wh_r
        Eigen::MatrixXd velocityRightHeelInWorld; //p_wh_r
        Eigen::MatrixXd JvRightHeelInWorld; //Jv_wh_r
        Eigen::MatrixXd dJvdqRightHeelInWorld; //dJvdq_wh_r
        Eigen::MatrixXd positionRightAnkleInWorld; //p_wa_r
        Eigen::MatrixXd velocityRightAnkleInWorld; //p_wa_r
        Eigen::MatrixXd JvRightAnkleInWorld; //Jv_wa_r
        Eigen::MatrixXd dJvdqRightAnkleInWorld; //dJvdq_wa_r

        //left foot
        Eigen::MatrixXd RotLeftFootInWorld; //R_wf_l
        Eigen::MatrixXd omegaLeftFoot; //w_ff_l
        Eigen::MatrixXd JwLeftFoot; //Jw_ff_l
        Eigen::MatrixXd dJwdqLeftFoot; //dJwdq_ff_l
        Eigen::MatrixXd positionLeftFootInWorld; //p_wf_l
        Eigen::MatrixXd velocityLeftFootInWorld; //v_wf_l
        Eigen::MatrixXd positionLeftToeInWorld; //p_wt_l
        Eigen::MatrixXd velocityLeftToeInWorld; //v_wt_l
        Eigen::MatrixXd JvLeftToeInWorld; //Jv_wt_l
        Eigen::MatrixXd dJvdqLeftToeInWorld; //dJvdq_wt_l
        Eigen::MatrixXd positionLeftHeelInWorld; //p_wh_l
        Eigen::MatrixXd velocityLeftHeelInWorld; //p_wh_l
        Eigen::MatrixXd JvLeftHeelInWorld; //Jv_wh_l
        Eigen::MatrixXd dJvdqLeftHeelInWorld; //dJvdq_wh_l
        Eigen::MatrixXd positionLeftAnkleInWorld; //p_wa_l
        Eigen::MatrixXd velocityLeftAnkleInWorld; //p_wa_l
        Eigen::MatrixXd JvLeftAnkleInWorld; //Jv_wa_l
        Eigen::MatrixXd dJvdqLeftAnkleInWorld; //dJvdq_wa_l

        //joints
        //{"key": {"key": value}}
        std::unordered_map<std::string, std::unordered_map<std::string, Eigen::MatrixXd>> joints;
        
        //gamepad data
        std::unordered_map<std::string, Eigen::MatrixXd> gamepad;


        //public methods
        void updateRobotStatus(void);
        void updateRobotDCMstatus(void);
        void updateSenseStatus(void);
        void updatePlanStatus(void);
        void updateInputStatus(void);
        void updateLegStatus(void);

        void setCommandLegPositions(void);
        void setCommandLegTorques(void);
        void setCommandLegValues(void);

        void updateArmStatus(void);
        void setCommandArmPositions(void);

        void updateGamepadStatus(void);

        static void stopRobot(void);
        static void dampingRobot(void);
        static bool isDamping(void);
        static double getTime(void);
        static void sleep(double t);

        //could not be implemented
        bool threadError(double dt);
        static void stopThreading(void);

};