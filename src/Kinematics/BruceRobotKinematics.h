#pragma once

#include "KinematicsInterface.h"
#include <memory>
#include "RobotModel.h"
#include <unordered_map>

class Kinematics: public KinematicsInterface{

    public:

        Kinematics(std::shared_ptr<RobotModel>);

        /**
         * @brief Inverse Kinematics of the foot
         * @param foot position MatrixXd(3,1) 0-x, 1-y, 2-z
         * @param foot direction MatrixXd(3,1) 0-roll, 1-picth, 2-yaw
         * @param leg index: 1-right, -1-left
         * 
         * @return vector of 5 elements
         * @return dq1,dq2,dq3,dq4,dq5
         */
        std::vector<double> inverseKinematicsFoot(Eigen::MatrixXd, Eigen::MatrixXd, LEG); 
        
        
        /**
         * @brief Inverse Kinematics of the ankle
         * @param foot position MatrixXd(3,1) 0-x, 1-y, 2-z
         * @param foot direction MatrixXd(3,1) 0-roll, 1-picth, 2-yaw
         * @param legindex: 1-right, -1-left
         * 
         * @return vector of 5 elements
         * @return dq1,dq2,dq3,dq4,dq5
         */
        std::vector<double> inverseKinematicsAnkle(Eigen::MatrixXd, Eigen::MatrixXd, LEG); 


        /** 
         * @brief Forward Kinematics Algotithm in robot's body frame
         * @param vector of 20 elements
         * @param 0-4 joints' angle of the right leg
         * @param 5-9 joints' angle of the left leg
         * @param 10-14 joints' angular velocity of the right leg
         * @param 15-19 joints' angular velocity of the left leg
         * 
         * @return unordered_map<string, MatrixXd>
         * @return keys: "b_right_toe_position", "b_right_toe_velocity", "b_right_toe_J", "b_right_toe_dJ",
         * @return       "b_right_heel_position", "b_right_heel_velocity", "b_right_heel_J", "b_right_heel_dJ",
         * @return       "b_right_ankle_position", "b_right_ankle_velocity", "b_right_ankle_J", "b_right_ankle_dJ",
         * @return       "b_right_foot_position", "b_right_foot_velocity", "b_right_foot_rot", "b_right_foot_Jw", "b_right_foot_dJw",
         * @return       "b_left_toe_position", "b_left_toe_velocity", "b_left_toe_J", "b_left_toe_dJ", 
         * @return       "b_left_heel_position", "b_left_heel_velocity", "b_left_heel_J", "b_left_heel_dJ",
         * @return       "b_left_ankle_position", "b_left_ankle_velocity", "b_left_ankle_J", "b_left_ankle_dJ",
         * @return       "b_left_foot_position", "b_left_foot_velocity", "b_left_foot_rot", "b_left_foot_Jw", "b_left_foot_dJw"
        */
        std::unordered_map<std::string, Eigen::MatrixXd> forwardKinematicsLeg(std::vector<double>);


        /**
         * @brief Forward Kinamatics Algorithm in world's frame
         * @param unordered_map<string, MatrixXd>
         * @param keys: "Rotation_Mat", "Body_Position", "Body_w", "Body_v",
         * @param       "b_right_toe_position", "b_right_toe_J", "b_right_toe_dJ",
         * @param       "b_right_heel_position", "b_right_heel_J", "b_right_heel_dJ",
         * @param       "b_right_ankle_position", "b_right_ankle_J", "b_right_ankle_dJ", "b_right_foot_rot", "b_right_foot_Jw", "b_right_foot_dJw",
         * @param       "b_left_toe_position", "b_left_toe_J", "b_left_toe_dJ",
         * @param       "b_left_heel_position", "b_left_heel_J", "b_left_heel_dJ",
         * @param       "b_left_ankle_position", "b_left_ankle_J", "b_left_ankle_dJ", "b_left_foot_rot", "b_left_foot_Jw", "b_left_foot_dJw",
         * @param       "rd1", "rd2", "rd3", "rd4", "rd5",
         * @param       "ld1", "ld2", "ld3", "ld4", "ld5"     
         * 
         * @return unordered_map<string, MatrixXd>
         * @return keys: "right_toe_position", "right_toe_velocity", "right_toe_Jv", "right_toe_dJvdq",
         * @return       "right_heel_position", "right_heel_velocity", "right_heel_Jv", "right_heel_dJvdq",
         * @return       "right_ankle_position", "right_ankle_velocity", "right_ankle_Jv", "right_ankle_dJvdq",
         * @return       "right_foot_position", "right_foot_velocity", "right_foot_rot_matrix", "right_foot_ang_rate", "right_foot_Jw", "right_foot_dJwdq",
         * @return       "left_toe_position", "left_toe_velocity", "left_toe_Jv", "left_toe_dJvdq",
         * @return       "left_heel_position", "left_heel_velocity", "left_heel_Jv", "left_heel_dJvdq",
         * @return       "left_ankle_position", "left_ankle_velocity", "left_ankle_Jv", "left_ankle_dJvdq",
         * @return       "left_foot_position", "left_foot_velocity", "left_foot_rot_matrix", "left_foot_ang_rate", "left_foot_Jw", "left_foot_dJwdq"
         */
        std::unordered_map<std::string, Eigen::MatrixXd> forwardKinematicsRobot(std::unordered_map<std::string, Eigen::MatrixXd>);


    private:
        std::shared_ptr<RobotModel> robotMode;
        //here I just assign fixed data in these parameters
        //later I will set these data automatically based on robot URDF model
        int numJointsPerLeg;
        double hx = 0.029216;
        double hy = 0.075856;
        double hz = 0.039765;
        double d2 = 0.0;
        double a3 = 0.204949;
        double a4 = 0.199881;
        double a5 = 0.024;
        double at = 0.05;
        double ah = 0.035;
        double a6 = ah;
};