#pragma once
#include "base_controller.h"
	
#include <eigen3/Eigen/Dense>
#include <franka/rate_limiting.h>

enum class OSCEEType{
    ORI,
    POS,
    POS_ORI,
};

enum class OSCImpedanceType{
    FIXED,
    VARIABLE_KP,
    VARIABLE,
};



class OSC: public ControllersBase{

public:
    OSC(int start, OSCEEType ee_type, OSCImpedanceType imp_type); 
    ~OSC();
    franka::Torques operator()(const franka::RobotState&, franka::Duration);
private: 
    OSCEEType osc_ee_type_; 
    OSCImpedanceType imp_type_;
    //std::array<double, 6> pose_command = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}; 
    Eigen::VectorXd ee_goal_pose;  
    bool uncoupling = true, first_step = true; 
    Eigen::Matrix<double, 7, 1> initial_joint_config;                                                       
    franka::Torques _compute_output_torque(const Eigen::Matrix<double, 6, 7> &jacobian, 
                                      const Eigen::Matrix<double, 7, 7> &mass, 
                                      const Eigen::Matrix<double, 6, 1> &ee_velocity,
                                      const Eigen::Matrix<double, 7, 1> &current_joint_pos, 
                                      const Eigen::Matrix<double, 7, 1> &current_joint_vel,                                       
                                      const Eigen::Matrix<double, 3, 1> &desired_ee_force,                                      
                                      const Eigen::Matrix<double, 3, 1> &desired_ee_torque);
};