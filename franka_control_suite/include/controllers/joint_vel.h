#pragma once
#include "base_controller.h"
	
#include <eigen3/Eigen/Dense>
#include <franka/rate_limiting.h>

class JointVelocity: public ControllersBase{

public:
    JointVelocity(int start, bool zmq_comms_flag=false); 
    ~JointVelocity();
    franka::Torques operator()(const franka::RobotState&, franka::Duration); 

private: 
    Eigen::VectorXd joint_goal_vel_eigen, summed_joint_vel_error, prev_joint_vel_error;
    std::vector<Eigen::VectorXd> joint_vel_error_buffer; 
    void clamp_joint_velocities(std::vector<double> &joint_vel_goal);
    bool is_saturated = false; 
    bool check_if_saturated(const std::array<double, DOF> &calculated_torque, 
                                    const std::array<double, DOF> &clamped_torque);
    
};