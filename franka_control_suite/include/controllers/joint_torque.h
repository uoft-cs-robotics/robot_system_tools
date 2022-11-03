#pragma once
#include "base_controller.h"
	
#include <eigen3/Eigen/Dense>
#include <franka/rate_limiting.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>

class JointTorque: public ControllersBase{

public:
    JointTorque(int start); 
    ~JointTorque();
    franka::Torques operator()(const franka::RobotState&, franka::Duration);   

private: 
    Eigen::VectorXd joint_goal_torque_eigen;
};