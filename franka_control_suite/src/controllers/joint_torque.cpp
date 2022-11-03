#include "controllers/joint_torque.h"

#include <algorithm>
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <chrono>



JointTorque::JointTorque(int start){
    count = start; 
    joint_goal_torque_eigen.resize(DOF); 
}

JointTorque::~JointTorque(){}

franka::Torques JointTorque::operator()(const franka::RobotState& robot_state, franka::Duration period){

    std::vector<double> joint_torque_goal; 
    Comms::actionSubscriber->readValues(joint_torque_goal);
    std::array<double, DOF> desired_tau, clipped_tau, desired_tau_rate_limited;
    std::copy(joint_torque_goal.begin(), joint_torque_goal.end(), desired_tau.begin());//convert torque command from eigen to std::array
    
    clipped_tau = clamp_torque(desired_tau);

    desired_tau_rate_limited = franka::limitRate(franka::kMaxTorqueRate, clipped_tau, robot_state.tau_J_d);

    return desired_tau_rate_limited;

}