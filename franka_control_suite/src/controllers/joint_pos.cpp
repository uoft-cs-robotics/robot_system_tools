#include "controllers/joint_pos.h"

#include <algorithm>
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <chrono>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>

JointPosition::JointPosition(int start,  bool zmq_comms_flag){
    count = start; 
    joint_goal_pos_eigen.resize(DOF);
    zmq_comms = zmq_comms_flag;
}

JointPosition::~JointPosition(){}

franka::Torques JointPosition::operator()(const franka::RobotState& robot_state, franka::Duration period){

    
    if(zmq_comms){
        std::vector<double> joint_pos_goal; 
        Comms::actionSubscriber->readValues(joint_pos_goal);
        clamp_joint_angles(joint_pos_goal);

        joint_goal_pos_eigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(joint_pos_goal.data(), joint_pos_goal.size());
        if((count-1)%4==0){
            std::vector<double> joints = {robot_state.q[0], robot_state.q[1], robot_state.q[2], robot_state.q[3],robot_state.q[4], robot_state.q[5], robot_state.q[6]};
            Comms::statePublisher->writeMessage(joints);
        }
        count++; 
    }
    else{
        joint_goal_pos_eigen = Eigen::Map<const Eigen::Matrix<double, DOF, 1>> ((robot_state.q_d).data());
    }
    
    Eigen::Map<const Eigen::Matrix<double, DOF, 1>> k_s_eigen(k_s.data());
    Eigen::Map<const Eigen::Matrix<double, DOF, 1>> k_d_eigen(k_d.data());


    Eigen::Map<const Eigen::Matrix<double, DOF, 1>> joint_pos_now((robot_state.q).data());
    Eigen::Map<const Eigen::Matrix<double, DOF, 1>> joint_vel_now((robot_state.dq).data());

    Eigen::Matrix<double, DOF, 1> joint_pos_error = joint_goal_pos_eigen - joint_pos_now;
    Eigen::Matrix<double, DOF, 1> joint_vel_error = -joint_vel_now;
    
    Eigen::Matrix<double, DOF, 1> desired_tau_eigen = k_s_eigen.cwiseProduct(joint_pos_error) + k_d_eigen.cwiseProduct(joint_vel_error); 
    
    std::array<double, DOF> desired_tau_calculated, desired_tau_rate_limited; 
    std::array<double, DOF> output_torque_array;
    Eigen::VectorXd::Map(&desired_tau_calculated[0], DOF) = desired_tau_eigen;
    output_torque_array = clamp_torque(desired_tau_calculated);
    desired_tau_rate_limited = franka::limitRate(franka::kMaxTorqueRate, output_torque_array, robot_state.tau_J_d);

    return desired_tau_rate_limited;
    
}


void JointPosition::clamp_joint_angles(std::vector<double> &joint_pos_goal){
    for (size_t i =0 ; i< DOF; i++){
        if (joint_pos_goal[i] > joint_max[i])
            joint_pos_goal[i] = joint_max[i];
        if (joint_pos_goal[i] < joint_min[i])
            joint_pos_goal[i] = joint_min[i];         
    }
}
