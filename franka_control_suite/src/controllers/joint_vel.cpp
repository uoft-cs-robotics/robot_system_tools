#include "controllers/joint_vel.h"

#include <algorithm>
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <chrono>
#include <numeric>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>

JointVelocity::JointVelocity(int start, bool zmq_comms_flag){
    count = start; 
    joint_goal_vel_eigen.resize(DOF);
    prev_joint_vel_error.resize(DOF);
    summed_joint_vel_error.resize(DOF);
    joint_vel_error_buffer.resize(100);//store last 100 joint vel error dot
    //sets to zero
    prev_joint_vel_error -= prev_joint_vel_error;
    summed_joint_vel_error -= summed_joint_vel_error; 
    zmq_comms = zmq_comms_flag;
    for(auto& vel_error: joint_vel_error_buffer)
        vel_error = Eigen::MatrixXd::Zero(DOF,1);
}


JointVelocity::~JointVelocity(){}

franka::Torques JointVelocity::operator()(const franka::RobotState& robot_state, franka::Duration period){
    
    if(zmq_comms){
        std::vector<double> joint_vel_goal; 
        Comms::actionSubscriber->readValues(joint_vel_goal);
        clamp_joint_velocities(joint_vel_goal);
        joint_goal_vel_eigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(joint_vel_goal.data(), joint_vel_goal.size());
        
        if((count-1)%4==0){
        std::vector<double> joints = {robot_state.q[0], robot_state.q[1], robot_state.q[2], robot_state.q[3],robot_state.q[4], robot_state.q[5], robot_state.q[6]};
        Comms::statePublisher->writeMessage(joints);
        }
        count++; 
    }
    else{
        joint_goal_vel_eigen = Eigen::Map<const Eigen::Matrix<double, DOF, 1>>((robot_state.dq_d).data());
    }
    std::array<double, DOF> k_s_ = {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0,10.0}};
    Eigen::Map<const Eigen::Matrix<double, DOF, 1>> k_s_eigen(k_s_.data());
    //Eigen::Map<const Eigen::Matrix<double, DOF, 1>> k_d_eigen(k_d.data()); 
    // k_i and k_d values taken from robosuite
    Eigen::Matrix<double, DOF, 1> k_i_eigen = k_s_eigen * 0.005;
    Eigen::Matrix<double, DOF, 1> k_d_eigen = 0.001*k_s_eigen;


    Eigen::Map<const Eigen::Matrix<double, DOF, 1>> joint_vel_now_eigen((robot_state.dq).data());
    Eigen::Matrix<double, DOF, 1> joint_vel_error = joint_goal_vel_eigen - joint_vel_now_eigen; 
    if(!is_saturated)
        summed_joint_vel_error += joint_vel_error; 


    Eigen::Matrix<double, DOF, 1> joint_vel_error_dot = joint_vel_error - prev_joint_vel_error;
    prev_joint_vel_error = joint_vel_error;
    joint_vel_error_buffer.push_back(joint_vel_error_dot);
    
    joint_vel_error_buffer.erase(joint_vel_error_buffer.begin());

    assert(joint_vel_error_buffer.size()==100);
    Eigen::Matrix<double, DOF, 1> average_joint_vel_error_dot; average_joint_vel_error_dot ;

    Eigen::Matrix<double, DOF, 1> ini = Eigen::MatrixXd::Zero(DOF,1);
    average_joint_vel_error_dot = std::accumulate(joint_vel_error_buffer.begin(),
                                                joint_vel_error_buffer.end(), 
                                                ini)/ joint_vel_error_buffer.size();
    

    Eigen::Matrix<double, DOF, 1> desired_tau_eigen = k_s_eigen.cwiseProduct(joint_vel_error) + 
                                                    k_i_eigen.cwiseProduct(summed_joint_vel_error) + 
                                                    k_d_eigen.cwiseProduct(average_joint_vel_error_dot);

    std::array<double, DOF> desired_tau_calculated, desired_tau_rate_limited; 
    Eigen::VectorXd::Map(&desired_tau_calculated[0], DOF) = desired_tau_eigen;
    std::array<double, DOF> clamped_torque = clamp_torque(desired_tau_calculated);
    is_saturated = check_if_saturated(desired_tau_calculated, clamped_torque);
    desired_tau_rate_limited =franka::limitRate(franka::kMaxTorqueRate, clamped_torque, robot_state.tau_J_d);
    return desired_tau_rate_limited;

}



bool JointVelocity::check_if_saturated(const std::array<double, DOF> &calculated_torque, 
                                    const std::array<double, DOF> &clamped_torque){
    double total_diff = 0.0; 
    for(size_t i =0; i < DOF; i++ ){
        total_diff += abs(calculated_torque[i] - clamped_torque[i]);
    }

    if(total_diff == 0.0)
        return false; 
    else 
        return true;

}

void JointVelocity::clamp_joint_velocities(std::vector<double> &joint_vel_goal){
    for (size_t i =0 ; i< DOF; i++){
        if (joint_vel_goal[i] > joint_vel_max[i])
            joint_vel_goal[i] = joint_vel_max[i];        
    }
}