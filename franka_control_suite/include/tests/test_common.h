#pragma once


#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <iterator>
#include <thread>
#include <atomic>
#include <fstream>

#include "context.h"
// Initialize data fields for the print thread.
struct {
    std::mutex mutex;
    bool has_data;
    std::array<double, 7> tau_d_last;
    franka::RobotState robot_state;
    std::array<double, 7> gravity;
} print_data{};

using StructType = decltype(print_data);
void log_data(const double& print_rate, StructType& print_data, std::atomic_bool& running, std::string& file_name, const bool&joint_space=true)
{
  std::fstream log_file(file_name);
  
  while (running) {
  
  // Sleep to achieve the desired print rate.
  std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));
      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock()) {
        log_file.open(file_name, std::ios::app);
        if (print_data.has_data) {  
          // log_file.seekp(0,std::ios::end);
          if(joint_space){
            
            for(size_t i =0; i < print_data.robot_state.q.size(); i++)
              log_file<<print_data.robot_state.q[i]<<",";
            for(size_t i =0; i < print_data.robot_state.q_d.size(); i++)
              log_file<<print_data.robot_state.q_d[i]<<",";              
            for(size_t i =0; i < print_data.robot_state.dq.size(); i++)
              log_file<<print_data.robot_state.dq[i]<<",";           
            for(size_t i =0; i < print_data.robot_state.dq_d.size(); i++)
              log_file<<print_data.robot_state.dq_d[i]<<",";    
            log_file<<"\n"; 
          }
          else{
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> joint_vel((print_data.robot_state.dq).data());
            auto jacobian_array = robotContext::model->zeroJacobian(franka::Frame::kEndEffector, print_data.robot_state);
            Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
            Eigen::Array<double, 6, 1> ee_current_velocity = (jacobian * joint_vel).array();
            std::cout<<print_data.robot_state.O_T_EE[12]<<"\n";
            double x_current = print_data.robot_state.O_T_EE[12],
                  y_current = print_data.robot_state.O_T_EE[13],
                  z_current = print_data.robot_state.O_T_EE[14];
            double x_desired = print_data.robot_state.O_T_EE_d[12],
                  y_desired = print_data.robot_state.O_T_EE_d[13],
                  z_desired = print_data.robot_state.O_T_EE_d[14];  
            log_file<<x_current<<","
                    <<y_current<<","
                    <<z_current<<","
                    <<x_desired<<","
                    <<y_desired<<","
                    <<z_desired<<",";                    
            for(size_t i =0; i < ee_current_velocity.size(); i++)
              log_file<<ee_current_velocity[i]<<","; 
            for(size_t i =0; i < print_data.robot_state.O_dP_EE_d.size(); i++)
              log_file<<print_data.robot_state.O_dP_EE_d[i]<<",";      
            log_file<<"\n";   
            
                        
          }
          print_data.has_data = false;
          log_file.close(); 
        }
        
        print_data.mutex.unlock();
      }  
    
    
  }
  
  std::cout<<"closing\n";
  

}