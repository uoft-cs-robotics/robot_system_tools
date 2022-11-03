#include "controllers/osc.h"

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>



OSC::OSC(int start, OSCEEType ee_type=OSCEEType::POS, OSCImpedanceType imp_type=OSCImpedanceType::FIXED){
    count = start;
    osc_ee_type_ = ee_type;
    ee_goal_pose.resize(7);
    imp_type_ = imp_type;
}

OSC::~OSC(){}

franka::Torques OSC::operator() (const franka::RobotState& robot_state, franka::Duration period){
    std::vector<double> goal_pose; 
    Comms::actionSubscriber->readValues(goal_pose);
    ee_goal_pose = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(goal_pose.data(), goal_pose.size());

    if(first_step){
      first_step = false; 
      initial_joint_config = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((robot_state.q).data());
    }
    if((count-1)%4==0){
      std::vector<double> joints = {robot_state.q[0], robot_state.q[1], robot_state.q[2], robot_state.q[3],robot_state.q[4], robot_state.q[5], robot_state.q[6]};
      Comms::statePublisher->writeMessage(joints);
    }
    count ++;
    Eigen::Affine3d ee_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d ee_translation_now(ee_transform.translation()), 
                    target_ee_translation;      
    Eigen::Quaterniond ee_ori_now_quat(ee_transform.linear());
    ee_ori_now_quat.normalize();
    Eigen::Quaterniond  target_ee_ori_quat;    

    if (ControllersBase::control_mode_ == ControlMode::DELTA){
        target_ee_translation = ee_translation_now + Eigen::Vector3d(ee_goal_pose.head(3)); 
        //Eigen::Vector4d ee_ori_goal_vec = ee_goal_pose.tail(4);
        Eigen::Quaterniond ee_ori_goal_vec(Eigen::Vector4d(ee_goal_pose.tail(4)));
        ee_ori_goal_vec.normalize();
        target_ee_ori_quat = utils::quat_multiplication(ee_ori_goal_vec, 
                                                        ee_ori_now_quat);
    }
    else{
        target_ee_translation = Eigen::Vector3d(ee_goal_pose.head(3));
        Eigen::Vector4d ee_ori_goal_vec = ee_goal_pose.tail(4);
        target_ee_ori_quat = Eigen::Quaterniond(ee_ori_goal_vec);
        target_ee_ori_quat.normalize();
    }

    Eigen::Vector3d ee_position_error = target_ee_translation - ee_translation_now;
    Eigen::Matrix<double, 3, 1> ori_error = utils::get_ori_error_matrix(target_ee_ori_quat, ee_ori_now_quat); 

    // joint space mass matrix
    auto massArray = robotContext::model->mass(robot_state);    
    // joint velocity
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> joint_velocity((robot_state.dq).data());  
    Eigen::Matrix<double, 7, 1> joint_velocity_error = - joint_velocity; 
    auto jacobianArray = robotContext::model->zeroJacobian(franka::Frame::kEndEffector, robot_state); 

    // convert to eigen objects
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobianArray.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> joint_pos(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(massArray.data());
    
    // ee velocity
    Eigen::Matrix<double, 6, 1> ee_velocity = jacobian * joint_velocity;
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> k_s_eigen(k_s.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> k_d_eigen(k_d.data());
    Eigen::Matrix<double, 3, 1> desired_ee_force, desired_ee_torque; 

    desired_ee_force = k_s_eigen.block<3,1>(0,0).cwiseProduct(ee_position_error) + 
                    k_d_eigen.block<3,1>(0,0).cwiseProduct(joint_velocity_error.block<3,1>(0,0));
    
    desired_ee_torque = k_s_eigen.block<3,1>(3,0).cwiseProduct(ori_error) + 
                     k_d_eigen.block<3,1>(3,0).cwiseProduct(joint_velocity_error.block<3,1>(3,0));
    
    franka::Torques output_joint_torque = _compute_output_torque(jacobian, 
                                                          mass, 
                                                          ee_velocity,
                                                          joint_pos, 
                                                          joint_velocity,
                                                          desired_ee_force,
                                                          desired_ee_torque);
  
    return output_joint_torque;
}


franka::Torques OSC::_compute_output_torque(const Eigen::Matrix<double, 6, 7> &jacobian, 
                                      const Eigen::Matrix<double, 7, 7> &mass, 
                                      const Eigen::Matrix<double, 6, 1> &ee_velocity,
                                      const Eigen::Matrix<double, 7, 1> &current_joint_pos, 
                                      const Eigen::Matrix<double, 7, 1> &current_joint_vel,                                       
                                      const Eigen::Matrix<double, 3, 1> &desired_ee_force, 
                                      const Eigen::Matrix<double, 3, 1> &desired_ee_torque){
  Eigen::Matrix<double, 7, 7> mass_inv = mass.inverse();
  Eigen::Matrix<double, 7, 6> jacobian_full_T = jacobian.transpose(); 
  Eigen::Matrix<double, 7, 3> jacobian_pos_T = (jacobian.block<3,7>(0,0)).transpose(); 
  Eigen::Matrix<double, 7, 3> jacobian_ori_T = (jacobian.block<3,7>(3,0)).transpose(); 

  Eigen::Matrix<double,6 ,6> lambda_full_inv = jacobian * mass_inv * jacobian_full_T; 
  Eigen::Matrix<double,3 ,3> lambda_pos_inv = (jacobian.block<3,7>(0,0)) * mass_inv * jacobian_pos_T; 
  Eigen::Matrix<double,3 ,3> lambda_ori_inv = (jacobian.block<3,7>(3,0)) * mass_inv * jacobian_pos_T;

  Eigen::Matrix<double,6 ,6> lambda_full = lambda_full.inverse();
  Eigen::Matrix<double,3 ,3> lambda_pos = lambda_pos.inverse();
  Eigen::Matrix<double,3 ,3> lambda_ori = lambda_ori.inverse(); 

  //nulspace 
  Eigen::Matrix<double, 7, 6> Jbar = mass_inv * jacobian_full_T * lambda_full;
  Eigen::Matrix<double, 7, 7> nullspace_matrix = Eigen::MatrixXd::Identity(7,7) - Jbar * jacobian;
  Eigen::Matrix<double, 6, 1> decoupled_wrench; 
  if (uncoupling){
    Eigen::Matrix<double, 3, 1> decoupled_force = lambda_pos * desired_ee_force; 
    Eigen::Matrix<double, 3, 1> decoupled_torque = lambda_ori * desired_ee_torque; 
    decoupled_wrench << decoupled_force, decoupled_torque; 
  }
  else{
    Eigen::Matrix<double, 6, 1> desired_wrench; 
    desired_wrench << desired_ee_force, desired_ee_torque; 
    decoupled_wrench = lambda_full * desired_wrench; 
  }
  Eigen::Matrix<double, 7, 1> output_torque = jacobian_full_T * decoupled_wrench; 

  double joint_kp = 10.0, joint_kv = 2.0* sqrt(joint_kp); 
  Eigen::Matrix<double, 7, 1> pose_torques = mass * (joint_kp * (initial_joint_config, current_joint_pos) - 
                                                    joint_kv * current_joint_pos);                                                  
  Eigen::Matrix<double, 7, 1> nullspace_torques = (nullspace_matrix.transpose()) * pose_torques; 
  output_torque = output_torque + nullspace_torques; 
  std::array<double, 7> output_torque_array{};
  Eigen::VectorXd::Map(&output_torque_array[0], 7) = output_torque;
  return output_torque_array;
}




    

//clip k_s commands 