#include "controllers/ik.h"

#include <algorithm>
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <chrono>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>

/*
Constructor must have atleast one parameter. Because when robot.cotrol()is run, 
it tries to pass the () operator with this class names, which will conflict with the base constructor
*/
InverseKinematics::InverseKinematics(int start, IKType type=IKType::M_P_PSEUDO_INVERSE){
  count =start;
  type_ = type;
  ee_goal_pose.resize(DOF);    
}

InverseKinematics::~InverseKinematics(){}



franka::JointVelocities InverseKinematics::operator() (const franka::RobotState& robot_state, franka::Duration period){
    //reads reference signal every 10ms
    // if((count-1) % 10 == 0) {
    //     Comms::actionSubscriber->readMessage();
    //     ee_goal_pose << Comms::actionSubscriber->ee_goal_pose[0],
    //                     Comms::actionSubscriber->ee_goal_pose[1],
    //                     Comms::actionSubscriber->ee_goal_pose[2],
    //                     Comms::actionSubscriber->ee_goal_pose[3],
    //                     Comms::actionSubscriber->ee_goal_pose[4],
    //                     Comms::actionSubscriber->ee_goal_pose[5],
    //                     Comms::actionSubscriber->ee_goal_pose[6];
    // }
    if((count-1)%4==0){
      std::vector<double> joints = {robot_state.O_T_EE[0], robot_state.O_T_EE[1], robot_state.O_T_EE[2], robot_state.O_T_EE[3],robot_state.O_T_EE[4], robot_state.O_T_EE[5], robot_state.O_T_EE[6],
                                  robot_state.O_T_EE[7], robot_state.O_T_EE[8], robot_state.O_T_EE[9], robot_state.O_T_EE[10], robot_state.O_T_EE[11], robot_state.O_T_EE[12], robot_state.O_T_EE[13],
                                  robot_state.O_T_EE[14], robot_state.O_T_EE[15]};
      Comms::statePublisher->writeMessage(joints);
    }    
    std::vector<double> goal_pose; 
    Comms::actionSubscriber->readValues(goal_pose);
    ee_goal_pose = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(goal_pose.data(), goal_pose.size());
    count++;
    Eigen::Affine3d ee_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d ee_translation_now(ee_transform.translation()), 
                    target_ee_translation;      
    Eigen::Quaterniond ee_ori_now_quat(ee_transform.linear());
    ee_ori_now_quat.normalize();
    Eigen::Quaterniond  target_ee_ori_quat;
    //Eigen::AngleAxisd ee_orientation_axis_now(ee_orientation_quat_now);
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
    auto jacobianArray = robotContext::model->zeroJacobian(franka::Frame::kEndEffector, robot_state); 
    Eigen::Map<const Eigen::Matrix<double, 6, DOF>> jacobian(jacobianArray.data());
    Eigen::Map<const Eigen::Matrix<double, DOF, 1>> joint_angles_now(robot_state.q.data());
    Eigen::Vector3d ee_position_error = target_ee_translation - ee_translation_now;
    Eigen::AngleAxisd ee_ori_error_aa = utils::get_ori_error_aa(target_ee_ori_quat, ee_ori_now_quat);
    Eigen::Quaterniond ee_ori_error_q = utils::get_ori_error_quat(target_ee_ori_quat, ee_ori_now_quat);
    std::array<double, DOF> output_arr = _compute_target_joint_angle_pos(joint_angles_now,
                                                                      jacobian, 
                                                                      ee_position_error, 
                                                                      ee_ori_error_aa);
    std::array<double, DOF> desired_j_pos_limited = franka::limitRate(franka::kMaxJointVelocity,
                                                                franka::kMaxJointAcceleration, 
                                                                franka::kMaxJointJerk,
                                                                output_arr,
                                                                robot_state.dq_d,
                                                                robot_state.ddq_d);
    franka::JointVelocities output_limited(desired_j_pos_limited);                                                                   
    return output_limited;                                                                
} 
std::array<double,DOF> InverseKinematics::_compute_target_joint_angle_pos(const Eigen::Matrix<double, DOF, 1> &q_now, 
                                                        const Eigen::Matrix<double, 6, DOF> &jacobian,
                                                        const Eigen::Vector3d &ee_position_error,
                                                        const Eigen::AngleAxisd &ee_ori_error_aa){
    Eigen::Vector3d ee_ori_error{ee_ori_error_aa.angle() * ee_ori_error_aa.axis()};                                                          
    Eigen::Matrix<double, DOF, 1> delta_joint_angles = _get_delta_joint_angles(jacobian, 
                                                                            ee_position_error,
                                                                            ee_ori_error) ;
    std::array<double, DOF> desired_joint_positions;
 
    Eigen::VectorXd::Map(&desired_joint_positions[0], DOF) = delta_joint_angles ;
    return desired_joint_positions;
}

//gets position and orientation error and return delta joint angles 
Eigen::Matrix<double, DOF, 1> InverseKinematics::_get_delta_joint_angles(const Eigen::Matrix<double, 6, DOF> &jacobian,
                                                    const Eigen::Vector3d &ee_position_error,
                                                    const Eigen::Vector3d &ee_ori_error){
    Eigen::Matrix<double, 6, 1> delta_ee_pose;
    delta_ee_pose << ee_position_error, ee_ori_error; 
    if(type_ == IKType::DAMPED_LS){
      Eigen::Matrix<double, DOF, 6> jacobian_T = jacobian.transpose();
      double lambda_val = 0.1;
      Eigen::Matrix<double, 6, 6> lambda_matrix = lambda_val * lambda_val * Eigen::Matrix< double, 6, 6 >::Identity(); 
      return jacobian_T * ((jacobian * jacobian_T + lambda_matrix).inverse()) * delta_ee_pose;
    }
    else if(type_ == IKType::M_P_PSEUDO_INVERSE){
      Eigen::MatrixXd jacobian_inverse = pseudoInverse<Eigen::MatrixXd>(jacobian);
      return jacobian_inverse * delta_ee_pose;
    }                
    else if (type_ == IKType::JACOBIAN_TRANSPOSE){
      Eigen::Matrix<double, DOF, 6> jacobian_T = jacobian.transpose();
      Eigen::Matrix<double, 6, 1> temp_variable = jacobian * jacobian_T * delta_ee_pose;
      double alpha = delta_ee_pose.dot(temp_variable) / temp_variable.dot(temp_variable);
      return alpha * jacobian_T * delta_ee_pose;
    }       
    // else if(type_ == IKType::ADAPTIVE_SVD){
    //   Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //   Eigen::Matrix<double, 6, 1> singular_values = svd.singularValues(); 
    //   Eigen::Matrix<double, DOF, 6> E_inv =  Eigen::Matrix<double, DOF, 6>::Zero();
    //   for(int i = 0; i < 6; i++){
    //     E_inv(i,i) = 1/singular_values(i);
    //   }
    //   std::cout<<"e "<<E_inv.rows()<<","<<E_inv.cols() <<"\n";
    //   std::cout<<"u "<<svd.matrixU().rows()<<","<<svd.matrixU().cols() <<"\n";
    //   std::cout<<"v "<<svd.matrixV().rows()<<","<<svd.matrixV().cols() <<"\n";
    //   return (svd.matrixV() * E_inv * ((svd.matrixU()).transpose())) * delta_ee_pose;
    // }             
}

