#include "controllers/cartesian_impedance.h"

#include <algorithm>
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <chrono>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>

CartesianImpedance::CartesianImpedance(int start,  bool zmq_comms_flag){
    count = start; 
    zmq_comms = zmq_comms_flag;
    //
    const double translational_stiffness{150.0};
    const double rotational_stiffness{10.0};
    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    damping.setZero();
    damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                        Eigen::MatrixXd::Identity(3, 3); 
    damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                            Eigen::MatrixXd::Identity(3, 3);    
}

CartesianImpedance::~CartesianImpedance(){}

franka::Torques CartesianImpedance::operator()(const franka::RobotState& robot_state, franka::Duration period){

    
    if(zmq_comms){
        // std::vector<double> joint_pos_goal; 
        // Comms::actionSubscriber->readValues(joint_pos_goal);
        // clamp_joint_angles(joint_pos_goal);

        // cartesian_goal_pos_eigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(joint_pos_goal.data(), joint_pos_goal.size());
        // if((count-1)%4==0){ Eigen::MatrixXd
        //     std::vector<double> joints = {robot_state.q[0], robot_state.q[1], robot_state.q[2], robot_state.q[3],robot_state.q[4], robot_state.q[5], robot_state.q[6]};
        //     Comms::statePublisher->writeMessage(joints);
        // }
        // count++; 
    }
    else{
        cartesian_goal_transform_eigen = Eigen::Matrix4d::Map(robot_state.O_T_EE_d.data());
    }

    // get robot_state variables
    std::array<double, 7> coriolis_array = robotContext::model->coriolis(robot_state);
    std::array<double, 42> jacobian_array = robotContext::model->zeroJacobian(franka::Frame::kEndEffector, robot_state);  

    // convert to Eigen
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());

    Eigen::Vector3d position_d(cartesian_goal_transform_eigen.translation());
    Eigen::Quaterniond orientation_d(cartesian_goal_transform_eigen.linear());

    // compute error to desired equilibrium pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d;

    // orientation error
    // "difference" quaternion
    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -transform.linear() * error.tail(3);        

    // compute control
    Eigen::VectorXd tau_task(7), tau_d(7);
    std::array<double, 7> tau_J_d, tau_J_d_rate_limited;  // NOLINT(readability-identifier-naming)

    // Spring damper system with damping ratio=1
    tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
    tau_d << tau_task + coriolis;
    Eigen::VectorXd::Map(&tau_J_d[0], 7) = tau_d;
    tau_J_d = clamp_torque(tau_J_d);
    tau_J_d_rate_limited = franka::limitRate(franka::kMaxTorqueRate, tau_J_d, robot_state.tau_J_d);

    return tau_J_d_rate_limited;
    
}
