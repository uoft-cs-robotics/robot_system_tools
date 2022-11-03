#pragma once
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <Poco/DateTimeFormatter.h>
#include <Poco/File.h>
#include <Poco/Path.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include <thread>
#include "common.h"
#include "tests/test_common.h"
#include <atomic>
#include <eigen3/Eigen/Dense>
namespace test_params{
    double vel_current = 0.0;
    const double acceleration_time = 2.0;
    const double runtime = 20.0;
    double angle = 0.0;
    const double radius = 0.15;
    const double vel_max = 0.25;
    const double vel_time_max = 1.0;
    const double omega_max = 1.0;
    const double cart_vel_time_max = 4.0;
    const double cart_vel_v_max = 0.1;
    const double cart_vel_angle = M_PI / 4.0;
}
franka::JointPositions test_joint_pos_motion_generator(const franka::RobotState&robot_state, franka::Duration period, const std::array<double, 7>& initial_position, double& index){
    
    double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * index));
    franka::JointPositions output = {{initial_position[0], initial_position[1],
                                    initial_position[2], initial_position[3] + delta_angle,
                                    initial_position[4] + delta_angle, initial_position[5],
                                    initial_position[6] + delta_angle}};    
    index += period.toSec();                                        
    return output;  
}

franka::CartesianPose test_cartesian_pose_motion_generator(const franka::RobotState&robot_state, franka::Duration period,const std::array<double, 16>& initial_pose, double& index){
    
        // Compute Cartesian velocity.
    if (test_params::vel_current < test_params::vel_max && index < test_params::runtime) {
        test_params::vel_current += period.toSec() * std::fabs(test_params::vel_max / test_params::acceleration_time);
    }
    if (test_params::vel_current > 0.0 && index > test_params::runtime) {
        test_params::vel_current -= period.toSec() * std::fabs(test_params::vel_max / test_params::acceleration_time);
    }
    test_params::vel_current = std::fmax(test_params::vel_current, 0.0);
    test_params::vel_current = std::fmin(test_params::vel_current, test_params::vel_max);

    // Compute new angle for our circular trajectory.
    test_params::angle += period.toSec() * test_params::vel_current / std::fabs(test_params::radius);
    if (test_params::angle > 2 * M_PI) {
        test_params::angle -= 2 * M_PI;
    }        
    // Compute relative y and z positions of desired pose.
    double delta_y = test_params::radius * (1 - std::cos(test_params::angle));
    double delta_z = test_params::radius * std::sin(test_params::angle);
    franka::CartesianPose pose_desired = initial_pose;
    pose_desired.O_T_EE[13] += delta_y;
    pose_desired.O_T_EE[14] += delta_z;   
    index += period.toSec();
    return pose_desired;

}

franka::JointVelocities test_joint_vel_motion_generator(const franka::RobotState&robot_state, franka::Duration period, double& index){

    double cycle = std::floor(std::pow(-1.0, (index - std::fmod(index, test_params::vel_time_max)) / test_params::vel_time_max));
    double omega = cycle * test_params::omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / test_params::vel_time_max * index));
    franka::JointVelocities velocities = {{0.0, 0.0, 0.0, omega, omega, omega, omega}};
    index += period.toSec();
    return velocities;
}

franka::CartesianVelocities test_cartesian_vel_motion_generator(const franka::RobotState&robot_state, franka::Duration period, double& index){
    index += period.toSec(); 
    double cycle = std::floor(pow(-1.0, (index - std::fmod(index, test_params::cart_vel_time_max)) / test_params::cart_vel_time_max));
    double v = cycle * test_params::cart_vel_v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / test_params::cart_vel_time_max * index));
    double v_x = std::cos(test_params::cart_vel_angle) * v;
    double v_z = -std::sin(test_params::cart_vel_angle) * v;
    franka::CartesianVelocities output = {{v_x, 0.0, v_z, 0.0, 0.0, 0.0}};
    return output;



}
