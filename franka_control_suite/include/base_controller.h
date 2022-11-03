#pragma once

#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include "context.h"
#include <eigen3/Eigen/Core>
#include "utils/utils.h"

#define DOF  7

enum class ControlMode{
    ABSOLUTE,
    DELTA
};

class ControllersBase{

public:
    /* virtual destructor to prevent memory leaks */
    virtual ~ControllersBase();

protected: 
    //virtual franka::Torques control_callback(const franka::RobotState&, franka::Duration) = 0; 
    // stiffness gain
    // const std::array<double, DOF> k_s = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    //const std::array<double, DOF> k_s = {{700.0, 700.0, 700.0, 700.0, 291.67, 175.0, 58.33}};
    const std::array<double, DOF> k_s = {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0,200.0}};
    const std::array<double, DOF> k_d = {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
    // damping gain
    //const std::array<double, DOF> k_d = {{100.0, 100.0, 100.0, 100.0, 60.0, 50.0, 30.0}};
    // const std::array<double, DOF> k_d = {{100.0, 100.0, 100.0, 100.0, 30.0, 25.0, 15.0}};
    // integral gain
    const std::array<double, DOF> k_i = {{100.0, 100.0, 100.0, 200.0, 120.0, 100.0, 30.0}};
    // joint limits
    const std::array<double, DOF> joint_min = {{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}};
    const std::array<double, DOF> joint_max = {{2.8973, 1.7628 	, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}};
    // joint velocity limits
    const std::array<double, DOF> joint_vel_max = {{2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61}};
    // torque limits
    const std::array<double, DOF> torque_max = {{87, 87, 87, 87, 12, 12, 12}};
    // count for receiving data 
    size_t count;
    //Cartesian impedance minimum : {N/m, N/m, N/m, Nm/rad, Nm/rad, Nm/rad}
    //values taken from libfranka documentation
    const std::array<double, 6> k_s_ee_min = {{10.0, 10.0, 10.0, 1.0, 1.0, 1.0}};
    //Cartesian impedance maximum
    const std::array<double, 6> k_s_ee_max = {{3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0}};

    std::array<double, DOF> clamp_torque(const std::array<double, 7> &torque_array);

    // by default control commands are relative and not absolute
    ControlMode control_mode_= ControlMode::ABSOLUTE; 

    bool zmq_comms = false;
};

inline ControllersBase::~ControllersBase(){}

inline std::array<double, DOF> ControllersBase::clamp_torque(const std::array<double, DOF> &torque_array){
    std::array<double, DOF> output_torque;
    for(size_t i = 0; i < torque_array.size(); i++){
        if (torque_array[i] > torque_max[i])
            output_torque[i] = torque_max[i];
        else
            output_torque[i] = torque_array[i];
    }
    return output_torque;
}


