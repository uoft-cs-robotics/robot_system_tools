#pragma once
#include "base_controller.h"

#include <eigen3/Eigen/Dense>
#include <franka/rate_limiting.h>


class CartesianImpedance: public ControllersBase{

public:
    CartesianImpedance(int start, bool zmq_comms_flag=false); 
    ~CartesianImpedance();
    franka::Torques operator()(const franka::RobotState&, franka::Duration); 

private: 
    Eigen::Affine3d cartesian_goal_transform_eigen;
    void clamp_joint_angles(std::vector<double> &joint_pos_goal);
    Eigen::Matrix<double, 6, 6> stiffness, damping;
};