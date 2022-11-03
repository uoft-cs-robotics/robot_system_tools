// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
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
#include <atomic>
#include <eigen3/Eigen/Dense>

#include "common.h"
#include "tests/test_common.h"
#include "controllers/cartesian_impedance.h"
#include "tests/motion_generators.h"

namespace robotContext {
    franka::Robot *robot;
    franka::Gripper *gripper;
    franka::Model *model;

}
namespace Comms {
    ActionSubscriber *actionSubscriber; 
    StatePublisher *statePublisher; 
}

// Compliance parameters
const double translational_stiffness{150.0};
const double rotational_stiffness{10.0};
Eigen::MatrixXd stiffness(6, 6), damping(6, 6);

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  // Parameters
  const size_t joint_number{3};
  const size_t filter_size{5};
  const double print_rate = 10.0;
  std::atomic_bool running{true};

  CartesianImpedance controller(1, false);
  std::string file_name = "/home/pairlab/log.txt";
  bool joint_space=false;
  std::thread print_thread = std::thread(log_data, std::cref(print_rate), std::ref(print_data), std::ref(running), std::ref(file_name), std::cref(joint_space));
  try {
    franka::Robot robot(argv[1]);
    franka::Model model_ = robot.loadModel();
    robotContext::model = &model_;
    setDefaultBehavior(robot);
    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    double index = 0.0;
    std::array<double, 16> initial_pose;
    robot.control(
        [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
          if (print_data.mutex.try_lock()) {
            print_data.has_data = true;
            print_data.robot_state = robot_state;
            print_data.mutex.unlock();
          }               
          return controller(robot_state, period);
        },
        [&](const franka::RobotState&robot_state, franka::Duration period) -> franka::CartesianPose {
        if (index == 0.0) {
            initial_pose = robot_state.O_T_EE_c;
        }          
        franka::CartesianPose pose_desired = test_cartesian_pose_motion_generator(robot_state,
                                                                                period, 
                                                                                initial_pose,
                                                                                index);
        if (index >= test_params::runtime + test_params::acceleration_time) {
            running = false;
            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
            return franka::MotionFinished(pose_desired);
        }                                        

        return pose_desired;
        });
  } catch (const franka::ControlException& e) {
    std::cout << e.what() << std::endl;
    return -1;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  if (print_thread.joinable()) {
    print_thread.join();
  }  
  return 0;
}  