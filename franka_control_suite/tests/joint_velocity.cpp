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
#include <franka/rate_limiting.h>

#include <thread>
#include "common.h"
#include <atomic>
#include "tests/test_common.h"
#include "tests/motion_generators.h"

#include "controllers/joint_vel.h"
#include "context.h"
namespace robotContext {
    franka::Robot *robot;
    franka::Gripper *gripper;
    franka::Model *model;

}
namespace Comms {
    ActionSubscriber *actionSubscriber; 
    StatePublisher *statePublisher; 
}

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

  // NOLINTNEXTLINE(readability-identifier-naming)
  const std::array<double, 7> K_P{{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}};
  // NOLINTNEXTLINE(readability-identifier-naming)
  const std::array<double, 7> K_D{{30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0}};
  const double max_acceleration{1.0};
  //JointVelController controller(filter_size, K_P, K_D);
  JointVelocity controller(1);
  std::string file_name = "/home/pairlab/log.txt";
  bool joint_space=true;
  std::thread print_thread = std::thread(log_data, std::cref(print_rate), std::ref(print_data), std::ref(running), std::ref(file_name), std::cref(joint_space));


  try {
    franka::Robot robot(argv[1]);
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
    double time_max = 1.0;
    double omega_max = 1.0;
    robot.control(
        [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
          if (print_data.mutex.try_lock()) {
            print_data.has_data = true;
            print_data.robot_state = robot_state;
            //print_data.tau_d_last = tau_J_d;
            //print_data.gravity = model.gravity(state);
            print_data.mutex.unlock();
          }   
          return controller(robot_state, period);
        },
        [&](const franka::RobotState&robot_state, franka::Duration period) -> franka::JointVelocities {
        franka::JointVelocities velocities = test_joint_vel_motion_generator(robot_state, period, index);
        if (index >= 5.0) {
            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
            running = false;
            return franka::MotionFinished(velocities);   
        }
        return velocities;
        });
    robot.control(motion_generator);
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