# franka_control_suite
**This repo is a work in progress **

Contains lowlevel torque controllers for Franka Emika panda and an interprocess communication interface using ZeroMQ. This code base's intended functionality is similar to [franka-interface](https://github.com/iamlab-cmu/franka-interface) but without any dependancy in ROS.


# Dependencies 

- libfranka
- ZeroMQ
- Eigen




To do:
- [ ] add python example test scripts 
- [ ] add interpolators to all implementations

Notes: 
- IK controller has been tested in the real world. 3 methods have been implemented
    - Moore-Penrose Inverse
    - Damped Least Square 
    - Jacobian transpose
- Joint torque controller listed to desired joint torque commands, clips it to real robot limits and limits the rate before applying.
- other controllers are being tested currently
- fixed impedance OSC has been implemented completely, variable impedance and K_p are partially implemented.
