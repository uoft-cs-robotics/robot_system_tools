## Description

Scripts to test controllers provided in frankapy. Inside the workstation docker, you can find the scripts at /root. We have a implemented a class that subscribes to synched messages of control command and the robot state and uses this to plot system responses to the control inputs. 

## Notes on Frankapy control APIs
- goto_joints
    - if dynamic is set as True:
        - This mode allows us to stream joint commands until the duration period passed to goto_joints is done 
        - custom joint impedance controller is used (with fixed impedance or variable not sure)
    - if dynamic is set as False
        - This is like a setpoint controller can't stream desired joint configuration.
        - if use_impedance is True 
            - custom joint impedance controller is used (with fixed impedance or variable not sure)
        - if use_impedance is False
            - libfranka's internal joint impedance controller is used
- goto_pose 
    - if dynamic is set as True: 
        - This mode allows us to stream joint commands until the duration period passed to goto_pose is done.
        - custom cartesian impedance controller is used (they claim variable impedance)
    - if dynamic is set as False
        - This is like a setpoint controller, can't stream desired pose. 
        - if use_impedance is True: 
            - custom cartesian impedance controller is used (they claim variable impedance)
        - is use_impedance is False: 
            - libfranka's internal IK + joint impedance controller is used     

- franka is precise for large magnitude motions but imprecise for smaller motions  

### [docker_frankapy_test.py](docker_frankapy_test.py):

This script can be used to test to see if both the realtime and workstation docker installations work without any errors along with running frankapy. Expected output: the robot arm "resets joints" to go to the home configuration. 

### [test_ee_pose_control.py](test_ee_pose_control.py):

This script uses dynamic pose control mode of frankapy to stream a square on the YZ plane (in robot base frame). We also set higher stiffness values than default values to get accurate pose tracking. 

In lines 20-30 you can specify the edge length of the square trajectory, speed at which we want the end-effector to trace one edge of the square, delta time steps at which we sample the desired square trajectory and the no.of cycles of tracing the square trajectory, rotational and translational stiffness of the cartesian impedance control (roll, pitch, yaw of the square in the robot base frame, but its not implemented right now). After executing the input trajectory the commanded and actual end-effector positions are plotted. 

### [test_ee_pose_impedance_control.py](test_ee_pose_impedance_control.py):

This script uses dynamic pose control mode of frankapy to stream a square on the YZ plane (in robot base frame). We also test variable impedance capability of the controller. 

In lines 20-30 you can specify the edge length of the square trajectory, speed at which we want the end-effector to trace one edge of the square, delta time steps at which we sample the desired square trajectory and the no.of cycles of tracing the square trajectory, rotational and translational stiffness of the cartesian impedance control (roll, pitch, yaw of the square in the robot base frame, but its not implemented right now). After executing the input trajectory the commanded and actual end-effector positions are plotted. We also plot the translational impedance value that we have commanded. 

### [test_joint_position_control.py]([test_joint_position_control.py]):

This script tests uses the dynamic joint control mode of frankapy to stream sinusoid reference trajectories to joints 1,4,5,6 of the franka robot arm. After executing the input trajectory the commanded and actual joint angles are plotted. 

### [test_joint_position_cartesian_impedance_control](test_joint_position_cartesian_impedance_control.py):

This script resets the robot joints to home configuration and tries to set low translation and rotational cartesian impedance. We can see that even though we send variable cartesian impedance, the expected stiffness response is not viewed in the robot. This means that using variable cartesian impedance doesn't work for goto_joints skill that runs a joint impedance controller. You can try pushing the robot end effector with various impedance value and we see the same stiffness response.
