# Documentation for using this camera-robot arm extrinsic calibration tool 


## Dependancies
If you are not using the workstation docker, you workstation computer should have the following installed 

- opencv 
- [perception](https://github.com/BerkeleyAutomation/perception) (this dependancy will be removed soon)
- zmq 
- pyrealsense2 

## Usage
Currently our data collection step is operated by moving the robot by hand using the guiding mode with the Franka Arms, so ensure the user stop is pressed down and the robot status LEDs are white in color. 
# In Real time Computer,
Bring the built docker container up 
```
sudo docker-compose -f docker/realtime_computer/docker-compose-gui.yml up 
```

To open a bash terminal inside the docker container 
```
sudo docker exec -it realtime_docker bash
```
Inside the real-time docekr run the server to send end-effector pose everytime we press "enter" from the work-station computer after running the calibration script
```
cd ~/git/franka_control_suite/build 
./read_states
```
# In workstation computer/Docker,
```
cd ..../franka_arm_infra/calibration
python CameraRobotCalibration ..args..
```
Now, move the robot to different configurations by hand, and press enter for the calibration script to record the calibration target pose and the end-effector pose. Collect 15-20 poses and then press any other key than "Enter" to signal end of data collection and for the calibration process to start. 

## Tips for data collection to get accurate calibration results
Few Referenced from [here](https://github.com/IFL-CAMP/easy_handeye#:~:text=can%27t%20hurt%20either.-,Tips%20for%20accuracy,-The%20following%20tips)
- Maximize rotation between poses.
- Minimize the translation between poses.
- make sure the AR calibration target is flat and that there are no bumps
- use opencv board with mulitple markers(eg. aruco board) instead of single tag 
- record transforms data closer to the camera/robot base 
    - by doing this, errors in rotation are not propagated as large errors in translation. 
- use high resolution mode of RGB camera especially if the calibration target is far or small
- only use transforms with low rmse reprojection error (< ~0.3 pixels) (already implemented in our tool)
- Minimize the distance from the target to the camera of the tracking system.
- Calibrate the camera intrinsics if necessary / applicable.


## Introduction 
Visual manipulation systems have two configurations- camera attached to gripper or camera attached to environment. For both the configurations, most manipulation approaches require the rigid transformation between the camera's coordinate frame and the robot's coordinate frame. 

Extrinsic calibration is the process of estimating this tranform. This process is usually done offline(like our tool) or sometimes online. When done offline, a calibration target with known dimensions(like [fiducial markers](/https://link.springer.com/content/pdf/10.1007/s10846-020-01307-9.pdf)) is used as they provide easy 3D-2D correspondences of detected keypoints in the image. The collected data is used to setup an optimization problem to estimate the rigid body transform that we require. Below, you can find further details regarding camera-robot calibration problem and details specific to our tool. 

## Camera robot calibration 

For both the configurations of camera-in-hand and camera-to-hand, the main constraint comes from the fact that the camera is <ins>**rigidly**</ins> attached to the gripper/environment and the calibration target is <ins>**rigidly**</ins> attached to the world/robot respectively. This gives rise to the popular AX=XB formulation where A,B,X are all homogeneous transformations. X is the transform we wish to estimate, A and B are transforms that we meaure i.e 1. tranform between gripper and the robot base and 2. transform between calibration target and camera. Refer [here](https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b) for more details on how we formulate this problem. Different solvers/approaches exist to solve this problem. In OpenCV there are 5 approaches. You can look at the comparison between these approaches [here](https://journals.plos.org/plosone/article?id=10.1371/journal.pone.0273261).   

## Data Collection 
Although, in principle we only need a minimum of 2 motions with non parallel rotation axes to determine the hand-eye transformation i.e  3 different poses. Our AX=XB solvers does not account for bias and variance in our measurements of calibration target pose and end-effector pose. So in real world we are trying to solve A $\delta$ A X = XB $\delta$ B when only AX=XB is true, and $\delta$ A and $\delta$ B are the errors in our measurement. The common approach to deal with this to collect as much data as possible, usually >15 pairs of poses. 

- ### Calibration Target Pose Estimation 
    For estimating the pose of the calibration target, we first detect the fiducials in the image using OpenCV. Since we know the 3D model of the fiducials, we know the corresponding 3D points of the detected 2D points, therefore we can use [P-n-P algorithm](https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga357634492a94efe8858d0ce1509da869) to estimate the pose of the calibration target with respect to the camera. 

    We can gauge the accuracy of the pose estimate by reprojecting the 3D points of the fiducials on the camera using calibration tag pose estimate and pre-estimated camera intrinsics. We can then calculate the rmse between the detected pixel coordinates and the expected coordinates based on reprojection. 

    Large reprojection error(bad pose estimates) usually occur for the following reasons. 
    - when the calibration tag is far away from the camera 
    - if the calibration tag's normal vector is not parallel to the z-axis of the camera frame i.e the tag is not frontal facing to the camera 
        - this affects greater with the calibration tag farther from the camera. For more info refer [here](https://www.semanticscholar.org/paper/Analysis-of-Tracking-Accuracy-for-Single-Camera-Pentenrieder/70c5d9b33a978ff2d03eeaa627afaf4f6f609a1f) 
  
- ### Robot Poses 
    vibrations
    non-rigid attachment 
    rotation error 
    error modelling




# Trouble shooting
Ensure the right end-effector is attached to the robot in the Franka Desk App as this the frame with respect to robot base that is returned by libfranka if we query for the end-effector's pose. 

## References: 
- [opencv documentation]( https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b)
- [TUM documentation](https://campar.in.tum.de/Chair/HandEyeCalibration)
