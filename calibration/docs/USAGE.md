
# Documentation for using this camera-robot arm extrinsic calibration tool

  

## Introduction

There are two scripts available for data collection and Hand eye calibration using Tsai algorithm from opencv,

1. CameraRobotCalibration.py

2. ROSCameraRobotCalibration.py

  

CameraRobotCalibration.py uses pyrealsense2 sdk and opencv to process images from Intel cameras, gets robot poses from a small zmq server in the realtime docker that gets the end effector pose using libfranka. Whereas the ROSCameraRobotCalibration.py requires a ROS Tf tree to be populated with the end effector and robot base frames and also a image and camera_info publisher to subscribe RGB images and the camera's intrinsic information and then uses opencv to estimate the pose of the calibration target.

  

## Dependancies

All Dependancies are automatically installed when building the workstation docker environment.

  

## Calibration Process and Usage Instructions

  

### a. <u>Calibration Tag preparation</u>

  

To perform the robot-camera calibration, we would have to first collect the calibration tag + End effector pose data for a number of configurations(15-30). To this end, to first create the calibration tag, print the PDF provided in calibration/calibration_tag directory. Affix this tag to a **completely** flat surface (eg. wooden board, exam pad, dibond aluminum etc), while ensuring there are no bumps when you stick the calibration tag on its surface using glue.

  

For the camera-in-hand case, the calibration tag is fixed to the environment rigidly. Ensure the tag doesn't move or vibrate too much throughout the calibration process.

  

For the camera-in-environment case, the calibration tag needs to be **rigidly** attached to the robot's End-Effector. You could use the (to be) provided CAD files for the finger tips and gripping points. The finger tips are to be attached to Franka End-effector and the gripping points are drilled/screwed onto the calibration tag. Now make the Franka End-effector with custom finger tips grasp the calibration tag with the attached custom gripping points, this ensures that the tag remains rigid with respect to the End-effector.

  

### b.<u> Preparation to provide End-Effector Poses. </u>

  

If you are using the ROS API ensure, there's a node that populates the TF tree with the robot base and end-effector frames. There should also be a node running that publishes RGB images and the camera's intrinsic in the camera_info topic. An example workflow of commands is shown below,

  

In Real time Computer,

Bring the built docker container up

```

sudo docker-compose -f docker/realtime_computer/docker-compose-gui.yml up

```

In workstation computer,

Bring the built workstation docker container up

```

xhost +local:docker

sudo docker-compose -f docker/workstation_computer/docker-compose-gui.yml up

```

open a bash terminal inside the workstation docker container

```

(sudo) docker exec -it workstation_computer_docker bash

```

In the worstation computer docker terminal, launch the nodes for publishing the robot's tf tree containing robot base and end effector frames, for example: bringing up frankapy

and running realsense launch file,

```

cd /root/git/frankapy/

bash ./bash_scripts/start_control_pc.sh -i (realtime computer ip) -u (realtimecomputer username) -d /root/git/franka-interface -a (robot_ip) -w (workstation IP)

  

roslaunch realsense2_camera rs_camera.launch

```

---

If you are not using the ROS APIs and are using Realsense cameras that can work with pyrealsense SDK then, first, ensure the camera is connected to the workstation computer, then run the read states server in the realtime docker.

  

In Real time Computer,

Bring the built docker container up

```

sudo docker-compose -f docker/realtime_computer/docker-compose-gui.yml up

```

open a bash terminal inside the realtime docker container

```

(sudo) docker exec -it realtime_docker bash

cd /root/git/franka_control_suite/build

```

run the readstates server that send end effector poses when requested,

```

./read_states <robot_ip> <realtime_pc_ip> <zmq_port_number>

```

  
  
  

### c.<u> Run Pose Data Collection + Calibration Script </u>

  

### In workstation computer,

Bring the built workstation docker container up

```

xhost +local:docker

sudo docker-compose -f docker/workstation_computer/docker-compose-gui.yml up

```

open a bash terminal inside the workstation docker container and go to appropriate directory

```

(sudo) docker exec -it workstation_computer_docker bash

cd /root/git/calibration

```

  

If not using the ROS API, In the worstation computer docker terminal, run the script to collect the pose data and run hand eye calibration i.e CameraRobotCalibration.py script with appropriate arguments, see the available ones here
```
python3 CameraRobotCalibration.py -h
```
You will see a terminal output like so
```
usage: CameraRobotCalibration.py [-h] [--camera_in_hand [CAMERA_IN_HAND]]
                                 [--move_robot_automatically [MOVE_ROBOT_AUTOMATICALLY]]
                                 [--only_calibration [ONLY_CALIBRATION]]
                                 [--run_ransac [RUN_RANSAC]] [--zmq_server_ip [ZMQ_SERVER_IP]]
                                 [--zmq_server_port [ZMQ_SERVER_PORT]]
                                 [--debug_image [DEBUG_IMAGE]]

Optional app description

optional arguments:
  -h, --help            show this help message and exit
  --camera_in_hand [CAMERA_IN_HAND]: True(default)/False
                        is the camera attached to the robots body or to the environment?
  --move_robot_automatically [MOVE_ROBOT_AUTOMATICALLY]: True/False(default)
                        should the EE automatically move for collecting data? In this case, the EE
                        is first manually moved to an initial pose and the script controls EE to
                        predefined relative poses. If false, the EE should be moved manually(white
                        status LED) and press enter to collect data
 --use_delta_poses [USE_DELTA_POSES]:True(default)/False
                        should we use delta end-effector poses to move the robot to collect
                        data for calibration
  --only_calibration [ONLY_CALIBRATION]: True/False(default)
                        if true, values stored in the data folder are used for calibration if
                        false, data is first collected, stored in /data folder and then
                        calibration routine is run
  --run_ransac [RUN_RANSAC]: True/False(default)
                        this option runs ransac to select EEposes and calibration target poses
                        based on how well they all agree to AX=XB
  --zmq_server_ip [ZMQ_SERVER_IP]
                        ip address of the zmq server running on the realtime PC to send robot hand
                        poses
  --zmq_server_port [ZMQ_SERVER_PORT]
                        port of the zmq server running on the realtime PC to send robot hand poses
                        the port number is an arbitrary number less than 65535 and ensure it is
                        not used by any other application in your computer
  --debug_image [DEBUG_IMAGE]: True(default)/False
                        this option draws aruco tag detections and the estimated tag frame on the
                        image and saves in the data/image folder

```

**Note: make sure the zmq_port passed to the ./read_states executable and passed to the CameraRobotCalibration.py script are the same**

run the ROSCameraRobotCalibration.py script with appropriate arguments, see the available ones here

```
python3 ROSCameraRobotCalibration.py -h

```
You will see a terminal output like so
```
usage: ROSCameraRobotCalibration.py [-h] [--camera_in_hand [CAMERA_IN_HAND]]
                                    [--move_robot_automatically [MOVE_ROBOT_AUTOMATICALLY]]
                                    [--only_calibration [ONLY_CALIBRATION]]
                                    [--robot_base_frame_name [ROBOT_BASE_FRAME_NAME]]
                                    [--ee_frame_name [EE_FRAME_NAME]]
                                    [--rgb_image_topic [RGB_IMAGE_TOPIC]]
                                    [--rgb_camera_info_topic [RGB_CAMERA_INFO_TOPIC]]
                                    [--run_ransac [RUN_RANSAC]] [--debug_image [DEBUG_IMAGE]]
                                    [--with_franka_gripper [WITH_FRANKA_GRIPPER]]

Optional app description

optional arguments:
  -h, --help            show this help message and exit
  --camera_in_hand [CAMERA_IN_HAND]: True(default)/False
                        is the camera attached to the robots body or to the environment?
  --move_robot_automatically [MOVE_ROBOT_AUTOMATICALLY]: True/False(default)
                        should the EE automatically move for collecting data? In this case, the EE
                        is first manually moved to an initial pose and the script controls EE to
                        predefined relative poses. If false, the EE should be moved manually(white
                        status LED) and press enter to collect data
  --only_calibration [ONLY_CALIBRATION]: True/False(default)
                        if true, values stored in the data folder are used for calibration if
                        false, data is first collected, stored in /data folder and then
                        calibration routine is run
  --robot_base_frame_name [ROBOT_BASE_FRAME_NAME]
                        robot base frames name in the /tf tree
  --ee_frame_name [EE_FRAME_NAME]
                        end-effector frames name in the /tf tree
  --rgb_image_topic [RGB_IMAGE_TOPIC]
                        RGB image raw topic name
  --rgb_camera_info_topic [RGB_CAMERA_INFO_TOPIC]
                        RGB image camera info name
  --run_ransac [RUN_RANSAC]: True/False(default)
                        this option runs ransac to select EEposes and calibration target poses
                        based on how well they all agree to AX=XB
  --debug_image [DEBUG_IMAGE]: True(default)/False
                        this option draws aruco tag detections and the estimated tag frame on the
                        image and saves in the data/image folder
  --with_franka_gripper [WITH_FRANKA_GRIPPER]: True/False(default)
                        are you using the franka arm with franka gripper?
```

For both the non ROS and ROS based pose data collection, move the robot to different configurations by hand, and press enter for the calibration script to record the calibration target pose and the end-effector pose. Collect 15-30 poses and then press any other key than "Enter" to signal end of data collection and for the calibration process to start.

The output hand eye calibration result for all available [methods](https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html#gad10a5ef12ee3499a0774c7904a801b99) in opencv will be printed on the terminal. If you choose to run RANSAC, the results after running the RANSAC scheme will be printed. 

For the camera in hand case the output is the Transformation of the camera frame in the EndEffector's frame. For the camera in the environment case. the output is the transformation of the camera frame in the robot's base frame. 

### c.<u> Instructions for Testing </u>
TBD