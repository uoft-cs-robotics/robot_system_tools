
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

  

To perform the robot-camera calibration, we would have to first collect the calibration tag + End effector pose data for a number of configurations(15-30). To this end, to first create the calibration tag, print the PDF provided in  [calibration/calibration_board](../calibration_board)  directory. Affix this tag to a **completely** flat surface (eg. wooden board, exam pad, dibond aluminum etc), while ensuring there are no bumps when you stick the calibration tag on its surface using glue.

  

For the camera-in-hand case, the calibration tag is fixed to the environment rigidly. Ensure the tag doesn't move or vibrate too much throughout the calibration process.

  

For the camera-in-environment case, the calibration tag needs to be **rigidly** attached to the robot's End-Effector. You could use the provided CAD files for the finger tips and gripping points in [models_4_3d_printing](../models_4_3d_printing). The [finger tips](../models_4_3d_printing/franka_custom_finger_tips.stl) are to be attached to Franka End-effector and the [gripping points](../models_4_3d_printing/finger_grasp_points.stl) or [handle plate](../models_4_3d_printing/finger_handle_plate.stl) are drilled/screwed onto the calibration tag. Now make the Franka End-effector with custom finger tips(figure 1) grasp the calibration tag(as show in figure 3) with the attached custom gripping points(figure 2), this ensures that the tag remains rigid with respect to the End-effector.

<img src="imgs/finger_tip.jpeg" width="120" height="80">

figure 1: custom finger tip

<img src="imgs/grasp_point.jpeg" width="120" height="80">
 
figure 2: grasping point

<img src="imgs/grasp_calib_tag.jpeg" width="120" height="80">

figure 3: grasping calibration tag 

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

cd /root/git/robot_toolkit

```

Now make sure to setup your config file like the one show [here](../config/robot_camera_calibration.yaml) and the run, 

```
python3 robot_camera_calibration.py --config_file <path to your config file eg: config/robot_camera_calibration.yaml>
```
  

For both the non ROS and ROS based pose data collection, move the robot to different configurations by hand, and press enter for the calibration script to record the calibration target pose and the end-effector pose. Collect 15-30 poses and then press any other key than "Enter" to signal end of data collection and for the calibration process to start.

The output hand eye calibration result for all available [methods](https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html#gad10a5ef12ee3499a0774c7904a801b99) in opencv will be stored in the output file mentioned in config YAML

For the camera in hand case the output is the Transformation of the camera frame in the EndEffector's frame. For the camera in the environment case. the output is the transformation of the camera frame in the robot's base frame. 

### c.<u> Instructions for Testing </u>
TBD
