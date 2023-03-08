# Documentation for using this camera-robot arm extrinsic calibration tool 

## Introduction
There are two scripts available for data collection and Hand eye calibration using Tsai algorithm from opencv, 
1. CameraRobotCalibration.py
2. ROSCameraRobotCalibration.py

CameraRobotCalibration.py uses pyrealsense2 sdk and opencv to process images from Intel cameras, gets robot poses from a small zmq server in the realtime docker that gets the end effector pose using libfranka. Whereas the ROSCameraRobotCalibration.py requires a ROS Tf tree to be populated with the end effector and robot base frames and also a image and camera_info publisher to subscribe RGB images and the camera's intrinsic information and then uses opencv to estimate the pose of the calibration target.  
## Dependancies
All Dependancies are automatically installed when building the workstation docker environment.

## Usage for non ROS based pose data collection for calibration 
### In Real time Computer,
Bring the built docker container up 
```
sudo docker-compose -f docker/realtime_computer/docker-compose-gui.yml up 
```
open a bash terminal inside the realtime docker container 
```
(sudo) docker exec -it realtime_docker bash
cd /root/git/franka_control_suite/build 
```
run the readstates server that send end effector poses when requested, ?
```
./read_states <robot_ip> <realtime_pc_ip> <zmq_port_number>
```
### In workstation computer,
Bring the built workstation docker container up 
```
xhost +local:docker 
sudo docker-compose -f docker/workstation_computer/docker-compose-gui.yml up 
```
open a bash terminal inside the workstation docker container 
```
(sudo) docker exec -it workstation_computer_docker bash
```
In the worstation computer docker terminal, run the script to collect the pose data and run hand eye calibration i.e CameraRobotCalibration.py script with appropriate arguments, see the available ones here
```
python3 CameraRobotCalibration.py -h

usage: CameraRobotCalibration.py [-h] [--camera_in_hand [CAMERA_IN_HAND]]
                                 [--move_robot_automatically [MOVE_ROBOT_AUTOMATICALLY]]
                                 [--only_calibration [ONLY_CALIBRATION]]
                                 [--run_ransac [RUN_RANSAC]] [--zmq_server_ip [ZMQ_SERVER_IP]]
                                 [--zmq_server_port [ZMQ_SERVER_PORT]]
                                 [--debug_image [DEBUG_IMAGE]]

Optional app description

optional arguments:
  -h, --help            show this help message and exit
  --camera_in_hand [CAMERA_IN_HAND]
                        is the camera attachedto the robots body or to the environment?
  --move_robot_automatically [MOVE_ROBOT_AUTOMATICALLY]
                        should the EEautomatically move for collecting data? In this case, the EE
                        is firstmanually moved to an initial pose and the script controls EE to
                        predefinedrelative poses. If false, the EE should be moved manually(white
                        status LEDand press enter to collect data
  --only_calibration [ONLY_CALIBRATION]
                        if true, valuesstored in the data folder are used for calibration if
                        false, data is first collected,stored in /data folder and then calibration
                        routine is run
  --run_ransac [RUN_RANSAC]
                        this optionruns ransac to select EEposes and calibration target poses
                        based on how well they all agree to AX=XB
  --zmq_server_ip [ZMQ_SERVER_IP]
                        ip addressof the zmq server running on the realtime PC to send robot hand
                        poses
  --zmq_server_port [ZMQ_SERVER_PORT]
                        portof the zmq server running on the realtime PC to send robot hand poses
  --debug_image [DEBUG_IMAGE]
                        this optiondraws aruco tag detections and the estimated tag frame on the
                        image and saves in the data/image folder

```
**Note: make sure the zmq_port passed to the ./read_states executable and passed to the CameraRobotCalibration.py script are the same**

## Usage for ROS based pose data collection for calibration 
### In Real time Computer,
Bring the built docker container up 
```
sudo docker-compose -f docker/realtime_computer/docker-compose-gui.yml up 
```
### In workstation computer,
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

In a new terminal inside workstation docker, 
```
cd /root/git/calibration 
```
run the ROSCameraRobotCalibration.py script with appropriate arguments, see the available ones here
```
python3 ROSCameraRobotCalibration.py -h 

usage: ROSCameraRobotCalibration.py [-h] [--camera_in_hand [CAMERA_IN_HAND]]
                                    [--move_robot_automatically [MOVE_ROBOT_AUTOMATICALLY]]
                                    [--only_calibration [ONLY_CALIBRATION]]
                                    [--robot_base_frame_name [ROBOT_BASE_FRAME_NAME]]
                                    [--ee_frame_name [EE_FRAME_NAME]]
                                    [--rgb_image_topic [RGB_IMAGE_TOPIC]]
                                    [--rgb_camera_info_topic [RGB_CAMERA_INFO_TOPIC]]
                                    [--run_ransac [RUN_RANSAC]] [--debug_image [DEBUG_IMAGE]]

Optional app description

optional arguments:
  -h, --help            show this help message and exit
  --camera_in_hand [CAMERA_IN_HAND]
                        is the camera attachedto the robots body or to the environment?
  --move_robot_automatically [MOVE_ROBOT_AUTOMATICALLY]
                        should the EEautomatically move for collecting data? In this case, the EE
                        is firstmanually moved to an initial pose and the script controls EE to
                        predefinedrelative poses. If false, the EE should be moved manually(white
                        status LEDand press enter to collect data
  --only_calibration [ONLY_CALIBRATION]
                        if true, valuesstored in the data folder are used for calibration if
                        false, data is first collected,stored in /data folder and then calibration
                        routine is run
  --robot_base_frame_name [ROBOT_BASE_FRAME_NAME]
                        robot base frames name in the /tf tree
  --ee_frame_name [EE_FRAME_NAME]
                        end-effector frames name in the /tf tree
  --rgb_image_topic [RGB_IMAGE_TOPIC]
                        RGB image raw topic name
  --rgb_camera_info_topic [RGB_CAMERA_INFO_TOPIC]
                        RGB image camera info name
  --run_ransac [RUN_RANSAC]
                        this optionruns ransac to select EEposes and calibration target poses
                        based on how well they all agree to AX=XB
  --debug_image [DEBUG_IMAGE]
                        this optiondraws aruco tag detections and the estimated tag frame on the
                        image and saves in the data/image folder
```





open a bash terminal inside the docker container 
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
