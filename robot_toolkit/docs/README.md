There are three documentation files- 

- [BACKGROUND.md](BACKGROUND.md): This gives a high level overview of how the robot camera calibration is performed and tips for data collection to get accurate results. 
- [MULTI_CAM_CALIB.md](MULTI_CAM_CALIB.md): This provides documentation for performing multi camera extrinsic calibration using [Kalibr](https://github.com/ethz-asl/kalibr)- a calibration tool for multicamera and IMU systems. 
- [ROBOT_CALIB_USAGE.md](ROBOT_CALIB_USAGE.md): This provides documentation for using the single camera-robot calibration. 

Description of scripts: 

In this repo, one could also find a list of "test" scripts that test functionalities of the code base, as well as, provide the user an idea on how to use the code base. You can find the description of each script below- 

### [test_aruco_board.py](../test_aruco_board.py):
- Operation: 
    * An arucoboard with its parameters specified by the ArucoBoardData object is detected and the its pose estimated on a test image that is stored locally in [../tests/data](../tests/data). 

- Expected output: 
    * The mean and variance of the reprojected error computed for each detected corner is to printed 
    * Detected markers and estimated frame of the aruco board is printed on drawn over the input image and displayed in a new window. 

### [test_aruco_tag.py](../test_aruco_tag.py):
- Operation:
    * An arucotag with its parameters specified by the ArucoTag is detected and its pose estimated on a test image that is stored localled in [../tests/data](../tests/data). 
- Expected output: 
    * The mean and variance of the reprojected error computed for each detected corner is to printed 
    * Detected markers and estimated frame of the aruco board is printed on drawn over the input image and displayed in a new window. 

### [test_create_tag.py](../test_create_tag.py):
- Operation:
    * Creates an image(.jpeg or .png) of an arucoboard with specified parameters passed as an argument. 
- Expected output: 
    * An image of an arucoboard with parameters specified in the script store in [here](../tests/data/aruco_board_2x2.png)

### [test_grasp_recorder.py](../test_grasp_recorder.py):
- Pre-requesites: 
    * This script is used to show how we could record desired grasps for any arbitrary object. using **default franka arm and franka gripper**
    * This script also assume only one realsense camera is connected to the workstation computer
    * The object can have any one of the available AR markers or we can use a pose estimator 
    * This script assumes the robot and camera already calibrate extrinsically and it works for both camera in hand and in env case
    * The robot needs to be in "white" status LED mode. 
    * In the realtime computer, ensure "read_states" server is running by running the following instruction in the realtime computer(docker). 
    ```
    cd <path_to_franka_control_suite>/franka_control_suite/build 
    ./read_states <robot_ip> <realtime_pc_ip> <zmq_port_number>
    ```
    * Edit lines 8-18 [test_grasp_recorder.py](../test_grasp_recorder.py) to your corresponding test case. 
    * **Ensure the object is not moved throught the time this script is run**
- Operation:
    * The object's pose is first estimated using the AR tag pose estimator(can also be replaced by any pose estimator for the object)
    * Now we can move the gripper by hand (in gravity compensation mode) to desired grasp poses. **Note: Do not close the gripper** and record this pose with respect to the object's frame of reference. 
    * Collect as many grasp poses as you want. 
- Expected Output: 
    * A yaml file specified at the location in the test_config dictionary to contain 4x4 arrays representing grasp poses. 


### [test_pick_n_place.py](../test_pick_n_place.py):
- Pre-requesites: 
    * This script is used to perform pick and place operations for objects for which we have already recorded desired grasps using [test_grasp_recorder.py](../test_grasp_recorder.py) using **default franka arm and franka gripper**
    * This script assumes the robot and camera already calibrate extrinsically and it works for both camera in hand and in env case.
    * This script also assumes only one realsense camera is connected to the workstation computer. 
    * Frankapy's server should have already started and the status LED should be blue. 
    * Edit lines 12-20 [test_pick_n_place.py](../test_pick_n_place.py) to your corresponding test case.     
- Operation:
    * The object to be grasped is first localized using pose estimation using AR markers(can also be replaced by any pose estimator for the object)
    * A grasp is selected from the grasps already recorded using [test_grasp_recorder.py](../test_grasp_recorder.py)
    * This grasp is used to pick the object and then place at a position offset in the +ve X direction in the robot base frame. 
- Expected output: 
    * The end-effector first goes to pregrasp pose and then to the grasp pose 
    * Picks up the object 
    * Then places the object at a position offset in the +ve X direction in the robot base frame. 
<!-- - Pre-requisites: 
    * This testscript is implemented for the **Franka Emika robot arm + Franka Emika gripper** with "camera in the hand" configuration. 
    * It detects the pose of a cube of dimension 5cm with an ArucoBoard affixed on one of its faces.
    * The frame of the object(cube) used for grasping is set as an offset to the arucoboard's frame attached to it when calling the constructor of the "Object" class. 
    * This script expects a ROS node of a camera to be running that publishes the RGB image and the camera's intrinsics in separate topics that are passed as arguments when instantiating "ros_camera" object. This script is specifically written with realsense camera node's topic names, so one can run, 
    ```
    roslaunch realsense2_camera rs_camera.launch
    ```
- Operation:
    * The "Object" to be picked (in this case the cube) is detected in the RGB image and its pose estimated in the camera frame
    * The estimated pose is used to pick the object using "pick_object" method
    * "place_object" is used to specify a pose to place the object and the robot arm and gripper executes this skill. 

- Expected output: 
    * The robot arm first resets itself to the home configuration (specified by Frankapy) 
    * The "object" to be picked (in this case the cube) is detected and its pose is estimated. 
    * The robot arm places its gripper at a fixed distance above the cube, set as "pre grasp" pose. 
    * The robot arm's gripper is moved to the object's pose that's used to grasp it. 
    * gripper closes once the object is in its tool center point i.e between two parallel fingers. 
    * The object is picked up 
    * The object is then placed in the "place pose" passed as an argument to the place_object method, which in this case is the pick pose. 
    * Therefore, the object is picked and placed back in its original pose.  -->


### [test_realsense_camera.py](../test_realsense_camera.py):
- Operation: 
    * This script tests the [Camera](../robot_arm_algos/src/camera/camera.py) class implemented for the realsense camera using the pyrealsense sdk.  
    * This script will read the depth and RGB images, and visualize the data

- Expected output: 
    * The depth and RGB images that are read by the camera object will be displayed in a window 
    * The depth and RGB images are also used to create a colored point cloud that is displayed in a window. 

### [test_ros_camera_arucoboard.py](../test_ros_camera_arucoboard.py):
- Operation: 
    * This tests arucoboard detection on images using camera intrinsics both subscribed from a ROS node publisher. 
    * Checkout the description of the arucoboard being used in this test. 
    * The topic names in this script are set for realsense camera ROS nodes, so run the command below in a new terminal before running this script. 
    ```
    roslaunch realsense2_camera rs_camera.launch
    ```    
- Expected output: 
    * Display of an image with the detected markers and estimated arucoboard frame drawn. 
    * Printing the mean and variance of the reprojection errors computed over each detected corner point.

### [test_ros_camera_arucotag.py](../test_ros_camera_arucotag.py):
- Operation: 
    * This tests arucotag detection on images using camera intrinsics both subscribed from a ROS node publisher. 
    * Checkout the description of the arucotag being used in this test.     
    * The topic names in this script are set for realsense camera ROS nodes, so run the command below in a new terminal before running this script. 
    ```
    roslaunch realsense2_camera rs_camera.launch
    ```    
- Expected output: 
    * Display of an image with the detected marker and estimated arucotag frame drawn. 
    * Printing the mean and variance of the reprojection errors computed over each detected corner point.

### [test_ros_camera_object.py](../test_ros_camera_object.py):
- Operation: 
    * This tests the pose estimation of objects that have an arucoboard/ aruco tag attached to it. 
    * The pose of the object is estimated by first estimating the pose of the arucoboard/aruco tag attached and then by multiplying the rigibody transform between the object's frame and the arucoboard/tag frame. 
- Expected output: 
    * Display of an image with the detected marker and estimated object frame drawn. 
    * Printing the mean and variance of the reprojection errors computed over each detected corner point of the arucoboard/arucotag  
