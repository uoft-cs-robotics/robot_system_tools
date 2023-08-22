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

### [test_pick_n_place.py](../test_pick_n_place.py):
- Pre-requisites: 
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
    * Therefore, the object is picked and placed back in its original pose. 


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
