# Documentation and steps on how to use Kalibr for multicamera calibration

## Introduction

[Kalibr](https://github.com/ethz-asl/kalibr) is a tool box that can be used to calibrate visual-inertial systems, i.e cameras and IMUs. We use it primarily for calibrating multiple cameras' intrinsics and the extrinsics between the cameras, below you will find the steps to achieve the same. 

Kalibr provides a docker container that includes all the dependancies and creates the ros workspace that is build the source code. They have a really nice [wiki](https://github.com/ethz-asl/kalibr/wiki), which was a great reference to create this documentation. You can think of this documentation of cherry picked instructions from the wiki and extra information that is not provided or is unclear in the wiki. 

The high level process of how kalibr should be used is:
- clone the reposiory and build the docker container 
- create a "data" folder that will be mounted to the container 
- create a calibration tag using the tool they provide [here](https://github.com/ethz-asl/kalibr/wiki/calibration-targets)
- Collect a ROSbag of the cameras' "image_raw" topics that will be used by Kalibr to compute the intrinsics and extrinsics 
- Create a .yaml file that describes the calibration tag used. 
- Both the bags and the .yaml should be present in the "data folder"
- run Kalibr calibration 
    - each topic in the ROS bag are processed to estimate the 2D keypoints of the checkerboard or apriltag corners 
    - These keypoints are used to estimate an initial guess for the intrinsics based on the camera model you pass as the argument. 
    - These keypoints are also used to estimate an initial guess of the extrinsics based on epipolar geometry 
    - The initial intrinsics and extrinsics guesses are then used to solve a batch optimization problem that minimizes reprojection error of the detected Keypoints keeping the cameras' intrinsics and extrinsics as the design variables.
- You can find more detailed description of these steps below- 

## Usage 

1. First clone kalibr repository and build the docker container they provide 

```
mkdir $USER/git 
cd $HOME/git 
git clone https://github.com/ethz-asl/kalibr
cd $HOME/git/kalibr

docker build -t kalibr -f Dockerfile_ros1_20_04 . # change this to whatever ubuntu version you want
```
2. 

3. Now, use the kalibr_create_target_pdf node to create a calibration target as described [here](https://github.com/ethz-asl/kalibr/wiki/calibration-targets)

- We suggest using Aprilgrid over the checkerboard as their points can be detected even under partial occlusion of tags 
- We also suggest to create a calibration tag as big as possible with maximum possible dimensions of the april tag marker and separation between them. 
- an example command would be: 

