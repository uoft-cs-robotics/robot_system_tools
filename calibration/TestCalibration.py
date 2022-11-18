
# tag pose estimation         
# rotate pose to change +ve z to -ve z
# ransac changed to use cv2 
import numpy as np
import os
import time
import cv2
import matplotlib.pyplot as plt


#Real Sense libraries#
from perception.realsense_sensor import RealSenseSensor
import pyrealsense2 as rs


#FrankaPy and ROS#
from frankapy import FrankaArm
from scipy.spatial.transform import Rotation as R
import tf.transformations as tf_utils


# test aruco marker objects 




# input calibration result here 
# R_cam2gripper = np.array([[ 0.03540295, -0.99904906,  0.02544817],
#  [ 0.99937248,  0.03536256 ,-0.00203574],
#  [ 0.00113389 , 0.02550427,  0.99967407]])

R_cam2gripper = np.array([[ 2.12889303e-02, -9.99406626e-01 , 2.70772296e-02],
 [ 9.99773198e-01,  2.12967822e-02,  1.60199835e-06],
 [-5.78258911e-04 , 2.70710544e-02,  9.99633345e-01]])

                        
# t_cam2gripper = np.array([ 0.04918928 ,-0.03774242 ,-0.05317044])
t_cam2gripper = np.array([0.04823794 ,-0.03761049, -0.05464881])
cam2gripper_pose = np.eye(4)
cam2gripper_pose[0:3, 0:3] = R_cam2gripper
cam2gripper_pose[0:3, -1] = t_cam2gripper
print(cam2gripper_pose)


#create frankapy object
fa = FrankaArm()
#fa.reset_joints()


ip = input("Press Enter to continue collecting current sample....else space bar to stop")

# realsense sensor 
ctx = rs.context()
device_id = ctx.devices[0].get_info(rs.camera_info.serial_number)
sensor = RealSenseSensor(device_id, frame="realsense", filter_depth=True)
sensor.start()
intr = sensor.color_intrinsics
#intr_list = [intr._fx, intr._fy, intr._cx, intr._cy]
camera_matrix = np.array([[intr._fx, 0.0, intr._cx], [0.0, intr._fy, intr._cy],[0.0,0.0,1.0]])
dist_coeffs =np.array([0.0,0.0,0.0,0.0])

color_im_, depth_im_ = sensor.frames()
color_im = color_im_.raw_data
image_gray = cv2.cvtColor(color_im, cv2.COLOR_BGR2GRAY)

markerLength = 0.015
aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_5X5_1000 )
arucoParams = cv2.aruco.DetectorParameters_create()
corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image_gray, aruco_dict, parameters=arucoParams,)  # First, detect markers


markerLength = 0.0265
markerSeparation = 0.0056
aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_6X6_1000 )
#aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_4X4_1000 )

board = cv2.aruco.GridBoard_create(5, 7, markerLength, markerSeparation, aruco_dict)
#img = cv2.aruco.drawPlanarBoard(board, (2550,3300))# for printing on A4 paper
#cv2.imwrite('/home/ruthrash/test.jpg', img)
arucoParams = cv2.aruco.DetectorParameters_create()
corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image_gray, aruco_dict, parameters=arucoParams)  # First, detect markers


if ids is not None: 
    for i in range(0, len(ids)):
        # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
        #rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], markerLength, camera_matrix, dist_coeffs)
        rvec = None 
        tvec = None
        retval, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, rvec, tvec)  # posture estimation from a diamond

        rotation_matrix = np.zeros(shape=(3,3))
        cv2.Rodrigues(rvec, rotation_matrix)
        # rotation_matrix[:, 2] = - rotation_matrix[:, 2]
        # t = rotation_matrix[:, 1].copy()
        # rotation_matrix[:, 1] = rotation_matrix[:, 0]
        # rotation_matrix[:, 0] = t
        # rvec_new = cv2.Rodrigues(rotation_matrix)
        # Draw a square around the markers
        cv2.aruco.drawDetectedMarkers(image_gray, corners) 

        # Draw Axis
        #cv2.drawFrameAxes(color_im, camera_matrix, dist_coeffs, rvec_new[0], tvec, 0.03)  
        cv2.drawFrameAxes(color_im, camera_matrix, dist_coeffs, rvec, tvec, 0.03)  
else:
    print('No aruco markers detected')
    #exit()

cv2.imshow('Estimated Pose', color_im)
key = cv2.waitKey(0)  
cv2.destroyAllWindows()

ee2base_ = fa.get_pose()


# print(ee2base_)
tag2cam = np.eye(4)
ee2base = np.eye(4)
cam2ee = np.eye(4)


tag2cam[0:3, 0:3] = rotation_matrix; tag2cam[0:3, -1] = tvec[:,0]
cam2ee = cam2gripper_pose
ee2base[0:3, 0:3] = ee2base_.rotation; ee2base[0:3, -1] = ee2base_.translation
# goal_matrix = np.matmul(np.matmul(tag2cam, cam2ee),ee2base)
goal_matrix = np.matmul(ee2base, np.matmul(cam2ee, tag2cam ))
goal = fa.get_pose()
# print(tag2cam)
# print(cam2ee)
# print(ee2base)

goal.rotation = goal_matrix[0:3, 0:3]; goal.translation = goal_matrix[0:3, -1]; goal.translation[2]+= 0.3   
print("goal_pose")
print(goal)
fa.goto_pose(goal)
print(fa.get_pose())
# goal.translation[2] -= 0.1
# fa.goto_pose(goal)
# R_tag2cam = 
# t_tag2cam = 

# R_tag2gripper = 
# t_tag2gripper = 

# rotate it upside down 

