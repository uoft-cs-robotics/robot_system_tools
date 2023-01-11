
# tag pose estimation         
# rotate pose to change +ve z to -ve z
# ransac changed to use cv2 
import numpy as np
import os
import time
import cv2
import matplotlib.pyplot as plt
from CalibrationUtils import *


#Real Sense libraries#
from perception.realsense_sensor import RealSenseSensor
import pyrealsense2 as rs


#FrankaPy and ROS#
from frankapy import FrankaArm
from scipy.spatial.transform import Rotation as R
import tf.transformations as tf_utils


# test aruco marker objects 

eye_in_hand = True

# input calibration result here 
R_cam2gripper = np.array([[ 0.00269485, -0.99983267,  0.01809318],
                        [ 0.99993556 , 0.00249471, -0.0110751 ],
                        [ 0.01102811 , 0.01812186 , 0.99977496]])
t_cam2gripper = np.array([ 0.05493054, -0.03262165, -0.06812028])  


# cam2gripper_pose = np.eye(4)
# cam2gripper_pose[0:3, 0:3] = R_cam2gripper
# cam2gripper_pose[0:3, -1] = t_cam2gripper
# print(cam2gripper_pose)
print("##")

#create frankapy object
fa = FrankaArm()
# print("##")
# fa.reset_joints()
# print("##")
# ini_rot = np.array([[ 0.99282895,  0.00339386, -0.119416  ],
#        [ 0.0083905 , -0.99909926,  0.04136483],
#        [-0.11916805, -0.04207016, -0.99198224]])
# trans = np.array([ 0.36782694, -0.41776979,  0.39976282])
# ini_pose = fa.get_pose() 
# ini_pose.translation = trans; ini_pose.rotation = ini_rot
# if(eye_in_hand):    
#     fa.goto_pose(ini_pose)
ip = input("Press Enter to continue collecting current sample....else space bar to stop")

###### realsense sensor 
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
######## realsense 

#####
if(eye_in_hand):
    Tcam2gripper = np.eye(4); Tgripper2base = np.eye(4); Tcam2base = np.eye(4)
    Tcam2gripper[0:3, 0:3] = R_cam2gripper; Tcam2gripper[0:3, -1] = t_cam2gripper
    ee_pose = fa.get_pose()
    Tgripper2base[0:3, 0:3] = ee_pose.rotation; Tgripper2base[0:3, -1] = ee_pose.translation
    Tcam2base = np.matmul(Tgripper2base, Tcam2gripper)
#####


######### aruco board 
markerLength = 0.0265
markerSeparation = 0.0056
aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_6X6_1000 )
#aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_4X4_1000 )

board = cv2.aruco.GridBoard_create(5, 7, markerLength, markerSeparation, aruco_dict)
#img = cv2.aruco.drawPlanarBoard(board, (2550,3300))# for printing on A4 paper
#cv2.imwrite('/home/ruthrash/test.jpg', img)
arucoParams = cv2.aruco.DetectorParameters_create()
#########

corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image_gray, aruco_dict, parameters=arucoParams)  # First, detect markers
refine_corners(image_gray, corners)

if ids is not None: 
    
    # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
    #rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], markerLength, camera_matrix, dist_coeffs)
    rvec = None 
    tvec = None
    retval, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, rvec, tvec)  # posture estimation from a diamond

    print("reprojection error in the camera frame: ")
    reproj_error =  reprojection_error(corners, ids, rvec, tvec, board, camera_matrix, dist_coeffs)
    print(reproj_error)
    # Tcam2base = np.eye(4)

    new_points = objPoints_in_robot_base(board, Tcam2base, rvec, tvec)

    print("reprojection error in the robot frame: ")
    reproj_error =  reprojection_error_in_robot_base(corners, ids, rvec, tvec, new_points, camera_matrix, dist_coeffs, Tcam2base)
    print(reproj_error)


    


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




'''

import cv2 
import numpy as np
markerLength = 0.0265
markerSeparation = 0.0057
aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_6X6_1000 )

board = cv2.aruco.GridBoard_create(5, 7, markerLength, markerSeparation, aruco_dict)
'''