
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
from CalibrationUtils import *

# test aruco marker objects 

eye_in_hand = True

# input calibration result here 
# R_cam2gripper = np.array([[ 0.01284593, -0.99982531,  0.01357677],
#  [ 0.99991016 , 0.01279266 ,-0.00400265],
#  [ 0.00382827 , 0.01362697 , 0.99989982]])
# t_cam2gripper = np.array([0.05031603, -0.03004549, -0.03017308])


# R_cam2gripper = np.array([[ 0.00593383, -0.99943719,  0.03301643],
#                             [ 0.99997778 , 0.00603086,  0.00284006],
#                             [-0.00303758 , 0.03299884,  0.99945077]])
# t_cam2gripper = np.array([0.04309507, -0.03661075, -0.05638443]) 


#tsai
R_cam2gripper = np.array([[ 0.00555508, -0.99973669,  0.02226396],
 [ 0.99985955,  0.00520096, -0.0159319 ],
 [ 0.01581191 , 0.02234934 , 0.99962518]])
t_cam2gripper = np.array([ 0.0610764,  -0.02725983, -0.06051241])

# #park 
# R_cam2gripper = np.array([[ 0.00406973, -0.99969235,  0.02446718],
#  [ 0.9998532,   0.00366071, -0.01673859],
#  [ 0.01664387 , 0.02453171 , 0.99956049]])
# t_cam2gripper = np.array([0.06024891 ,-0.02663721 ,-0.06075412])

# #horaud 
# R_cam2gripper = np.array([[ 0.00405254, -0.99969247 , 0.02446506],
#  [ 0.99985166 , 0.00364119, -0.01683466],
#  [ 0.0167404 ,  0.02452965 , 0.99955893]])
# t_cam2gripper = np.array([ 0.06024806, -0.02659598, -0.06075694])


# #andreff
# R_cam2gripper = np.array([[ 2.28728086e-04, -9.99675590e-01,  2.54688687e-02],
#  [ 9.99871854e-01, -1.79055587e-04 ,-1.60076319e-02],
#  [ 1.60069992e-02 , 2.54692664e-02 , 9.99547444e-01]])
# t_cam2gripper = np.array([0.05703949, -0.02572831 ,-0.04573494])

# #daniilidis
# R_cam2gripper = np.array([[-0.001155,   -0.99963885,  0.02684832],
#  [ 0.99989996, -0.00153297, -0.01406162],
#  [ 0.0140977 ,  0.02682939,  0.99954061]])
# t_cam2gripper = np.array([0.06018967, -0.02653409 ,-0.06085566])
#####################RANSAC####################33
#tsai
# R_cam2gripper = np.array([[-0.02469115, -0.99935464,  0.02608922],
#  [ 0.9995544 , -0.02511714, -0.01612843],
#  [ 0.01677331 , 0.02567937 , 0.9995295 ]])
# t_cam2gripper = np.array([0.05925489, -0.02886778, -0.0480761])

# # # park 
# R_cam2gripper = np.array([[ 0.01043955, -0.99911444 , 0.04075958],
#  [ 0.99994138 , 0.01031375 ,-0.0032954 ],
#  [ 0.0028721  , 0.04079159,  0.99916355]])
# t_cam2gripper = np.array([ 0.04151907, -0.029871,   -0.05880735])

# # #horaud 
# R_cam2gripper = np.array([[ 0.00958403, -0.99915697 , 0.03991863],
#  [ 0.99994929 , 0.00945294 ,-0.00347133],
#  [ 0.00309106 , 0.03994987 , 0.9991969 ]])
# t_cam2gripper = np.array([0.04407418, -0.03598013,-0.05371199])

# #andreff
# R_cam2gripper = np.array([[-0.02247551, -0.99940345,  0.0262221 ],
#  [ 0.99967142, -0.02278941, -0.01173394],
#  [ 0.01232453 , 0.02594976,  0.99958727]])
# t_cam2gripper = np.array([  0.05898052, -0.02831087, -0.0456534])

# #daniilidis
# R_cam2gripper = np.array([[-0.01577934, -0.9992651,   0.03493251],
#  [ 0.99946251, -0.01676724, -0.02817019],
#  [ 0.02873521 , 0.03446923,  0.99899257]])
# t_cam2gripper = np.array([0.05009148, -0.02226693, -0.05529808])


cam2gripper_pose = np.eye(4)
cam2gripper_pose[0:3, 0:3] = R_cam2gripper
cam2gripper_pose[0:3, -1] = t_cam2gripper
print(cam2gripper_pose)
print("##")

#create frankapy object
fa = FrankaArm()
print("##")
# fa.reset_joints()
print("##")
ini_rot = np.array([[ 0.99984526,  0.00864872, -0.01467651],
       [ 0.00810089, -0.99927337, -0.03698498],
       [-0.01498572,  0.03686036, -0.99920804]])
trans = np.array([0.55879714, 0.03248001, 0.27579337])
ini_pose = fa.get_pose() 
ini_pose.translation = trans; ini_pose.rotation = ini_rot
# if(eye_in_hand):    
#     fa.goto_pose(ini_pose)
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


markerLength = 0.025
aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_5X5_1000 )
arucoParams = cv2.aruco.DetectorParameters_create()
corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image_gray, aruco_dict, parameters=arucoParams,)  # First, detect markers
refine_corners(image_gray, corners)

tvecs = []
rvecs = []
r_offset = R.from_euler('z', 45, degrees=True)

if ids is not None: 
    for i in range(0, len(ids)):
        # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], markerLength, camera_matrix, dist_coeffs)
        print(reprojection_error_single_aruco_tag(corners, markerPoints, rvec, tvec, camera_matrix, dist_coeffs, id=i))
        #print(i, rvec)
        rotation_matrix = np.zeros(shape=(3,3))
        cv2.Rodrigues(rvec, rotation_matrix) 
        t = np.eye(4); t[0:3, 0:3] = rotation_matrix 
        print(i, (np.array(tf_utils.euler_from_matrix(t))*180.0/np.pi))
        print(rotation_matrix)
        #new = np.matmul(np.array(rotation_matrix), r_offset.as_matrix())    
        # rvec = None 
        # tvec = None
        #retval, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, rvec, tvec)  # posture estimation from a diamond
        # Draw a square around the markers
        
        tvecs.append(tvec)
        # Draw Axisimport rospy

avg_tvec = None
for tvec in tvecs: 
    if (avg_tvec is None):
        avg_tvec = tvec
    else:
        avg_tvec += tvec
avg_tvec = avg_tvec/len(tvecs)
print(tvecs)
print(avg_tvec) 

cv2.drawFrameAxes(color_im, camera_matrix, dist_coeffs, rvec, avg_tvec, 0.03)
cv2.imshow('Estimated Pose', color_im)
key = cv2.waitKey(0)  
cv2.destroyAllWindows()
'''
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
    
    # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
    #rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], markerLength, camera_matrix, dist_coeffs)
    rvec = None 
    tvec = None
    retval, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, rvec, tvec)  # posture estimation from a diamond

    rotation_matrix = np.zeros(shape=(3,3))
    cv2.Rodrigues(rvec, rotation_matrix)
    print(rotation_matrix)
    # print(rvec)
    # rotation_matrix[:, 2] = -1* rotation_matrix[:, 2]
    # t = rotation_matrix[:, 1].copy()
    # rotation_matrix[:, 1] = rotation_matrix[:, 0].copy()
    # rotation_matrix[:, 0] = t
    # rvec_new = cv2.Rodrigues(rotation_matrix)
    print(rotation_matrix)
    #print(rvec, rvec_new[0])
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
'''



ee2base_ = fa.get_pose()


# print(ee2base_)
tag2cam = np.eye(4)
ee2base = np.eye(4)
cam2ee = np.eye(4)
cam2base = np.eye(4)

tag2cam[0:3, 0:3] = rotation_matrix; 
#tag2cam[0:3, -1] = tvec[:, 0]
tag2cam[0:3, -1] = avg_tvec
cam2ee = cam2gripper_pose


if(eye_in_hand):
    ee2base[0:3, 0:3] = ee2base_.rotation; ee2base[0:3, -1] = ee2base_.translation
    goal_matrix = np.matmul(ee2base, np.matmul(cam2ee, tag2cam ))

else: 
    cam2base[0:3, 0:3] = np.array([[-0.06493062 ,-0.68077015 , 0.7296136 ],
                                [-0.99759733,  0.02658313, -0.0639758 ],
                                [ 0.02415741, -0.73201457 ,-0.68086055]])
    cam2base[0:3, -1] = np.array([-0.10220491, -0.37519443,  1.06997212])
    goal_matrix = np.matmul(cam2base, tag2cam )




goal = fa.get_pose()
# print(tag2cam)
# print(cam2ee)goal.translation[2]-= 0.15
# goal.translation[2]+= 0.042

# fa.goto_pose(goal)
# print(ee2base)


print("before")
rot = goal_matrix[0:3, 0:3]
print(rot)
rot[:, 2] = -1* rot[:, 2]
t = rot[:, 1].copy()
rot[:, 1] = rot[:, 0].copy()
rot[:, 0] = t
print("after")
print(rot)

rot = np.array([[ 9.97775978e-01, -5.61477743e-02, -3.56552590e-02],
       [-5.60788130e-02, -9.98412398e-01,  2.93206582e-03],
       [-3.57632816e-02, -9.26040242e-04, -9.99359848e-01]])

goal.rotation = rot
goal.translation = goal_matrix[0:3, -1]; goal.translation[2]+= 0.15
print("goal_pose")

print(goal)
fa.goto_pose(goal)


goal.translation[2]-= 0.15
goal.translation[2]+= 0.052

fa.goto_pose(goal)
current_pose = fa.get_pose()
#error = np.linalg.norm(np.array(goal.translation[:2])- np.array(current_pose.translation[:2]))
time.sleep(2)
true = np.array([ 0.42233412, -0.07618003])
error = np.linalg.norm(np.array(goal.translation[:2])- true)
print('error', error)
print(current_pose)

    
# goal.translation[2] -= 0.1
# fa.goto_pose(goal)
# R_tag2cam = 
# t_tag2cam = 

# R_tag2gripper = 
# t_tag2gripper = 

# rotate it upside down 

