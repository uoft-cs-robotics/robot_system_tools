
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

import copy
import time
####### test aruco marker objects 
markerLength = 0.0404
aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_4X4_1000 )
arucoParams = cv2.aruco.DetectorParameters_create()
#######

####### realsense sensor 
ctx = rs.context()
device_id = ctx.devices[0].get_info(rs.camera_info.serial_number)
sensor = RealSenseSensor(device_id, frame="realsense", filter_depth=True)
sensor.start()
intr = sensor.color_intrinsics
#intr_list = [intr._fx, intr._fy, intr._cx, intr._cy]
camera_matrix = np.array([[intr._fx, 0.0, intr._cx], [0.0, intr._fy, intr._cy],[0.0,0.0,1.0]])
dist_coeffs =np.array([0.0,0.0,0.0,0.0])
#######

eye_in_hand = True
print("yes1")
fa = FrankaArm()
print("yes2")

if not eye_in_hand: 
    fa.reset_joints()


if eye_in_hand: 
    R_cam2gripper = np.array([[ 1.76727722e-02, -9.99540758e-01,  2.46159712e-02],
 [ 9.99843823e-01 , 1.76660665e-02, -4.89871083e-04],
 [ 5.47787289e-05,  2.46207842e-02 , 9.99696861e-01]])

    t_cam2gripper = np.array([ 48.7694127 , -33.28287964 ,-38.80161595])/1000.0












#     R_cam2gripper = np.array([[ 0.02864086, -0.9995428 ,  0.00968959],
#  [ 0.99955481,  0.02871958,  0.00808479],
#  [-0.00835937,  0.00945372  ,0.99992037]]


# )
#     t_cam2gripper = np.array([ 53.00361167, -38.211703 ,  -61.15352875])/1000.0
#     R_cam2gripper = np.array([[ 0.00481826, -0.99986074 , 0.0159775 ],
#  [ 0.99998557 , 0.00477971, -0.00245062],
#  [ 0.00237391 , 0.01598908,  0.99986935]]

# ) best stuff
#     t_cam2gripper = np.array([ 54.91350466, -35.04783465, -56.64075378])/1000.0
        
#     R_cam2gripper = np.array([[ 0.02276629 ,-0.99950498,  0.02171403],
#  [ 0.99973003,  0.02286155 , 0.00414871],
#  [-0.00464308,  0.02161371,  0.99975561]]

# )
#     t_cam2gripper = np.array([ 0.05003657, -0.03845091, -0.06112441])
       
#     R_cam2gripper = np.array([[ 0.00587679, -0.99971619,  0.02308704],
#  [ 0.99997304 , 0.00597684,  0.00426689],
#  [-0.00440367, 0.02306135 , 0.99972435]]

# )
#     t_cam2gripper = np.array([ 0.05136074 -0.03452872 -0.05884213]) good stuff
   

#     R_cam2gripper = np.array([[ 0.01807741 ,-0.99931022 , 0.03243911],
#  [ 0.99979569, 0.01777368, -0.00962724],
#  [ 0.00904404 , 0.03260652  ,0.99942735]])
#     t_cam2gripper = np.array([ 0.04748442, -0.03227636 ,-0.05801888])
    Tcam2gripper = np.eye(4); Tcam2gripper[0:3, 0:3] = R_cam2gripper; Tcam2gripper[0:3, -1] = t_cam2gripper
    
    ee2base_fa = fa.get_pose() 
    Tgripper2base = np.eye(4); Tgripper2base[0:3, 0:3] = ee2base_fa.rotation; Tgripper2base[0:3, -1] = ee2base_fa.translation
    Tcam2base = np.matmul(Tgripper2base, Tcam2gripper)
    print("current height: ", Tcam2base[2, -1])
else: 
    pass
#     Tcam2base = np.eye(4); 
#     Tcam2base[0:3, 0:3] = np.array([[ 2.37290625e-02, -9.99505536e-01,  2.06304568e-02],
#  [ 9.99718394e-01 , 2.37292629e-02, -2.35119432e-04],
#  [-2.54542360e-04 , 2.06302263e-02 , 9.99787142e-01]])
#     Tcam2base[0:3, -1] = np.array([ 0.05467272 ,-0.03602785, -0.06702594])      
    # R_cam2base = np.array([[-0.17833759, -0.86612861 , 0.46692283],
    #                     [-0.98123311 , 0.19190743, -0.01879162],
    #                     [-0.07333  ,  -0.46151139, -0.88409838]])
    # t_cam2base = np.array([-0.27941828, -0.56453168,  0.87719238])    
    #Tcam2base = np.eye(4); Tcam2base[0:3, 0:3] = R_cam2base; Tcam2base[0:3, -1] = t_cam2base  

rvecs = []
tvecs = []
rvec = None; tvec = None
for i in range(10):
    print(i)
    color_im_, depth_im_ = sensor.frames()
    color_im = color_im_.raw_data
    image_gray = cv2.cvtColor(color_im, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image_gray, aruco_dict, parameters=arucoParams,)  # First, detect markers
    refine_corners(image_gray, corners)
    cv2.aruco.drawDetectedMarkers(color_im, corners, borderColor=(0, 0, 255))
    # cv2.drawFrameAxes(color_im, camera_matrix, dist_coeffs, rvec, tvec, 0.03)
    # cv2.imshow('Estimated Pose', image_gray)
    # key = cv2.waitKey(0)  
    # cv2.destroyAllWindows()
    time.sleep(0.5)

    if ids is not None: 
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], markerLength, camera_matrix, dist_coeffs)
            #print(markerPoints)
            print(reprojection_error_single_aruco_tag(corners, markerPoints, rvec, tvec, camera_matrix, dist_coeffs))
            rvecs.append(rvec)
            tvecs.append(tvec)
    time.sleep(0.3)
avg_tvec = None
for tvec in tvecs: 
    if (avg_tvec is None):
        avg_tvec = tvec
    else:
        avg_tvec += tvec
avg_tvec = avg_tvec/len(tvecs)

cv2.drawFrameAxes(color_im, camera_matrix, dist_coeffs, rvec, avg_tvec, 0.03)
cv2.imshow('Estimated Pose', color_im)
key = cv2.waitKey(0)  
cv2.destroyAllWindows()
sensor.stop()

Ttag2cam = np.eye(4); rotation_matrix = np.eye(3) ; print(rvec)
cv2.Rodrigues(rvec, rotation_matrix); print(rotation_matrix)

# rotation_matrix[:,2] = -1*rotation_matrix[:,2]
# t = copy.copy(rotation_matrix[:,0])
# rotation_matrix[:,0] = rotation_matrix[:,1]
# rotation_matrix[:,1] = t; print(rotation_matrix)



Ttag2cam[0:3, 0:3] = rotation_matrix; Ttag2cam[0:3, -1] = avg_tvec #Ttag2cam[0:3, -1] = tvec[:,0]

Ttag2base = np.matmul(Tcam2base, Ttag2cam)

rotation_matrix = Ttag2base[0:3, 0:3]
rotation_matrix[:,2] = -1*rotation_matrix[:,2]
t = copy.copy(rotation_matrix[:,0])
rotation_matrix[:,0] = rotation_matrix[:,1]
rotation_matrix[:,1] = t; print(rotation_matrix)
Ttag2base[0:3, 0:3] = rotation_matrix
pose = fa.get_pose() 

pose.translation = Ttag2base[0:3, -1]; 

# pose.rotation = np.array([[ 0.99774194, -0.04250232,  0.05182053],
#        [-0.04126788, -0.99883403, -0.02466388],
#        [ 0.05280838,  0.02246966, -0.9983518 ]])
       
pose.rotation = Ttag2base[0:3, 0:3]


# Tright2left = np.array([[ 0.99849453,  0.01417743, -0.05298756,  0.02435545],
#        [-0.01404807,  0.99989736,  0.00281301,  0.97666475],
#        [ 0.053022  , -0.0020644 ,  0.99859122, -0.02058212],
#        [ 0.        ,  0.        ,  0.        ,  1.        ]])
# # Tright2left = np.array([[ 1.0,  0.0, 0.0,  0.0],
# #                         [0.0,  1.0,  0.0,  0.97666475],
# #                         [ 0.0  , 0.0 ,  1.0, -0.0],
# #                          [ 0.        ,  0.        ,  0.        ,  1.        ]])
# Tcube2right = np.eye(4); Tcube2right[0:3, 0:3] = pose.rotation; Tcube2right[0:3, -1] = pose.translation
# T = np.matmul(  Tright2left,Tcube2right)
# print("in left", T)
# print("in left", np.array(tf_utils.euler_from_matrix(T))*180.0/np.pi)

pose.translation[2] += 0.2
print(pose)
fa.goto_pose(pose)


pose.translation[2] -= 0.2
print("goal:", pose.translation)
fa.goto_pose(pose)

current = fa.get_pose()
print("current:", current.translation)


'''
import numpy as np 
import tf.transformations as tf_utils 
[ 0.04894636 ,-0.0344041 , -0.06674745 ]
Tee2right  = np.eye(4)
Tee2right[0:3, 0:3] = np.array([[ 0.99912148,  0.00342975, -0.0415366 ],
                                [ 0.00304987, -0.99994335, -0.00920581],
                                [-0.04156582,  0.00907104, -0.99909457]])
Tee2right[0:3, -1] = np.array([ 0.44321501, -0.45064716,  0.03013582])

Tee2left = np.eye(4)
Tee2left[0:3, 0:3] = np.array([[ 0.99986304, -0.01123269,  0.011335  ],
                                [-0.0111031 , -0.99986338, -0.01143182],
                                [ 0.01146186,  0.0113044 , -0.99987041]])
Tee2left[0:3, -1] =   np.array([0.45891737, 0.5199223 , 0.03394171])                              



Tright2left = np.matmul(Tee2left, tf_utils.inverse_matrix(Tee2right))
'''
exit()
import numpy as np
from frankapy import FrankaArm

fa = FrankaArm() 

truth = fa.get_pose().translation
# print(truth)


goal = np.array([0.35856752, 0.04358505, 0.04719299])
goal[2] -= 0.025
goal2 = np.array([0.41535317 ,0.00652426, 0.04415178])
print(truth)
print(goal)
print(truth-goal)
print(np.linalg.norm(goal - truth))


truth = np.array([[0.44527214, 0.0526369 , 0.01031098]])
[ 48.7694127  -33.28287964 -38.80161595]





# [[ 0.02164965 -0.99938713  0.02750737]
#  [ 0.9997642   0.02159501 -0.00228191]
#  [ 0.00168649  0.02755029  0.999619  ]]
# [ 41.42451182 -36.81290169 -53.97740803]

# [[ 0.05155956 -0.99838984  0.02365017]
#  [ 0.99809937  0.05231605  0.0325682 ]
#  [-0.03375305  0.02192602  0.99918966]]
# [ 32.17782609 -58.45954057 -65.36996991]
