
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
    
    R_cam2gripper = np.array([[ 0.04327561, -0.99861027 , 0.03007906],
 [ 0.99906139 , 0.04319899 ,-0.00319282],
 [ 0.00188899,  0.030189 ,   0.99954242]])
    t_cam2gripper = np.array([ 0.04777699, -0.03524564, -0.0507563]) 

    # R_cam2gripper = np.array([[ 0.01143509, -0.99975713,  0.01883957],
    #         [ 0.99990166,  0.01158566,  0.00790228],
    #         [-0.00811863,  0.01874735,  0.99979129]])
    # t_cam2gripper = np.array([0.05296211 ,  -0.03722842 ,-0.05813381]) ##most correct yet
    # R_cam2gripper = np.array([[ 0.01840936, -0.99965802,  0.01857263],
    #                         [ 0.99980491 , 0.01827272, -0.00749979],
    #                         [ 0.00715785,  0.01870707 , 0.99979939]])
    # t_cam2gripper = np.array([ 0.05071377, -0.03182113, -0.04429567])        
    Tcam2gripper = np.eye(4); Tcam2gripper[0:3, 0:3] = R_cam2gripper; Tcam2gripper[0:3, -1] = t_cam2gripper
    
    ee2base_fa = fa.get_pose() 
    Tgripper2base = np.eye(4); Tgripper2base[0:3, 0:3] = ee2base_fa.rotation; Tgripper2base[0:3, -1] = ee2base_fa.translation
    Tcam2base = np.matmul(Tgripper2base, Tcam2gripper)
    print("current height: ", Tcam2base[2, -1])
else: 
    Tcam2base = np.eye(4); 
    Tcam2base[0:3, 0:3] = np.array([[ 3.55710140e-01, -7.52908289e-01,  5.53714191e-01],
                                    [-9.34595965e-01 ,-2.87081277e-01 , 2.10035051e-01],
                                    [ 8.23846442e-04, -5.92210645e-01, -8.05782770e-01]])
    Tcam2base[0:3, -1] = np.array([-0.15937439, -0.51703974 , 0.8664317 ])      
    # R_cam2base = np.array([[-0.17833759, -0.86612861 , 0.46692283],
    #                     [-0.98123311 , 0.19190743, -0.01879162],
    #                     [-0.07333  ,  -0.46151139, -0.88409838]])
    # t_cam2base = np.array([-0.27941828, -0.56453168,  0.87719238])    
    #Tcam2base = np.eye(4); Tcam2base[0:3, 0:3] = R_cam2base; Tcam2base[0:3, -1] = t_cam2base  

rvecs = []
tvecs = []
rvec = None; tvec = None
for i in range(10):
    color_im_, depth_im_ = sensor.frames()
    color_im = color_im_.raw_data
    image_gray = cv2.cvtColor(color_im, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image_gray, aruco_dict, parameters=arucoParams,)  # First, detect markers
    refine_corners(image_gray, corners)
    cv2.aruco.drawDetectedMarkers(color_im, corners, borderColor=(0, 0, 255))

    
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


Tright2left = np.array([[ 0.99849453,  0.01417743, -0.05298756,  0.02435545],
       [-0.01404807,  0.99989736,  0.00281301,  0.97666475],
       [ 0.053022  , -0.0020644 ,  0.99859122, -0.02058212],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
# Tright2left = np.array([[ 1.0,  0.0, 0.0,  0.0],
#                         [0.0,  1.0,  0.0,  0.97666475],
#                         [ 0.0  , 0.0 ,  1.0, -0.0],
#                          [ 0.        ,  0.        ,  0.        ,  1.        ]])
Tcube2right = np.eye(4); Tcube2right[0:3, 0:3] = pose.rotation; Tcube2right[0:3, -1] = pose.translation
T = np.matmul(  Tright2left,Tcube2right)
print("in left", T)
print("in left", np.array(tf_utils.euler_from_matrix(T))*180.0/np.pi)

pose.translation[2] += 0.2
print(pose)
fa.goto_pose(pose)

# print(goal)
pose.translation[2] -= 0.2
print(pose)
fa.goto_pose(pose)



'''
import numpy as np 
import tf.transformations as tf_utils 

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