
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

eye_in_hand = False
fa = FrankaArm()


if not eye_in_hand: 
    fa.reset_joints()


if eye_in_hand: 
    
    R_cam2gripper = np.array([[ 3.01473874e-04, -9.99535968e-01 , 3.04591042e-02],
                            [ 9.99999331e-01 , 3.35343425e-04 , 1.10686584e-03],
                            [-1.11656648e-03 , 3.04587502e-02 , 9.99535401e-01]])
    t_cam2gripper = np.array([ 0.04466579, -0.03646603, -0.06275917])    
    Tcam2gripper = np.eye(4); Tcam2gripper[0:3, 0:3] = R_cam2gripper; Tcam2gripper[0:3, -1] = t_cam2gripper
    
    ee2base_fa = fa.get_pose() 
    Tgripper2base = np.eye(4); Tgripper2base[0:3, 0:3] = ee2base_fa.rotation; Tgripper2base[0:3, -1] = ee2base_fa.translation
    Tcam2base = np.matmul(Tgripper2base, Tcam2gripper)
    print("current height: ", Tcam2base[2, -1])
else: 
    R_cam2base = np.array([[-0.14252794, -0.87081441,  0.47049766],
                            [-0.98855273,  0.1490066 , -0.02367556],
                            [-0.04949023, -0.46848618, -0.88208357]])
    t_cam2base = np.array([-0.27909548, -0.56676275,  0.87199486])        
    # R_cam2base = np.array([[-0.17833759, -0.86612861 , 0.46692283],
    #                     [-0.98123311 , 0.19190743, -0.01879162],
    #                     [-0.07333  ,  -0.46151139, -0.88409838]])
    # t_cam2base = np.array([-0.27941828, -0.56453168,  0.87719238])    
    Tcam2base = np.eye(4); Tcam2base[0:3, 0:3] = R_cam2base; Tcam2base[0:3, -1] = t_cam2base  

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
pose.translation = Ttag2base[0:3, -1]; pose.rotation = np.array([[ 0.99774194, -0.04250232,  0.05182053],
       [-0.04126788, -0.99883403, -0.02466388],
       [ 0.05280838,  0.02246966, -0.9983518 ]])#pose.rotation = Ttag2base[0:3, 0:3]


pose.translation[2] += 0.2
print(pose)
fa.goto_pose(pose)

pose.translation[2] -= 0.2
print(pose)
fa.goto_pose(pose)