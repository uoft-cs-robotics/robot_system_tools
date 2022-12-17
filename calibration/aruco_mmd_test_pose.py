import matplotlib.pyplot as plt
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import math
from scipy.spatial.transform import Rotation as R

import rospy
import tf.transformations as tf_utils
from CalibrationUtils import * 
import tf

#Real Sense libraries#
from perception.realsense_sensor import RealSenseSensor
import pyrealsense2 as rs


camera_in_hand = True 

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

markerLength = 0.0404
aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_4X4_1000 )
arucoParams = cv2.aruco.DetectorParameters_create()
corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image_gray, aruco_dict, parameters=arucoParams,)  # First, detect markers

rvec = None; tvec = None

refine_corners(image_gray, corners)
cv2.aruco.drawDetectedMarkers(color_im, corners, borderColor=(0, 0, 255))
if ids is not None: 
    for i in range(0, len(ids)):
        # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], markerLength, camera_matrix, dist_coeffs)

cv2.drawFrameAxes(color_im, camera_matrix, dist_coeffs, rvec, tvec, 0.03)
cv2.imshow('Estimated Pose', color_im)
key = cv2.waitKey(0)  
cv2.destroyAllWindows()
sensor.stop()

if __name__ == '__main__':
    print(rvec, tvec)
    rotation_matrix = np.zeros(shape=(3,3))
    cv2.Rodrigues(rvec, rotation_matrix)
    print(rotation_matrix)
    Tcube2camera = np.eye(4); Tcube2camera[0:3, 0:3] = rotation_matrix
    Tcube2camera[0:3,-1] = tvec[:,0]

    rospy.init_node('cube', anonymous=True)
    try:
        while not rospy.core.is_shutdown():
            br = tf.TransformBroadcaster() 
            br.sendTransform(Tcube2camera[0:3,-1],
                    tf_utils.quaternion_from_matrix(Tcube2camera),
                    rospy.Time.now(),
                    'cube',
                    'camera_color_optical_frame'
                    )   

    except KeyboardInterrupt:
            rospy.core.signal_shutdown('keyboard interrupt')                      