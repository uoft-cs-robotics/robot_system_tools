
# tag pose estimation         
# rotate pose to change +ve z to -ve z
# ransac changed to use cv2 
import numpy as np
import os
import time
import cv2
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import tf 

#Real Sense libraries#
from perception.realsense_sensor import RealSenseSensor
import pyrealsense2 as rs


#FrankaPy and ROS#
# from frankapy import FrankaArm
from scipy.spatial.transform import Rotation as R
import tf.transformations as tf_utils
from CalibrationUtils import *
import rospy
from std_msgs.msg import Float64

ip = input("Press Enter to continue collecting current sample....else space bar to stop")


# realsense sensor 
# ctx = rs.context()
# device_id = ctx.devices[0].get_info(rs.camera_info.serial_number)
# sensor = RealSenseSensor(device_id, frame="realsense", filter_depth=True)
# sensor.start()
# intr = sensor.color_intrinsics
# #intr_list = [intr._fx, intr._fy, intr._cx, intr._cy]
# camera_matrix = np.array([[intr._fx, 0.0, intr._cx], [0.0, intr._fy, intr._cy],[0.0,0.0,1.0]])
# dist_coeffs =np.array([0.0,0.0,0.0,0.0])

br = tf.TransformBroadcaster()
bridge = CvBridge()
    
count = 0
color_im = None
def callback(data):
    tvecs = []
    rots = []
    global count, color_im

    try:
        color_im = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    count = count + 1
    camera_matrix = np.array([[908.7789916992188, 0.0, 638.7603149414062], [0.0, 907.6327514648438,341.215576171875],[0.0,0.0,1.0]])
    dist_coeffs = np.array([0.0,0.0,0.0,0.0])
    image_gray = cv2.cvtColor(color_im, cv2.COLOR_BGR2GRAY)

    markerLength = 0.025
    aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_5X5_1000 )
    arucoParams = cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image_gray, aruco_dict, parameters=arucoParams,)  # First, detect markers
    refine_corners(image_gray, corners)

    if ids is not None: 
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], markerLength, camera_matrix, dist_coeffs)
            print(reprojection_error_single_aruco_tag(corners, markerPoints, rvec, tvec, camera_matrix, dist_coeffs, id=i))
            cv2.drawFrameAxes(color_im, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

            #print(i, rvec)
            rotation_matrix = np.zeros(shape=(3,3))
            cv2.Rodrigues(rvec, rotation_matrix) 
            t = np.eye(4); t[0:3, 0:3] = rotation_matrix 
            print(i, (np.array(tf_utils.euler_from_matrix(t))*180.0/np.pi))
            print(rotation_matrix)
            rots.append(rotation_matrix)
            tvecs.append(tvec)
    i = 0
    for rot, tvec in zip(rots, tvecs):
        rot_ = np.eye(4); rot_[0:3, 0:3] = rot
        # print(tvec.shape, rot.shape)
        br.sendTransform(tvec[0,0,:], tf_utils.quaternion_from_matrix(rot_), rospy.Time.now(), 'tag_'+str(i), 'camera_color_optical_frame')
        i += 1
        print('publishing tf')



def process_image(event=None):
    color_im_, depth_im_ = sensor.frames()
    color_im = color_im_.raw_data
    image_gray = cv2.cvtColor(color_im, cv2.COLOR_BGR2GRAY)

    markerLength = 0.025
    aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_5X5_1000 )
    arucoParams = cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image_gray, aruco_dict, parameters=arucoParams,)  # First, detect markers
    refine_corners(image_gray, corners)

    if ids is not None: 
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], markerLength, camera_matrix, dist_coeffs)
            print(reprojection_error_single_aruco_tag(corners, markerPoints, rvec, tvec, camera_matrix, dist_coeffs, id=i))
            cv2.drawFrameAxes(color_im, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

            #print(i, rvec)
            rotation_matrix = np.zeros(shape=(3,3))
            cv2.Rodrigues(rvec, rotation_matrix) 
            t = np.eye(4); t[0:3, 0:3] = rotation_matrix 
            print(i, (np.array(tf_utils.euler_from_matrix(t))*180.0/np.pi))
            print(rotation_matrix)
            rots.append(rotation_matrix)
            tvecs.append(tvec)
    i = 0
    for rot, tvec in zip(rots, tvecs):
        rot_ = np.eye(4); rot_[0:3, 0:3] = rot
        print(tvec.shape, rot.shape)
        br.sendTransform(tvec[0,0,:], tf_utils.quaternion_from_matrix(rot_), rospy.Time.now(), 'tag_'+str(i), 'camera_color_optical_frame')
        i += 1
        print('publishing tf')


if __name__ == '__main__':
    rospy.init_node("your_sensor_node")
    # Create an instance of Temperature sensor
    rospy.Subscriber("/camera/color/image_raw", Image, callback)
    # Create a ROS Timer for reading data
    # rospy.Timer(rospy.Duration(1.0/10.0), process_image)
    # Create another ROS Timer for publishing data
    # Don't forget this or else the program will exit
    rospy.spin()