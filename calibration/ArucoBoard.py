#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import tf 
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf.transformations as tf_utils


br = tf.TransformBroadcaster()
bridge = CvBridge()

def callback(data):
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    markerLength = 0.0262
    markerSeparation = 0.0054
    aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_6X6_1000 )

    board = cv2.aruco.GridBoard_create(5, 7, markerLength, markerSeparation, aruco_dict)
    dist_coeffs = np.array([0.0,0.0,0.0,0.0])
    camera_matrix = np.array([[909.341796875, 0.0, 643.876220703125], [0.0, 908.0641479492188, 348.6051330566406],[0.0,0.0,1.0]])
    arucoParams = cv2.aruco.DetectorParameters_create()    
    image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image_gray, aruco_dict, parameters=arucoParams)  # First, detect markers
    rvec = None; tvec = None; 
    retval, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, rvec, tvec)  # posture estimation from a diamond
    rotation_matrix = np.zeros(shape=(3,3))
    cv2.Rodrigues(rvec, rotation_matrix)
    tf_rot = np.eye(4); tf_rot[0:3, 0:3] = rotation_matrix
    br.sendTransform(tvec, tf_utils.quaternion_from_matrix(tf_rot), rospy.Time.now(), 'board', 'camera_depth_optical_frame')
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/camera/color/image_raw", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()