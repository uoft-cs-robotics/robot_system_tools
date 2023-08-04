import numpy as np 

#Real Sense libraries#
from perception.realsense_sensor import RealSenseSensor
import pyrealsense2 as rs

import cv2 
import tf.transformations as tf_utils
import matplotlib.pyplot as plt

def refine_corners( image, corners):
    winSize = [5, 5]
    zeroZone = [-1, -1]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 10, 0.001)
    for corner in corners: 
        cv2.cornerSubPix(image, corner, winSize, zeroZone, criteria)

def reprojection_error( all_corners, ids,  rvec, tvec, board, camera_matrix, dist_coeffs): 
    mean_error = 0.0
    singular_mean_error = 0.0
    for id_, corners in zip(ids, all_corners):
        proj_img_point, _ = cv2.projectPoints(board.getObjPoints()[id_[0]], rvec, tvec, camera_matrix, dist_coeffs )
        error = cv2.norm(corners[0], proj_img_point[:,0,:], cv2.NORM_L2)/len(proj_img_point)
        mean_error += error
    return mean_error/len(ids)

Tcam2gripper = np.eye(4)
Tcam2gripper[0:3,0:3] = np.array([[-0.00679546 ,-0.99993656 , 0.00898346],
 [ 0.99996937 ,-0.00676026 , 0.00394327],
 [-0.00388229 , 0.00900998 , 0.99995187]])

Tcam2gripper[0:3,-1] = np.array([0.05467995, -0.03614509, -0.039079 ])

from frankapy import FrankaArm 

fa = FrankaArm()
fa.reset_joints()
fa.close_gripper()
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

ARUCO_BOARD_N_ROWS= 5
ARUCO_BOARD_N_COLS = 7 
ARUCO_MARKER_LENGTH = 0.025
ARUCO_MARKER_SEPARATION = 0.005


# ARUCO_BOARD_N_ROWS= 4
# ARUCO_BOARD_N_COLS = 5 
# ARUCO_MARKER_LENGTH = 0.039
# ARUCO_MARKER_SEPARATION = 0.005

aruco_dict = cv2.aruco.getPredefinedDictionary( cv2.aruco.DICT_6X6_1000 )
board = cv2.aruco.GridBoard((ARUCO_BOARD_N_ROWS, ARUCO_BOARD_N_COLS), 
                                ARUCO_MARKER_LENGTH, 
                                ARUCO_MARKER_SEPARATION, 
                                aruco_dict)
arucoParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, 
                                        arucoParams)
for _ in range(20):
    color_im_, depth_im_ = sensor.frames()
    color_im = color_im_.raw_data
    image_gray = cv2.cvtColor(color_im, cv2.COLOR_BGR2GRAY)

corners, ids, rejectedImgPoints = detector.detectMarkers(image_gray)  # First, detect markers
# refine_corners(image_gray, corners)

if ids is not None: 
    rvec = None 
    tvec = None
    objPoints= None; imgPoints = None
    objPoints, imgPoints = board.matchImagePoints(corners, ids, objPoints, imgPoints)
    retval, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, camera_matrix, rvec, tvec)

    cv2.aruco.drawDetectedMarkers(color_im, corners, borderColor=(0, 0, 255))
    cv2.drawFrameAxes(color_im, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
    reproj_error =  reprojection_error(corners,
                                        ids,
                                        rvec, tvec,
                                        board, 
                                        camera_matrix, 
                                        dist_coeffs)  
    print("reprojection_error:", reproj_error)
    plt.imshow(color_im)
    plt.show()

ee2base_fa = fa.get_pose()
Tgripper2base = np.eye(4); Tgripper2base[0:3, 0:3] = ee2base_fa.rotation; Tgripper2base[0:3, -1] = ee2base_fa.translation
Tcam2base = np.matmul(Tgripper2base, Tcam2gripper)


Ttag2cam = np.eye(4); rotation_matrix = np.eye(3) ; print("rvec", rvec)
cv2.Rodrigues(rvec, rotation_matrix); print("rotation",rotation_matrix)

Ttag2cam[0:3, 0:3] = rotation_matrix; Ttag2cam[0:3, -1] = np.squeeze(tvec) #Ttag2cam[0:3, -1] = tvec[:,0]

Ttag2base = np.matmul(Tcam2base, Ttag2cam)

goal_pose = fa.get_pose()
goal_pose.translation = Ttag2base[0:3,-1] + np.array([0.0,0.0,0.02])
print(goal_pose.translation)
# goal_pose.rotation = Ttag2base[0:3,0:3]

fa.goto_pose(goal_pose,ignore_virtual_walls=True, use_impedance=False)

current_pose = fa.get_pose()
# current_pose.translation[2] -= 0.1
print(np.linalg.norm(current_pose.translation - goal_pose.translation))
print(current_pose.translation)
print(goal_pose.translation)
