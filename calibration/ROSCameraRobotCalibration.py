import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np
from CalibrationUtils import *
from RANSAC import RANSAC 
import threading
import time
from cv_bridge import CvBridge
from tf import TransformListener
import tf.transformations as tf_utils

class ROSCameraRobotCalibration:
    def __init__(self, file_name:str='data/file.txt')-> None:     
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/camera/color/image_raw", Image, self.subscribeImage)# subscriber for RGB image
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.getCameraInfo)#subscriber for RGM image camera info 
        self.camera_matrix = None#np.array([[intr._fx, 0.0, intr._cx], [0.0, intr._fy, intr._cy],[0.0,0.0,1.0]])
        self.dist_coeffs = None#np.array([0.0,0.0,0.0,0.0])   
        self.create_aruco_objects()     
        self.img_msg = None
        self.tf_listener_ = TransformListener()
        self.robot_base_frame = "panda_link0"
        self.ee_frame = "panda_end_effector"
        self.file_name = file_name


    def create_aruco_objects(self):
        markerLength = 0.0265
        markerSeparation = 0.0057   
        self.aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_6X6_1000 )
        self.board = cv2.aruco.GridBoard_create(5, 7, markerLength, markerSeparation, self.aruco_dict)
        self.arucoParams = cv2.aruco.DetectorParameters_create()

    def subscribeImage(self, img_msg):
        if self.camera_matrix is None: 
            return 
        else: 
            self.img_msg = img_msg


    def getCameraInfo(self, info_msg):
        if(self.camera_matrix is None):
            self.dist_coeffs = np.array(info_msg.D[:4])
            self.camera_matrix = np.array(info_msg.K).reshape(3,3)
        else: 
            return        

    def userInteractionFunction(self,):
        cv_bridge = CvBridge()
        R_gripper2base = []; t_gripper2base = []     
        R_tag2cam = []; t_tag2cam = []  
        detections_count = 0
        processed_image = 0                     
        while(self.img_msg is None):#waits here until a new image is received
            pass
        while(True):      
            ip = input("Press Enter to continue collecting current sample....else space bar to stop")
            if (ip==""):
                time.sleep(0.5)      
                # assert(self.tf_listener_.frameExists(self.robot_base_frame))
                # assert(self.tf_listener_.frameExists(self.ee_frame))

                t = self.tf_listener_.getLatestCommonTime(self.robot_base_frame, self.ee_frame)
                ee_position, ee_quaternion = self.tf_listener_.lookupTransform(self.robot_base_frame, self.ee_frame,t)
                ee_rotation_matrix = tf_utils.quaternion_matrix(ee_quaternion)
                ee_rotation_matrix = ee_rotation_matrix[0:3, 0:3]
                color_im = cv_bridge.imgmsg_to_cv2(self.img_msg)         
                image_gray = cv2.cvtColor(color_im, cv2.COLOR_BGR2GRAY)      
                corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image_gray, self.aruco_dict, parameters=self.arucoParams)  # First, detect markers 
                refine_corners(image_gray, corners)
                if ids is not None: 
                    detections_count +=1
                    rvec = None 
                    tvec = None
                    retval, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, self.board, self.camera_matrix, self.dist_coeffs, rvec, tvec)  # p
                    #cv2.drawFrameAxes(color_im, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                    file_name = "data/image/image_"+str(detections_count-1)+".jpg"
                    # cv2.imwrite(file_name, color_im)
                    print("tag", rvec, tvec, retval )
                    print("ee_rot", ee_rotation_matrix)
                    print("ee",cv2.Rodrigues(ee_rotation_matrix)[0], ee_position  )
                    reproj_error =  reprojection_error(corners,
                                                        ids,
                                                        rvec, tvec,
                                                        self.board, 
                                                        self.camera_matrix, 
                                                        self.dist_coeffs)
                    print("reprojection error",reproj_error)
                    thresh = 0.3
                    if reproj_error >  thresh:
                        print("#### Very high reprojection error ####")
                        print("#### Ignoring this sample ####")
                        continue
                    else:
                        R_gripper2base.append(cv2.Rodrigues(ee_rotation_matrix)[0])
                        t_gripper2base.append(ee_position) 
                        R_tag2cam.append(rvec)
                        t_tag2cam.append(tvec)
                print(detections_count, processed_image)                           
            else:
                print("stopping data collection")
                if processed_image ==0 :
                    exit()
                break    
                # break
            processed_image +=1 
        print("writing to file")
        for ee_rot, ee_trans, tag_rot, tag_trans in zip(R_gripper2base, t_gripper2base, R_tag2cam, t_tag2cam):
            ee_pose_line = [str(i) for i in [ee_rot[0][0],ee_rot[1][0], ee_rot[2][0], ee_trans[0], ee_trans[1], ee_trans[2]]]
            tag_pose_line = [str(i) for i in [tag_rot[0][0], tag_rot[1][0], tag_rot[2][0], tag_trans[0][0], tag_trans[1][0], tag_trans[2][0] ]]
            line = ee_pose_line + tag_pose_line
            write_to_file(line, self.file_name)    
                



if __name__ == '__main__':
    calibration_object = ROSCameraRobotCalibration()
    rate = rospy.Rate(30) # 30hz
    x = threading.Thread(target=calibration_object.userInteractionFunction)
    x.start()
    if not rospy.core.is_initialized():
        raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
    try:
        while not rospy.core.is_shutdown():
            rate.sleep()
    except KeyboardInterrupt:
        rospy.core.signal_shutdown('keyboard interrupt')    
    x.join()
    

