import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np
import argparse 
import threading
import time
from cv_bridge import CvBridge
from tf import TransformListener
import tf.transformations as tf_utils

from CalibrationUtils import *
from RANSAC import RANSAC 

class ROSCameraRobotCalibration:
    def __init__(self, args, file_name:str='data/file.txt')-> None:     
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber(args.rgb_image_topic, Image, self.subscribeImage)# subscriber for RGB image
        rospy.Subscriber(args.rgb_camera_info_topic, CameraInfo, self.getCameraInfo)#subscriber for RGM image camera info 
        self.camera_matrix = None#np.array([[intr._fx, 0.0, intr._cx], [0.0, intr._fy, intr._cy],[0.0,0.0,1.0]])
        self.dist_coeffs = None#np.array([0.0,0.0,0.0,0.0])   
        self.create_aruco_objects()     
        self.img_msg = None
        self.tf_listener_ = TransformListener()
        self.robot_base_frame = args.robot_base_frame_name #"panda_link0"
        self.ee_frame = args.ee_frame_name#"panda_end_effector"
        self.file_name = file_name
        self.args = args
        self.calib_data_As = []; self.calib_data_As_tf = []
        self.calib_data_Bs = []; self.calib_data_Bs_tf = []      
          
        if(not self.args.only_calibration):
            open(self.file_name, 'w').close()#empty the file in which poses are recorded
    def create_aruco_objects(self):
        markerLength = 0.0265
        markerSeparation = 0.0057   
        self.aruco_dict = cv2.aruco.getPredefinedDictionary( cv2.aruco.DICT_6X6_1000 )
        self.board = cv2.aruco.GridBoard((5, 7), markerLength, markerSeparation, self.aruco_dict)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.arucoParams)

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
        prev_pose = None 
        while(True): 
                
            ip = input("Press Enter to continue collecting current sample....else space bar to stop")
            if (ip==""):
                time.sleep(0.5)      
                processed_image +=1
                # yaml API used in tf python breaks for some reason 
                # assert(self.tf_listener_.frameExists(self.robot_base_frame))
                # assert(self.tf_listener_.frameExists(self.ee_frame))

                t = self.tf_listener_.getLatestCommonTime(self.robot_base_frame, self.ee_frame)
                ee_position, ee_quaternion = self.tf_listener_.lookupTransform(self.robot_base_frame, self.ee_frame,t)
                ee_rotation_matrix = tf_utils.quaternion_matrix(ee_quaternion)
                ee_rotation_matrix = ee_rotation_matrix[0:3, 0:3]
                color_im = cv_bridge.imgmsg_to_cv2(self.img_msg)         
                image_gray = cv2.cvtColor(color_im, cv2.COLOR_BGR2GRAY)      
                corners, ids, rejectedImgPoints = self.detector.detectMarkers(image_gray)  # First, detect markers
                refine_corners(image_gray, corners)
                if ids is not None: 
                    
                    rvec = None 
                    tvec = None
                    objPoints= None; imgPoints = None
                    objPoints, imgPoints = self.board.matchImagePoints(corners, ids, objPoints, imgPoints)
                    # print(objPoints, imgPoints)                    
                    retval, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, self.camera_matrix, rvec, tvec)                    #cv2.drawFrameAxes(color_im, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                    if(args.debug_image):
                        cv2.aruco.drawDetectedMarkers(color_im, corners, borderColor=(0, 0, 255))
                        cv2.drawFrameAxes(color_im, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                        img_file_name = "data/image/image_"+str(detections_count-1)+".jpg"
                        cv2.imwrite(img_file_name, color_im)
                    # print("tag", rvec, tvec, retval )
                    # print("ee_rot", ee_rotation_matrix)
                    # print("ee",cv2.Rodrigues(ee_rotation_matrix)[0], ee_position  )
                    reproj_error =  reprojection_error(corners,
                                                        ids,
                                                        rvec, tvec,
                                                        self.board, 
                                                        self.camera_matrix, 
                                                        self.dist_coeffs)
                    print("reprojection error",reproj_error)
                    thresh = 0.3
                    if reproj_error >  thresh:
                        if(detections_count > 0 ):
                            current = np.eye(4); current[0:3, 0:3] = ee_rotation_matrix; current[0:3, -1] = ee_position
                            difference = np.matmul(np.linalg.inv(current), prev_pose) 
                            print("relative rotation in degrees, translation in m: ", 
                                    np.array(tf_utils.euler_from_matrix(difference))*180.0/np.pi,
                                    difference[0:3,-1])                        
                        print("#### Very high reprojection error ####")
                        print("#### Ignoring this sample ####")
                        continue
                    else:
                        R_gripper2base.append(cv2.Rodrigues(ee_rotation_matrix)[0])
                        t_gripper2base.append(ee_position) 
                        R_tag2cam.append(rvec)
                        t_tag2cam.append(tvec)
                    if (detections_count==0):
                        prev_pose = np.eye(4); prev_pose[0:3, 0:3] = ee_rotation_matrix; prev_pose[0:3, -1] = ee_position
                    else: 
                        current = np.eye(4); current[0:3, 0:3] = ee_rotation_matrix; current[0:3, -1] = ee_position
                        difference = np.matmul(np.linalg.inv(current), prev_pose) 
                        print("relative rotation in degrees, translation in m : ",
                                np.array(tf_utils.euler_from_matrix(difference))*180.0/np.pi,
                                difference[0:3,-1])
                        prev_pose = current 
                    detections_count +=1                           
                print("accepted pairs of pose, no. of frames processed", detections_count, processed_image)                           
            else:
                print("stopping data collection")
                if processed_image ==0 :
                    rospy.core.signal_shutdown('keyboard interrupt')
                    exit()
                rospy.core.signal_shutdown('keyboard interrupt')
                break    
                
                # break
            # rospy.core.signal_shutdown('keyboard interrupt')
            
        for ee_rot, ee_trans, tag_rot, tag_trans in zip(R_gripper2base, t_gripper2base, R_tag2cam, t_tag2cam):
            ee_pose_line = [str(i) for i in [ee_rot[0][0],ee_rot[1][0], ee_rot[2][0], ee_trans[0], ee_trans[1], ee_trans[2]]]
            tag_pose_line = [str(i) for i in [tag_rot[0][0], tag_rot[1][0], tag_rot[2][0], tag_trans[0][0], tag_trans[1][0], tag_trans[2][0] ]]
            line = ee_pose_line + tag_pose_line
            write_to_file(line, self.file_name)    
                
    def Calibrate(self,):
        with open(self.file_name, 'r') as fp:
            lines = fp.readlines()
        for line in lines:
            data = line.split('\n')[0].split(',')
            ee_pose = tuple(((float(data[0]),
                                float(data[1]),
                                    float(data[2])), 
                            (float(data[3]),
                                float(data[4]),
                                float(data[5]))))
            tag_pose = tuple(((float(data[6]),
                                float(data[7]),
                                float(data[8])), 
                            (float(data[9]),
                                float(data[10]),
                                float(data[11]))))
            ee_pose_tf = np.eye(4)
            tag_pose_tf = np.eye(4)
            ee_pose_tf[0:3, 0:3] = cv2.Rodrigues(ee_pose[0])[0]; ee_pose_tf[0:3, -1] = ee_pose[1]
            tag_pose_tf[0:3, 0:3] = cv2.Rodrigues(tag_pose[0])[0]; tag_pose_tf[0:3, -1] = tag_pose[1]  

            if(self.args.camera_in_hand):          
                self.calib_data_As.append(ee_pose)
                self.calib_data_As_tf.append(ee_pose_tf)
                self.calib_data_Bs.append(tag_pose)
                self.calib_data_Bs_tf.append(tag_pose_tf)     
            else:
                ee_pose_tf_inv = np.linalg.inv(ee_pose_tf)#tf_utils.inverse_matrix(ee_pose_tf)
                ee_pose = list(ee_pose)
                ee_pose[0] = cv2.Rodrigues(ee_pose_tf_inv[0:3, 0:3])[0]
                ee_pose[1] = ee_pose_tf_inv[0:3, -1]
                ee_pose = tuple(ee_pose)
                self.calib_data_As.append(ee_pose)
                self.calib_data_As_tf.append(ee_pose_tf_inv)
                self.calib_data_Bs.append(tag_pose)
                self.calib_data_Bs_tf.append(tag_pose_tf)                

        solvers = [cv2.CALIB_HAND_EYE_TSAI,
                    cv2.CALIB_HAND_EYE_PARK,
                    cv2.CALIB_HAND_EYE_HORAUD,
                    cv2.CALIB_HAND_EYE_ANDREFF,
                    cv2.CALIB_HAND_EYE_DANIILIDIS]

        for solver in solvers: 
            print("solver = ", solver)
            rs = RANSAC(As=self.calib_data_As, 
                        As_tf=self.calib_data_As_tf, 
                        Bs=self.calib_data_Bs, 
                        Bs_tf=self.calib_data_Bs_tf,
                        solver=solver, 
                        run_ransac=self.args.run_ransac)
            rs.Run()



def main(args):
    calibration_object = ROSCameraRobotCalibration(args)
    if (not args.only_calibration):
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
        calibration_object.Calibrate()
    else: 
        calibration_object.Calibrate()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Optional app description')
    parser.add_argument("--camera_in_hand", default=True, nargs='?', type=str2bool, help='is the camera attached'\
                        'to the robots body or to the environment?')
    parser.add_argument("--move_robot_automatically", default=False, nargs='?',  type=str2bool, help='should the EE'\
                        'automatically move for collecting data? In this case, the EE is first' \
                        'manually moved to an initial pose and the script controls EE to predefined'\
                        'relative poses. If false, the EE should be moved manually(white status LED'\
                        'and press enter to collect data')
    parser.add_argument("--only_calibration", default=False, nargs='?',type=str2bool, help='if true, values'\
                        'stored in the data folder are used for calibration if false, data is first collected,'\
                        'stored in /data folder and then calibration  routine is run')  
    parser.add_argument("--robot_base_frame_name",default="panda_link0", nargs='?',  type=str, help='robot base frames name in the /tf tree')    
    parser.add_argument("--ee_frame_name",default="panda_end_effector", nargs='?',  type=str, help='end-effector frames name in the /tf tree')                                               
    parser.add_argument("--rgb_image_topic",default="/camera/color/image_raw", nargs='?',  type=str, help='RGB image raw topic name')                                               
    parser.add_argument("--rgb_camera_info_topic",default="/camera/color/camera_info", nargs='?',  type=str, help='RGB image camera info name')                                               
    parser.add_argument("--run_ransac",default=False, nargs='?',  type=str2bool, help='this option'\
                        'runs ransac to select EEposes and calibration target poses based on how well they all agree to AX=XB')
    parser.add_argument("--debug_image",default=False, nargs='?',  type=str2bool, help='this option'\
                        'draws aruco tag detections and the estimated tag frame on the image and saves in the data/image folder')                                                                   
    args = parser.parse_args()
    main(args)



