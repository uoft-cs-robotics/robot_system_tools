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
from scipy.spatial.transform import Rotation as R

from calibration_utils import *
from ransac import RANSAC 
from base_robot_camera_calibration import BaseRobotCameraCalibration
from frankapy import FrankaArm

class ROSCameraRobotCalibration(BaseRobotCameraCalibration):
    def __init__(self, args, file_name:str='data/file.txt', movement_pose_file:str='data/file_moves.txt')-> None: 
        super().__init__(args, data_file_name=file_name) 
        self.args = args
        if(self.args.move_robot_automatically):
            self.fa_object = FrankaArm()           
        if(not args.move_robot_automatically):# as frankapy inits its own node
            rospy.init_node('listener', anonymous=True)
        rospy.Subscriber(args.rgb_image_topic, Image, self.subscribe_image_cb)# subscriber for RGB image
        rospy.Subscriber(args.rgb_camera_info_topic, CameraInfo, self.camera_info_cb)#subscriber for RGM image camera info 
        self.img_msg = None
        self.tf_listener_ = TransformListener()
        self.robot_base_frame = args.robot_base_frame_name #"panda_link0"
        self.ee_frame = args.ee_frame_name#"panda_end_effector"  
        self.cv_bridge = CvBridge()
        self.detections_count = 0
        self.processed_image = 0   
        self.bool_kill_thread = False
        if self.args.move_robot_automatically:
            self.abs_poses = get_absolute_poses(movement_pose_file)#gets command poses for frankapy to move robot automatically
        else: 
            self.abs_poses = None

    def subscribe_image_cb(self, img_msg):
        if self.camera_matrix is None: 
            return 
        else: 
            self.img_msg = img_msg

    def camera_info_cb(self, info_msg):
        if(self.camera_matrix is None):
            self.dist_coeffs = np.array(info_msg.D[:4])
            self.camera_matrix = np.array(info_msg.K).reshape(3,3)
        else: 
            return        

    def user_robot_move_function(self,):

        while(self.img_msg is None):#waits here until a new image is received
            print("waiting for image msg")
            time.sleep(1)
            if self.bool_kill_thread: 
                return

        R_gripper2base = []; t_gripper2base = []     
        R_tag2cam = []; t_tag2cam = []  
        prev_tag_pose = None 
        prev_ee_pose = None         
        while(True): 
            if(self.abs_poses is not None and len(self.abs_poses)!=0 ):
                self.fa_object.reset_joints()
                self.fa_object.goto_pose(self.abs_poses.pop(0),ignore_virtual_walls=True, use_impedance=False)
                time.sleep(1.0) 
            elif (self.abs_poses is not None and len(self.abs_poses)==0):
                rospy.core.signal_shutdown('keyboard interrupt')
                break

            if(args.move_robot_automatically):
                ip = ""
            else:
                ip = input("Press Enter to continue collecting current sample....else space bar to stop")

            if (ip==""):
                time.sleep(0.5)      
                # yaml API used in tf python breaks for some reason 
                # assert(self.tf_listener_.frameExists(self.robot_base_frame))
                # assert(self.tf_listener_.frameExists(self.ee_frame))
                t = self.tf_listener_.getLatestCommonTime(self.robot_base_frame, self.ee_frame)
                ee_position, ee_quaternion = self.tf_listener_.lookupTransform(self.robot_base_frame, self.ee_frame,t)
                ee_rotation_matrix = tf_utils.quaternion_matrix(ee_quaternion)
                ee_rotation_matrix = ee_rotation_matrix[0:3, 0:3]# ee_rotation_matrix = ee_rotation_matrix[0:3, 0:3]                
                corners, ids, rvec, tvec, reproj_error = self.process_image_msg_for_aruco(self.img_msg, prev_tag_pose)
                if ids is not None:                    
                    if reproj_error > self.REPROJ_THRESH:
                        continue
                    else:
                        R_gripper2base.append(cv2.Rodrigues(ee_rotation_matrix)[0])
                        t_gripper2base.append(ee_position) 
                        R_tag2cam.append(rvec)
                        t_tag2cam.append(tvec)
                        
                    if (self.detections_count==0):
                        prev_ee_pose = np.eye(4); prev_ee_pose[0:3, 0:3] = ee_rotation_matrix; prev_ee_pose[0:3, -1] = ee_position
                        prev_tag_pose = np.eye(4); prev_tag_pose[0:3, 0:3] = cv2.Rodrigues(rvec)[0]; prev_tag_pose[0:3, -1] = np.squeeze(tvec)
                        self.detections_count +=1  

                    elif (self.detections_count > 0 and reproj_error < self.REPROJ_THRESH): 
                        current_ee = np.eye(4); current_ee[0:3, 0:3] = ee_rotation_matrix; current_ee[0:3, -1] = ee_position
                        current_tag = np.eye(4); current_tag[0:3, 0:3] = cv2.Rodrigues(rvec)[0]; current_tag[0:3, -1] = np.squeeze(tvec)
                        difference = np.matmul(np.linalg.inv(current_tag), prev_tag_pose) 
                        diff_r = R.from_matrix(difference[0:3,0:3])
                        print("relative calibration tag rotation in degrees, translation in m : ",
                                diff_r.as_euler('xyz', degrees=True),
                                difference[0:3,-1])
                        prev_ee_pose = current_ee
                        prev_tag_pose = current_tag
                        self.detections_count +=1  
                        
                else:
                    print("No aruco tag detected in this sample")  
                    self.processed_image +=1                        
                print("accepted pairs of pose, no. of frames processed", self.detections_count, self.processed_image)                           
            else:
                print("stopping data collection")
                if self.processed_image ==0 :
                    rospy.core.signal_shutdown('keyboard interrupt')
                    exit()
                rospy.core.signal_shutdown('keyboard interrupt')
                break    

        for ee_rot, ee_trans, tag_rot, tag_trans in zip(R_gripper2base, t_gripper2base, R_tag2cam, t_tag2cam):
            ee_pose_line = [str(i) for i in [ee_rot[0][0],ee_rot[1][0], ee_rot[2][0], ee_trans[0], ee_trans[1], ee_trans[2]]]
            tag_pose_line = [str(i) for i in [tag_rot[0][0], tag_rot[1][0], tag_rot[2][0], tag_trans[0][0], tag_trans[1][0], tag_trans[2][0] ]]
            line = ee_pose_line + tag_pose_line
            write_to_file(line, self.DATA_FILE_NAME)    
 
def main(args):
    calibration_object = ROSCameraRobotCalibration(args)
    if (not args.only_calibration):
        rate = rospy.Rate(30) # 30hz
        x = threading.Thread(target=calibration_object.user_robot_move_function)
        x.daemon = True
        x.start()
        if not rospy.core.is_initialized():
            raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
        try:
            while not rospy.core.is_shutdown():
                rate.sleep()
        except KeyboardInterrupt:
            print("killing ROS node")
            rospy.core.signal_shutdown('keyboard interrupt') 
        calibration_object.bool_kill_thread = True
        x.join()
        calibration_object.run_robot_camera_calibration()
    else: 
        calibration_object.run_robot_camera_calibration()

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



