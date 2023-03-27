import argparse
import numpy as np
import os
import time
import cv2

##
import zmq

#Real Sense libraries#
from perception.realsense_sensor import RealSenseSensor## unnecessary dependancy 
import pyrealsense2 as rs
#FrankaPy#
#from frankapy import FrankaArm
from scipy.spatial.transform import Rotation as R

#Calibration
from calibration_utils import *
from base_robot_camera_calibration import BaseRobotCameraCalibration

class CameraRobotCalibration(BaseRobotCameraCalibration): 
    def __init__(self, args,
                reproj_error_thresh=0.3,
                aruco_marker_length=0.025,
                aruco_marker_separation=0.005,
                aruco_board_n_rows=5,
                aruco_board_n_cols=7,
                file_name:str='data/file.txt')-> None: 
        super().__init__(args, 
                        reproj_error_thresh=reproj_error_thresh,
                        aruco_marker_length=aruco_marker_length,
                        aruco_marker_separation=aruco_marker_separation,
                        aruco_board_n_rows=aruco_board_n_rows,
                        aruco_board_n_cols=aruco_board_n_cols,        
                        data_file_name=file_name)         
        self.args = args
        ctx = rs.context()
        if(not self.args.only_calibration):
            device_id = ctx.devices[0].get_info(rs.camera_info.serial_number)
            self.sensor = RealSenseSensor(device_id, frame="realsense", filter_depth=True)
            self.sensor.start()  # need yaml config here 
            intr = self.sensor.color_intrinsics
            self.camera_matrix = np.array([[intr._fx, 0.0, intr._cx], [0.0, intr._fy, intr._cy],[0.0,0.0,1.0]])
            self.dist_coeffs = np.array([0.0,0.0,0.0,0.0])

        self.zmq_port = args.zmq_server_port
        self.zmq_ip = args.zmq_server_ip

    def get_ee_pose_zmq(self,):
        self.socket.send_string("data")#even an empty message would do
        message = self.socket.recv()# receives EE pose 
        zmqPose = np.frombuffer(message).astype(np.float32)
        zmqPose = np.reshape(a=zmqPose, newshape=(4,4), order='F')
        zmq_position = np.array(zmqPose[:3,3])
        zmq_rot = np.array(zmqPose[:3,:3])
        return zmq_rot, zmq_position

    def collect_data(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        socket_address = "tcp://" + self.zmq_ip + ":" + self.zmq_port 
        self.socket.connect(socket_address)

        R_gripper2base = []; t_gripper2base = []     
        R_tag2cam = []; t_tag2cam = [] 
        counter = []
        all_corners = []
        all_ids = []

        prev_tag_pose = None 
        while(True):
            ip = input("Press Enter to continue collecting current sample....else space bar to stop")
            tag_poses = []
            ee_poses = []
            if (ip==""):
                time.sleep(0.5)
                ee_rotation_matrix, ee_position  = self.get_ee_pose_zmq() 
                color_im_, depth_im_ = self.sensor.frames()
                color_im = color_im_.raw_data
                image_gray = cv2.cvtColor(color_im, cv2.COLOR_BGR2GRAY)
                corners, ids, rvec, tvec, reproj_error = self.process_image_for_aruco(image_gray, prev_tag_pose)
                if ids is not None:                    
                    if reproj_error > self.REPROJ_THRESH:
                        continue                
                    else:
                        R_gripper2base.append(cv2.Rodrigues(ee_rotation_matrix)[0])
                        t_gripper2base.append(ee_position) 
                        R_tag2cam.append(rvec)
                        t_tag2cam.append(tvec)

                    if (self.detections_count==0):
                        prev_tag_pose = np.eye(4); prev_tag_pose[0:3, 0:3] = cv2.Rodrigues(rvec)[0]; prev_tag_pose[0:3, -1] = np.squeeze(tvec)
                        self.detections_count +=1  
                    elif (self.detections_count > 0 and reproj_error < self.REPROJ_THRESH): 
                        current_tag = np.eye(4); current_tag[0:3, 0:3] = cv2.Rodrigues(rvec)[0]; current_tag[0:3, -1] = np.squeeze(tvec)
                        difference_tag = np.matmul(np.linalg.inv(current_tag), prev_tag_pose) 
                        diff_r = R.from_matrix(difference_tag[0:3,0:3])
                        print("relative calibration tag rotation in degrees, translation in m : ",
                                diff_r.as_euler('xyz', degrees=True),
                                difference_tag[0:3,-1])
                        prev_tag_pose = current_tag
                        self.detections_count +=1    
                else:
                    print("No aruco tag detected in this sample")  
                    self.processed_image +=1     

                print("accepted pairs of pose, no. of frames processed", self.detections_count, self.processed_image)                                                                           
            else:
                print("stopping data collection")
                if self.detections_count==0 :
                    exit()
                break    
            self.processed_image += 1    
        for ee_rot, ee_trans, tag_rot, tag_trans in zip(R_gripper2base, t_gripper2base, R_tag2cam, t_tag2cam):
            ee_pose_line = [str(i) for i in [ee_rot[0][0],ee_rot[1][0], ee_rot[2][0], ee_trans[0], ee_trans[1], ee_trans[2]]]
            tag_pose_line = [str(i) for i in [tag_rot[0][0], tag_rot[1][0], tag_rot[2][0], tag_trans[0][0], tag_trans[1][0], tag_trans[2][0] ]]
            line = ee_pose_line + tag_pose_line
            write_to_file(line, self.DATA_FILE_NAME)     
                   
def main(args):
    calib = CameraRobotCalibration(args)
    #fa.reset_joints()
    if (not args.only_calibration):
        if(args.move_robot_automatically):
            print('THE ROBOT IS GOING TO MOVE TO POSES RELATIVE TO INITIAL'\
                'CONFIGURATION TO COLLECT DATA!')
            ip = input('have you configured the EE and camera to a desired'\
                'initial configuration? if yes, press Enter')
            if (ip==""):
                calib.collect_data()
            else:
                print('move the end effector to initial configuration and restart'\
                    'the script')
                exit()
        else: 
            calib.collect_data()
            calib.run_robot_camera_calibration()
    else:
        calib.run_robot_camera_calibration()
                 

    pass 

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Optional app description')
    parser.add_argument("--camera_in_hand", default=True, nargs='?', type=str2bool, help='is the camera attached '\
                        'to the robots body or to the environment?')
    parser.add_argument("--move_robot_automatically", default=False, nargs='?',  type=str2bool, help='should the EE '\
                        'automatically move for collecting data? In this case, the EE is first ' \
                        'manually moved to an initial pose and the script controls EE to predefined '\
                        'relative poses. If false, the EE should be moved manually(white status LED) '\
                        'and press enter to collect data')
    parser.add_argument("--only_calibration", default=False, nargs='?',type=str2bool, help='if true, values '\
                        'stored in the data folder are used for calibration if false, data is first collected, '\
                        'stored in /data folder and then calibration  routine is run')                        
    # parser.add_argument("--create_calibration_target",default=False, nargs='?',  type=str2bool, help='this options'\
    #                     'only creates a calibration target and stores the image in the data folder')         
    parser.add_argument("--run_ransac",default=False, nargs='?',  type=str2bool, help='this option '\
                        'runs ransac to select EEposes and calibration target poses based on how well they all agree to AX=XB ')                                          
    parser.add_argument("--zmq_server_ip",default='192.168.0.3', nargs='?',  type=str, help='ip address '\
                        'of the zmq server running on the realtime PC to send robot hand poses ')  
    parser.add_argument("--zmq_server_port",default='2000', nargs='?',  type=str, help='port ' \
                        'of the zmq server running on the realtime PC to send robot hand poses ' \
                        'the port number is an arbitrary number less than 65535 and ensure it is ' \
                        'not used by any other application in your computer')  
    parser.add_argument("--debug_image",default=True, nargs='?',  type=str2bool, help='this option '\
                        'draws aruco tag detections and the estimated tag frame on the image and saves in the data/image folder')                                                                        
                                                                      

    args = parser.parse_args()
    main(args)

