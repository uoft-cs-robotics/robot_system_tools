# either eye-in-hand or eye-2-hand 
# either move with frankapy or move with hand and press enter 
# which algorithm to use
# ransac needed?
# 'store' data & calibration or only calibration 

import argparse
import numpy as np
import os
import time
import cv2

##
import zmq
import tf.transformations as tf_utils

#Real Sense libraries#
from perception.realsense_sensor import RealSenseSensor## unnecessary dependancy 
import pyrealsense2 as rs

#FrankaPy#
#from frankapy import FrankaArm
from scipy.spatial.transform import Rotation as R
#from april_tag_pick_place import point_to_object

#Calibration
from CalibrationUtils import *
from RANSAC import RANSAC 

class CameraRobotCalibration: 
    def __init__(self, args, file_name:str='data/file.txt')-> None: 
        self.args = args
        self.create_aruco_objects()
        ctx = rs.context()
        if(not self.args.only_calibration):
            device_id = ctx.devices[0].get_info(rs.camera_info.serial_number)
            self.sensor = RealSenseSensor(device_id, frame="realsense", filter_depth=True)
            # self.sensor.COLOR_IM_HEIGHT = 1080
            # self.sensor.COLOR_IM_WIDTH = 1920
            print("color image size: ", self.sensor.COLOR_IM_HEIGHT, self.sensor.COLOR_IM_WIDTH)
            self.sensor.start()  # need yaml config here 
            intr = self.sensor.color_intrinsics
            print("orig", np.array([[intr._fx, 0.0, intr._cx], [0.0, intr._fy, intr._cy],[0.0,0.0,1.0]]))
            # print("dup", np.array([[908.7789916992188, 0.0, 638.7603149414062], [0.0, 907.6327514648438, 341.215576171875],[0.0,0.0,1.0]]))
            #self.intr_list = [intr._fx, intr._fy, intr._cx, intr._cy]
            self.camera_matrix = np.array([[intr._fx, 0.0, intr._cx], [0.0, intr._fy, intr._cy],[0.0,0.0,1.0]])
            #self.camera_matrix = np.array([[908.7789916992188, 0.0, 638.7603149414062], [0.0, 907.6327514648438, 341.215576171875],[0.0,0.0,1.0]])
            self.dist_coeffs = np.array([0.0,0.0,0.0,0.0])
        self.file_name =  file_name
        # tuple of (rvec,tvec) and 4x4 tf matrices for tag's pose and ee's pose
        self.calib_data_As = []; self.calib_data_As_tf = []
        self.calib_data_Bs = []; self.calib_data_Bs_tf = []

    def get_ee_pose_zmq(self,):
        self.socket.send_string("data")#even an empty message would do
        message = self.socket.recv()
        zmqPose = np.frombuffer(message).astype(np.float32)
        zmqPose = np.reshape(a=zmqPose, newshape=(4,4), order='F')
        zmq_position = np.array(zmqPose[:3,3])
        zmq_rot = np.array(zmqPose[:3,:3])
        return zmq_rot, zmq_position

    def create_aruco_objects(self):
        markerLength = 0.0265
        markerSeparation = 0.0057
        self.aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_6X6_1000 )
        #aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_4X4_1000 )

        self.board = cv2.aruco.GridBoard_create(5, 7, markerLength, markerSeparation, self.aruco_dict)
        #img = cv2.aruco.drawPlanarBoard(board, (3300,3300))# for printing on A4 paper
        #cv2.imwrite('/home/ruthrash/test.jpg', img)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
    
    def refine_corners(self, image, corners):
        winSize = [5, 5]
        zeroZone = [-1, -1]
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 40, 0.001)
        for corner in corners: 
            cv2.cornerSubPix(image, corner, winSize, zeroZone, criteria)

    def reprojection_error(self, all_corners, ids,  rvec, tvec): 
        mean_error = 0.0 
        for id_, corners in zip(ids, all_corners):
            #print(id_[0])
            proj_img_point, _ = cv2.projectPoints(self.board.objPoints[id_[0]], rvec, tvec, self.camera_matrix, self.dist_coeffs )
            #print(self.board.objPoints[id_[0]], corners)
            #print(np.shape(self.board.objPoints[id_[0]]), np.shape(corners[0]), np.shape(proj_img_point[:,0,:]))
            # print(corners[0], proj_img_point[:,0,:])
            # print(len(proj_img_point))
            error = cv2.norm(corners[0], proj_img_point[:,0,:], cv2.NORM_L2)/len(proj_img_point)
            mean_error += error
        return mean_error/len(ids)

    def WaitAndCollectData(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://192.168.0.3:2000")
        #self.fa = FrankaArm()
        R_gripper2base = []; t_gripper2base = []     
        R_tag2cam = []; t_tag2cam = [] 
        counter = []
        all_corners = []
        all_ids = []
        detections_count = 0
        i = 0
        while(True):
            ip = input("Press Enter to continue collecting current sample....else space bar to stop")
            tag_poses = []
            ee_poses = []
            if (ip==""):
                time.sleep(0.5)

                color_im_, depth_im_ = self.sensor.frames()
                color_im = color_im_.raw_data
                image_gray = cv2.cvtColor(color_im, cv2.COLOR_BGR2GRAY)
                corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image_gray, self.aruco_dict, parameters=self.arucoParams)  # First, detect markers
                self.refine_corners(image_gray, corners)
                cv2.aruco.drawDetectedMarkers(color_im, corners, borderColor=(0, 0, 255))

                if ids is not None: 
                    detections_count +=1
                    rvec = None 
                    tvec = None
                    retval, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, self.board, self.camera_matrix, self.dist_coeffs, rvec, tvec)  # posture estimation from a diamond
                    
                    

                    cv2.drawFrameAxes(color_im, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                    file_name = "data/image/image_"+str(detections_count)+".jpg"
                    cv2.imwrite(file_name, color_im)
                    ee_rotation, ee_position  = self.get_ee_pose_zmq() 
                    print("tag", rvec, tvec, retval )
                    print("ee",cv2.Rodrigues(ee_rotation)[0], ee_position  )
                    reproj_error =  self.reprojection_error(corners,ids, rvec, tvec)
                    print("reprojection error",reproj_error)
                    if self.args.camera_in_hand:
                        thresh = 0.3
                    else: 
                        thresh = 0.3
                    if reproj_error >  thresh:
                        print("#### Very high reprojection error ####")
                        continue
                        
                    else:
                        R_gripper2base.append(cv2.Rodrigues(ee_rotation)[0])
                        t_gripper2base.append(ee_position) 
                        R_tag2cam.append(rvec)
                        t_tag2cam.append(tvec)
                    
                print(detections_count, i)
                cv2.aruco.drawDetectedMarkers(color_im, corners, borderColor=(0, 0, 255))
            else:
                print("stopping data collection")
                if i ==0 :
                    exit()
                break    
            i += 1    
        for ee_rot, ee_trans, tag_rot, tag_trans in zip(R_gripper2base, t_gripper2base, R_tag2cam, t_tag2cam):
            ee_pose_line = [str(i) for i in [ee_rot[0][0],ee_rot[1][0], ee_rot[2][0], ee_trans[0], ee_trans[1], ee_trans[2]]]
            tag_pose_line = [str(i) for i in [tag_rot[0][0], tag_rot[1][0], tag_rot[2][0], tag_trans[0][0], tag_trans[1][0], tag_trans[2][0] ]]
            line = ee_pose_line + tag_pose_line
            write_to_file(line, self.file_name)     
                   
    def Calibrate(self,):
        with open(self.file_name, 'r') as fp:
            lines = fp.readlines()
        i = 0 
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

            # self.calib_data.append(tuple((ee_pose, tag_pose)))
            # self.calib_data_tf.append(tuple((ee_pose_tf, tag_pose_tf)))
            ee_pose_tf[0:3, 0:3] = cv2.Rodrigues(ee_pose[0])[0]; ee_pose_tf[0:3, -1] = ee_pose[1]
            tag_pose_tf[0:3, 0:3] = cv2.Rodrigues(tag_pose[0])[0]; tag_pose_tf[0:3, -1] = tag_pose[1]  

            if(self.args.camera_in_hand):          
                self.calib_data_As.append(ee_pose)
                self.calib_data_As_tf.append(ee_pose_tf)
                self.calib_data_Bs.append(tag_pose)
                self.calib_data_Bs_tf.append(tag_pose_tf)     
            else:
                ee_pose_tf_inv = tf_utils.inverse_matrix(ee_pose_tf)
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
                        solver=solver)
            rs.Run()
        # best_x, best_error= rs.Run()
        # print(best_x[0:3, 0:3], best_x[0:3, -1] )

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
                calib.MoveAndCollectData()
            else:
                print('move the end effector to initial configuration and restart'\
                    'the script')
                exit()
        else: 
            calib.WaitAndCollectData()
            calib.Calibrate()
    else:
        calib.Calibrate()
                 

    pass 

if __name__ == "__main__":
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
    parser.add_argument("--create_calibration_target",default=False, nargs='?',  type=str2bool, help='this options'\
                        'only creates a calibration target and stores the image in the data folder')                        
    args = parser.parse_args()
    main(args)

