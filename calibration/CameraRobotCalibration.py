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
#import tf.transformations as tf_utils

#Real Sense libraries#
from perception.realsense_sensor import RealSenseSensor
import pyrealsense2 as rs

#FrankaPy#
from frankapy import FrankaArm
from scipy.spatial.transform import Rotation as R
#from april_tag_pick_place import point_to_object

def point_to_object(obj_loc, distance, pan, yaw, rotate):
    r = R.from_rotvec(np.array([0, -pan, 0]))
    r1 = R.from_rotvec(np.array([0,0, yaw]))
    r2 = R.from_rotvec(np.array([0, pan, 0]))
    r3 = R.from_rotvec(np.array([np.pi, 0 ,0]))
    r4 = R.from_rotvec(np.array([0, 0, rotate]))
    translation = r1.as_matrix() @ r.as_matrix() @ np.array([distance, 0, 0])
    translation += obj_loc
    return translation, r1.as_matrix() @ r2.as_matrix() @ r3.as_matrix() @ r4.as_matrix() 


class CameraRobotCalibration: 
    def __init__(self, args)-> None: 
        self.args = args
        self.create_aruco_objects()
        ctx = rs.context()
        #device_id = ctx.devices[cfg['rs']['id']].get_info(rs.camera_info.serial_number)
        #self.sensor = RealSenseSensor(device_id, frame=cfg['rs']['frame'], filter_depth=cfg['rs']['filter_depth'])
        device_id = ctx.devices[0].get_info(rs.camera_info.serial_number)
        self.sensor = RealSenseSensor(device_id, frame="realsense", filter_depth=True)

        self.sensor.start()  # need yaml config here 
        intr = self.sensor.color_intrinsics
        #self.intr_list = [intr._fx, intr._fy, intr._cx, intr._cy]
        self.camera_matrix = np.array([[intr._fx, 0.0, intr._cx], [0.0, intr._fy, intr._cy],[0.0,0.0,1.0]])
        self.dist_coeffs =np.array([0.0,0.0,0.0,0.0])

    def create_aruco_objects(self):
        markerLength = 0.03
        markerSeparation = 0.006
        self.aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_6X6_1000 )
        #aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_4X4_1000 )

        self.board = cv2.aruco.GridBoard_create(5, 7, markerLength, markerSeparation, self.aruco_dict)
        #img = cv2.aruco.drawPlanarBoard(board, (2550,3300)) for printing on A4 paper
        #cv2.imwrite('/home/ruthrash/test.jpg', img)

        self.arucoParams = cv2.aruco.DetectorParameters_create()

    def WaitAndCollectData(self):
        # self.context = zmq.Context()
        # self.socket = self.context.socket(zmq.REQ)
        # self.socket.connect("tcp://192.168.0.3:2000")
        self.fa = FrankaArm()
        R_gripper2base = []
        t_gripper2base = []
        R_base2gripper = []
        t_base2gripper = []        
        counter = []
        all_corners = []
        all_ids = []
        detections_count = 0
        i = 0
        while(True):
            ip = input("Press Enter to continue collecting current sample....else space bar to stop")
            if (ip==""):
                time.sleep(2.0)
                color_im_, depth_im_ = self.sensor.frames()
                color_im = color_im_.raw_data
                image_gray = cv2.cvtColor(color_im, cv2.COLOR_BGR2GRAY)
                corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image_gray, self.aruco_dict, parameters=self.arucoParams)  # First, detect markers
                if ids is not None: 
                    detections_count +=1
                    all_corners+= list(corners)
                    all_ids+=list(ids)
                    counter.append(len(ids))
                    T_ready_world = self.fa.get_pose() 
                    print((T_ready_world.rotation, T_ready_world.translation))
                    if(self.args.camera_in_hand):
                        R_gripper2base.append(T_ready_world.rotation)
                        t_gripper2base.append(T_ready_world.translation)
                    else:
                        R_b2g = np.transpose(T_ready_world.rotation)
                        t_b2g = -R_b2g @ T_ready_world.translation
                        R_base2gripper.append(R_b2g)
                        t_base2gripper.append(t_b2g)
                    # ee_rotation, ee_position   = self.getpose_zmq() 
                    # print(ee_rotation, ee_position)
                    # R_gripper2base.append(ee_rotation)
                    # t_gripper2base.append(ee_position) 
                print(detections_count, i)
            else:
                print("stopping data collection")
                if i ==0 :
                    exit()
                break    
            i += 1
        print("running calibration")
        if(self.args.camera_in_hand):#camera in hand
            retval, cameraMatrix, distCoeffs, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors =cv2.aruco.calibrateCameraArucoExtended(all_corners,
                                                                                                                np.array(all_ids),
                                                                                                                np.array(counter),
                                                                                                                self.board,
                                                                                                                image_gray.shape,
                                                                                                                self.camera_matrix, 
                                                                                                                self.dist_coeffs,
                                                                                                                flags=cv2.CALIB_USE_INTRINSIC_GUESS)

            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(R_gripper2base, t_gripper2base, rvecs, tvecs)
            print(t_gripper2base)
            print(tvecs)
            print(R_cam2gripper)
            print(t_cam2gripper)  
        else:# camera in environment
            retval, cameraMatrix, distCoeffs, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors =cv2.aruco.calibrateCameraArucoExtended(all_corners,
                                                                                                                np.array(all_ids),
                                                                                                                np.array(counter),
                                                                                                                self.board,
                                                                                                                image_gray.shape,
                                                                                                                self.camera_matrix, 
                                                                                                                self.dist_coeffs,
                                                                                                                flags=cv2.CALIB_USE_INTRINSIC_GUESS)

            R_cam2base, t_cam2base = cv2.calibrateHandEye(R_base2gripper, t_base2gripper, rvecs, tvecs)
            print(t_base2gripper)
            print(tvecs)
            print(R_cam2base)
            print(t_cam2base)              


    # def getpose_zmq(self,):
    #     self.socket.send_string("data")#even an empty message would do
    #     message = self.socket.recv()
    #     zmqPose = np.frombuffer(message).astype(np.float32)
    #     zmqPose = np.reshape(a=zmqPose, newshape=(4,4), order='F')
    #     print(zmqPose)
    #     zmq_position = np.array(zmqPose[:3,3])
    #     zmq_rot = np.array(zmqPose[:3,:3])
    #     return zmq_rot, zmq_position


    def MoveAndCollectData(self):
        self.fa = FrankaArm()
        pan = np.pi/4.0
        location = np.array([0.7, 0 , 0])
        R_gripper2base = []
        t_gripper2base = []
        R_base2gripper = []
        t_base2gripper = []          
        counter = []
        all_corners = []
        all_ids = []
        detections_count = 0
        self.fa.reset_joints()
        EE_initial_pose = self.fa.get_pose() 
        self.fa.reset_joints()
        for i in range(10):
            distance = 0.3 + 0.1 * i // 3.0
            yaw = (0.7 + 0.07 * i) * np.pi
            EE_pose = self.fa.get_pose() 
            EE_pose.translation, EE_pose.rotation = point_to_object(location,
                                                                    distance,
                                                                    pan,
                                                                    yaw,
                                                                    0)
            self.fa.goto_pose(EE_pose)
            time.sleep(4.0)
            color_im_, depth_im_ = self.sensor.frames()
            color_im = color_im_.raw_data
            image_gray = cv2.cvtColor(color_im, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image_gray, self.aruco_dict, parameters=self.arucoParams)  # First, detect markers
            if ids is not None: 
                detections_count +=1
                all_corners+= list(corners)
                all_ids+=list(ids)
                counter.append(len(ids))
                T_ready_world = self.fa.get_pose() 
                print((T_ready_world.rotation, T_ready_world.translation))
                if(self.args.camera_in_hand):
                    R_gripper2base.append(T_ready_world.rotation)
                    t_gripper2base.append(T_ready_world.translation)
                else:
                    R_b2g = np.transpose(T_ready_world.rotation)
                    t_b2g = -R_b2g @ T_ready_world.translation
                    R_base2gripper.append(R_b2g)
                    t_base2gripper.append(t_b2g)                    
                # ee_rotation, ee_position   = self.getpose_zmq() 
                # print(ee_rotation, ee_position)
                # R_gripper2base.append(ee_rotation)
                # t_gripper2base.append(ee_position)                
            print(detections_count, i)
            if i == 4: 
                #self.fa.goto_pose(EE_initial_pose)
                self.fa.reset_joints()
        #self.fa.goto_pose(EE_initial_pose)
        self.fa.reset_joints()
        
        print("running calibration")
        if(self.args.camera_in_hand):#camera in hand
            retval, cameraMatrix, distCoeffs, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors =cv2.aruco.calibrateCameraArucoExtended(all_corners,
                                                                                                                np.array(all_ids),
                                                                                                                np.array(counter),
                                                                                                                self.board,
                                                                                                                image_gray.shape,
                                                                                                                self.camera_matrix, 
                                                                                                                self.dist_coeffs,
                                                                                                                flags=cv2.CALIB_USE_INTRINSIC_GUESS)

            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(R_gripper2base, t_gripper2base, rvecs, tvecs)
            print(t_gripper2base)
            print(tvecs)
            print(R_cam2gripper)
            print(t_cam2gripper)  
        else:# camera in environment
            retval, cameraMatrix, distCoeffs, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors =cv2.aruco.calibrateCameraArucoExtended(all_corners,
                                                                                                                np.array(all_ids),
                                                                                                                np.array(counter),
                                                                                                                self.board,
                                                                                                                image_gray.shape,
                                                                                                                self.camera_matrix, 
                                                                                                                self.dist_coeffs,
                                                                                                                flags=cv2.CALIB_USE_INTRINSIC_GUESS)

            R_cam2base, t_cam2base = cv2.calibrateHandEye(R_base2gripper, t_base2gripper, rvecs, tvecs)
            print(t_base2gripper)
            print(tvecs)
            print(R_cam2base)
            print(t_cam2base)         
        

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
                 

    pass 

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Optional app description')
    parser.add_argument("--camera_in_hand", action='store',  type=bool, nargs='?', default=True, help='is the camera attached'\
                        'to the robots body or to the environment?')
    parser.add_argument("--move_robot_automatically", action='store', type=bool, nargs='?', default=False, help='should the EE'\
                        'automatically move for collecting data? In this case, the EE is first' \
                        'manually moved to an initial pose and the script controls EE to predefined'\
                        'relative poses. If false, the EE should be moved manually(white status LED'\
                        'and press enter to collect data')
    parser.add_argument("--only_calibration", action='store', type=bool, nargs='?', default=False, help='if true, values'\
                        'stored in the data folder are used for calibration if false, data is first collected,'\
                        'stored in /data folder and then calibration  routine is run')                        
    parser.add_argument("--create_calibration_target", action='store', type=bool, nargs='?', default=False, help='this options'\
                        'only creates a calibration target and stores the image in the data folder')                        
    args = parser.parse_args()
    main(args)

