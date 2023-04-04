import cv2 
import numpy as np
from calibration_utils import *
from scipy.spatial.transform import Rotation as R
from backend import BACKEND

class BaseRobotCameraCalibration:
    def __init__(self, args,
                    data_file_name:str='data/file.txt',
                    reproj_error_thresh=0.3,
                    aruco_marker_length=0.025,
                    aruco_marker_separation=0.005,
                    aruco_board_n_rows=5,
                    aruco_board_n_cols=7):
        self.args = args
        self.camera_matrix = None
        self.dist_coeffs = None           
        self.calib_data_As = []; self.calib_data_As_tf = []
        self.calib_data_Bs = []; self.calib_data_Bs_tf = []   
        self.REPROJ_THRESH = reproj_error_thresh    
        self.DATA_FILE_NAME = data_file_name   
        self.ARUCO_MARKER_LENGTH = aruco_marker_length
        self.ARUCO_MARKER_SEPARATION = aruco_marker_separation
        self.ARUCO_BOARD_N_COLS = aruco_board_n_cols
        self.ARUCO_BOARD_N_ROWS = aruco_board_n_rows
        self.detections_count = 0
        self.processed_image = 0    
        self.create_aruco_objects()
        if(not self.args.only_calibration):
            open(self.DATA_FILE_NAME, 'w').close()#empty the file in which poses are recorded


    def create_aruco_objects(self):  
        self.aruco_dict = cv2.aruco.getPredefinedDictionary( cv2.aruco.DICT_6X6_1000 )
        self.board = cv2.aruco.GridBoard((self.ARUCO_BOARD_N_ROWS, self.ARUCO_BOARD_N_COLS), 
                                        self.ARUCO_MARKER_LENGTH, 
                                        self.ARUCO_MARKER_SEPARATION, 
                                        self.aruco_dict)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, 
                                                self.arucoParams)

    def read_calibration_data_from_file(self,file_name=None):
        if file_name is None: 
            file_name = self.DATA_FILE_NAME

        with open(file_name, 'r') as fp:
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
            ee_pose_tf[0:3, 0:3] = cv2.Rodrigues(ee_pose[0])[0]
            ee_pose_tf[0:3, -1] = ee_pose[1]
            tag_pose_tf[0:3, 0:3] = cv2.Rodrigues(tag_pose[0])[0]
            tag_pose_tf[0:3, -1] = tag_pose[1]  

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

    def run_robot_camera_calibration(self,):
        self.read_calibration_data_from_file(self.DATA_FILE_NAME)
        assert(len(self.calib_data_As)!=0)
        assert(len(self.calib_data_Bs)!=0)
        # solvers = [cv2.CALIB_HAND_EYE_TSAI,
        #             cv2.CALIB_HAND_EYE_PARK,
        #             cv2.CALIB_HAND_EYE_HORAUD,
        #             cv2.CALIB_HAND_EYE_ANDREFF,
        #             cv2.CALIB_HAND_EYE_DANIILIDIS]
        solver_names = {cv2.CALIB_HAND_EYE_TSAI:"cv2.CALIB_HAND_EYE_TSAI",
                        cv2.CALIB_HAND_EYE_PARK:"cv2.CALIB_HAND_EYE_PARK",
                        cv2.CALIB_HAND_EYE_HORAUD:"cv2.CALIB_HAND_EYE_HORAUD",
                        cv2.CALIB_HAND_EYE_ANDREFF:"cv2.CALIB_HAND_EYE_ANDREFF",
                        cv2.CALIB_HAND_EYE_DANIILIDIS:"cv2.CALIB_HAND_EYE_DANIILIDIS"}
        for solver in solver_names.keys(): 
            print("solver = ", solver)
            backend = BACKEND(As=self.calib_data_As, 
                        As_tf=self.calib_data_As_tf, 
                        Bs=self.calib_data_Bs, 
                        Bs_tf=self.calib_data_Bs_tf,
                        solver=solver, 
                        run_ransac=self.args.run_ransac)
            backend.Run()

    def process_image_msg_for_aruco(self, img_msg, prev_tag_pose=None):
        color_im = self.cv_bridge.imgmsg_to_cv2(img_msg)         
        image_gray = cv2.cvtColor(color_im, cv2.COLOR_BGR2GRAY)
        return self.process_image_for_aruco(image_gray, prev_tag_pose)

    def process_image_for_aruco(self, image_gray, prev_tag_pose=None):      
        corners, ids, rejectedImgPoints = self.detector.detectMarkers(image_gray)  # First, detect markers
        refine_corners(image_gray, corners)
        self.processed_image += 1
        if ids is not None: 
            rvec = None 
            tvec = None
            objPoints= None; imgPoints = None
            objPoints, imgPoints = self.board.matchImagePoints(corners, ids, objPoints, imgPoints)
            # print(objPoints, imgPoints)                    
            retval, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, self.camera_matrix, rvec, tvec)
            if(self.args.debug_image):
                cv2.aruco.drawDetectedMarkers(image_gray, corners, borderColor=(0, 0, 255))
                cv2.drawFrameAxes(image_gray, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                img_file_name = "data/image/image_"+str(self.detections_count-1)+".jpg"
                cv2.imwrite(img_file_name, image_gray)
            reproj_error =  reprojection_error(corners,
                                                ids,
                                                rvec, tvec,
                                                self.board, 
                                                self.camera_matrix, 
                                                self.dist_coeffs)  
        else: 
            return None, None, None, None, None,
   
        print("reprojection error",reproj_error)
        if reproj_error > self.REPROJ_THRESH:
            if(self.detections_count > 0 and prev_tag_pose is not None):
                current = np.eye(4); current[0:3, 0:3] = cv2.Rodrigues(rvec)[0]; current[0:3, -1] = np.squeeze(tvec)
                # difference = np.matmul(np.linalg.inv(current), prev_tag_pose)
                difference = np.matmul(np.linalg.inv(prev_tag_pose), current) 
                diff_r = R.from_matrix(difference[0:3,0:3])
                print("relative calibration tag rotation in degrees, translation in m: ", 
                        diff_r.as_euler('xyz', degrees=True),
                        difference[0:3,-1])                        
            print("#### Very high reprojection error ####")
            print("#### Ignoring this sample ####")          

        return corners, ids, rvec, tvec, reproj_error
