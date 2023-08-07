import numpy as np
from ._calibration_solver import CalibrationSolver
from ..camera.camera import RGBCamera
from .data_collector._data_collector import RobotCameraCalibrationDataCollector
from ._calibration_data_utils import write_calibration_data_file
from ..logger import logger
from ..config_reader import write_dict_to_yaml
"""
collects pose data for robot camera extrinsics calibration 
and outputs results from calibration problem solver
"""
class RobotCameraCalibrator(RobotCameraCalibrationDataCollector, CalibrationSolver):
    def __init__(self, robot_pose_collector, 
                tag_pose_collector,
                camera,
                n_data,
                calib_data_file_name,
                output_file,
                move_robot_automatically,                
                robot_arm_object = None,
                fg_data_folder_path = None,
                flag_camera_in_hand = True,
                collect_data_and_calibrate = True):
        RobotCameraCalibrationDataCollector.__init__(self, robot_pose_collector,
                                                    tag_pose_collector,
                                                    camera,
                                                    n_data,
                                                    robot_arm_object = robot_arm_object,
                                                    move_robot_automatically = move_robot_automatically,
                                                    fg_data_folder_path = fg_data_folder_path)     
        if(collect_data_and_calibrate):
            self.collect_data()
            try:
                assert(len(self.calib_data.rvecs_ee2base) >= 3)
            except:
                logger.error(f"You need more than 3 data points, right now you only have {len(self.calib_data.rvecs_ee2base)} ")
                return
            write_calibration_data_file(self.calib_data, calib_data_file_name)
        
        CalibrationSolver.__init__(self, calib_data_file_name, flag_camera_in_hand) 
        self.output_file = output_file
    
    def run_calibration(self, solver = None):
        if solver is None:
            results = self.run_calibration_all_solvers()
            file_output = dict()
            for result in results:
                file_output[result[1]] =  result[0].tolist()
            write_dict_to_yaml(file_output, self.output_file)
            return 
        
        self.run_calibration_solver(solver)


    








            

            







