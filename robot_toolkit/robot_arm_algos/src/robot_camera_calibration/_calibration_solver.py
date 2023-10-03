import numpy as np
import cv2
import tf.transformations as tf_utils
from ._calibration_data_utils import read_calibration_data_from_file, tf_from_rvectvec, error_rot_trans, solvers
from ..logger import logger



class CalibrationSolver:
    def __init__(self, calib_data_file_name, flag_camera_in_hand=True):
        self.calib_solver_data = read_calibration_data_from_file(calib_data_file_name, 
                                                            flag_camera_in_hand)
        self.A_rvecs = [self.calib_solver_data.As[i][0] for i in range(len(self.calib_solver_data.As))]
        self.A_tvecs = [self.calib_solver_data.As[i][1] for i in range(len(self.calib_solver_data.As))]
        self.B_rvecs = [self.calib_solver_data.Bs[i][0] for i in range(len(self.calib_solver_data.Bs))]
        self.B_tvecs = [self.calib_solver_data.Bs[i][1] for i in range(len(self.calib_solver_data.Bs))]
        self.solvers = solvers                                                             
        pass

    def run_calibration_solver(self, solver_name):
        if(solver_name not in list(self.solvers.keys())):
            logger.error("Invalid robot camera calibration solver requested")
            return 

        output_rvec, output_tvec = cv2.calibrateHandEye(self.A_rvecs,
                                                        self.A_tvecs,
                                                        self.B_rvecs,
                                                        self.B_tvecs,
                                                        method=self.solvers[solver_name])
        output_tf = tf_from_rvectvec(output_rvec, output_tvec)
        """(rot_error_mean, rot_error_var), (trans_error_mean, trans_error_var)""" 
        # rot_error_var = trans_error_var = 0.0
        (rot_error_mean, rot_error_var) , (trans_error_mean, trans_error_var) = self.compute_calibration_error_fulldataset(output_tf)
        logger.debug(f"Output of calibration for solver: {solver_name} is: {output_tf}")
        logger.debug(f"This yields rotation error (mean, variance) in degrees = ({rot_error_mean},{rot_error_var})")
        logger.debug(f"This yields translation error (mean, variance) in cms - ({trans_error_mean},{trans_error_var})")
        return output_tf
    
    def run_calibration_all_solvers(self,):
        out = []
        for solver_name, _ in self.solvers.items():
            out.append((self.run_calibration_solver(solver_name), solver_name))
        return out
      
    def compute_calibration_error_fulldataset(self, output_tf):
        indices = range(len(self.calib_solver_data.As_tf))
        pairs = [(a, b) for idx, a in enumerate(indices) for b in indices[idx + 1:]]
        rot_errors = []
        trans_errors = []
        for pair in pairs:
            A = np.matmul(np.linalg.inv(self.calib_solver_data.As_tf[pair[0]]),
                        self.calib_solver_data.As_tf[pair[1]])
            AX = np.matmul(A, output_tf)
            B = np.matmul(self.calib_solver_data.Bs_tf[pair[0]],
                        np.linalg.inv(self.calib_solver_data.Bs_tf[pair[1]]))
            XB = np.matmul(output_tf, B)
            rot_error, trans_error = error_rot_trans(AX, XB)
            rot_errors.append(rot_error); trans_errors.append(trans_error)
        return (np.mean(rot_errors, axis = 0), np.var(rot_errors, axis = 0)), \
                (np.mean(trans_errors, axis = 0), np.var(trans_errors, axis = 0))

    # def compute_calibration_error_fulldataset(self, output_tf):
    #     indices = range(len(self.calib_solver_data.As_tf))
    #     pairs = [(a, b) for idx, a in enumerate(indices) for b in indices[idx + 1:]]
    #     rot_error = 0.0 
    #     trans_error = 0.0
    #     for pair in pairs:
    #         A = np.matmul(np.linalg.inv(self.calib_solver_data.As_tf[pair[0]]),
    #                     self.calib_solver_data.As_tf[pair[1]])
    #         AX = np.matmul(A, output_tf)
    #         B = np.matmul(self.calib_solver_data.Bs_tf[pair[0]],
    #                         np.linalg.inv(self.calib_solver_data.Bs_tf[pair[1]]))
    #         XB = np.matmul(output_tf, B)
    #         AXrpy = np.array(tf_utils.euler_from_matrix(AX))
    #         XBrpy = np.array(tf_utils.euler_from_matrix(XB))
    #         rot_error += np.linalg.norm(AXrpy-XBrpy)*(180.00/np.pi)# converts errors to degrees
    #         trans_error += 100.00*np.linalg.norm(AX[:3, 3]-XB[:3, 3])# convert errors to cm

    #     return  trans_error / len(pairs), rot_error / len(pairs)

""" 
todo run ransac
"""