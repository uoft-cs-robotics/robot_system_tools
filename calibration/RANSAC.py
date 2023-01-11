import random
import numpy as np
#import baldor as br
import tf.transformations as tf_utils
random.seed(10)
import cv2

from CalibrationUtils import *

class RANSAC:
    def __init__(self,As, As_tf, Bs, Bs_tf, solver=cv2.CALIB_HAND_EYE_TSAI,  min_pts=4, iterations=5000, thresh=1.0, run_ransac=False) -> None:
        self.As = As
        self.As_tf = As_tf
        self.Bs = Bs
        self.Bs_tf = Bs_tf

        self.iterations = iterations 
        self.thresh = thresh 
        self.min_pts = min_pts
        # self.req_agreeable_pts = req_agreeable_pts
        self.inliers_idxs = list()
        self.besterr = (60000,60000)
        self.bestlen = 0
        self.solver = solver
        self.run_ransac = run_ransac
        pass

    def compute_estimation_error_fulldataset(self, X_hat):
        indices = range(len(self.As_tf))
        pairs = [(a, b) for idx, a in enumerate(indices) for b in indices[idx + 1:]]
        rot_error = 0.0 
        trans_error = 0.0
        for pair in pairs:
            # print(pair, pair[0], pair[1])
            A = np.matmul(tf_utils.inverse_matrix(self.As_tf[pair[0]]), self.As_tf[pair[1]])
            AX = np.matmul(A, X_hat)
            B = np.matmul(self.Bs_tf[pair[0]], tf_utils.inverse_matrix(self.Bs_tf[pair[1]]))
            XB = np.matmul(X_hat, B)
            AXrpy = np.array(tf_utils.euler_from_matrix(AX))
            XBrpy = np.array(tf_utils.euler_from_matrix(XB))
            rot_error += np.linalg.norm(AXrpy-XBrpy)*(180.00/np.pi)# converts errors to degrees
            trans_error += 100.00*np.linalg.norm(AX[:3, 3]-XB[:3, 3])# convert errors to cm
            # print("##########")
            # print(B[0:3, -1], np.array(tf_utils.euler_from_matrix(B[0:3, 0:3]))*(180.00/np.pi))
            # print(100.00*np.linalg.norm(AX[:3, 3]-XB[:3, 3]), np.linalg.norm(AXrpy-XBrpy)*(180.00/np.pi))
            # print("##########")
        return  (trans_error/len(pairs)), (rot_error/len(pairs))

    def compute_estimation_error_list(self, X_hat, idx,  maybe_inliers_idxs):
        rot_error = 0.0 
        trans_error = 0.0        
        for maybe_inliers_idx in maybe_inliers_idxs:
            assert(idx != maybe_inliers_idx)
            A = np.matmul(tf_utils.inverse_matrix(self.As_tf[idx]), self.As_tf[maybe_inliers_idx])
            AX = np.matmul(A, X_hat)
            B = np.matmul(self.Bs_tf[idx], tf_utils.inverse_matrix(self.Bs_tf[maybe_inliers_idx]))
            XB = np.matmul(X_hat, B)
            AXrpy = np.array(tf_utils.euler_from_matrix(AX))
            XBrpy = np.array(tf_utils.euler_from_matrix(XB))
            rot_error += np.linalg.norm(AXrpy-XBrpy)*(180.00/np.pi)# converts errors to degrees
            trans_error += 100.00*np.linalg.norm(AX[:3, 3]-XB[:3, 3])# convert errors to cm
        # print(trans_error/len(maybe_inliers_idxs), rot_error/len(maybe_inliers_idxs))
        return  trans_error/len(maybe_inliers_idxs), rot_error/len(maybe_inliers_idxs)

    def compute_estimation_error_inliers(self, X_hat, maybe_inliers_idxs):
        pairs = [(a, b) for idx, a in enumerate(maybe_inliers_idxs) for b in maybe_inliers_idxs[idx + 1:]]
        rot_error = 0.0 
        trans_error = 0.0
        for pair in pairs:
            A = np.matmul(tf_utils.inverse_matrix(self.As_tf[pair[0]]), self.As_tf[pair[1]])
            AX = np.matmul(A, X_hat)
            B = np.matmul(self.Bs_tf[pair[0]], tf_utils.inverse_matrix(self.Bs_tf[pair[1]]))
            XB = np.matmul(X_hat, B)
            AXrpy = np.array(tf_utils.euler_from_matrix(AX))
            XBrpy = np.array(tf_utils.euler_from_matrix(XB))
            rot_error += np.linalg.norm(AXrpy-XBrpy)*(180.00/np.pi)# converts errors to degrees
            trans_error += 100.00*np.linalg.norm(AX[:3, 3]-XB[:3, 3])# convert errors to cm
        # print(trans_error/len(pairs), rot_error/len(pairs))
        return  trans_error/len(pairs), rot_error/len(pairs)

    def Run(self,):
        A_rot = [self.As[i][0] for i in range(len(self.As))]
        A_trans = [self.As[i][1] for i in range(len(self.As))]
        B_rot = [self.Bs[i][0] for i in range(len(self.Bs))]
        B_trans = [self.Bs[i][1] for i in range(len(self.Bs))]    
   
        rot, trans = cv2.calibrateHandEye(A_rot, A_trans, B_rot, B_trans,method=self.solver)
        X_full = tf_from_rvectvec(rot, trans)
        print("full dataset error", self.compute_estimation_error_fulldataset(X_full))
        print(X_full)
        print(np.array(X_full[0:3, 0:3]))
        print(np.array(X_full[0:3, -1]))

        if(self.run_ransac):
            pass
        else:
            return
            
        for i in range(self.iterations):
            if(i %500 ==0):
                print(i)
            inliers = []
            index_list = range(len(self.As_tf))
            maybe_inliers_idxs = random.sample(index_list, k=int(len(self.As_tf)/2))
            #maybe_inliers_idxs = random.sample(index_list, k=self.min_pts)

            A_rot = [self.As[i][0] for i in maybe_inliers_idxs]
            A_trans = [self.As[i][1] for i in maybe_inliers_idxs]
            B_rot = [self.Bs[i][0] for i in maybe_inliers_idxs]
            B_trans = [self.Bs[i][1] for i in maybe_inliers_idxs]

            r_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(A_rot, A_trans, B_rot, B_trans, method=self.solver)          
            X = tf_from_rvectvec(r_cam2gripper, t_cam2gripper)  

            for idx in index_list:
                if idx not in maybe_inliers_idxs:
                    t_error, r_error = self.compute_estimation_error_list(X, idx, maybe_inliers_idxs) 
                    this_error = (t_error+r_error)/2.0                 
                    if (this_error) < self.thresh:
                        maybe_inliers_idxs.append(idx)
            if len(maybe_inliers_idxs) > 5:
                inlierAs_rot = [self.As[i][0] for i in maybe_inliers_idxs]
                inlierAs_trans = [self.As[i][1] for i in maybe_inliers_idxs]

                inlierBs_rot = [self.Bs[i][0] for i in maybe_inliers_idxs]
                inlierBs_trans = [self.Bs[i][1] for i in maybe_inliers_idxs]

                r_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(inlierAs_rot, inlierAs_trans, inlierBs_rot, inlierBs_trans, method=self.solver)  
                better_X = tf_from_rvectvec(r_cam2gripper, t_cam2gripper)  
                t_error, rot_err = self.compute_estimation_error_inliers(better_X, maybe_inliers_idxs)     
                thiserror = (t_error, r_error)
                if (thiserror[0]+thiserror[1])/2.0 < (self.besterr[0]+self.besterr[1])/2.0  :
                    best_X = better_X
                    self.besterr = thiserror
                    self.bestlen = len(maybe_inliers_idxs)
        print(thiserror)                    
        print('best samples', self.bestlen)
        print('best error', self.besterr)
        print(np.array(best_X[0:3, 0:3]))
        print(np.array(best_X[0:3,-1]))
        print(best_X)
        inverse_matrix = tf_utils.inverse_matrix(best_X)
        print(tf_utils.quaternion_from_matrix(inverse_matrix))
        print(np.array(inverse_matrix[0:3, -1]))        
        return best_X, self.besterr




