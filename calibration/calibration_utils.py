import numpy as np
import numpy.matlib as npm
import cv2
import copy
from autolab_core import RigidTransform



def str2bool(value):
    if isinstance(value, bool):
        return value
    if value.lower() in {'false', 'f', '0', 'no', 'n'}:
        return False
    elif value.lower() in {'true', 't', '1', 'yes', 'y'}:
        return True
    raise ValueError(f'{value} is not a valid boolean value')

    
def point_to_object(obj_loc, distance, pan, yaw, rotate):
    r = R.from_rotvec(np.array([0, -pan, 0]))
    r1 = R.from_rotvec(np.array([0,0, yaw]))
    r2 = R.from_rotvec(np.array([0, pan, 0]))
    r3 = R.from_rotvec(np.array([np.pi, 0 ,0]))
    r4 = R.from_rotvec(np.array([0, 0, rotate]))
    translation = r1.as_matrix() @ r.as_matrix() @ np.array([distance, 0, 0])
    translation += obj_loc
    return translation, r1.as_matrix() @ r2.as_matrix() @ r3.as_matrix() @ r4.as_matrix() 


def write_to_file(line_list, file_name):
    # open file in append mode
    line  = ",".join(line_list)
    with open(file_name, 'a') as f:
        f.write(line)
        f.write('\n') 

# def sample_view_poses(initial_pose):
#     delta_position = [[],[],[],[],[],[],[],[],[],[]]
#     return

def get_delta_poses(file_name, initial_pose_fa):
    with open(file_name, 'r') as fp:
        lines = fp.readlines()
        ee_poses_tf = []
    for line in lines:
        data = line.split('\n')[0].split(',')
        ee_pose = tuple(((float(data[0]),
                            float(data[1]),
                                float(data[2])), 
                        (float(data[3]),
                            float(data[4]),
                            float(data[5]))))
        # tag_pose = tuple(((float(data[6]),
        #                     float(data[7]),
        #                     float(data[8])), 
        #                 (float(data[9]),
        #                     float(data[10]),
        #                     float(data[11]))))
        ee_pose_tf = np.eye(4)
        # tag_pose_tf = np.eye(4)
        ee_pose_tf[0:3, 0:3] = cv2.Rodrigues(ee_pose[0])[0]; ee_pose_tf[0:3, -1] = ee_pose[1]
        ee_pose_tf[0:3, -1] = ee_pose_tf[0:3, -1]/1000.0
        # tag_pose_tf[0:3, 0:3] = cv2.Rodrigues(tag_pose[0])[0]; tag_pose_tf[0:3, -1] = tag_pose[1] 
        ee_poses_tf.append(ee_pose_tf)

    delta_poses = []
    initial_pose = np.eye(4)
    initial_pose[0:3, 0:3] = initial_pose_fa.rotation
    initial_pose[0:3, -1] = initial_pose_fa.translation    
    for abs_ee_pose_tf in ee_poses_tf: 
        # if prev is None: 
        #     delta_pose = RigidTransform(from_frame='franka_tool', 
        #                                 to_frame='franka_tool',
        #                                 rotation=np.eye(3),
        #                                 translation=np.array([0.0, 0.0,0.0]))
        #     prev = abs_ee_pose_tf
        # else: 
            # difference = np.matmul(np.linalg.inv(abs_ee_pose_tf), prev)
            # print(prev)
            # print(abs_ee_pose_tf)
        # difference = np.matmul(np.linalg.inv(abs_ee_pose_tf), initial_pose)
        difference = np.matmul(np.linalg.inv(initial_pose), abs_ee_pose_tf )

        delta_pose = RigidTransform(from_frame='franka_tool', 
                                    to_frame='franka_tool',
                                    rotation=difference[0:3,0:3],
                                    translation=difference[0:3,-1])
        delta_poses.append(delta_pose)
    first_delta_pose = RigidTransform(from_frame='franka_tool', 
                                    to_frame='franka_tool',
                                    rotation=np.eye(3),
                                    translation=np.array([0.0, 0.0, 0.0]))
    delta_poses = [first_delta_pose] + delta_poses
    return delta_poses

def get_absolute_poses(file_name):
    with open(file_name, 'r') as fp:
        lines = fp.readlines()

    ee_poses = []
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
        # tag_pose_tf = np.eye(4)
        ee_pose_tf[0:3, 0:3] = cv2.Rodrigues(ee_pose[0])[0]; ee_pose_tf[0:3, -1] = ee_pose[1]
        # tag_pose_tf[0:3, 0:3] = cv2.Rodrigues(tag_pose[0])[0]; tag_pose_tf[0:3, -1] = tag_pose[1] 
        abs_pose = RigidTransform(from_frame='franka_tool', 
                                        to_frame='world',
                                        # to_frame='franka_tool_base',
                                        rotation=ee_pose_tf[0:3, 0:3],
                                        translation=ee_pose_tf[0:3, -1]/1000.0)
        ee_poses.append(abs_pose)
    return ee_poses
    
# print(ee_rotation)
# rvec = cv2.Rodrigues(ee_rotation)[0]
# print(rvec)
# rotation_matrix = np.zeros(shape=(3,3))
# rotation_matrix = cv2.Rodrigues(rvec)
# print(rotation_matrix[0])        



#########
#  reference: https://github.com/christophhagen/averaging-quaternions/blob/master/averageQuaternions.py
#########


def rotationmatrix_error(R2, R1):
    rc1 = R1[0:3, 0]
    rc2 = R1[0:3, 1]
    rc3 = R1[0:3, 2]
    rd1 = R2[0:3, 0]
    rd2 = R2[0:3, 1]
    rd3 = R2[0:3, 2]    
    error = 0.5 * (np.cross(rc1, rd1) + np.cross(rc2, rd2) + np.cross(rc3, rd3))
    return  error



# Q is a Nx4 np matrix and contains the quaternions to average in the rows.
# The quaternions are arranged as (w,x,y,z), with w being the scalar
# The result will be the average quaternion of the input. Note that the signs
# of the output quaternion can be reversed, since q and -q describe the same orientation
def averageQuaternions(Q):
    # Number of quaternions to average
    M = Q.shape[0]
    A = npm.zeros(shape=(4,4))

    for i in range(0,M):
        q = Q[i,:]
        # multiply q with its transposed version q' and add A
        A = np.outer(q,q) + A

    # scale
    A = (1.0/M)*A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:,0].A1)


# Average multiple quaternions with specific weights
# The weight vector w must be of the same length as the number of rows in the
# quaternion maxtrix Q
def weightedAverageQuaternions(Q, w):
    # Number of quaternions to average
    M = Q.shape[0]
    A = npm.zeros(shape=(4,4))
    weightSum = 0

    for i in range(0,M):
        q = Q[i,:]
        A = w[i] * np.outer(q,q) + A
        weightSum += w[i]

    # scale
    A = (1.0/weightSum) * A

    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)

    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]

    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:,0].A1)

# def AverageTransformations(transforms_list):
#     avg_tvec = np.array([0.0, 0.0, 0.0])
#     avg_rvec = None
#     quaternions = []
#     for transform in transforms_list:
#         print(transform[1])
#         avg_tvec += np.array(transform[1])

#         rotation_matrix = np.zeros(shape=(3,3))
#         cv2.Rodrigues(transform[0], rotation_matrix)
#         transform_matrix = np.eye(4); transform_matrix[0:3, 0:3] = rotation_matrix
#         quaternion = tf_utils.quaternion_from_matrix(transform_matrix)
#         quaternions.append(quaternion)

#     avg_tvec = avg_tvec/len(transforms_list)
#     avg_quat = averageQuaternions(np.array(quaternions))
#     avg_transmatrix = tf_utils.quaternion_matrix(avg_quat)
#     return cv2.Rodrigues(avg_transmatrix[0:3, 0:3])[0], avg_tvec

def refine_corners( image, corners):
    winSize = [5, 5]
    zeroZone = [-1, -1]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 10, 0.001)
    for corner in corners: 
        cv2.cornerSubPix(image, corner, winSize, zeroZone, criteria)

def reprojection_error( all_corners, ids,  rvec, tvec, board, camera_matrix, dist_coeffs): 
    mean_error = 0.0 
    for id_, corners in zip(ids, all_corners):
        #print(id_[0])
        proj_img_point, _ = cv2.projectPoints(board.getObjPoints()[id_[0]], rvec, tvec, camera_matrix, dist_coeffs )
        #print(self.board.getObjPoints()[id_[0]], corners)
        #print(np.shape(self.board.getObjPoints()[id_[0]]), np.shape(corners[0]), np.shape(proj_img_point[:,0,:]))
        # print(corners[0], proj_img_point[:,0,:])
        # print(len(proj_img_point))
        error = cv2.norm(corners[0], proj_img_point[:,0,:], cv2.NORM_L2)/len(proj_img_point)
        mean_error += error
    return mean_error/len(ids)

def reprojection_error_in_robot_base( all_corners, ids,  rvec, tvec, new_points, camera_matrix, dist_coeffs, Tcam2base):
    mean_error = 0.0 
    # Ttag2cam = np.eye(4) ; Ttag2base = np.eye(4) 

    # Ttag2cam[0:3, 0:3] = cv2.Rodrigues(rvec)[0]; Ttag2cam[0:3, -1] = tvec[:,0]

    # Ttag2base= np.matmul(Tcam2base, Ttag2cam)

    Tbase2cam = np.linalg.inv(Tcam2base)#tf_utils.inverse_matrix(Tcam2base)
    rvec_new = cv2.Rodrigues(Tbase2cam[0:3, 0:3])[0]
    tvec_new = Tbase2cam[0:3, -1]

    for id_, corners in zip(ids, all_corners):
        #print(id_[0])
        proj_img_point, _ = cv2.projectPoints(new_points[id_[0]], rvec_new, tvec_new, camera_matrix, dist_coeffs )
        #print(self.board.getObjPoints()[id_[0]], corners)
        #print(np.shape(self.board.getObjPoints()[id_[0]]), np.shape(corners[0]), np.shape(proj_img_point[:,0,:]))
        # print(corners[0], proj_img_point[:,0,:])
        # print(len(proj_img_point))
        error = cv2.norm(corners[0], proj_img_point[:,0,:], cv2.NORM_L2)/len(proj_img_point)
        mean_error += error
    return mean_error/len(ids)

def objPoints_in_robot_base(board, Tcam2base, rvec, tvec):
    Ttag2cam = tf_from_rvectvec(rvec, tvec)
    Ttag2base = np.matmul(Tcam2base, Ttag2cam)
    print("sanity", Ttag2base)
    new_points = copy.copy(board.getObjPoints())
    print("before", new_points[0])
    for square_idx in range(len(new_points)):
        for point_idx in range(len(new_points[square_idx])):
            homogeneous_coordinate = np.array([0.0, 0.0, 0.0, 1.0])
            homogeneous_coordinate[0:3] = new_points[square_idx][point_idx]
            # print("before transform", homogeneous_coordinate)
            new_points[square_idx][point_idx] = np.matmul(Ttag2base, np.transpose(homogeneous_coordinate))[0:3]
            # print("after transform", np.matmul(Tcam2base, np.transpose(homogeneous_coordinate)))
            # print("pt",new_points[square_idx][point_idx])
    print("after", new_points[0]) 
    return new_points         


def tf_from_rvectvec(rvec, tvec):
    out = np.eye(4)
    out[0:3, -1] = tvec[:,0]
    if np.shape(rvec) == (3,1):
        out[0:3, 0:3] = cv2.Rodrigues(rvec)[0]
    else: 
        out[0:3, 0:3] = rvec
    return out

def reprojection_error( all_corners, ids,  rvec, tvec, board, camera_matrix, dist_coeffs): 
    mean_error = 0.0
    singular_mean_error = 0.0
    for id_, corners in zip(ids, all_corners):
        proj_img_point, _ = cv2.projectPoints(board.getObjPoints()[id_[0]], rvec, tvec, camera_matrix, dist_coeffs )
        error = cv2.norm(corners[0], proj_img_point[:,0,:], cv2.NORM_L2)/len(proj_img_point)
        mean_error += error
    return mean_error/len(ids)

def reprojection_error_single_aruco_tag(corners, markerPoints, rvec, tvec, camera_matrix, dist_coeffs, id=0):
    mean_error = 0.0
    for point, corner in zip(markerPoints, corners[id][0]):
        # print('pt', point)
        # print('corner', corner)
        proj_img_point, _ = cv2.projectPoints(point, rvec, tvec, camera_matrix, dist_coeffs )
        # print(proj_img_point, corner)
        error = cv2.norm(corner, proj_img_point[0][0], cv2.NORM_L2)
        mean_error += error
    return mean_error/len(markerPoints)


# def reprojection_error( objPoints, imgPoints,  rvec, tvec, board, camera_matrix, dist_coeffs): 
#     mean_error = 0.0 
#     for objPoint, imgPoint in zip(objPoints, imgPoints):
#         #print(id_[0])
#         # print(objPoint, imgPoint, rvec, tvec, camera_matrix, dist_coeffs)
#         proj_img_point, _ = cv2.projectPoints(objPoint, rvec, tvec, camera_matrix, dist_coeffs )
#         #print(self.board.getObjPoints()[id_[0]], corners)
#         #print(np.shape(self.board.getObjPoints()[id_[0]]), np.shape(corners[0]), np.shape(proj_img_point[:,0,:]))
#         # print(corners[0], proj_img_point[:,0,:])
#         # print(len(proj_img_point))
#         print(imgPoint, proj_img_point[0])
#         error = cv2.norm(imgPoint, proj_img_point[0], cv2.NORM_L2)/len(proj_img_point)
#         mean_error += error
#     return mean_error/len(imgPoints)