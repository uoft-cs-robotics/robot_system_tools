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


# print(ee_rotation)
# rvec = cv2.Rodrigues(ee_rotation)[0]
# print(rvec)
# rotation_matrix = np.zeros(shape=(3,3))
# rotation_matrix = cv2.Rodrigues(rvec)
# print(rotation_matrix[0])        


import numpy as np
import numpy.matlib as npm
import cv2
import tf.transformations as tf_utils
#########
#  reference: https://github.com/christophhagen/averaging-quaternions/blob/master/averageQuaternions.py
#########


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

def AverageTransformations(transforms_list):
    avg_tvec = np.array([0.0, 0.0, 0.0])
    avg_rvec = None
    quaternions = []
    for transform in transforms_list:
        print(transform[1])
        avg_tvec += np.array(transform[1])

        rotation_matrix = np.zeros(shape=(3,3))
        cv2.Rodrigues(transform[0], rotation_matrix)
        transform_matrix = np.eye(4); transform_matrix[0:3, 0:3] = rotation_matrix
        quaternion = tf_utils.quaternion_from_matrix(transform_matrix)
        quaternions.append(quaternion)

    avg_tvec = avg_tvec/len(transforms_list)
    avg_quat = averageQuaternions(np.array(quaternions))
    avg_transmatrix = tf_utils.quaternion_matrix(avg_quat)
    return cv2.Rodrigues(avg_transmatrix[0:3, 0:3])[0], avg_tvec


