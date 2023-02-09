import matplotlib.pyplot as plt
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import math
from scipy.spatial.transform import Rotation as R

import rospy
import tf.transformations as tf_utils
from CalibrationUtils import * 
import tf
camera_in_hand = True 

# if camera_in_hand: 
#     from frankapy import FrankaArm
#     fa = FrankaArm() 

def tf_from_rvectvec(rvec, tvec):
    out = np.eye(4)
    out[0:3, -1] = tvec[:,0]
    out[0:3, 0:3] = rvec
    return out

def create_aruco_boards(save=True):
    boards = []
    aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_6X6_1000 )
    for i in range(10):
        markerLength = 2.25 / 100
        markerSeparation = 0.25 / 100
        dictionary = cv2.aruco.Dictionary_create(4, 6)
        dictionary.bytesList = aruco_dict.bytesList[i * 4: (i + 1) * 4]

        #aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_4X4_1000 )
        board = cv2.aruco.GridBoard_create(2, 2, markerLength, markerSeparation, dictionary)
        boards.append(board)
        if save:
            save_board(board, i)
    return boards

def save_board(board, fname):
    dpi = 96
    dpcm = dpi * 2.54
    pixels = int(5 * dpcm)
    margincm = 0.125
    marginpx = int(dpcm * margincm)
    img = cv2.aruco.drawPlanarBoard(board, (pixels,pixels), marginSize=marginpx)# for printing on A4 paper
    cv2.imwrite(f'{fname}.png', img)
    plt.imsave(f'{fname}.pdf', img, cmap='gray', dpi=dpi)

class Camera:
    def __init__(self):
        device_id = "148122060186"  # Lab: "828112071102" home:"829212070352" handcamera="148122061435" mountcamera=""148122060186""
        #device_id = "148122061435"
        # Configure streams

        pipeline = None
        pipeline = rs.pipeline()
        config = rs.config()

        config.enable_device(device_id)
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)  # rgb
        # Start streaming
        profile = pipeline.start(config)

        # eat some frames to allow autoexposure to settle
        for i in range(0, 5):
            pipeline.wait_for_frames()

        self.pipeline = pipeline

    def __enter__(self):
        def __get_frame():
            while True:
                frames = self.pipeline.wait_for_frames(10000)
                color_frame = frames.get_color_frame()
                yield color_frame



        return __get_frame()

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.pipeline.stop()

def draw_robot_base_frame(color_frame, draw_on):
    # from calibration
    rvec, tvec = (np.array([[ 1.86737514],
                [-2.18129036],
                [ 0.3117456 ]]),
        np.array([-0.49845678,  0.33394825 , 1.15018519]))

    intr = color_frame.profile.as_video_stream_profile().intrinsics
    camera_matrix = np.array([[intr.fx, 0.0, intr.ppx], [0.0, intr.fy, intr.ppy],[0.0,0.0,1.0]])
    dist_coeffs =np.array([0.0,0.0,0.0,0.0])
    cv2.drawFrameAxes(draw_on, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

def detect(color_frame, board, draw_on=None):
    intr = color_frame.profile.as_video_stream_profile().intrinsics
    camera_matrix = np.array([[intr.fx, 0.0, intr.ppx], [0.0, intr.fy, intr.ppy],[0.0,0.0,1.0]])
    dist_coeffs =np.array([0.0,0.0,0.0,0.0])
    parameters =  cv2.aruco.DetectorParameters_create()
    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())

    # Aruco marker part
    # Detect the markers in the image
    image_gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(image_gray, board.dictionary, parameters=parameters)
    #refine_corners(image_gray, markerCorners)
    rvec = None
    tvec = None
    if markerIds is not None:
        retval, rvec, tvec = cv2.aruco.estimatePoseBoard(markerCorners, markerIds, board, camera_matrix, dist_coeffs, rvec, tvec)
        print("reproj_error", reprojection_error(markerCorners, markerIds, rvec, tvec, board, camera_matrix, dist_coeffs))
        # rot_tag = np.array(cv2.Rodrigues(rvec)[0])
        # offset_direction = rot_tag[:, 0] + rot_tag[:,1]
        # offset_direction = offset_direction/np.linalg.norm(offset_direction)
        # offset = (np.sqrt(50.0)/200.0)*offset_direction #+ 0.025*rot_tag[:,2]
        # tvec[0] += offset[0]; tvec[1] += offset[1]; tvec[2] += offset[2]
    if draw_on is not None and tvec is not None:

        cv2.drawFrameAxes(draw_on, camera_matrix, dist_coeffs, rvec, tvec, 0.03)
        for corners in markerCorners:
            centre = corners[0, 0] + corners[0, 1] + corners[0, 2] + corners[0, 3]
            centre = [int(x / 4) for x in centre]
            centre = tuple(centre)
            cv2.circle(draw_on,centre,1,(0,0,255),8)
        cv2.aruco.drawDetectedMarkers(draw_on, markerCorners, borderColor=(0, 0, 255))

    return rvec, tvec

def run_loop(times=1,display=True):
    boards = create_aruco_boards()
    if display:
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    poses = {}
    with Camera() as gen:
        for _ in range(times):
            color_frame = next(gen)
            color_image = np.asanyarray(color_frame.get_data())

            for tidx, board in enumerate(boards):
                rvec, tvec = detect(color_frame, board, color_image if display else None)
                if tvec is not None:
                    poses.setdefault(tidx, []).append(get_cube_pose((rvec, tvec)))

            if display:
                draw_robot_base_frame(color_frame, color_image)
                cv2.imshow('RealSense',color_image)

                # Press esc or 'q' to close the image window
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q') or key == 27:
                    display = False
                    cv2.destroyAllWindows()
                    break
    if display:
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return poses


def get_cube_pose(detection):
    rot_CB, p_CB_C = detection
    R_CB, _ = cv2.Rodrigues(rot_CB)
    R_CB = R.from_matrix(R_CB)
    p_CB_C = p_CB_C.flatten()

    # if p_CB_C[2] < 0:
    #     # means that the camera is in the negative z-axis of the cube, which shouldnt happen
    #     R = np.array([
    #         [1, 0, 0],
    #         [0, 1, 0],
    #         [0, 0, -1]
    #     ])
    #     p_BC_B = R @ p_BC_B
    #     R_BC = R @ R_BC

    #     assert p_BC_B[2] > 0

    assert R_CB.apply([0, 0, 1])[2] >= 0

    #p_BO_B = np.array([2.5 / 100, 2.5 / 100, 2.5 / 100])
    p_BO_B = np.array([0.0, 0.0, 0.0])
    p_BO_C = R_CB.apply(p_BO_B)

    p_CO_C = p_CB_C + p_BO_C
    R_BO = R.identity()

    R_CO = R_CB * R_BO

    # assert p_CO_C[2] > p_CB_C[2]
    return R_CO.as_mrp(), p_CB_C.reshape((3, 1))

# if camera_in_hand: 
#     R_cam2gripper = np.array([[-0.00776021, -0.99987852,  0.01351743],
#  [ 0.9999001,  -0.00791866, -0.01170802],
#  [ 0.01181364 , 0.01342522 , 0.99984009]])
#     t_cam2gripper = np.array([ 0.05970745, -0.0319058,  -0.06840195])
#     Tcam2gripper = np.eye(4); Tcam2gripper[0:3, 0:3] = R_cam2gripper; Tcam2gripper[0:3, -1] = t_cam2gripper
#     ee2base_ = fa.get_pose()
#     Tgripper2base = np.eye(4); Tgripper2base[0:3, 0:3] = ee2base_.rotation; Tgripper2base[0:3, -1] = ee2base_.translation
#     Tcamera2base = np.matmul(Tgripper2base, Tcam2gripper)
#     print(Tcamera2base)

# else: 
#     Tcamera2base = np.array([[-0.10301954, -0.94251086,  0.31789974, -0.09129307],
#     [-0.9942128 ,  0.08778318, -0.06192753, -0.48345847],
#     [ 0.03046112, -0.32243974, -0.94609975 , 1.1155266 ],
#     [ 0.    ,      0.   ,       0.       ,   1.        ]])

if __name__ == '__main__':
    poses = run_loop(10, display=True)

    for tidx in poses:
        print(f"Detected (object_id={tidx}) {len(poses[tidx])} times. Printing first pose:")
        i = 0 
        #for i in range(len(poses[tidx])):
        print(poses[tidx][i][0], poses[tidx][i][1].flatten())
        T_block = tf_from_rvectvec(cv2.Rodrigues(poses[tidx][0][0])[0], poses[tidx][0][1])
        print(T_block)
        # Tblock2base = np.matmul(Tcamera2base, T_block)
        # print('in robot frame', Tblock2base)
        # pose = fa.get_pose() 
        # pose.translation = Tblock2base[0:3, -1]
        # pose.translation[2]+= 0.1
        # fa.goto_pose(pose)
        # pose.translation[2]-= 0.1
        # fa.goto_pose(pose)
    rospy.init_node('cube', anonymous=True)
    try:
        while not rospy.core.is_shutdown():
            print("in while loop")
            br = tf.TransformBroadcaster() 
            br.sendTransform(T_block[0:3,-1],
                    tf_utils.quaternion_from_matrix(T_block),
                    rospy.Time.now(),
                    'cube',
                    'camera_color_optical_frame'
                    )   

    except KeyboardInterrupt:
            rospy.core.signal_shutdown('keyboard interrupt')                      