import logging
import argparse
import numpy as np

from autolab_core import RigidTransform, YamlConfig
from visualization import Visualizer3D as vis3d
from visualization import Visualizer2D as vis2d
import apriltag
from perception.realsense_sensor import RealSenseSensor
import pyrealsense2 as rs

from frankapy import FrankaArm
from scipy.spatial.transform import Rotation as R
import OpenEXR
import Imath
import matplotlib.pyplot as plt
import pickle
import time
import os
import cv2
from timeit import default_timer as timer

def exr_saver(EXR_PATH, ndarr, ndim=3):
    '''Saves a numpy array as an EXR file with HALF precision (float16)
    Args:
        EXR_PATH (str): The path to which file will be saved
        ndarr (ndarray): A numpy array containing img data
        ndim (int): The num of dimensions in the saved exr image, either 3 or 1.
                        If ndim = 3, ndarr should be of shape (height, width) or (3 x height x width),
                        If ndim = 1, ndarr should be of shape (height, width)
    Returns:
        None
    '''
    if ndim == 3:
        # Check params
        if len(ndarr.shape) == 2:
            # If a depth image of shape (height x width) is passed, convert into shape (3 x height x width)
            ndarr = np.stack((ndarr, ndarr, ndarr), axis=0)

        if ndarr.shape[0] != 3 or len(ndarr.shape) != 3:
            raise ValueError(
                'The shape of the tensor should be (3 x height x width) for ndim = 3. Given shape is {}'.format(
                    ndarr.shape))

        # Convert each channel to strings
        Rs = ndarr[0, :, :].astype(np.float16).tostring()
        Gs = ndarr[1, :, :].astype(np.float16).tostring()
        Bs = ndarr[2, :, :].astype(np.float16).tostring()

        # Write the three color channels to the output file
        HEADER = OpenEXR.Header(ndarr.shape[2], ndarr.shape[1])
        half_chan = Imath.Channel(Imath.PixelType(Imath.PixelType.HALF))
        HEADER['channels'] = dict([(c, half_chan) for c in "RGB"])

        out = OpenEXR.OutputFile(EXR_PATH, HEADER)
        out.writePixels({'R': Rs, 'G': Gs, 'B': Bs})
        out.close()
    elif ndim == 1:
        # Check params
        if len(ndarr.shape) != 2:
            raise ValueError(('The shape of the tensor should be (height x width) for ndim = 1. ' +
                              'Given shape is {}'.format(ndarr.shape)))

        # Convert each channel to strings
        Rs = ndarr[:, :].astype(np.float16).tostring()

        # Write the color channel to the output file
        HEADER = OpenEXR.Header(ndarr.shape[1], ndarr.shape[0])
        half_chan = Imath.Channel(Imath.PixelType(Imath.PixelType.HALF))
        HEADER['channels'] = dict([(c, half_chan) for c in "R"])

        out = OpenEXR.OutputFile(EXR_PATH, HEADER)
        out.writePixels({'R': Rs})
        out.close()


def detect(april, sensor, intr, tag_size):
    img, depth = sensor.frames()
    detections, det_img = april.detect(img.to_grayscale().data,  return_image=True)
    poses = []
    for i, detection in enumerate(detections):
        pose, e0, e1 = april.detection_pose(detection,intr,tag_size)
        poses.append([pose, e0, e1])
    return detections, poses


def subsample(data, rate=0.1):
    idx = np.random.choice(np.arange(len(data)), size=int(rate * len(data)))
    return idx


def make_det_one(R):
    U, _, Vt = np.linalg.svd(R)
    return U @ np.eye(len(R)) @ Vt


def get_closest_grasp_pose(T_tag_world, T_ee_world):
    tag_axes = [
        T_tag_world.rotation[:,0], -T_tag_world.rotation[:,0],
        T_tag_world.rotation[:,1], -T_tag_world.rotation[:,1]
    ]
    x_axis_ee = T_ee_world.rotation[:,0]
    dots = [axis @ x_axis_ee for axis in tag_axes]
    grasp_x_axis = tag_axes[np.argmax(dots)]
    grasp_z_axis = np.array([0, 0, -1])
    grasp_y_axis = np.cross(grasp_z_axis, grasp_x_axis)
    grasp_R = make_det_one(np.c_[grasp_x_axis, grasp_y_axis, grasp_z_axis])
    grasp_translation = T_tag_world.translation + np.array([0, 0, -cfg['cube_size'] / 2])
    return RigidTransform(
        rotation=grasp_R,
        translation=grasp_translation,
        from_frame=T_ee_world.from_frame, to_frame=T_ee_world.to_frame
    )


def point_to_object(obj_loc, distance, pan, yaw, rotate):
    r = R.from_rotvec(np.array([0, -pan, 0]))
    r1 = R.from_rotvec(np.array([0,0, yaw]))
    r2 = R.from_rotvec(np.array([0, pan, 0]))
    r3 = R.from_rotvec(np.array([np.pi, 0 ,0]))
    r4 = R.from_rotvec(np.array([0, 0, rotate]))
    translation = r1.as_matrix() @ r.as_matrix() @ np.array([distance, 0, 0])
    translation += obj_loc
    return translation, r1.as_matrix() @ r2.as_matrix() @ r3.as_matrix() @ r4.as_matrix()


def record_video(data_dir,i):
    logging.info('Detecting April Tags')
    intr = sensor.color_intrinsics
    intr_list = [intr._fx, intr._fy, intr._cx, intr._cy]
    detections, poses = detect(april, sensor, intr_list, tag_size=cfg['april_tag']['tag_size'])
    logging.info('Visualizing poses')
    img, depth_im = sensor.frames()
    exr_saver(f"{data_dir}/{i}.exr", depth_im.data.squeeze(), ndim=1)
    plt.imsave(f"{data_dir}/{i}.jpg", img.data)
    with open(f"{data_dir}/{i}.pkl", 'wb') as handle:
        pickle.dump([detections, poses], handle)


def point_and_shot(location,distance, pan, yaw, rotate, fa, cfg, duration):
    endfactor_pose = fa.get_pose()
    endfactor_pose.translation, endfactor_pose.rotation = point_to_object(location, distance, pan, yaw, rotate)
    fa.goto_pose(endfactor_pose, duration=duration)
    T_ready_world = fa.get_pose()


    logging.info('Detecting April Tags')
    detect_options = cfg['april_tag']['detector']
    options = apriltag.DetectorOptions(families=detect_options['families'],
                                       nthreads=detect_options['nthreads'],
                                       border=detect_options['border'],
                                       quad_decimate=detect_options['quad_decimate'],
                                       quad_blur=detect_options['quad_blur'],
                                       refine_edges=detect_options['refine_edges'],
                                       refine_decode=detect_options['refine_decode'],
                                       refine_pose=detect_options['refine_pose'],
                                       debug=detect_options['debug'],
                                       quad_contours=detect_options['quad_contours'], )

    april = apriltag.Detector(options=options)
    intr = sensor.color_intrinsics
    intr_list = [intr._fx, intr._fy, intr._cx, intr._cy]
    detections, poses = detect(april, sensor, intr_list, tag_size=cfg['april_tag']['tag_size'])
    if len(detections) == 0:
        return
    T_tag_cameras = [
        RigidTransform(from_frame='april_tag', to_frame='realsense', rotation=p[0][:3, :3], translation=p[0][:3, 3])
        for p in poses]
    T_camera_world = T_ready_world * T_camera_ee
    # T_tag_world = T_camera_world * T_tag_camera
    logging.info('Visualizing poses')
    img, depth_im = sensor.frames()
    points_world = T_camera_world * intr.deproject(depth_im)
    data_dir = f"{data_path}/{time.time()}"
    os.mkdir(data_dir)
    exr_saver(f"{data_dir}/d_{distance}_p_{pan}_y_{yaw}_r{rotate}.exr", depth_im.data.squeeze(), ndim=1)
    plt.imsave(f"{data_dir}/d_{distance}_p_{pan}_y_{yaw}_r{rotate}.jpg", img.data)
    with open(f"{data_dir}/d_{distance}_p_{pan}_y_{yaw}_r{rotate}.pkl", 'wb') as handle:
        pickle.dump([detections, poses, T_ready_world.matrix], handle)

    if cfg['vis_detect']:
        vis2d.imshow(img)
        vis2d.show()
        vis3d.figure()
        vis3d.pose(RigidTransform())
        idx = subsample(points_world.data.reshape(-1, 3))
        vis3d.points(points_world.data.T, color=(img.data.reshape(-1, 3) / 255.0), scale=None)
        vis3d.pose(T_ready_world)
        vis3d.pose(T_camera_world)
        for t_pose in T_tag_cameras:
            vis3d.pose(T_camera_world * t_pose)
        vis3d.show()


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.INFO)
    parser = argparse.ArgumentParser()
    parser.add_argument('--cfg', '-c', type=str, default='cfg/april_tag_pick_place_cfg.yaml')
    parser.add_argument('--no_grasp', '-ng', action='store_true')
    args = parser.parse_args()
    cfg = YamlConfig(args.cfg)
    T_camera_ee = RigidTransform.load(cfg['T_camera_ee_path'])
    T_camera_mount_delta = RigidTransform.load(cfg['T_camera_mount_path'])
    ctx = rs.context()
    device_id = ctx.devices[cfg['rs']['id']].get_info(rs.camera_info.serial_number)
    sensor = RealSenseSensor(device_id, frame=cfg['rs']['frame'], filter_depth=cfg['rs']['filter_depth'])
    sensor.start()

    detect_options = cfg['april_tag']['detector']
    options = apriltag.DetectorOptions(families=detect_options['families'],
                                       nthreads=detect_options['nthreads'],
                                       border=detect_options['border'],
                                       quad_decimate=detect_options['quad_decimate'],
                                       quad_blur=detect_options['quad_blur'],
                                       refine_edges=detect_options['refine_edges'],
                                       refine_decode=detect_options['refine_decode'],
                                       refine_pose=detect_options['refine_pose'],
                                       debug=detect_options['debug'],
                                       quad_contours=detect_options['quad_contours'], )

    april = apriltag.Detector(options=options)
    # e1 = timer()
    #
    #
    # i = 0
    # save_path = "test1"
    # if not os.path.exists(save_path):
    #     os.mkdir(save_path)
    # while True:
    #     record_video(save_path, i)
    #     i += 1
    #     e2 = timer()
    #     t = e2 - e1
    #     if t>60:
    #         break
    # exit(0)
    logging.info('Starting robot')
    fa = FrankaArm()
    endfactor_pose =  fa.get_pose()
    object="7_white_iron_3"
    logging.info('Init camera')

    for o in range(3):
        if o== 0:
            pan = np.pi / 3
            location = np.array([0.7, 0.05, 0])
        elif o == 1:
            pan = np.pi / 4
            location = np.array([0.8, 0, 0])
        else:
            pan = np.pi/6
            location = np.array([0.87, 0, 0])
        data_path = f"{object}_{pan}"
        if not os.path.exists(data_path):
            os.mkdir(data_path)
        for k in range(3):
            distance = 0.3 + 0.1 * k
            for j in range(2):
                print((o, k, j))
                m = False
                fa.reset_joints()
                for i in range(5):
                    start = (0.7 if o != 2 else 0.8) if j == 0 else (1.3 if o != 2 else 1.2)
                    yaw = np.pi * (start + 0.07 * i * (-1) ** j)
                    if o == 2 and i == 4:
                        continue
                    for l in range(6):
                        if l != 0:
                            duration = 0.5
                        elif o != 2:
                            duration = 5
                        else:
                            duration = 6
                        rotate = np.pi * ((5 * m + l * (-1) ** m) / 6.0) * (-1) ** j
                        print((pan,yaw,rotate))
                        point_and_shot(location, distance, pan, yaw, rotate, fa, cfg, duration)
                    m = not m
