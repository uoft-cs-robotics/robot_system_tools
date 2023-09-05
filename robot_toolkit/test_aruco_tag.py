import numpy as np
import matplotlib.pyplot as plt
import cv2
from robot_arm_algos.src.logger import logger
from robot_arm_algos.src.camera.camera import Camera
from robot_arm_algos.src.tags_detection.aruco_tag import ArucoTag, ArucoTagData


def main():
    camera_matrix = np.array([[909.34179688, 0.0, 643.8762207 ],
                            [0.0, 908.06414795, 348.60513306],
                            [0.0, 0.0, 1.0]])
    dist_coeffs = np.array([0.0,0.0,0.0,0.0])                            
    camera = Camera(camera_matrix, dist_coeffs)

    aruco_tag_data = ArucoTagData(marker_length = 0.0404,
                                dictionary = "DICT_4X4_1000")
    aruco_tag = ArucoTag(aruco_tag_data)
    color_image = cv2.imread("tests/data/aruco_tag_test.png")
    corners, ids, _  = aruco_tag.detect_markers(color_image)
    image_gray =  cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY) 
    aruco_tag.refine_corners(image_gray, corners)
    obj_points, img_points = aruco_tag.get_matched_object_points(corners)
    rvec, tvec = aruco_tag.estimate_pose(obj_points, img_points, camera)
    reproj_error, error_variance = aruco_tag.compute_reprojection_error(rvec,
                                                        tvec,
                                                        obj_points,
                                                        img_points,
                                                        camera)    

    logger.info(f"reprojection error = {reproj_error}, error_variance = {error_variance}")

    aruco_tag.draw_detections(color_image, corners)
    aruco_tag.draw_estimated_frame(color_image, camera, rvec, tvec)
    plt.imshow(color_image)
    plt.show(block=False)
    plt.pause(4.0)
    plt.close()

if __name__ == "__main__":
    main()    