import numpy as np
from robot_arm_algos.src.pick_and_place.object import Object
from robot_arm_algos.src.tags_detection.aruco_board import ArucoBoard, ArucoBoardData
from robot_arm_algos.src.camera.realsense_camera import RealSenseCamera
from robot_arm_algos.src.pick_and_place.record_grasps import RecordGrasps
from robot_arm_algos.src.robot_camera_calibration.data_collector.zmq_robot_pose_collector import ZmqRobotPoseCollector
def main():
    aruco_board_data = ArucoBoardData(dictionary="DICT_6X6_1000",
                                marker_length = 0.0225,
                                marker_separation = 0.0025,
                                n_rows = 2,
                                n_cols = 2,
                                ids = np.array([0,1,2,3]))  
    aruco_board_tag = ArucoBoard(aruco_board_data)  
    object_ = Object(object_name = "cube 4 with arucoboard tag",
                    tag = aruco_board_tag)          
    camera = RealSenseCamera(camera_id='145522066546')
    zmq_robot_pose_collector = ZmqRobotPoseCollector("192.168.0.3", "2233")
    rg = RecordGrasps(object = object_,
                      camera = camera,
                      extrinsics_file = "robot_arm_algos/robot_camera_extrinsics.yaml",
                      grasps_file = "test.txt",
                      robot_pose_collector = zmq_robot_pose_collector,
                      camera_in_hand = True,
                      output_file = "tests/data/grasps.yaml")
    rg.run_grasp_recorder()

if __name__ == "__main__":
    main()      