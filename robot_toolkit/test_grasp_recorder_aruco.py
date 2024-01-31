import numpy as np
from robot_arm_algos.src.pick_and_place.object import Object
from robot_arm_algos.src.tags_detection.aruco_board import ArucoBoard, ArucoBoardData
from robot_arm_algos.src.tags_detection.aruco_tag import ArucoTag, ArucoTagData
from robot_arm_algos.src.camera.realsense_camera import RealSenseCamera
from robot_arm_algos.src.pick_and_place.record_grasps import RecordGrasps
from robot_arm_algos.src.robot_camera_calibration.data_collector.zmq_robot_pose_collector import ZmqRobotPoseCollector
test_config = {
    "aruco_board_data":  ArucoBoardData(dictionary="DICT_6X6_1000",
                            marker_length = 0.0225,
                            marker_separation = 0.0025,
                            n_rows = 2,
                            n_cols = 2,
                            ids = [8,9,10,11]),
    "zmq_ip": "192.168.0.3",
    "zmq_port": "2233",
    "extrinsics_file" : "robot_arm_algos/robot_camera_extrinsics.yaml",    
    "grasps_file": "tests/data/grasps_aruco_drill.yaml",    
    "camera_in_hand": True    
}


def main():
 
    aruco_board_tag = ArucoBoard(test_config["aruco_board_data"])
    
    # aruco_tag_data = ArucoTagData(marker_length = 0.049,
    #                             dictionary = "DICT_6X6_1000",
    #                             tag_id=2)
    # aruco_tag = ArucoTag(aruco_tag_data)
    
    object_ = Object(object_name = "cube 4 with aruco tag",
                    tag = aruco_board_tag)          
    camera = RealSenseCamera()
    zmq_robot_pose_collector = ZmqRobotPoseCollector(test_config["zmq_ip"], test_config["zmq_port"])
    rg = RecordGrasps(object = object_,
                      camera = camera,
                      extrinsics_file = test_config["extrinsics_file"],
                      robot_pose_collector = zmq_robot_pose_collector,
                      camera_in_hand = test_config["camera_in_hand"],
                      output_file = test_config["grasps_file"])
    rg.run_grasp_recorder()

if __name__ == "__main__":
    main()      