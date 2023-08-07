import numpy as np
import cv2
import zmq
from ._data_collector import RobotPoseCollector

"""
queries a zmq server located in the realtime machine to get ee frame. 
Useful to move Franka manually when status LED is white. TF tree maybe corrupted when light is white
"""
class ZmqRobotPoseCollector(RobotPoseCollector):
    def __init__(self, zmq_ip, zmq_port):
        RobotPoseCollector.__init__(self)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        socket_address = "tcp://" + zmq_ip + ":" + zmq_port 
        self.socket.connect(socket_address)
    
    """
    todo: see if blocking call takes long and throws error that not getting reply from server
    """
    def get_ee_frame(self,):
        self.socket.send_string("data")#even an empty message would do
        message = self.socket.recv()# receives EE pose 
        zmq_pose = np.frombuffer(message).astype(np.float32)
        zmq_pose = np.reshape(a=zmq_pose, newshape=(4,4), order='F')
        zmq_position = np.array(zmq_pose[:3,3])
        zmq_rot = np.array(zmq_pose[:3,:3])
        return cv2.Rodrigues(zmq_rot)[0], zmq_position