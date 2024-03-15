from abc import ABC, abstractmethod

class ObjectPoseEstimator(ABC):
    def __init__(self, algorithm_name=None):
        self.algorithm_name = algorithm_name

    @abstractmethod
    def estimate_pose(self, camera):
        pass