from abc import ABC, abstractmethod

class GraspPredictor(ABC):
    def __init__(self, algorithm_name=None):
        self.algorithm_name = algorithm_name

    @abstractmethod
    def generate_grasps(self, camera):
        pass