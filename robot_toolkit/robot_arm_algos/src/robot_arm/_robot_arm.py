from abc import ABC, abstractmethod
class RobotArm(ABC):
    def __init__(self, robot_id = 0):
        self.robot_id = robot_id
    
    @abstractmethod    
    def get_ee_frame(self,):
        pass

    @abstractmethod
    def get_joint_angles(self,):
        pass

    @abstractmethod
    def go_to_joint_angles(self, goal_joint_config):
        pass

    @abstractmethod
    def go_to_ee_pose(self, goal_ee_pose):
        pass

    @abstractmethod
    def go_to_ee_pose_delta(self, delta_ee_pose):
        pass

    # @abstractmethod 
    # def open_gripper(self,):
    #     pass

    # @abstractmethod
    # def close_gripper(self,):
    #     pass

    # @abstractmethod 
    # def close_gripper(self,):
    #     pass

