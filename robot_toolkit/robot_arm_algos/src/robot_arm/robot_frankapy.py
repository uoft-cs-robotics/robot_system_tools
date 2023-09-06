import numpy as np
import random
import math
import tf.transformations as ros_tf_utils
from autolab_core import RigidTransform
from frankapy import FrankaArm
from ._robot_arm import RobotArm

class RobotFrankaPy(RobotArm):
    def __init__(self, with_franka_gripper = True ,robot_id = 1, init_node = True):
        self.fpy_object = FrankaArm(robot_num = robot_id,
                                    with_gripper = with_franka_gripper,
                                    init_node = init_node)

    def get_ee_frame(self,):
        ee_frame = self.fpy_object.get_pose()
        output_ee_frame = np.eye(4)
        output_ee_frame[0:3, 0:3] = ee_frame.rotation
        output_ee_frame[0:3,-1] = ee_frame.translation
        return output_ee_frame
    
    def get_joint_angles(self,):
        return self.fpy_object.get_joints()
    
    def go_to_joint_angles(self, goal_joint_config, 
                                ignore_virtual_walls = True, 
                                use_impedance = False, 
                                duration = 5):
        self.fpy_object.goto_joints(joints = goal_joint_config,
                                    ignore_virtual_walls = ignore_virtual_walls,
                                    use_impedance = use_impedance,
                                    duration = duration)
        return
    
    def go_to_ee_pose(self, goal_ee_pose, 
                            ignore_virtual_walls = True, 
                            use_impedance = False, 
                            duration = 3):
        self.fpy_object.goto_pose(tool_pose = goal_ee_pose, 
                                ignore_virtual_walls=ignore_virtual_walls, 
                                use_impedance=use_impedance, 
                                duration = duration)
        return

    def go_to_ee_pose_delta(self, delta_ee_pose, 
                                ignore_virtual_walls = True, 
                                use_impedance = False, 
                                duration = 3):
        self.fpy_object.goto_pose_delta(delta_tool_pose = delta_ee_pose,
                                        ignore_virtual_walls = ignore_virtual_walls,
                                        use_impedance = use_impedance, 
                                        duration = duration
                                        )
        return

    def reset_arm(self,):
        self.fpy_object.reset_joints(self,)
    
    def open_gripper(self,):
        self.fpy_object.open_gripper(self,)
    
    def close_gripper(self,):
        self.fpy_object.close_gripper(self,)
    
    def get_randomized_absolute_poses(self, n_poses, flag_camera_in_hand = True):
        initial_pose = self.fpy_object.get_pose()
        from itertools import cycle
        toggle_signage = cycle([-1.0000,1.0000])
        def toggle_sign():
            return toggle_signage.__next__()

        ee_poses = []
        ee_poses.append(initial_pose)
        r, p ,y = ros_tf_utils.euler_from_matrix(initial_pose.rotation, 'rxyz')
        initial_translation = initial_pose.translation
        z_grid = np.array([1.0, 0.0, -1.0]) * 0.05
        signage = [-1.0000,1.0000]
        goal_pose = initial_pose.copy()
        for i in range(n_poses):
            x_offset = i % int(n_poses/len(z_grid))
            y_offset = i % int(n_poses/len(z_grid))
            sign = toggle_sign()
            x_offset *= 0.01 * random.choice(signage)
            y_offset *= 0.01 * random.choice(signage)    
            z_offset = random.choice(z_grid)            
            r_new = r + math.radians(random.uniform(0.0,30.0)) * random.choice(signage)
            p_new = p + math.radians(random.uniform(0.0,30.0)) * random.choice(signage)
            y_new = y + math.radians(random.uniform(0.0,80.0)) * random.choice(signage)
            goal_pose = RigidTransform(from_frame='franka_tool', 
                                    to_frame='world',
                                    rotation=ros_tf_utils.euler_matrix(r_new, p_new, y_new, 'rxyz')[0:3,0:3],
                                    translation=initial_pose.translation + np.array([x_offset, y_offset, z_offset]))
            ee_poses.append(goal_pose)
            # print(math.degrees(r_new),math.degrees(p_new),math.degrees(y_new),x_offset,y_offset,z_offset)

        return ee_poses

    # def get_randomized_absolute_poses(self, n_poses, flag_camera_in_hand = True):
    #     initial_pose = self.fpy_object.get_pose()
    #     from itertools import cycle
    #     toggle_signage = cycle([-1.0000,1.0000])
    #     def toggle_sign():
    #         return toggle_signage.__next__()

    #     ee_poses = []
    #     ee_poses.append(initial_pose)
    #     r, p ,y = ros_tf_utils.euler_from_matrix(initial_pose.rotation, 'rxyz')
    #     initial_translation = initial_pose.translation
    #     z_grid = np.array([1.0, 0.0, -1.0]) * 0.05
    #     signage = [-1.0000,1.0000]
    #     goal_pose = initial_pose.copy()
    #     for i in range(n_poses):
    #         x_offset = i % int(n_poses/len(z_grid))
    #         y_offset = i % int(n_poses/len(z_grid))
    #         sign = toggle_sign()
    #         x_offset *= 0.01 * random.choice(signage)
    #         y_offset *= 0.01 * random.choice(signage)    
    #         z_offset = random.choice(z_grid)            
    #         r_new = r + math.radians(random.uniform(25.0, 40.0)) * random.choice(signage)
    #         p_new = p + math.radians(random.uniform(25.0, 40.0)) * random.choice(signage)
    #         y_new = y + math.radians(random.uniform(50.0, 80.0)) * random.choice(signage)
    #         goal_pose = RigidTransform(from_frame='franka_tool', 
    #                                 to_frame='world',
    #                                 rotation=ros_tf_utils.euler_matrix(r_new, p_new, y_new, 'rxyz')[0:3,0:3],
    #                                 translation=initial_pose.translation + np.array([x_offset, y_offset, z_offset]))
    #         ee_poses.append(goal_pose)
    #     return ee_poses        

    
