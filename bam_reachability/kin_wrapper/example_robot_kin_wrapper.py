#!/usr/bin/env python3

# BAM
from bam_reachability.kin_wrapper.kin_wrapper import KinWrapper
from bam_reachability.utils.math_utils import get_matrix, matrix_to_xyzrpy, xyzrpy_to_matrix
from bam_reachability.kin_wrapper.pin_kinematics import PinKinematics
# PYTHON
from typing import Tuple, Optional
import os
import numpy as np
import example_robot_data



"""
Simple class to mock and test the kinematics

Assume its a 2 DOF robot arm...

I would expect this to look like a cylinder workspace instead of a sphere!

    - With a uniform chance of failure, the histogram should have more points
      towards theouter edges,as there is more area inside the circle there

- you could do a 3 DOF arm, but the kinematics are slightly more complex... can do if needed later
"""

class ExampleRobotKinWrapper(KinWrapper):

    def __init__(self, robot_name: str, base_link: str, ik_tip_link: str, verbose: bool = False):

        super().__init__(robot_name)

        robot = example_robot_data.load(robot_name)

        self.model = robot.model
        self.collision_model = robot.collision_model
        self.visual_model = robot.visual_model

        self.K = PinKinematics(self.model, base_link, ik_tip_link, verbose)


    def IK(self, pose_matrix: np.ndarray, q_seed: Optional[np.ndarray] = None)-> Tuple[bool, np.ndarray]:
        success, joint_positions = self.K.IK(pose_matrix, q_seed)

        return success, joint_positions


    def FK(self, joint_positions: np.ndarray)-> Tuple[bool, np.ndarray]:
        success, pose_matrix = self.K.FK(joint_positions)

        return success, pose_matrix

class UR5KinWrapper(ExampleRobotKinWrapper):

    def __init__(self, verbose: bool = False):
        assert False, "UR5 Mesh does not display properly in meshcat"
        super().__init__('ur5', 'base_link', 'tool0', verbose)

class UR3KinWrapper(ExampleRobotKinWrapper):

    def __init__(self, verbose: bool = False):
        super().__init__('ur3', 'base_link', 'tool0', verbose)

class UR10KinWrapper(ExampleRobotKinWrapper):

    def __init__(self, verbose: bool = False):
        super().__init__('ur10', 'base_link', 'tool0', verbose)


class PandaKinWrapper(ExampleRobotKinWrapper):

    def __init__(self, verbose: bool = False):
        super().__init__('panda', 'panda_link0', 'panda_link7', verbose)



if __name__ == '__main__':
    from bam_reachability.generators import rectangle_point_generator, generate_deviation_vectors


    # frames = rectangle_point_generator(scale=(1, 1, 1), step=0.05)
    # orientations = generate_deviation_vectors([0,0,1], np.deg2rad(180), np.deg2rad(30))
    # print("Frame: ", frames.shape)
    # print("Orientation: ", orientations.shape)
    # print("")

    # frame = frames[0, :]
    # orientation = orientations[0, :]
    # pose = np.hstack((frame, orientation))  # shape (N, 6)

    # print("Frame: ", frame.shape)
    # print("Orientation: ", orientation.shape)
    # print("Pose: ", pose.shape)

    panda = PandaKinWrapper()
    ur3 = UR3KinWrapper()
    ur5 = UR5KinWrapper()

    # kinematics = ExampleRobotKinWrapper(robot_name="panda", base_link="panda_link0", ik_tip_link="panda_link7")

    # ik_success, ik_sol = kinematics.IK(pose)
    # fk_success, fk_sol = kinematics.FK(ik_sol)

    # print(pose)
    # print(np.round(ik_sol,3))
    # print(np.round(fk_sol,3))