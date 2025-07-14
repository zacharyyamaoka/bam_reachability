#!/usr/bin/env python3

# BAM
from bam_reachability.kin_wrapper.kin_wrapper import KinWrapper
from bam_reachability.utils.math_utils import get_matrix, matrix_to_xyzrpy, xyzrpy_to_matrix

# PYTHON
from typing import Tuple
import os
import numpy as np



"""
Simple class to mock and test the kinematics

Assume its a 2 DOF robot arm...

I would expect this to look like a cylinder workspace instead of a sphere!

    - With a uniform chance of failure, the histogram should have more points
      towards theouter edges,as there is more area inside the circle there

- you could do a 3 DOF arm, but the kinematics are slightly more complex... can do if needed later
"""

class MockKinWrapper(KinWrapper):

    def __init__(self, L1=0.25, L2=0.25, seed=1337, random_ik_fail=True):
        """
        L1: Length of link 1 (m)
        L2: Length of link 2 (m)
        """
        super().__init__("Mock")
        self.L1 = L1
        self.L2 = L2
        self.seed = seed
        self.rng = np.random.default_rng(seed)  # create a per-instance random generator
        self.random_ik_fail = random_ik_fail

    def seed_reset(self):
        """Reset the RNG to the original seed."""
        self.rng = np.random.default_rng(self.seed)

    def IK(self, pose_matrix: np.ndarray)-> Tuple[bool, np.ndarray]:
        xyz, rpy = matrix_to_xyzrpy(pose_matrix)

        # calculate J1,J2 by assuming a 2 DOF arm
        x = xyz[0]
        y = xyz[1]


        pose_radius = np.sqrt(x**2 + y**2)
        max_reach = self.L1 + self.L2

        if pose_radius >= max_reach:
            return False, np.zeros(6)

        # to test out different colors, its helpful to give give different poses different reachability scores!
        if self.random_ik_fail:

            # prob is 0.0 at pose_radius = max_reach, and 1.0 at pose_radius = 0.0
            prob = 1.0 - (pose_radius / max_reach)
            if prob <= 0.2: # add area with 100% success    
                pass
            
            elif (self.rng.random() < prob): 
                return False, np.zeros(6)

        q1, q2, success = rr_ik(self.L1, self.L2, x, y)

        J1 = q1
        J2 = q2
        J3 = xyz[2] #Although it's strange to assign z to J3, it works just like the wrist pass through values below
        J4 = rpy[0]
        J5 = rpy[1]
        J6 = rpy[2]

        return success, np.array([J1, J2, J3, J4, J5, J6])

    def FK(self, joint_positions: np.ndarray)-> Tuple[bool, np.ndarray]:

        q1 = joint_positions[0]
        q2 = joint_positions[1]
        x1, y1, x2, y2 = rr_fk(self.L1, self.L2, q1, q2)

        x = x2
        y = y2
        z = joint_positions[2] # Pass through value as well! just like wrist
        rx = joint_positions[3]
        ry = joint_positions[4]
        rz = joint_positions[5]

        pose_matrix = xyzrpy_to_matrix([x, y, z], [rx, ry, rz])

        return True, pose_matrix


# from bam_kinematics_dynamics/rr_kinematics.py
# Nice to not have to maintain yet another set of functions...
def rr_fk(L1, L2, q1, q2):
        """
        Input:
        Output:
        - x1
        - y1
        - x2
        - y2
        """

        x1, y1 = np.cos(q1) * L1, np.sin(q1) * L1
        x2, y2 = x1 + np.cos(q1 + q2) * L2, y1 + np.sin(q1 + q2) * L2

        return x1, y1, x2, y2

def rr_ik(L1, L2, x, y):

    d2 = x**2 + y**2

    d = d2**0.5
    if d < 1e-6:
        return 0, 0, False # near origin
    
    base_angle = np.arctan2(y, x)

    # C:= cos of the first angle in a triangle defined by L1 and L2
    # the formula comes from the Law of Cosines

    C = (L1**2 + d2 - L2**2) / (2*L1*d)

    # Check if the target is reachable.
    # If C > 1, it means the target is outside the workspace of the robot
    # If C < -1, it means the target is inside the workspace, but not reachable
    if not -1 <= C <= 1:
        # The target is not reachable
        return 0, 0, False

    c = np.arccos(C)

    # B:= cos of the second angle
    B = (L1**2 + L2**2 - d2) / (2*L1*L2)
    b = np.arccos(B)

    return base_angle+c, b-np.pi, True



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

    kinematics = MockKinWrapper(L1=0.5, L2=0.5, random_ik_fail=False)

    # ik_success, ik_sol = kinematics.IK(pose)
    # fk_success, fk_sol = kinematics.FK(ik_sol)

    # print(pose)
    # print(np.round(ik_sol,3))
    # print(np.round(fk_sol,3))