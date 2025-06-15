#!/usr/bin/env python3

"""
*** this requires Pinnochio to be setup on computer

"""

# ROS
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from tf_transformations import euler_from_quaternion

# BAM
from bam_descriptions import get_robot_params, RobotParam
from bam_moveit_client import MoveItClient
from bam_ros_utils.msgs import get_joint_state, get_pose_stamped
from bam_ros_utils.np_helper import to_numpy_xyzrpy
from bam_ros_utils.geometry import move_relative

# to avoid any naming issues naming these files as kin
#from bam_kinematics_dynamics.six_dof.offset_wrist_kinematics import OffsetWristKinematics as Kinematics
from bam_kinematics_dynamics import OffsetWristKinematics

# PYTHON
from typing import Tuple
import os
import numpy as np


class OffsetWristKinWrapper():
    
    def __init__(self, rp: RobotParam, use_tool0=True):

        # Limits can also be read from URDF, but this is nice because no other dependecies...
        self.K = OffsetWristKinematics(rp.dh_list, rp.base_link, rp.lower_limits, rp.upper_limits, use_tool0=use_tool0, verbose=False)
        self.robot_params = rp
        self.name = rp.name + "_offset"
    
    def IK(self, pose: np.ndarray)-> Tuple[bool, np.ndarray]:

        pose_msg = get_pose_stamped(pose[:3], pose[3:], self.robot_params.base_link)

        success_list, ik_sol_list = self.K.ik(pose_msg)

        success, ik_sol = self.K.select_sol_by_id(success_list, ik_sol_list, 2)

        return success, np.array(ik_sol)

    def FK(self, joint_positions: np.ndarray)-> Tuple[bool, np.ndarray]:

        pose_stamped = self.K.fk(joint_positions.tolist())

        pose_array = to_numpy_xyzrpy(pose_stamped)

        return True, pose_array


if __name__ == '__main__':

    rp = get_robot_params("ur")
    K = OffsetWristKinWrapper(rp, use_tool0=True)
    lower_bounds = K.robot_params.lower_limits
    upper_bounds = K.robot_params.upper_limits

    num_tests = 100
    success_count = 0

    for i in range(num_tests):
        q_rand = np.random.uniform(lower_bounds, upper_bounds)
        fk_success, q_rand_pose = K.FK(q_rand)
        if not fk_success:
            print(f"[{i}] FK q_rand failed")
            continue

        ik_success, ik_sol = K.IK(q_rand_pose)
        if not ik_success:
            print(f"[{i}] IK q_rand_pose failed")
            continue

        fk_success, fk_sol = K.FK(ik_sol)
        if not fk_success:
            print(f"[{i}] FK ik_sol failed")
            continue

        # You cannot directly compare as IK has 8 possible solutions!
        if np.allclose(q_rand_pose, fk_sol):
            success_count += 1
        else:
            print(f"[{i}] FK mismatch\n  q: {np.round(q_rand,3)}\n q_pose: {np.round(q_rand_pose,3)}\n  ik_sol : {np.round(fk_sol,3)}")
            break

    print(f"\n{success_count}/{num_tests} solutions matched")
