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
from bam_descriptions import get_robot_params
from bam_moveit_client import MoveItClient
from bam_ros_utils.msgs import get_joint_state, get_pose_stamped
from bam_ros_utils.np_helper import to_numpy_xyzrpy
from bam_ros_utils.geometry import move_relative

from bam_kinematics_dynamics import PinKinematics, urdf_to_models

# PYTHON
from typing import Tuple
import os
import numpy as np


class PinKinWrapper():
    
    def __init__(self, arm="ur", verbose=True):

        rp = get_robot_params(arm)

        model, collision_model, visual_model = urdf_to_models(urdf_xml=rp.urdf_xml, package_name=rp.mesh_package_name)
        # K_urdf = PinKinematics(model, rp.base_link, rp.ik_tip, verbose)
        self.K = PinKinematics(model, rp.base_link, "tool0", verbose) # it can do the transform for me! which is helpful to confirm
        # self.K = PinKinematics(model, rp.base_link, rp.ik_tip, verbose) # it can do the transform for me! which is helpful to confirm

        self.robot_params = rp


    def IK(self, pose: np.ndarray)-> Tuple[bool, np.ndarray]:
        """
        Very cool! So PinKinWrapper can not by it self completely the map
        beacuse it cannot calculate the IK to get the joint positions to use for the FK
        I can replace it with the Offset Wrist IK though... then you can check consitentcy!
        """
        raise NotImplementedError("IK method must be provided by another IK")

        return False, None

    def FK(self, joint_positions: np.ndarray)-> Tuple[bool, np.ndarray]:

        pose_stamped = self.K.fk(joint_positions)

        pose = to_numpy_xyzrpy(pose_stamped)

        return True, pose


if __name__ == '__main__':

    K = PinKinWrapper(arm="ur")

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

        success_count += 1

    print(f"\n{success_count}/{num_tests} solutions")

