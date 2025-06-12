#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node

# BAM
from bam_descriptions import get_robot_params
from bam_moveit_client import MoveItClient

# PYTHON
from typing import Tuple
import os
import numpy as np

"""
The most minimal setup to test connection is:

Launch Moveit:
ros2 launch bam_core_bringup moveit.launch.py robot:bam_arm plugin:=none rviz:=true 

Launch Joint State Buffer:
ros2 run bam_ros_utils joint_state_buffer

"""

class MoveItKinematics():
    
    def __init__(self, arm="ur"):

        rclpy.init(args=None)

        self.rp = get_robot_params(arm)

        node = Node('generic_node', namespace=f'bam_{os.environ.get('ROBOT_ID')}')
        client = MoveItClient(node,  self.rp.move_group_arm,  self.rp.base_link,  self.rp.ik_tip, spin=True)

    
    def IK(self, joint_positions)-> Tuple[bool, np.ndarray]:
        pass

    def IK(self, pose)-> Tuple[bool, np.ndarray]:
        pass




if __name__ == '__main__':
    pass

# rclpy.shutdown()
# exit(0)
