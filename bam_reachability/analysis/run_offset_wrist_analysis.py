#!/usr/bin/env python3

"""
I am getting some numerical instability:

[ERROR] Target poses do not match
pose_1:  [ 0.5       0.3       0.5      -3.141593 -1.570796  0.      ]
pose_2:  [ 0.5       0.3       0.5       3.141593 -1.570796  0.      ]
Trying to compare angles directly
Success
Processed 2933 / 3024 poses
Processed 2934 / 3024 poses
Processed 2935 / 3024 poses
[ERROR] Target poses do not match
pose_1:  [ 0.5       0.3       0.5      -2.356194  1.570796  0.      ]
pose_2:  [ 0.5       0.3       0.5      -1.735945  1.570796  0.753151]
Trying to compare angles directly
XYZ error: 3.3306690738754696e-16 > 1e-06
RPY error: 0.1329017949793698 > 0.001
Failed
IK/FK not consistent
Processed 2936 / 3024 poses
[ERROR] Target poses do not match
pose_1:  [ 0.5       0.3       0.5      -3.141593  1.570796  0.      ]
pose_2:  [0.5      0.3      0.5      3.141593 1.570796 0.      ]
Trying to compare angles directly
Success

It sure is interesting how you start to see the edge cases as you test a greater number of poses!

"""
# ROS
import rclpy

# BAM
from bam_reachability.generators import rectangular_generator, view_generator, visualize_vectors, matrix_to_rpy, visualize_frames, table_workspace_generator
from bam_reachability.visualizer import AlignedSlicer
from bam_reachability.kin_wrapper import OffsetWristKinWrapper
from bam_reachability.utils import xyzrpy_to_matrix

from bam_reachability.reachability_map import ReachabilityMap

# PYTHON
import time
import numpy as np
import os

arm_name = "ur"

# 1. Generate Frames

# frames = rectangular_generator(scale=(1, 1, 1), step=0.05)
# positions = rectangular_generator(scale=(1, 1, 1), step=0.1)

table_pose = xyzrpy_to_matrix([0,0.5,0], [-0.3, 0, 0])
positions = table_workspace_generator(table_pose, scale=(1, 0.5, 0.2), step=0.1)

# slicer = AlignedSlicer(positions, step=0.1)
# slicer.run()

z_axis = table_pose[:3, 2]

R_list = view_generator(
    inital_view=-1*z_axis,
    hemisphere_angle=np.deg2rad(0),
    view_step=np.deg2rad(85/2),
    rotation_step=np.deg2rad(360/8)
)

# visualize_frames(R_list, only_z=False)

orientations = np.array(matrix_to_rpy(R_list))

# n_orientations = 20
# orientations = np.random.uniform(low=-3.14, high=3.14, size=(positions.shape[0], n_orientations, 3))

print("Frames: ", positions.shape)
print("Orientations: ", orientations.shape)

# 2. Create IK/FK functions

# You must run:
# ros2 launch ur5e_moveit_config demo.launch.py

K = OffsetWristKinWrapper(arm=arm_name, use_tool0=True)

reachability = ReachabilityMap(positions, orientations, K.IK, K.FK)

base_dir = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(base_dir, f'{arm_name}_table_offset_wrist_map')

reachability.save(file_path)


