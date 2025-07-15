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
from bam_descriptions import get_robot_params
from bam_reachability.generators import TablePoseGenerator, PlacePoseGenerator
from bam_reachability.visualizer import Open3DMapViewer
from bam_reachability.kin_wrapper import OffsetWristKinWrapper
from bam_reachability.utils import xyzrpy_to_matrix, make_map_path

from bam_reachability.reachability_map import ReachabilityMap

# PYTHON
import time
import numpy as np
import os

arm_name = "ur"


# 1. Generate Frames

# 4 DOF, Low point count for fast testing with moveit
# You don't want to have points over origin or cannot reach them
generator = TablePoseGenerator(
    pose = ([0, 0.3, 0.1], [-0.05, 0, 0]),
    scale = (0.75, 0.4, 0.1),
    xyz_step = 0.25,
    hemisphere_angle = np.deg2rad(0),
    view_step = np.deg2rad(0),
    rotation_step = np.deg2rad(360),
    viz = False,
    )

# generator = PlacePoseGenerator()

positions, orientations = generator.generate(viz=False)


print("Positions: ", positions.shape)
print("Orientations: ", orientations.shape)
# print(positions)

# 2. Create IK/FK functions
rp = get_robot_params(arm_name)
K = OffsetWristKinWrapper(rp, use_tool0=True, check_collision=True)


# 3. Create Map and save result
reachability = ReachabilityMap(positions, orientations, K.IK, K.FK)

file_path = make_map_path(__file__, arm_name, K.name, generator.name)

reachability.save(file_path)


