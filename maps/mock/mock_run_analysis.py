#!/usr/bin/env python3

from bam_reachability.generators import rectangle_point_generator, view_generator, visualize_frames, matrix_to_rpy
from bam_reachability.visualizer import O3DMapViewer
from bam_reachability.kin_wrapper import MockKin

from bam_reachability.reachability_map import ReachabilityMap

import numpy as np
import os

positions = rectangle_point_generator(scale=(1, 1, 1), step=0.2)

R_list = view_generator(
    inital_view=[0, 0, -1],
    hemisphere_angle=np.deg2rad(90),
    view_step=np.deg2rad(45),
    rotation_step=np.deg2rad(360)
)

# visualize_frames(R_list, only_z=True)

orietnations = [matrix_to_rpy(R) for R in R_list]

print("Frames: ", positions.shape)
print("Orientations: ", orientations.shape)

kinematics = MockKin(L1=0.3, L2=0.3)

reachability = ReachabilityMap(positions, orientations, kinematics.IK, kinematics.FK)

base_dir = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(base_dir, 'mock_map')

reachability.save(file_path)
