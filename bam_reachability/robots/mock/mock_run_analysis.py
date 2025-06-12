#!/usr/bin/env python3

from bam_reachability.generators import rectangular_generator, generate_deviation_vectors
from bam_reachability.visualizer import AlignedSlicer
from bam_reachability.kinematics import MockKinematics

from bam_reachability.reachability_map import ReachabilityMap

import numpy as np
import os

frames = rectangular_generator(scale=(1, 1, 1), step=0.05)
orientations = generate_deviation_vectors([0,0,1], np.deg2rad(180), np.deg2rad(30))

print("Frames: ", frames.shape)
print("Orientations: ", orientations.shape)

kinematics = MockKinematics(L1=0.3, L2=0.3)

# slicer = AlignedSlicer(frames, step=0.05)
# slicer.run()

reachability = ReachabilityMap(frames, orientations, kinematics.IK, kinematics.FK)


base_dir = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(base_dir, 'mock_map.pkl')

reachability.save(file_path)
