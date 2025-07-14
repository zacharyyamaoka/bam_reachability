# BAM
from bam_descriptions import get_robot_params
from bam_reachability.generators import TablePoseGenerator, PlacePoseGenerator
from bam_reachability.utils.math_utils import get_matrix

from bam_reachability.reachability_map import ReachabilityMap, make_map_path
from bam_reachability.kin_wrapper import MockKinWrapper
# PYTHON
import time
import numpy as np
import os
from typing import Callable

arm_name = "mock"

# 1. Generate Frames

generator = TablePoseGenerator(
    pose_matrix = get_matrix(([0, 0, 0], [0, 0, 0])),
    scale = (1.25, 1.25, 1),
    xyz_step = 0.1,
    hemisphere_angle = np.deg2rad(90),
    view_step = np.deg2rad(45),
    rotation_step = np.deg2rad(360),
    viz = False,
    )

# generator = PlacePoseGenerator()

positions, orientations = generator.generate(viz=False)

print("Positions: ", positions.shape)
print("Orientations: ", orientations.shape)

# 2. Create IK/FK functions
K = MockKinWrapper(L1=0.25, L2=0.25)


# 3. Create Map and save result
reachability = ReachabilityMap(positions, orientations, K.IK, K.FK)
reachability.calculate_ik_fk_map()

